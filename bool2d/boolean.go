// Package bool2d implements topology-preserving boolean operations on
// github.com/unixpickle/model3d/model2d meshes.
//
// Inputs are interpreted as polygon boundaries. They may contain multiple
// connected components and holes, but should be closed. Output segments are
// oriented with the result interior on their right, matching model3d's mesh
// convention.
package bool2d

import (
	"fmt"
	"math"
	"sort"

	model "github.com/unixpickle/model3d/model2d"
)

const (
	defaultMaxInputSegments     = 10_000
	defaultMaxIntersectionCuts  = 250_000
	defaultMaxIntersectionPairs = 5_000_000
	defaultMaxContactDegree     = 1_024
)

// Options configures safety limits for boolean operations. A zero field uses
// the corresponding value from DefaultOptions. Negative values are invalid.
type Options struct {
	MaxInputSegments     int
	MaxIntersectionCuts  int
	MaxIntersectionPairs int
	MaxContactDegree     int
}

// DefaultOptions returns the limits used by Union, Intersection, and
// Difference.
func DefaultOptions() Options {
	return Options{
		MaxInputSegments:     defaultMaxInputSegments,
		MaxIntersectionCuts:  defaultMaxIntersectionCuts,
		MaxIntersectionPairs: defaultMaxIntersectionPairs,
		MaxContactDegree:     defaultMaxContactDegree,
	}
}

func normalizeOptions(options Options) (Options, error) {
	defaults := DefaultOptions()
	fields := []struct {
		name                string
		value, valueDefault *int
	}{
		{"MaxInputSegments", &options.MaxInputSegments, &defaults.MaxInputSegments},
		{"MaxIntersectionCuts", &options.MaxIntersectionCuts, &defaults.MaxIntersectionCuts},
		{"MaxIntersectionPairs", &options.MaxIntersectionPairs, &defaults.MaxIntersectionPairs},
		{"MaxContactDegree", &options.MaxContactDegree, &defaults.MaxContactDegree},
	}
	for _, field := range fields {
		if *field.value < 0 {
			return Options{}, fmt.Errorf("meshbool/bool2d: option %s must not be negative", field.name)
		}
		if *field.value == 0 {
			*field.value = *field.valueDefault
		}
	}
	return options, nil
}

// Validate checks that no option is negative. Zero values are valid and mean
// to use the corresponding default.
func (o Options) Validate() error {
	_, err := normalizeOptions(o)
	return err
}

// ComplexityError indicates that an operation was stopped before an
// adversarial arrangement could consume unbounded memory.
type ComplexityError struct {
	Stage string
	Limit int
}

func (c *ComplexityError) Error() string {
	return fmt.Sprintf("meshbool/bool2d: %s exceeds safety limit of %d", c.Stage, c.Limit)
}

// TopologyError indicates that numerical degeneracy prevented construction of
// a closed, consistently oriented manifold result.
type TopologyError struct {
	Problem string
	Count   int
}

func (t *TopologyError) Error() string {
	if t.Count != 0 {
		return fmt.Sprintf("meshbool/bool2d: invalid output topology: %s (%d)", t.Problem, t.Count)
	}
	return fmt.Sprintf("meshbool/bool2d: invalid output topology: %s", t.Problem)
}

// Union computes the union of zero or more meshes.
func Union(meshes ...*model.Mesh) *model.Mesh {
	return UnionWithOptions(DefaultOptions(), meshes...)
}

// UnionWithOptions computes a union with configurable safety limits. It
// panics on complexity or topology failure.
func UnionWithOptions(options Options, meshes ...*model.Mesh) *model.Mesh {
	result, err := UnionCheckedWithOptions(options, meshes...)
	if err != nil {
		panic(err)
	}
	return result
}

// UnionChecked is like Union, but reports complexity and output-topology
// failures as errors.
func UnionChecked(meshes ...*model.Mesh) (result *model.Mesh, err error) {
	return UnionCheckedWithOptions(DefaultOptions(), meshes...)
}

// UnionCheckedWithOptions is like UnionWithOptions, but returns complexity,
// topology, and invalid-option failures as errors.
func UnionCheckedWithOptions(options Options, meshes ...*model.Mesh) (result *model.Mesh, err error) {
	options, err = normalizeOptions(options)
	if err != nil {
		return nil, err
	}
	defer recoverComplexity(&err)
	return boolean(options, booleanUnion, meshes), nil
}

// Intersection computes the intersection of zero or more meshes. With no
// arguments, it returns an empty mesh.
func Intersection(meshes ...*model.Mesh) *model.Mesh {
	return IntersectionWithOptions(DefaultOptions(), meshes...)
}

// IntersectionWithOptions computes an intersection with configurable safety
// limits. It panics on complexity or topology failure.
func IntersectionWithOptions(options Options, meshes ...*model.Mesh) *model.Mesh {
	result, err := IntersectionCheckedWithOptions(options, meshes...)
	if err != nil {
		panic(err)
	}
	return result
}

// IntersectionChecked is like Intersection, but reports complexity and
// output-topology failures as errors.
func IntersectionChecked(meshes ...*model.Mesh) (result *model.Mesh, err error) {
	return IntersectionCheckedWithOptions(DefaultOptions(), meshes...)
}

// IntersectionCheckedWithOptions is like IntersectionWithOptions, but returns
// complexity, topology, and invalid-option failures as errors.
func IntersectionCheckedWithOptions(options Options, meshes ...*model.Mesh) (result *model.Mesh, err error) {
	options, err = normalizeOptions(options)
	if err != nil {
		return nil, err
	}
	defer recoverComplexity(&err)
	return boolean(options, booleanIntersection, meshes), nil
}

// Difference subtracts every mesh in subtract from the first mesh.
func Difference(first *model.Mesh, subtract ...*model.Mesh) *model.Mesh {
	return DifferenceWithOptions(DefaultOptions(), first, subtract...)
}

// DifferenceWithOptions computes a difference with configurable safety
// limits. It panics on complexity or topology failure.
func DifferenceWithOptions(options Options, first *model.Mesh, subtract ...*model.Mesh) *model.Mesh {
	result, err := DifferenceCheckedWithOptions(options, first, subtract...)
	if err != nil {
		panic(err)
	}
	return result
}

// DifferenceChecked is like Difference, but reports complexity and
// output-topology failures as errors.
func DifferenceChecked(first *model.Mesh, subtract ...*model.Mesh) (result *model.Mesh, err error) {
	return DifferenceCheckedWithOptions(DefaultOptions(), first, subtract...)
}

// DifferenceCheckedWithOptions is like DifferenceWithOptions, but returns
// complexity, topology, and invalid-option failures as errors.
func DifferenceCheckedWithOptions(options Options, first *model.Mesh, subtract ...*model.Mesh) (result *model.Mesh, err error) {
	options, err = normalizeOptions(options)
	if err != nil {
		return nil, err
	}
	defer recoverComplexity(&err)
	if first == nil {
		return model.NewMesh(), nil
	}
	meshes := make([]*model.Mesh, 1, 1+len(subtract))
	meshes[0] = first
	meshes = append(meshes, subtract...)
	return boolean(options, booleanDifference, meshes), nil
}

func recoverComplexity(err *error) {
	if value := recover(); value != nil {
		switch failure := value.(type) {
		case *ComplexityError:
			*err = failure
			return
		case *TopologyError:
			*err = failure
			return
		}
		panic(value)
	}
}

type booleanKind int

const (
	booleanUnion booleanKind = iota
	booleanIntersection
	booleanDifference
)

type sourceSegment struct {
	p    [2]model.Coord
	cuts []float64
}

type atomicSegment struct {
	p [2]model.Coord
}

func boolean(options Options, kind booleanKind, meshes []*model.Mesh) *model.Mesh {
	meshes = nonNilMeshes(meshes)
	if len(meshes) == 0 {
		return model.NewMesh()
	}

	totalSegments := 0
	for _, mesh := range meshes {
		totalSegments += mesh.NumSegments()
		if totalSegments > options.MaxInputSegments {
			panic(&ComplexityError{Stage: "input segments", Limit: options.MaxInputSegments})
		}
	}
	segments := collectSegments(meshes)
	if len(segments) == 0 {
		return model.NewMesh()
	}
	if center, ok := normalizationCenter(segments); ok {
		local := make([]*model.Mesh, len(meshes))
		for i, mesh := range meshes {
			local[i] = mesh.Translate(center.Scale(-1))
		}
		return boolean(options, kind, local).Translate(center)
	}
	tol := meshTolerance(segments)
	splitSegments(options, segments, tol)
	atoms := atomicSegments(segments, tol)
	classifiers := make([]pointClassifier, len(meshes))
	for i, m := range meshes {
		classifiers[i] = newPointClassifier(m)
	}

	result := model.NewMesh()
	for _, atom := range atoms {
		a, b := atom.p[0], atom.p[1]
		delta := b.Sub(a)
		length := delta.Norm()
		if length <= tol {
			continue
		}
		mid := a.Mid(b)
		rightUnit := model.XY(delta.Y/length, -delta.X/length)
		offset := math.Max(tol*8, length*1e-8)
		// Keep probes within the local edge neighborhood, but never force an
		// ordinary short edge's probe below the operation tolerance.
		offset = math.Min(offset, length*0.25)
		right := evalBoolean(kind, classifiers, mid.Add(rightUnit.Scale(offset)))
		left := evalBoolean(kind, classifiers, mid.Sub(rightUnit.Scale(offset)))
		if right == left {
			continue
		}
		if right {
			result.Add(&model.Segment{a, b})
		} else {
			result.Add(&model.Segment{b, a})
		}
	}
	result = regularizePointContacts(options, result, tol)
	return validateResultTopology(options, result)
}

func validateResultTopology(options Options, mesh *model.Mesh) *model.Mesh {
	if !mesh.Manifold() {
		panic(&TopologyError{Problem: "vertices do not each have two incident segments"})
	}
	if inconsistent := mesh.InconsistentVertices(); len(inconsistent) != 0 {
		panic(&TopologyError{Problem: "inconsistent segment orientation", Count: len(inconsistent)})
	}
	type boundedSegment struct {
		segment  *model.Segment
		min, max model.Coord
	}
	segments := mesh.SegmentSlice()
	ordered := make([]boundedSegment, len(segments))
	for i, segment := range segments {
		ordered[i] = boundedSegment{segment: segment, min: segment.Min(), max: segment.Max()}
	}
	sort.Slice(ordered, func(i, j int) bool {
		if ordered[i].min.X != ordered[j].min.X {
			return ordered[i].min.X < ordered[j].min.X
		}
		if ordered[i].min.Y != ordered[j].min.Y {
			return ordered[i].min.Y < ordered[j].min.Y
		}
		return coordLess(ordered[i].max, ordered[j].max)
	})
	pairs := 0
	active := make([]boundedSegment, 0, 32)
	for _, current := range ordered {
		remaining := active[:0]
		for _, candidate := range active {
			if candidate.max.X < current.min.X {
				continue
			}
			remaining = append(remaining, candidate)
			if candidate.min.Y > current.max.Y || current.min.Y > candidate.max.Y {
				continue
			}
			pairs++
			if pairs > options.MaxIntersectionPairs {
				panic(&ComplexityError{Stage: "output validation candidate pairs", Limit: options.MaxIntersectionPairs})
			}
			if invalidSegmentIntersection(candidate.segment, current.segment) {
				panic(&TopologyError{Problem: "crossing or overlapping output segments"})
			}
		}
		active = append(remaining, current)
	}
	return mesh
}

func invalidSegmentIntersection(a, b *model.Segment) bool {
	for i, point := range a {
		for j, other := range b {
			if point == other {
				first := a[1-i].Sub(point)
				second := b[1-j].Sub(point)
				return cross(first, second) == 0 && first.Dot(second) > 0
			}
		}
	}
	p, r := a[0], a[1].Sub(a[0])
	q, s := b[0], b[1].Sub(b[0])
	determinant := cross(r, s)
	if determinant != 0 {
		delta := q.Sub(p)
		t := cross(delta, s) / determinant
		u := cross(delta, r) / determinant
		return t >= 0 && t <= 1 && u >= 0 && u <= 1
	}
	if cross(q.Sub(p), r) != 0 {
		return false
	}
	denominator := r.NormSquared()
	if denominator == 0 {
		return true
	}
	t0 := q.Sub(p).Dot(r) / denominator
	t1 := b[1].Sub(p).Dot(r) / denominator
	if t0 > t1 {
		t0, t1 = t1, t0
	}
	return math.Max(0, t0) <= math.Min(1, t1)
}

// regularizePointContacts separates boundary loops which meet at an exact
// point. Such a contact has four or more incident segments and therefore
// cannot be represented as a manifold model.Mesh, whose connectivity is
// determined solely by coordinate equality.
func regularizePointContacts(options Options, mesh *model.Mesh, tol float64) *model.Mesh {
	segments := mesh.SegmentSlice()
	if len(segments) == 0 || mesh.Manifold() {
		return mesh
	}
	sort.Slice(segments, func(i, j int) bool {
		a := orderedSegmentKey(segments[i][0], segments[i][1])
		b := orderedSegmentKey(segments[j][0], segments[j][1])
		if a[0] != b[0] {
			return coordLess(a[0], b[0])
		}
		return coordLess(a[1], b[1])
	})
	vertexSegments := map[model.Coord][]int{}
	for i, segment := range segments {
		for _, point := range segment {
			vertexSegments[point] = append(vertexSegments[point], i)
		}
	}
	type segmentEndpoint struct {
		segment, endpoint int
	}
	replacements := map[segmentEndpoint]model.Coord{}
	for point, incident := range vertexSegments {
		if len(incident) <= 2 {
			continue
		}
		var incoming, outgoing []int
		for _, index := range incident {
			if segments[index][1] == point {
				incoming = append(incoming, index)
			}
			if segments[index][0] == point {
				outgoing = append(outgoing, index)
			}
		}
		if len(incoming) < 2 || len(incoming) != len(outgoing) {
			continue
		}
		if len(incoming) > options.MaxContactDegree {
			panic(&ComplexityError{Stage: "loops at one contact point", Limit: options.MaxContactDegree})
		}
		assignment := matchContactEdges(segments, point, incoming, outgoing)
		directions := make([]model.Coord, len(incoming))
		for i, incomingIndex := range incoming {
			previous := segments[incomingIndex][0]
			next := segments[outgoing[assignment[i]]][1]
			directions[i] = previous.Add(next).Scale(0.5).Sub(point)
		}
		for i, incomingIndex := range incoming {
			direction := directions[i]
			if direction.Norm() <= tol {
				// A point inserted into a straight edge has no local chord
				// direction. Move it away from the other contacting loops.
				for j, other := range directions {
					if i != j {
						direction = direction.Sub(other)
					}
				}
			}
			if direction.Norm() <= tol {
				angle := float64(i+1) * 2.399963229728653
				direction = model.NewCoordPolar(angle, 1)
			} else {
				direction = direction.Normalize()
			}
			outgoingIndex := outgoing[assignment[i]]
			shift := tol * 4
			shift = math.Min(shift, point.Dist(segments[incomingIndex][0])*0.25)
			shift = math.Min(shift, point.Dist(segments[outgoingIndex][1])*0.25)
			replacement := point.Add(direction.Scale(shift))
			replacements[segmentEndpoint{segment: incomingIndex, endpoint: 1}] = replacement
			replacements[segmentEndpoint{segment: outgoingIndex, endpoint: 0}] = replacement
		}
	}
	if len(replacements) == 0 {
		return mesh
	}
	result := model.NewMesh()
	for i, segment := range segments {
		copy := *segment
		for j := range copy {
			if replacement, ok := replacements[segmentEndpoint{segment: i, endpoint: j}]; ok {
				copy[j] = replacement
			}
		}
		result.Add(&copy)
	}
	return result
}

// matchContactEdges returns the outgoing edge paired with each incoming edge.
// With the result interior on the right, every occupied angular sector starts
// at an incoming ray and ends at the next counterclockwise outgoing ray. This
// local ordering remains unambiguous for sharp lobes where a smoothest-turn
// heuristic would incorrectly splice different loops together.
func matchContactEdges(segments []*model.Segment, point model.Coord, incoming, outgoing []int) []int {
	type contactRay struct {
		angle    float64
		incoming bool
		position int
	}
	rays := make([]contactRay, 0, len(incoming)+len(outgoing))
	for position, segment := range incoming {
		direction := segments[segment][0].Sub(point)
		rays = append(rays, contactRay{
			angle: math.Atan2(direction.Y, direction.X), incoming: true, position: position,
		})
	}
	for position, segment := range outgoing {
		direction := segments[segment][1].Sub(point)
		rays = append(rays, contactRay{
			angle: math.Atan2(direction.Y, direction.X), position: position,
		})
	}
	sort.Slice(rays, func(i, j int) bool {
		if rays[i].angle != rays[j].angle {
			return rays[i].angle < rays[j].angle
		}
		if rays[i].incoming != rays[j].incoming {
			return rays[i].incoming
		}
		return rays[i].position < rays[j].position
	})
	result := make([]int, len(incoming))
	usedOutgoing := make([]bool, len(outgoing))
	valid := true
	for i, ray := range rays {
		if !ray.incoming {
			continue
		}
		next := rays[(i+1)%len(rays)]
		if next.incoming || usedOutgoing[next.position] {
			valid = false
			break
		}
		result[ray.position] = next.position
		usedOutgoing[next.position] = true
	}
	if valid {
		return result
	}
	// Collinear duplicate rays are an ambiguous degenerate case. Preserve a
	// deterministic bijection so regularization still cannot leave open ends.
	for i := range result {
		result[i] = i
	}
	return result
}

func normalizationCenter(segments []*sourceSegment) (model.Coord, bool) {
	min, max := segments[0].p[0], segments[0].p[0]
	for _, segment := range segments {
		for _, point := range segment.p {
			min, max = min.Min(point), max.Max(point)
		}
	}
	span := max.Sub(min).MaxCoord()
	center := min.Mid(max)
	return center, center.Abs().MaxCoord() > math.Max(span, 1)*1024
}

func nonNilMeshes(meshes []*model.Mesh) []*model.Mesh {
	result := make([]*model.Mesh, 0, len(meshes))
	for _, m := range meshes {
		if m != nil {
			result = append(result, m)
		}
	}
	return result
}

func collectSegments(meshes []*model.Mesh) []*sourceSegment {
	var result []*sourceSegment
	for _, mesh := range meshes {
		mesh.Iterate(func(s *model.Segment) {
			if s[0] != s[1] {
				result = append(result, &sourceSegment{p: *s, cuts: []float64{0, 1}})
			}
		})
	}
	sort.Slice(result, func(i, j int) bool {
		ai, bi := orderedSegmentKey(result[i].p[0], result[i].p[1]), orderedSegmentKey(result[j].p[0], result[j].p[1])
		if ai[0] != bi[0] {
			return coordLess(ai[0], bi[0])
		}
		return coordLess(ai[1], bi[1])
	})
	return result
}

func meshTolerance(segments []*sourceSegment) float64 {
	min, max := segments[0].p[0], segments[0].p[0]
	maxAbs := 0.0
	for _, s := range segments {
		for _, p := range s.p {
			min = min.Min(p)
			max = max.Max(p)
			maxAbs = math.Max(maxAbs, p.Abs().MaxCoord())
		}
	}
	scale := max.Sub(min).MaxCoord()
	if scale == 0 || math.IsInf(scale, 0) || math.IsNaN(scale) {
		scale = 1
	}
	// This is large enough to coalesce independently-computed intersections,
	// but small enough to retain ordinary modeling details.
	ulp := math.Nextafter(maxAbs, math.Inf(1)) - maxAbs
	return math.Max(math.Max(scale*1e-10, ulp*16), math.SmallestNonzeroFloat64*1024)
}

func splitSegments(options Options, segments []*sourceSegment, tol float64) {
	type boundedSegment struct {
		segment  *sourceSegment
		min, max model.Coord
	}
	ordered := make([]boundedSegment, len(segments))
	for i, segment := range segments {
		ordered[i] = boundedSegment{
			segment: segment,
			min:     segment.p[0].Min(segment.p[1]),
			max:     segment.p[0].Max(segment.p[1]),
		}
	}
	sort.SliceStable(ordered, func(i, j int) bool {
		if ordered[i].min.X != ordered[j].min.X {
			return ordered[i].min.X < ordered[j].min.X
		}
		if ordered[i].min.Y != ordered[j].min.Y {
			return ordered[i].min.Y < ordered[j].min.Y
		}
		return coordLess(ordered[i].max, ordered[j].max)
	})

	totalCuts := len(segments) * 2
	totalPairs := 0
	active := make([]boundedSegment, 0, 32)
	for _, current := range ordered {
		remaining := active[:0]
		for _, candidate := range active {
			totalPairs++
			if totalPairs > options.MaxIntersectionPairs {
				panic(&ComplexityError{Stage: "intersection candidate pairs", Limit: options.MaxIntersectionPairs})
			}
			if candidate.max.X+tol < current.min.X {
				continue
			}
			remaining = append(remaining, candidate)
			if candidate.min.Y > current.max.Y+tol || current.min.Y > candidate.max.Y+tol {
				continue
			}
			before := len(current.segment.cuts) + len(candidate.segment.cuts)
			addIntersections(current.segment, candidate.segment, tol)
			totalCuts += len(current.segment.cuts) + len(candidate.segment.cuts) - before
			if totalCuts > options.MaxIntersectionCuts {
				panic(&ComplexityError{Stage: "intersection cuts", Limit: options.MaxIntersectionCuts})
			}
		}
		active = append(remaining, current)
	}
}

func addIntersections(a, b *sourceSegment, tol float64) {
	p, r := a.p[0], a.p[1].Sub(a.p[0])
	q, s := b.p[0], b.p[1].Sub(b.p[0])
	rxs := cross(r, s)
	qmp := q.Sub(p)
	scale := math.Max(r.Norm(), s.Norm())
	detTol := tol * math.Max(scale, tol)
	if math.Abs(rxs) > detTol {
		t := cross(qmp, s) / rxs
		u := cross(qmp, r) / rxs
		paramTolA := tol / math.Max(r.Norm(), tol)
		paramTolB := tol / math.Max(s.Norm(), tol)
		if t >= -paramTolA && t <= 1+paramTolA && u >= -paramTolB && u <= 1+paramTolB {
			a.cuts = append(a.cuts, clamp01(t))
			b.cuts = append(b.cuts, clamp01(u))
		}
		return
	}
	if math.Abs(cross(qmp, r)) > detTol {
		return
	}
	// Collinear segments must be cut at all endpoints in their shared range.
	for _, point := range b.p {
		if t, ok := segmentParam(a.p, point, tol); ok {
			a.cuts = append(a.cuts, t)
		}
	}
	for _, point := range a.p {
		if t, ok := segmentParam(b.p, point, tol); ok {
			b.cuts = append(b.cuts, t)
		}
	}
}

func segmentParam(s [2]model.Coord, p model.Coord, tol float64) (float64, bool) {
	d := s[1].Sub(s[0])
	denom := d.NormSquared()
	if denom == 0 {
		return 0, false
	}
	t := p.Sub(s[0]).Dot(d) / denom
	closest := s[0].Add(d.Scale(t))
	paramTol := tol / math.Max(math.Sqrt(denom), tol)
	if p.Dist(closest) > tol || t < -paramTol || t > 1+paramTol {
		return 0, false
	}
	return clamp01(t), true
}

func atomicSegments(segments []*sourceSegment, tol float64) []atomicSegment {
	canon := newCanonicalizer(tol)
	unique := map[segmentKey]atomicSegment{}
	for _, s := range segments {
		sort.Float64s(s.cuts)
		cuts := s.cuts[:0]
		paramTol := tol / math.Max(s.p[0].Dist(s.p[1]), tol)
		for _, x := range s.cuts {
			if len(cuts) == 0 || x-cuts[len(cuts)-1] > paramTol {
				cuts = append(cuts, x)
			}
		}
		delta := s.p[1].Sub(s.p[0])
		for i := 1; i < len(cuts); i++ {
			a := canon.add(s.p[0].Add(delta.Scale(cuts[i-1])))
			b := canon.add(s.p[0].Add(delta.Scale(cuts[i])))
			if a == b || a.Dist(b) <= tol {
				continue
			}
			key := orderedSegmentKey(a, b)
			unique[key] = atomicSegment{p: [2]model.Coord{a, b}}
		}
	}
	result := make([]atomicSegment, 0, len(unique))
	for _, s := range unique {
		result = append(result, s)
	}
	return result
}

type segmentKey [2]model.Coord

func orderedSegmentKey(a, b model.Coord) segmentKey {
	if coordLess(b, a) {
		a, b = b, a
	}
	return segmentKey{a, b}
}

func coordLess(a, b model.Coord) bool {
	return a.X < b.X || (a.X == b.X && a.Y < b.Y)
}

type gridKey struct{ x, y int64 }

type canonicalizer struct {
	tol     float64
	buckets map[gridKey][]model.Coord
}

func newCanonicalizer(tol float64) *canonicalizer {
	return &canonicalizer{tol: tol, buckets: map[gridKey][]model.Coord{}}
}

func (c *canonicalizer) add(p model.Coord) model.Coord {
	k := c.key(p)
	for dx := int64(-1); dx <= 1; dx++ {
		for dy := int64(-1); dy <= 1; dy++ {
			for _, old := range c.buckets[gridKey{k.x + dx, k.y + dy}] {
				if old.Dist(p) <= c.tol {
					return old
				}
			}
		}
	}
	c.buckets[k] = append(c.buckets[k], p)
	return p
}

func (c *canonicalizer) key(p model.Coord) gridKey {
	return gridKey{safeFloor(p.X / c.tol), safeFloor(p.Y / c.tol)}
}

func safeFloor(x float64) int64 {
	if x >= math.MaxInt64 {
		return math.MaxInt64
	}
	if x <= math.MinInt64 {
		return math.MinInt64
	}
	return int64(math.Floor(x))
}

type pointClassifier struct {
	root *classifierNode
}

func newPointClassifier(mesh *model.Mesh) pointClassifier {
	return pointClassifier{root: newClassifierNode(mesh.SegmentSlice())}
}

func (p pointClassifier) contains(point model.Coord) bool {
	crossings := 0
	p.root.countRayCrossings(point, &crossings)
	return crossings%2 == 1
}

type classifierNode struct {
	min, max    model.Coord
	segments    []*model.Segment
	left, right *classifierNode
}

func newClassifierNode(segments []*model.Segment) *classifierNode {
	if len(segments) == 0 {
		return nil
	}
	segments = append([]*model.Segment(nil), segments...)
	node := &classifierNode{min: segments[0].Min(), max: segments[0].Max()}
	for _, segment := range segments[1:] {
		node.min, node.max = node.min.Min(segment.Min()), node.max.Max(segment.Max())
	}
	if len(segments) <= 8 {
		node.segments = segments
		return node
	}
	axisX := node.max.X-node.min.X >= node.max.Y-node.min.Y
	sort.Slice(segments, func(i, j int) bool {
		centerI, centerJ := segments[i].Mid(), segments[j].Mid()
		if axisX {
			return centerI.X < centerJ.X
		}
		return centerI.Y < centerJ.Y
	})
	middle := len(segments) / 2
	node.left = newClassifierNode(segments[:middle])
	node.right = newClassifierNode(segments[middle:])
	return node
}

func (p *classifierNode) countRayCrossings(point model.Coord, result *int) {
	if p == nil || p.max.X <= point.X || p.min.Y > point.Y || p.max.Y <= point.Y {
		return
	}
	if p.left != nil {
		p.left.countRayCrossings(point, result)
		p.right.countRayCrossings(point, result)
		return
	}
	// Odd/even classification supports islands and holes independently of the
	// winding orientation supplied by the caller. The half-open Y interval
	// avoids double-counting vertices.
	for _, s := range p.segments {
		a, b := s[0], s[1]
		if (a.Y > point.Y) == (b.Y > point.Y) {
			continue
		}
		x := a.X + (point.Y-a.Y)*(b.X-a.X)/(b.Y-a.Y)
		if x > point.X {
			(*result)++
		}
	}
}

func evalBoolean(kind booleanKind, classifiers []pointClassifier, p model.Coord) bool {
	switch kind {
	case booleanUnion:
		for _, c := range classifiers {
			if c.contains(p) {
				return true
			}
		}
		return false
	case booleanIntersection:
		for _, c := range classifiers {
			if !c.contains(p) {
				return false
			}
		}
		return true
	case booleanDifference:
		if !classifiers[0].contains(p) {
			return false
		}
		for _, c := range classifiers[1:] {
			if c.contains(p) {
				return false
			}
		}
		return true
	default:
		panic("unknown boolean operation")
	}
}

func cross(a, b model.Coord) float64 { return a.X*b.Y - a.Y*b.X }

func clamp01(x float64) float64 {
	return math.Max(0, math.Min(1, x))
}
