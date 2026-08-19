package meshbool

import (
	"fmt"
	"math"
	"sort"

	"github.com/unixpickle/meshbool/bool2d"
	model2d "github.com/unixpickle/model3d/model2d"
	"github.com/unixpickle/model3d/model3d"
)

const (
	defaultMaxInputTriangles         = 200_000
	defaultMaxFragmentsPerTriangle   = 4_096
	defaultMaxTotalFragments         = 200_000
	defaultMaxOutputTriangles        = 200_000
	defaultMaxContactEdgeTriangles   = 1_024
	defaultMaxTriangleCandidatePairs = 5_000_000
)

// Options configures safety limits for boolean operations. A zero numeric
// field uses the corresponding value from DefaultOptions. Negative values are
// invalid. PlanarOptions controls coplanar surface processing.
type Options struct {
	MaxInputTriangles         int
	MaxFragmentsPerTriangle   int
	MaxTotalFragments         int
	MaxOutputTriangles        int
	MaxContactEdgeTriangles   int
	MaxTriangleCandidatePairs int
	PlanarOptions             bool2d.Options
}

// DefaultOptions returns the limits used by Union, Intersection, and
// Difference.
func DefaultOptions() Options {
	return Options{
		MaxInputTriangles:         defaultMaxInputTriangles,
		MaxFragmentsPerTriangle:   defaultMaxFragmentsPerTriangle,
		MaxTotalFragments:         defaultMaxTotalFragments,
		MaxOutputTriangles:        defaultMaxOutputTriangles,
		MaxContactEdgeTriangles:   defaultMaxContactEdgeTriangles,
		MaxTriangleCandidatePairs: defaultMaxTriangleCandidatePairs,
		PlanarOptions:             bool2d.DefaultOptions(),
	}
}

func normalizeOptions(options Options) (Options, error) {
	defaults := DefaultOptions()
	fields := []struct {
		name                string
		value, valueDefault *int
	}{
		{"MaxInputTriangles", &options.MaxInputTriangles, &defaults.MaxInputTriangles},
		{"MaxFragmentsPerTriangle", &options.MaxFragmentsPerTriangle, &defaults.MaxFragmentsPerTriangle},
		{"MaxTotalFragments", &options.MaxTotalFragments, &defaults.MaxTotalFragments},
		{"MaxOutputTriangles", &options.MaxOutputTriangles, &defaults.MaxOutputTriangles},
		{"MaxContactEdgeTriangles", &options.MaxContactEdgeTriangles, &defaults.MaxContactEdgeTriangles},
		{"MaxTriangleCandidatePairs", &options.MaxTriangleCandidatePairs, &defaults.MaxTriangleCandidatePairs},
	}
	for _, field := range fields {
		if *field.value < 0 {
			return Options{}, fmt.Errorf("meshbool: option %s must not be negative", field.name)
		}
		if *field.value == 0 {
			*field.value = *field.valueDefault
		}
	}
	if options.PlanarOptions == (bool2d.Options{}) {
		options.PlanarOptions = defaults.PlanarOptions
	}
	if err := options.PlanarOptions.Validate(); err != nil {
		return Options{}, err
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
// adversarial intersection arrangement could consume unbounded memory.
type ComplexityError struct {
	Stage string
	Limit int
}

func (c *ComplexityError) Error() string {
	return fmt.Sprintf("meshbool: %s exceeds safety limit of %d", c.Stage, c.Limit)
}

// TopologyError indicates that numerical degeneracy prevented construction of
// a closed, manifold, orientable, and self-intersection-free result.
type TopologyError struct {
	Problem string
	Count   int
}

func (t *TopologyError) Error() string {
	if t.Count != 0 {
		return fmt.Sprintf("meshbool: invalid output topology: %s (%d)", t.Problem, t.Count)
	}
	return fmt.Sprintf("meshbool: invalid output topology: %s", t.Problem)
}

// Union computes the closed-set union of zero or more triangle meshes.
// Input meshes are not modified.
func Union(meshes ...*model3d.Mesh) *model3d.Mesh {
	return UnionWithOptions(DefaultOptions(), meshes...)
}

// UnionWithOptions computes a union with configurable safety limits. It
// panics on complexity or topology failure.
func UnionWithOptions(options Options, meshes ...*model3d.Mesh) *model3d.Mesh {
	result, err := UnionCheckedWithOptions(options, meshes...)
	if err != nil {
		panic(err)
	}
	return result
}

// UnionChecked is like Union, but reports complexity and output-topology
// failures as errors instead of panicking.
func UnionChecked(meshes ...*model3d.Mesh) (result *model3d.Mesh, err error) {
	return UnionCheckedWithOptions(DefaultOptions(), meshes...)
}

// UnionCheckedWithOptions is like UnionWithOptions, but returns complexity,
// topology, and invalid-option failures as errors.
func UnionCheckedWithOptions(options Options, meshes ...*model3d.Mesh) (result *model3d.Mesh, err error) {
	options, err = normalizeOptions(options)
	if err != nil {
		return nil, err
	}
	defer recoverComplexity(&err)
	return unionUnchecked(options, meshes...), nil
}

func unionUnchecked(options Options, meshes ...*model3d.Mesh) *model3d.Mesh {
	meshes = nonNilMeshes(meshes)
	if len(meshes) == 0 {
		return model3d.NewMesh()
	}
	checkInputMeshes(options, meshes)
	result := meshes[0].DeepCopy()
	for _, mesh := range meshes[1:] {
		result = booleanMeshPair(options, result, mesh, meshUnion)
	}
	return result
}

// Intersection computes the intersection of zero or more triangle meshes.
// With no arguments, it returns an empty mesh. Input meshes are not modified.
func Intersection(meshes ...*model3d.Mesh) *model3d.Mesh {
	return IntersectionWithOptions(DefaultOptions(), meshes...)
}

// IntersectionWithOptions computes an intersection with configurable safety
// limits. It panics on complexity or topology failure.
func IntersectionWithOptions(options Options, meshes ...*model3d.Mesh) *model3d.Mesh {
	result, err := IntersectionCheckedWithOptions(options, meshes...)
	if err != nil {
		panic(err)
	}
	return result
}

// IntersectionChecked is like Intersection, but reports complexity and
// output-topology failures as errors instead of panicking.
func IntersectionChecked(meshes ...*model3d.Mesh) (result *model3d.Mesh, err error) {
	return IntersectionCheckedWithOptions(DefaultOptions(), meshes...)
}

// IntersectionCheckedWithOptions is like IntersectionWithOptions, but returns
// complexity, topology, and invalid-option failures as errors.
func IntersectionCheckedWithOptions(options Options, meshes ...*model3d.Mesh) (result *model3d.Mesh, err error) {
	options, err = normalizeOptions(options)
	if err != nil {
		return nil, err
	}
	defer recoverComplexity(&err)
	return intersectionUnchecked(options, meshes...), nil
}

func intersectionUnchecked(options Options, meshes ...*model3d.Mesh) *model3d.Mesh {
	meshes = nonNilMeshes(meshes)
	if len(meshes) == 0 {
		return model3d.NewMesh()
	}
	checkInputMeshes(options, meshes)
	result := meshes[0].DeepCopy()
	for _, mesh := range meshes[1:] {
		result = booleanMeshPair(options, result, mesh, meshIntersection)
		if result.NumTriangles() == 0 {
			break
		}
	}
	return result
}

// Difference subtracts every mesh in subtract from first. Input meshes are not
// modified.
func Difference(first *model3d.Mesh, subtract ...*model3d.Mesh) *model3d.Mesh {
	return DifferenceWithOptions(DefaultOptions(), first, subtract...)
}

// DifferenceWithOptions computes a difference with configurable safety
// limits. It panics on complexity or topology failure.
func DifferenceWithOptions(options Options, first *model3d.Mesh, subtract ...*model3d.Mesh) *model3d.Mesh {
	result, err := DifferenceCheckedWithOptions(options, first, subtract...)
	if err != nil {
		panic(err)
	}
	return result
}

// DifferenceChecked is like Difference, but reports complexity and
// output-topology failures as errors instead of panicking.
func DifferenceChecked(first *model3d.Mesh, subtract ...*model3d.Mesh) (result *model3d.Mesh, err error) {
	return DifferenceCheckedWithOptions(DefaultOptions(), first, subtract...)
}

// DifferenceCheckedWithOptions is like DifferenceWithOptions, but returns
// complexity, topology, and invalid-option failures as errors.
func DifferenceCheckedWithOptions(options Options, first *model3d.Mesh,
	subtract ...*model3d.Mesh) (result *model3d.Mesh, err error) {
	options, err = normalizeOptions(options)
	if err != nil {
		return nil, err
	}
	defer recoverComplexity(&err)
	return differenceUnchecked(options, first, subtract...), nil
}

func differenceUnchecked(options Options, first *model3d.Mesh, subtract ...*model3d.Mesh) *model3d.Mesh {
	if first == nil {
		return model3d.NewMesh()
	}
	meshes := make([]*model3d.Mesh, 1, 1+len(subtract))
	meshes[0] = first
	meshes = append(meshes, nonNilMeshes(subtract)...)
	checkInputMeshes(options, meshes)
	result := first.DeepCopy()
	for _, mesh := range meshes[1:] {
		result = booleanMeshPair(options, result, mesh, meshDifference)
		if result.NumTriangles() == 0 {
			break
		}
	}
	return result
}

func recoverComplexity(err *error) {
	if value := recover(); value != nil {
		switch complexity := value.(type) {
		case *ComplexityError:
			*err = complexity
			return
		case *bool2d.ComplexityError:
			*err = &ComplexityError{
				Stage: "planar " + complexity.Stage,
				Limit: complexity.Limit,
			}
			return
		case *TopologyError:
			*err = complexity
			return
		case *bool2d.TopologyError:
			*err = &TopologyError{Problem: "planar " + complexity.Problem, Count: complexity.Count}
			return
		}
		panic(value)
	}
}

func checkComplexity(stage string, count, limit int) {
	if count > limit {
		panic(&ComplexityError{Stage: stage, Limit: limit})
	}
}

func checkInputMeshes(options Options, meshes []*model3d.Mesh) {
	for _, mesh := range meshes {
		checkComplexity("triangles in one input mesh", mesh.NumTriangles(), options.MaxInputTriangles)
	}
}

type meshBooleanKind int

const (
	meshUnion meshBooleanKind = iota
	meshIntersection
	meshDifference
)

func booleanMeshPair(options Options, a, b *model3d.Mesh, kind meshBooleanKind) *model3d.Mesh {
	checkInputMeshes(options, []*model3d.Mesh{a, b})
	if a.NumTriangles() == 0 || b.NumTriangles() == 0 {
		switch kind {
		case meshUnion:
			if a.NumTriangles() == 0 {
				return b.DeepCopy()
			}
			return a.DeepCopy()
		case meshIntersection:
			return model3d.NewMesh()
		case meshDifference:
			return a.DeepCopy()
		}
	}
	min := a.Min().Min(b.Min())
	max := a.Max().Max(b.Max())
	span := max.Sub(min).MaxCoord()
	center := min.Mid(max)
	if center.Abs().MaxCoord() > math.Max(span, 1)*1024 {
		localA := a.Translate(center.Scale(-1))
		localB := b.Translate(center.Scale(-1))
		return booleanMeshPairLocal(options, localA, localB, kind).Translate(center)
	}
	return booleanMeshPairLocal(options, a, b, kind)
}

func booleanMeshPairLocal(options Options, a, b *model3d.Mesh, kind meshBooleanKind) *model3d.Mesh {
	checkComplexity("triangles in one input mesh", a.NumTriangles(), options.MaxInputTriangles)
	checkComplexity("triangles in one input mesh", b.NumTriangles(), options.MaxInputTriangles)
	trianglesA, trianglesB := sortedTriangles(a), sortedTriangles(b)
	indexA, indexB := newTriangleIndex(trianglesA), newTriangleIndex(trianglesB)
	colliderA := model3d.GroupedTrianglesToCollider(trianglesA)
	colliderB := model3d.GroupedTrianglesToCollider(trianglesB)
	solidA := model3d.NewColliderSolid(colliderA)
	solidB := model3d.NewColliderSolid(colliderB)
	scale := meshScale([]*model3d.Mesh{a, b})
	tol := math.Max(scale*1e-9, math.SmallestNonzeroFloat64*1024)
	candidatePairs := 0
	var fragments []*polygon
	fragments = append(fragments, splitAndClassifyMesh(
		options, trianglesA, colliderB, indexB, solidA, solidB, kind, tol, true, &candidatePairs)...)
	checkComplexity("surface fragments", len(fragments), options.MaxTotalFragments)
	fragments = append(fragments, splitAndClassifyMesh(
		options, trianglesB, colliderA, indexA, solidA, solidB, kind, tol, false, &candidatePairs)...)
	checkComplexity("surface fragments", len(fragments), options.MaxTotalFragments)
	return polygonsMesh(options, fragments, scale)
}

func splitAndClassifyMesh(options Options, triangles []*model3d.Triangle, other model3d.TriangleCollider, otherIndex *triangleIndex,
	solidA, solidB model3d.Solid, kind meshBooleanKind, tol float64, sourceA bool,
	candidatePairs *int) []*polygon {
	var result []*polygon
	for _, tri := range triangles {
		if tri.Area() <= tol*tol {
			continue
		}
		normal := tri.Normal()
		u, v := planeBasis(normal)
		origin := tri[0]
		project := func(point model3d.Coord3D) model2d.Coord {
			delta := point.Sub(origin)
			return model2d.XY(u.Dot(delta), v.Dot(delta))
		}
		lift := func(point model2d.Coord) model3d.Coord3D {
			return origin.Add(u.Scale(point.X)).Add(v.Scale(point.Y))
		}
		pieces := [][]model2d.Coord{{project(tri[0]), project(tri[1]), project(tri[2])}}
		var splitLines [][2]model2d.Coord
		for _, collision := range other.TriangleCollisions(tri) {
			splitLines = append(splitLines, [2]model2d.Coord{project(collision[0]), project(collision[1])})
		}
		var nearby []*model3d.Triangle
		otherIndex.query(tri.Min().AddScalar(-tol), tri.Max().AddScalar(tol), &nearby)
		*candidatePairs += len(nearby)
		checkComplexity("triangle candidate pairs", *candidatePairs, options.MaxTriangleCandidatePairs)
		projectedTri := []model2d.Coord{project(tri[0]), project(tri[1]), project(tri[2])}
		for _, candidate := range nearby {
			candidateNormal := candidate.Normal()
			if math.Abs(normal.Dot(candidateNormal)) < 1-1e-12 ||
				math.Abs(normal.Dot(candidate[0].Sub(tri[0]))) > tol {
				continue
			}
			projectedCandidate := []model2d.Coord{
				project(candidate[0]), project(candidate[1]), project(candidate[2]),
			}
			if !convexPolygonsOverlap2D(projectedTri, projectedCandidate, tol) {
				continue
			}
			for i, point := range projectedCandidate {
				splitLines = append(splitLines, [2]model2d.Coord{point, projectedCandidate[(i+1)%3]})
			}
		}
		for _, splitLine := range splitLines {
			p1, p2 := splitLine[0], splitLine[1]
			if p1.Dist(p2) <= tol {
				continue
			}
			var next [][]model2d.Coord
			for _, piece := range pieces {
				frontPiece, backPiece := splitPolygon2D(piece, p1, p2, tol)
				if len(frontPiece) >= 3 {
					next = append(next, frontPiece)
					checkComplexity("fragments for one triangle", len(next), options.MaxFragmentsPerTriangle)
				}
				if len(backPiece) >= 3 {
					next = append(next, backPiece)
					checkComplexity("fragments for one triangle", len(next), options.MaxFragmentsPerTriangle)
				}
			}
			pieces = next
		}
		for _, piece := range pieces {
			center2d := model2d.Coord{}
			vertices := make([]model3d.Coord3D, len(piece))
			for i, point := range piece {
				center2d = center2d.Add(point)
				vertices[i] = lift(point)
			}
			center := lift(center2d.Scale(1 / float64(len(piece))))
			offset := tol * 4
			plusPoint := center.Add(normal.Scale(offset))
			minusPoint := center.Sub(normal.Scale(offset))
			var plus, minus bool
			if sourceA {
				// The source triangle itself defines A exactly on either side;
				// asking a ray collider to rediscover this near a coplanar seam is
				// less reliable, especially for very thin results.
				plus = evalMeshBoolean(kind, false, solidB.Contains(plusPoint))
				minus = evalMeshBoolean(kind, true, solidB.Contains(minusPoint))
			} else {
				plus = evalMeshBoolean(kind, solidA.Contains(plusPoint), false)
				minus = evalMeshBoolean(kind, solidA.Contains(minusPoint), true)
			}
			if plus == minus {
				continue
			}
			poly := newPolygon(vertices)
			if poly == nil {
				continue
			}
			// Surface normals point from result interior to result exterior.
			if plus && !minus {
				poly.flip()
			}
			result = append(result, poly)
			checkComplexity("surface fragments", len(result), options.MaxTotalFragments)
		}
	}
	return result
}

func sortedTriangles(mesh *model3d.Mesh) []*model3d.Triangle {
	triangles := mesh.TriangleSlice()
	type keyedTriangle struct {
		triangle *model3d.Triangle
		key      triangleKey
	}
	keyed := make([]keyedTriangle, len(triangles))
	for i, triangle := range triangles {
		keyed[i].triangle = triangle
		keyed[i].key, _ = makeTriangleKey(*triangle)
	}
	sort.Slice(keyed, func(i, j int) bool {
		return triangleKeyLess(keyed[i].key, keyed[j].key)
	})
	for i := range keyed {
		triangles[i] = keyed[i].triangle
	}
	return triangles
}

func splitPolygon2D(vertices []model2d.Coord, lineA, lineB model2d.Coord, tol float64) ([]model2d.Coord, []model2d.Coord) {
	direction := lineB.Sub(lineA)
	frontVertices := make([]model2d.Coord, 0, len(vertices)+1)
	backVertices := make([]model2d.Coord, 0, len(vertices)+1)
	distances := make([]float64, len(vertices))
	for i, point := range vertices {
		distances[i] = cross2D(direction, point.Sub(lineA))
	}
	detTol := tol * math.Max(direction.Norm(), tol)
	for i, point := range vertices {
		j := (i + 1) % len(vertices)
		nextPoint := vertices[j]
		d1, d2 := distances[i], distances[j]
		if d1 >= -detTol {
			frontVertices = append(frontVertices, point)
		}
		if d1 <= detTol {
			backVertices = append(backVertices, point)
		}
		if (d1 < -detTol && d2 > detTol) || (d1 > detTol && d2 < -detTol) {
			t := d1 / (d1 - d2)
			intersection := point.Add(nextPoint.Sub(point).Scale(t))
			frontVertices = append(frontVertices, intersection)
			backVertices = append(backVertices, intersection)
		}
	}
	return cleanPolygon2D(frontVertices), cleanPolygon2D(backVertices)
}

func cleanPolygon2D(vertices []model2d.Coord) []model2d.Coord {
	result := vertices[:0]
	for _, point := range vertices {
		if len(result) == 0 || point != result[len(result)-1] {
			result = append(result, point)
		}
	}
	if len(result) > 1 && result[0] == result[len(result)-1] {
		result = result[:len(result)-1]
	}
	return result
}

func cross2D(a, b model2d.Coord) float64 { return a.X*b.Y - a.Y*b.X }

func convexPolygonsOverlap2D(a, b []model2d.Coord, tol float64) bool {
	for _, polygon := range [][]model2d.Coord{a, b} {
		for i, point := range polygon {
			edge := polygon[(i+1)%len(polygon)].Sub(point)
			axis := model2d.XY(-edge.Y, edge.X)
			if axis.NormSquared() == 0 {
				continue
			}
			amin, amax := projectionRange2D(a, axis)
			bmin, bmax := projectionRange2D(b, axis)
			projectionTol := tol * axis.Norm()
			if amax < bmin-projectionTol || bmax < amin-projectionTol {
				return false
			}
		}
	}
	return true
}

func projectionRange2D(points []model2d.Coord, axis model2d.Coord) (float64, float64) {
	min, max := points[0].Dot(axis), points[0].Dot(axis)
	for _, point := range points[1:] {
		value := point.Dot(axis)
		min, max = math.Min(min, value), math.Max(max, value)
	}
	return min, max
}

func evalMeshBoolean(kind meshBooleanKind, inA, inB bool) bool {
	switch kind {
	case meshUnion:
		return inA || inB
	case meshIntersection:
		return inA && inB
	case meshDifference:
		return inA && !inB
	default:
		panic("invalid mesh boolean kind")
	}
}

type polygon struct {
	vertices []model3d.Coord3D
	plane    plane
}

type plane struct {
	normal model3d.Coord3D
	w      float64
}

func newPolygon(vertices []model3d.Coord3D) *polygon {
	vertices = cleanVertices(vertices)
	if len(vertices) < 3 {
		return nil
	}
	n := vertices[1].Sub(vertices[0]).Cross(vertices[2].Sub(vertices[0]))
	if n.NormSquared() == 0 {
		return nil
	}
	n = n.Normalize()
	return &polygon{vertices: vertices, plane: plane{normal: n, w: n.Dot(vertices[0])}}
}

func (p *polygon) flip() {
	for i, j := 0, len(p.vertices)-1; i < j; i, j = i+1, j-1 {
		p.vertices[i], p.vertices[j] = p.vertices[j], p.vertices[i]
	}
	p.plane.normal = p.plane.normal.Scale(-1)
	p.plane.w = -p.plane.w
}

func cleanVertices(vertices []model3d.Coord3D) []model3d.Coord3D {
	result := make([]model3d.Coord3D, 0, len(vertices))
	for _, v := range vertices {
		if len(result) == 0 || v != result[len(result)-1] {
			result = append(result, v)
		}
	}
	if len(result) > 1 && result[0] == result[len(result)-1] {
		result = result[:len(result)-1]
	}
	// Splitting can leave exactly collinear points. Removing them reduces
	// sliver triangles and makes coincident fragments easier to deduplicate.
	for changed := true; changed && len(result) >= 3; {
		changed = false
		for i := range result {
			prev := result[(i+len(result)-1)%len(result)]
			cur := result[i]
			next := result[(i+1)%len(result)]
			a, b := cur.Sub(prev), next.Sub(cur)
			if a.Cross(b).NormSquared() <= 1e-28*a.NormSquared()*b.NormSquared() && a.Dot(b) >= 0 {
				result = append(result[:i], result[i+1:]...)
				changed = true
				break
			}
		}
	}
	return result
}

type coplanarGroup struct {
	normal             model3d.Coord3D
	w                  float64
	positive, negative []*model2d.Mesh
}

type planeGroupKey [4]int64

func polygonsMesh(options Options, polygons []*polygon, scale float64) *model3d.Mesh {
	tol := math.Max(scale*1e-9, math.SmallestNonzeroFloat64*1024)
	var groups []*coplanarGroup
	normalStep := 1e-10
	wStep := tol * 4
	buckets := map[planeGroupKey][]*coplanarGroup{}
	keyFor := func(normal model3d.Coord3D, w float64) planeGroupKey {
		return planeGroupKey{
			roundToInt64(normal.X / normalStep), roundToInt64(normal.Y / normalStep),
			roundToInt64(normal.Z / normalStep), roundToInt64(w / wStep),
		}
	}
	for _, poly := range polygons {
		normal, w, positive := canonicalPlane(poly.plane.normal, poly.plane.w)
		var group *coplanarGroup
		key := keyFor(normal, w)
		for dx := int64(-1); dx <= 1 && group == nil; dx++ {
			for dy := int64(-1); dy <= 1 && group == nil; dy++ {
				for dz := int64(-1); dz <= 1 && group == nil; dz++ {
					for dw := int64(-1); dw <= 1 && group == nil; dw++ {
						neighbor := planeGroupKey{key[0] + dx, key[1] + dy, key[2] + dz, key[3] + dw}
						for _, candidate := range buckets[neighbor] {
							if candidate.normal.Dot(normal) >= 1-1e-12 && math.Abs(candidate.w-w) <= tol {
								group = candidate
								break
							}
						}
					}
				}
			}
		}
		if group == nil {
			group = &coplanarGroup{normal: normal, w: w}
			groups = append(groups, group)
			buckets[key] = append(buckets[key], group)
		}
		u, v := planeBasis(group.normal)
		mesh := model2d.NewMesh()
		projected := make([]model2d.Coord, len(poly.vertices))
		for i, point := range poly.vertices {
			projected[i] = model2d.XY(u.Dot(point), v.Dot(point))
		}
		for i, point := range projected {
			next := projected[(i+1)%len(projected)]
			if point != next {
				mesh.Add(&model2d.Segment{point, next})
			}
		}
		if positive {
			group.positive = append(group.positive, mesh)
		} else {
			group.negative = append(group.negative, mesh)
		}
	}

	var raw []model3d.Triangle
	for _, group := range groups {
		positive := bool2d.UnionWithOptions(options.PlanarOptions, group.positive...)
		negative := bool2d.UnionWithOptions(options.PlanarOptions, group.negative...)
		if positive.NumSegments() != 0 {
			raw = append(raw, liftPlanarMesh(positive, group, true)...)
			checkComplexity("triangulated coplanar surfaces", len(raw), options.MaxOutputTriangles)
		}
		if negative.NumSegments() != 0 {
			raw = append(raw, liftPlanarMesh(negative, group, false)...)
			checkComplexity("triangulated coplanar surfaces", len(raw), options.MaxOutputTriangles)
		}
	}
	return finalizeTriangles(options, raw, tol)
}

func canonicalPlane(normal model3d.Coord3D, w float64) (model3d.Coord3D, float64, bool) {
	positive := true
	if normal.X < 0 || (normal.X == 0 && (normal.Y < 0 || (normal.Y == 0 && normal.Z < 0))) {
		normal, w, positive = normal.Scale(-1), -w, false
	}
	return normal, w, positive
}

func planeBasis(normal model3d.Coord3D) (model3d.Coord3D, model3d.Coord3D) {
	u, v := normal.OrthoBasis()
	if u.Cross(v).Dot(normal) < 0 {
		v = v.Scale(-1)
	}
	return u, v
}

func liftPlanarMesh(mesh *model2d.Mesh, group *coplanarGroup, positive bool) []model3d.Triangle {
	u, v := planeBasis(group.normal)
	lift := func(c model2d.Coord) model3d.Coord3D {
		return u.Scale(c.X).Add(v.Scale(c.Y)).Add(group.normal.Scale(group.w))
	}
	triangles2d := triangulatePlanarMesh(mesh)
	result := make([]model3d.Triangle, 0, len(triangles2d))
	desired := group.normal
	if !positive {
		desired = desired.Scale(-1)
	}
	for _, tri2d := range triangles2d {
		tri := model3d.Triangle{lift(tri2d[0]), lift(tri2d[1]), lift(tri2d[2])}
		if tri.Normal().Dot(desired) < 0 {
			tri[0], tri[1] = tri[1], tri[0]
		}
		result = append(result, tri)
	}
	return result
}

func triangulatePlanarMesh(mesh *model2d.Mesh) [][3]model2d.Coord {
	if !mesh.Manifold() {
		panic(&TopologyError{Problem: "non-manifold planar triangulation boundary"})
	}
	return model2d.TriangulateMesh(mesh)
}

func finalizeTriangles(options Options, raw []model3d.Triangle, tol float64) *model3d.Mesh {
	checkComplexity("output triangles", len(raw), options.MaxOutputTriangles)
	canon := newCoordCanonicalizer(tol)
	for i := range raw {
		for j := range raw[i] {
			raw[i][j] = canon.add(raw[i][j])
		}
	}
	index := newPointIndex(canon.points)
	conformed := make([]model3d.Triangle, 0, len(raw))
	for _, tri := range raw {
		conformed = append(conformed, conformTriangle(tri, index, tol)...)
		checkComplexity("conforming output triangles", len(conformed), options.MaxOutputTriangles)
	}
	raw = conformed
	balances := map[triangleKey]triangleBalance{}
	for _, tri := range raw {
		if tri.Area() <= tol*tol {
			continue
		}
		key, orientation := makeTriangleKey(tri)
		balance := balances[key]
		if orientation {
			balance.value++
			balance.positive = tri
		} else {
			balance.value--
			balance.negative = tri
		}
		balances[key] = balance
	}
	var candidates []model3d.Triangle
	for _, balance := range balances {
		if balance.value == 0 {
			continue
		}
		tri := balance.positive
		if balance.value < 0 {
			tri = balance.negative
		}
		candidates = append(candidates, tri)
	}
	// Numerical boundary classification can leave a zero-neighborhood sliver:
	// a triangle whose three edges are all exposed and which is not connected
	// to the closed result. Such a patch cannot bound volume and retaining it
	// would make the result non-manifold.
	for {
		edgeCounts := map[model3d.Segment]int{}
		for _, tri := range candidates {
			for i := range tri {
				edgeCounts[model3d.NewSegment(tri[i], tri[(i+1)%3])]++
			}
		}
		filtered := make([]model3d.Triangle, 0, len(candidates))
		for _, tri := range candidates {
			exposed := 0
			for i := range tri {
				if edgeCounts[model3d.NewSegment(tri[i], tri[(i+1)%3])] != 2 {
					exposed++
				}
			}
			if exposed < 2 {
				filtered = append(filtered, tri)
			}
		}
		if len(filtered) == len(candidates) {
			break
		}
		candidates = filtered
	}
	result := model3d.NewMesh()
	for _, tri := range candidates {
		triCopy := tri
		result.Add(&triCopy)
	}
	result = separateContactEdges(options, result, tol)
	result = separateContactComponents(result, tol)
	result = separateSingularVertexFans(result, tol)
	return validateResultTopology(result)
}

func validateResultTopology(mesh *model3d.Mesh) *model3d.Mesh {
	if mesh.NumTriangles() == 0 {
		return mesh
	}
	if mesh.NeedsRepair() {
		panic(&TopologyError{Problem: "edges do not each have two incident triangles"})
	}
	if singular := mesh.SingularVertices(); len(singular) != 0 {
		panic(&TopologyError{Problem: "singular vertices", Count: len(singular)})
	}
	if !mesh.Orientable() {
		panic(&TopologyError{Problem: "non-orientable surface"})
	}
	if intersections := mesh.SelfIntersections(); intersections != 0 {
		panic(&TopologyError{Problem: "self-intersections", Count: intersections})
	}
	return mesh
}

type contactEdgeTriangle struct {
	triangle  *model3d.Triangle
	theta     float64
	normalDir bool
}

// separateContactEdges replaces an edge used by more than two triangles with
// one slightly displaced V-shaped edge per oriented surface pair. This turns
// an exact edge contact into point contacts at its endpoints, which the later
// vertex cleanup can resolve without opening either surface.
func separateContactEdges(options Options, mesh *model3d.Mesh, tol float64) *model3d.Mesh {
	edgeCounts := map[model3d.Segment]int{}
	mesh.Iterate(func(triangle *model3d.Triangle) {
		for i := range triangle {
			edgeCounts[model3d.NewSegment(triangle[i], triangle[(i+1)%3])]++
		}
	})
	var contacts []model3d.Segment
	for edge, count := range edgeCounts {
		if count > 2 {
			contacts = append(contacts, edge)
		}
	}
	if len(contacts) == 0 {
		return mesh
	}
	sort.Slice(contacts, func(i, j int) bool {
		if contacts[i][0] != contacts[j][0] {
			return coord3Less(contacts[i][0], contacts[j][0])
		}
		return coord3Less(contacts[i][1], contacts[j][1])
	})
	result := mesh.DeepCopy()
	for _, edge := range contacts {
		incident := result.Find(edge[0], edge[1])
		if len(incident) <= 2 || len(incident)%2 != 0 {
			continue
		}
		checkComplexity("triangles at one contact edge", len(incident), options.MaxContactEdgeTriangles)
		groups, directions := pairContactEdgeTriangles(edge, incident)
		midpoint := edge.Mid()
		for groupIndex, group := range groups {
			shift := math.Min(tol*4, edge.Length()*0.25)
			for _, triangle := range group {
				shift = math.Min(shift, midpoint.Dist(contactEdgeOtherVertex(edge, triangle))*0.25)
			}
			newMidpoint := midpoint.Add(directions[groupIndex].Scale(shift))
			for _, triangle := range group {
				other := contactEdgeOtherVertex(edge, triangle)
				first := model3d.Triangle{other, edge[0], newMidpoint}
				second := model3d.Triangle{other, newMidpoint, edge[1]}
				shared := model3d.NewSegment(other, edge[0])
				if triangleSegmentOrientation(&first, shared) != triangleSegmentOrientation(triangle, shared) {
					first[0], first[1] = first[1], first[0]
					second[0], second[1] = second[1], second[0]
				}
				result.Remove(triangle)
				result.Add(&first)
				result.Add(&second)
			}
			checkComplexity("edge-contact regularization triangles", result.NumTriangles(), options.MaxOutputTriangles)
		}
	}
	return result
}

func pairContactEdgeTriangles(edge model3d.Segment, triangles []*model3d.Triangle) ([][2]*model3d.Triangle, []model3d.Coord3D) {
	axis := edge[0].Sub(edge[1]).Normalize()
	basis1, basis2 := axis.OrthoBasis()
	midpoint := edge.Mid()
	angular := make([]contactEdgeTriangle, len(triangles))
	for i, triangle := range triangles {
		triangleVector := contactEdgeOtherVertex(edge, triangle).Sub(midpoint).Normalize()
		x, y := basis1.Dot(triangleVector), basis2.Dot(triangleVector)
		normal := triangle.Normal()
		normalX, normalY := basis1.Dot(normal), basis2.Dot(normal)
		angular[i] = contactEdgeTriangle{
			triangle:  triangle,
			theta:     math.Atan2(y, x),
			normalDir: normalX*y-normalY*x > 0,
		}
	}
	sort.Slice(angular, func(i, j int) bool {
		if angular[i].theta != angular[j].theta {
			return angular[i].theta < angular[j].theta
		}
		keyI, _ := makeTriangleKey(*angular[i].triangle)
		keyJ, _ := makeTriangleKey(*angular[j].triangle)
		return triangleKeyLess(keyI, keyJ)
	})
	if len(angular) > 2 && angular[0].normalDir {
		first := angular[0]
		copy(angular, angular[1:])
		first.theta += 2 * math.Pi
		angular[len(angular)-1] = first
	}
	for i := 0; i < len(angular); i += 2 {
		for j := i + 1; j < len(angular); j++ {
			if triangleSegmentOrientation(angular[i].triangle, edge) !=
				triangleSegmentOrientation(angular[j].triangle, edge) {
				angular[i+1], angular[j] = angular[j], angular[i+1]
				break
			}
		}
	}
	groups := make([][2]*model3d.Triangle, 0, len(angular)/2)
	directions := make([]model3d.Coord3D, 0, len(angular)/2)
	for i := 0; i < len(angular); i += 2 {
		groups = append(groups, [2]*model3d.Triangle{angular[i].triangle, angular[i+1].triangle})
		theta := (angular[i].theta + angular[i+1].theta) / 2
		directions = append(directions,
			basis1.Scale(math.Cos(theta)).Add(basis2.Scale(math.Sin(theta))))
	}
	return groups, directions
}

func contactEdgeOtherVertex(edge model3d.Segment, triangle *model3d.Triangle) model3d.Coord3D {
	for _, point := range triangle {
		if point != edge[0] && point != edge[1] {
			return point
		}
	}
	panic("meshbool: degenerate triangle on contact edge")
}

func triangleSegmentOrientation(triangle *model3d.Triangle, segment model3d.Segment) bool {
	for i, point := range triangle {
		if point == segment[0] {
			return triangle[(i+2)%3] == segment[1]
		}
	}
	panic("meshbool: segment is not in triangle")
}

func separateContactComponents(mesh *model3d.Mesh, tol float64) *model3d.Mesh {
	triangles := sortedTriangles(mesh)
	if len(triangles) == 0 {
		return mesh
	}
	edgeTriangles := map[model3d.Segment][]int{}
	for i, tri := range triangles {
		for j := range tri {
			edge := model3d.NewSegment(tri[j], tri[(j+1)%3])
			edgeTriangles[edge] = append(edgeTriangles[edge], i)
		}
	}
	adjacent := make([][]int, len(triangles))
	for _, indices := range edgeTriangles {
		if len(indices) == 2 {
			a, b := indices[0], indices[1]
			adjacent[a] = append(adjacent[a], b)
			adjacent[b] = append(adjacent[b], a)
		}
	}
	components := make([]int, len(triangles))
	for i := range components {
		components[i] = -1
	}
	var centers []model3d.Coord3D
	var centerCounts []int
	for start := range triangles {
		if components[start] >= 0 {
			continue
		}
		component := len(centers)
		centers = append(centers, model3d.Coord3D{})
		centerCounts = append(centerCounts, 0)
		queue := []int{start}
		components[start] = component
		for len(queue) != 0 {
			index := queue[len(queue)-1]
			queue = queue[:len(queue)-1]
			for _, point := range triangles[index] {
				centers[component] = centers[component].Add(point)
				centerCounts[component]++
			}
			for _, neighbor := range adjacent[index] {
				if components[neighbor] < 0 {
					components[neighbor] = component
					queue = append(queue, neighbor)
				}
			}
		}
	}
	if len(centers) <= 1 {
		return mesh
	}
	for i := range centers {
		centers[i] = centers[i].Scale(1 / float64(centerCounts[i]))
	}
	type vertexComponent struct {
		vertex    model3d.Coord3D
		component int
	}
	vertexComponents := map[model3d.Coord3D]map[int]bool{}
	vertexComponentMinEdge := map[vertexComponent]float64{}
	for i, tri := range triangles {
		for vertex, point := range tri {
			if vertexComponents[point] == nil {
				vertexComponents[point] = map[int]bool{}
			}
			vertexComponents[point][components[i]] = true
			key := vertexComponent{vertex: point, component: components[i]}
			minimum := vertexComponentMinEdge[key]
			for _, neighbor := range []model3d.Coord3D{tri[(vertex+1)%3], tri[(vertex+2)%3]} {
				length := point.Dist(neighbor)
				if minimum == 0 || length < minimum {
					minimum = length
				}
			}
			vertexComponentMinEdge[key] = minimum
		}
	}
	shared := false
	for _, componentSet := range vertexComponents {
		if len(componentSet) > 1 {
			shared = true
			break
		}
	}
	if !shared {
		return mesh
	}
	result := model3d.NewMesh()
	for i, oldTriangle := range triangles {
		triangle := *oldTriangle
		component := components[i]
		for j, point := range triangle {
			componentSet := vertexComponents[point]
			if len(componentSet) <= 1 || component == minimumComponent(componentSet) {
				continue
			}
			direction := centers[component].Sub(point)
			if direction.NormSquared() == 0 {
				direction = model3d.X(1)
			} else {
				direction = direction.Normalize()
			}
			shift := math.Min(tol*4, vertexComponentMinEdge[vertexComponent{
				vertex: point, component: component,
			}]*0.25)
			triangle[j] = point.Add(direction.Scale(shift))
		}
		result.Add(&triangle)
	}
	return result
}

func minimumComponent(components map[int]bool) int {
	minimum := -1
	for component := range components {
		if minimum < 0 || component < minimum {
			minimum = component
		}
	}
	return minimum
}

// separateSingularVertexFans handles a point contact even when the two local
// surface sheets are connected elsewhere. Edge-connected triangle fans at the
// singular vertex are moved together, preserving every ordinary mesh edge.
// Contacts involving an edge with more than two triangles are left to
// separateContactComponents, since pairing coincident edge fans locally would
// require a different ambiguity policy.
func separateSingularVertexFans(mesh *model3d.Mesh, tol float64) *model3d.Mesh {
	singular := mesh.SingularVertices()
	if len(singular) == 0 {
		return mesh
	}
	sort.Slice(singular, func(i, j int) bool { return coord3Less(singular[i], singular[j]) })
	triangles := sortedTriangles(mesh)
	edgeTriangles := map[model3d.Segment][]int{}
	vertexTriangles := map[model3d.Coord3D][]int{}
	for i, triangle := range triangles {
		for j, point := range triangle {
			vertexTriangles[point] = append(vertexTriangles[point], i)
			edge := model3d.NewSegment(point, triangle[(j+1)%3])
			edgeTriangles[edge] = append(edgeTriangles[edge], i)
		}
	}
	type triangleVertex struct {
		triangle, vertex int
	}
	replacements := map[triangleVertex]model3d.Coord3D{}
	for _, point := range singular {
		incident := vertexTriangles[point]
		eligible := true
		for _, triangleIndex := range incident {
			triangle := triangles[triangleIndex]
			for vertex, candidate := range triangle {
				if candidate != point {
					continue
				}
				for _, neighbor := range []model3d.Coord3D{
					triangle[(vertex+1)%3], triangle[(vertex+2)%3],
				} {
					if len(edgeTriangles[model3d.NewSegment(point, neighbor)]) > 2 {
						eligible = false
					}
				}
			}
		}
		if !eligible {
			continue
		}

		incidentSet := map[int]bool{}
		for _, triangleIndex := range incident {
			incidentSet[triangleIndex] = true
		}
		fan := map[int]int{}
		var fanTriangles [][]int
		for _, seed := range incident {
			if _, ok := fan[seed]; ok {
				continue
			}
			fanID := len(fanTriangles)
			fan[seed] = fanID
			stack := []int{seed}
			var members []int
			for len(stack) != 0 {
				last := len(stack) - 1
				current := stack[last]
				stack = stack[:last]
				members = append(members, current)
				triangle := triangles[current]
				for vertex, candidate := range triangle {
					if candidate != point {
						continue
					}
					for _, neighbor := range []model3d.Coord3D{
						triangle[(vertex+1)%3], triangle[(vertex+2)%3],
					} {
						for _, next := range edgeTriangles[model3d.NewSegment(point, neighbor)] {
							if incidentSet[next] {
								if _, ok := fan[next]; !ok {
									fan[next] = fanID
									stack = append(stack, next)
								}
							}
						}
					}
				}
			}
			fanTriangles = append(fanTriangles, members)
		}
		if len(fanTriangles) < 2 {
			continue
		}
		for fanID, members := range fanTriangles {
			center := model3d.Coord3D{}
			centerCount := 0
			shift := tol * 4
			for _, triangleIndex := range members {
				for _, candidate := range triangles[triangleIndex] {
					if candidate != point {
						center = center.Add(candidate)
						centerCount++
						shift = math.Min(shift, point.Dist(candidate)*0.25)
					}
				}
			}
			direction := center.Scale(1 / float64(centerCount)).Sub(point)
			if direction.NormSquared() == 0 {
				angle := float64(fanID+1) * 2.399963229728653
				direction = model3d.XYZ(math.Cos(angle), math.Sin(angle), 0.5).Normalize()
			} else {
				direction = direction.Normalize()
			}
			replacement := point.Add(direction.Scale(shift))
			for _, triangleIndex := range members {
				for vertex, candidate := range triangles[triangleIndex] {
					if candidate == point {
						replacements[triangleVertex{triangle: triangleIndex, vertex: vertex}] = replacement
					}
				}
			}
		}
	}
	if len(replacements) == 0 {
		return mesh
	}
	result := model3d.NewMesh()
	for triangleIndex, oldTriangle := range triangles {
		triangle := *oldTriangle
		for vertex := range triangle {
			if replacement, ok := replacements[triangleVertex{triangle: triangleIndex, vertex: vertex}]; ok {
				triangle[vertex] = replacement
			}
		}
		result.Add(&triangle)
	}
	return result
}

type coordGridKey struct{ x, y, z int64 }

type coordCanonicalizer struct {
	tol     float64
	buckets map[coordGridKey][]model3d.Coord3D
	points  []model3d.Coord3D
}

func newCoordCanonicalizer(tol float64) *coordCanonicalizer {
	return &coordCanonicalizer{tol: tol, buckets: map[coordGridKey][]model3d.Coord3D{}}
}

func (c *coordCanonicalizer) add(p model3d.Coord3D) model3d.Coord3D {
	k := c.key(p)
	for dx := int64(-1); dx <= 1; dx++ {
		for dy := int64(-1); dy <= 1; dy++ {
			for dz := int64(-1); dz <= 1; dz++ {
				for _, old := range c.buckets[coordGridKey{k.x + dx, k.y + dy, k.z + dz}] {
					if old.Dist(p) <= c.tol {
						return old
					}
				}
			}
		}
	}
	c.buckets[k] = append(c.buckets[k], p)
	c.points = append(c.points, p)
	return p
}

func (c *coordCanonicalizer) key(p model3d.Coord3D) coordGridKey {
	return coordGridKey{floorInt64(p.X / c.tol), floorInt64(p.Y / c.tol), floorInt64(p.Z / c.tol)}
}

func floorInt64(x float64) int64 {
	if x >= math.MaxInt64 {
		return math.MaxInt64
	}
	if x <= math.MinInt64 {
		return math.MinInt64
	}
	return int64(math.Floor(x))
}

func roundToInt64(x float64) int64 {
	return floorInt64(x + 0.5)
}

type pointIndex struct {
	min, max    model3d.Coord3D
	points      []model3d.Coord3D
	left, right *pointIndex
}

type triangleIndex struct {
	min, max    model3d.Coord3D
	triangles   []*model3d.Triangle
	left, right *triangleIndex
}

func newTriangleIndex(triangles []*model3d.Triangle) *triangleIndex {
	if len(triangles) == 0 {
		return nil
	}
	return buildTriangleIndex(append([]*model3d.Triangle(nil), triangles...))
}

func buildTriangleIndex(triangles []*model3d.Triangle) *triangleIndex {
	node := &triangleIndex{min: triangles[0].Min(), max: triangles[0].Max()}
	for _, triangle := range triangles[1:] {
		node.min = node.min.Min(triangle.Min())
		node.max = node.max.Max(triangle.Max())
	}
	if len(triangles) <= 8 {
		node.triangles = triangles
		return node
	}
	span := node.max.Sub(node.min)
	axis := 0
	if span.Y > span.X {
		axis = 1
	}
	if coordAxis(span, 2) > coordAxis(span, axis) {
		axis = 2
	}
	sort.Slice(triangles, func(i, j int) bool {
		centerI := triangles[i].Min().Mid(triangles[i].Max())
		centerJ := triangles[j].Min().Mid(triangles[j].Max())
		return coordAxis(centerI, axis) < coordAxis(centerJ, axis)
	})
	mid := len(triangles) / 2
	node.left = buildTriangleIndex(triangles[:mid])
	node.right = buildTriangleIndex(triangles[mid:])
	return node
}

func (t *triangleIndex) query(min, max model3d.Coord3D, result *[]*model3d.Triangle) {
	if t == nil || t.min.X > max.X || t.max.X < min.X ||
		t.min.Y > max.Y || t.max.Y < min.Y || t.min.Z > max.Z || t.max.Z < min.Z {
		return
	}
	if t.left == nil {
		for _, triangle := range t.triangles {
			triMin, triMax := triangle.Min(), triangle.Max()
			if triMin.X <= max.X && triMax.X >= min.X && triMin.Y <= max.Y && triMax.Y >= min.Y &&
				triMin.Z <= max.Z && triMax.Z >= min.Z {
				*result = append(*result, triangle)
			}
		}
		return
	}
	t.left.query(min, max, result)
	t.right.query(min, max, result)
}

func newPointIndex(points []model3d.Coord3D) *pointIndex {
	if len(points) == 0 {
		return nil
	}
	points = append([]model3d.Coord3D(nil), points...)
	return buildPointIndex(points)
}

func buildPointIndex(points []model3d.Coord3D) *pointIndex {
	n := &pointIndex{min: points[0], max: points[0]}
	for _, p := range points[1:] {
		n.min, n.max = n.min.Min(p), n.max.Max(p)
	}
	if len(points) <= 12 {
		n.points = points
		return n
	}
	span := n.max.Sub(n.min)
	axis := 0
	if span.Y > span.X {
		axis = 1
	}
	if coordAxis(span, 2) > coordAxis(span, axis) {
		axis = 2
	}
	sort.Slice(points, func(i, j int) bool { return coordAxis(points[i], axis) < coordAxis(points[j], axis) })
	mid := len(points) / 2
	n.left = buildPointIndex(points[:mid])
	n.right = buildPointIndex(points[mid:])
	return n
}

func (p *pointIndex) query(min, max model3d.Coord3D, result *[]model3d.Coord3D) {
	if p == nil || p.min.X > max.X || p.max.X < min.X ||
		p.min.Y > max.Y || p.max.Y < min.Y || p.min.Z > max.Z || p.max.Z < min.Z {
		return
	}
	if p.left == nil {
		for _, point := range p.points {
			if point.X >= min.X && point.X <= max.X && point.Y >= min.Y && point.Y <= max.Y &&
				point.Z >= min.Z && point.Z <= max.Z {
				*result = append(*result, point)
			}
		}
		return
	}
	p.left.query(min, max, result)
	p.right.query(min, max, result)
}

func conformTriangle(t model3d.Triangle, index *pointIndex, tol float64) []model3d.Triangle {
	min := t.Min().AddScalar(-tol)
	max := t.Max().AddScalar(tol)
	var candidates []model3d.Coord3D
	index.query(min, max, &candidates)
	normal := t.Normal()
	filtered := candidates[:0]
	for _, point := range candidates {
		if point == t[0] || point == t[1] || point == t[2] {
			continue
		}
		if math.Abs(normal.Dot(point.Sub(t[0]))) <= tol && pointInTriangle3D(t, point, tol) {
			filtered = append(filtered, point)
		}
	}
	sort.Slice(filtered, func(i, j int) bool { return coord3Less(filtered[i], filtered[j]) })
	triangles := []model3d.Triangle{t}
	for _, point := range filtered {
		alreadyVertex := false
		for _, triangle := range triangles {
			if triangle[0] == point || triangle[1] == point || triangle[2] == point {
				alreadyVertex = true
				break
			}
		}
		if alreadyVertex {
			continue
		}
		splitEdge := false
		next := make([]model3d.Triangle, 0, len(triangles)+2)
		for _, triangle := range triangles {
			edge := pointOnTriangleEdge3D(triangle, point, tol)
			if edge < 0 {
				next = append(next, triangle)
				continue
			}
			splitEdge = true
			a, b, c := triangle[edge], triangle[(edge+1)%3], triangle[(edge+2)%3]
			next = appendNondegenerate(next, model3d.Triangle{a, point, c}, tol)
			next = appendNondegenerate(next, model3d.Triangle{point, b, c}, tol)
		}
		if splitEdge {
			triangles = next
			continue
		}
		for i, triangle := range triangles {
			if pointInTriangle3D(triangle, point, tol) {
				replacement := make([]model3d.Triangle, 0, 3)
				replacement = appendNondegenerate(replacement, model3d.Triangle{triangle[0], triangle[1], point}, tol)
				replacement = appendNondegenerate(replacement, model3d.Triangle{triangle[1], triangle[2], point}, tol)
				replacement = appendNondegenerate(replacement, model3d.Triangle{triangle[2], triangle[0], point}, tol)
				triangles = append(triangles[:i], append(replacement, triangles[i+1:]...)...)
				break
			}
		}
	}
	return triangles
}

func appendNondegenerate(triangles []model3d.Triangle, triangle model3d.Triangle, tol float64) []model3d.Triangle {
	if triangle.Area() > tol*tol {
		return append(triangles, triangle)
	}
	return triangles
}

func pointOnTriangleEdge3D(triangle model3d.Triangle, point model3d.Coord3D, tol float64) int {
	for i, a := range triangle {
		b := triangle[(i+1)%3]
		delta := b.Sub(a)
		lengthSquared := delta.NormSquared()
		if lengthSquared == 0 {
			continue
		}
		parameter := point.Sub(a).Dot(delta) / lengthSquared
		if parameter > 0 && parameter < 1 && a.Add(delta.Scale(parameter)).Dist(point) <= tol {
			return i
		}
	}
	return -1
}

func pointInTriangle3D(triangle model3d.Triangle, point model3d.Coord3D, tol float64) bool {
	a := triangle[1].Sub(triangle[0])
	b := triangle[2].Sub(triangle[0])
	p := point.Sub(triangle[0])
	d00, d01, d11 := a.Dot(a), a.Dot(b), b.Dot(b)
	d20, d21 := p.Dot(a), p.Dot(b)
	denominator := d00*d11 - d01*d01
	if denominator <= 0 {
		return false
	}
	v := (d11*d20 - d01*d21) / denominator
	w := (d00*d21 - d01*d20) / denominator
	u := 1 - v - w
	parameterTolerance := tol / math.Max(math.Sqrt(math.Max(d00, d11)), tol)
	return u >= -parameterTolerance && v >= -parameterTolerance && w >= -parameterTolerance
}

func coordAxis(c model3d.Coord3D, axis int) float64 {
	switch axis {
	case 0:
		return c.X
	case 1:
		return c.Y
	default:
		return c.Z
	}
}

type triangleKey [3]model3d.Coord3D

type triangleBalance struct {
	value              int
	positive, negative model3d.Triangle
}

func makeTriangleKey(t model3d.Triangle) (triangleKey, bool) {
	coords := [3]model3d.Coord3D{t[0], t[1], t[2]}
	inversions := 0
	for end := len(coords) - 1; end > 0; end-- {
		for i := 0; i < end; i++ {
			if coord3Less(coords[i+1], coords[i]) {
				coords[i], coords[i+1] = coords[i+1], coords[i]
				inversions++
			}
		}
	}
	return triangleKey(coords), inversions%2 == 0
}

func triangleKeyLess(a, b triangleKey) bool {
	for i := range a {
		if a[i] != b[i] {
			return coord3Less(a[i], b[i])
		}
	}
	return false
}

func coord3Less(a, b model3d.Coord3D) bool {
	return a.X < b.X || (a.X == b.X && (a.Y < b.Y || (a.Y == b.Y && a.Z < b.Z)))
}

func meshScale(meshes []*model3d.Mesh) float64 {
	initialized := false
	maxAbs := 0.0
	var min, max model3d.Coord3D
	for _, mesh := range meshes {
		mesh.IterateVertices(func(v model3d.Coord3D) {
			maxAbs = math.Max(maxAbs, v.Abs().MaxCoord())
			if !initialized {
				min, max, initialized = v, v, true
			} else {
				min, max = min.Min(v), max.Max(v)
			}
		})
	}
	if !initialized {
		return 1
	}
	scale := max.Sub(min).MaxCoord()
	if scale == 0 || math.IsInf(scale, 0) || math.IsNaN(scale) {
		scale = 1
	}
	// At large coordinate offsets, arithmetic cannot resolve distances below
	// a handful of ULPs even when the mesh's local extent is small. Inflate the
	// effective scale so all callers' relative tolerances honor that floor.
	ulp := math.Nextafter(maxAbs, math.Inf(1)) - maxAbs
	scale = math.Max(scale, ulp*16/1e-9)
	return scale
}

func nonNilMeshes(meshes []*model3d.Mesh) []*model3d.Mesh {
	result := make([]*model3d.Mesh, 0, len(meshes))
	for _, mesh := range meshes {
		if mesh != nil {
			result = append(result, mesh)
		}
	}
	return result
}
