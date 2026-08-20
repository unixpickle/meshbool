package meshbool

import (
	"errors"
	"fmt"
	"math"
	"math/big"
	"sort"

	"github.com/unixpickle/meshbool/internal/bool2d"
	model2d "github.com/unixpickle/model3d/model2d"
	"github.com/unixpickle/model3d/model3d"
)

const (
	arithmeticToleranceRelative      = 1e-12
	fallbackToleranceRelative        = 5e-15
	defaultMaxInputTriangles         = 200_000
	defaultMaxFragmentsPerTriangle   = 4_096
	defaultMaxTotalFragments         = 200_000
	defaultMaxOutputTriangles        = 200_000
	defaultMaxContactEdgeTriangles   = 1_024
	defaultMaxTriangleCandidatePairs = 5_000_000
)

// Options3D configures safety limits for 3-D boolean operations. A zero
// numeric field uses the corresponding value from DefaultOptions3D. Negative
// values are invalid. PlanarOptions controls coplanar surface processing.
type Options3D struct {
	MaxInputTriangles         int
	MaxFragmentsPerTriangle   int
	MaxTotalFragments         int
	MaxOutputTriangles        int
	MaxContactEdgeTriangles   int
	MaxTriangleCandidatePairs int
	PlanarOptions             Options2D
}

// DefaultOptions3D returns the standard limits for 3-D operations.
func DefaultOptions3D() Options3D {
	return Options3D{
		MaxInputTriangles:         defaultMaxInputTriangles,
		MaxFragmentsPerTriangle:   defaultMaxFragmentsPerTriangle,
		MaxTotalFragments:         defaultMaxTotalFragments,
		MaxOutputTriangles:        defaultMaxOutputTriangles,
		MaxContactEdgeTriangles:   defaultMaxContactEdgeTriangles,
		MaxTriangleCandidatePairs: defaultMaxTriangleCandidatePairs,
		PlanarOptions:             DefaultOptions2D(),
	}
}

func normalizeOptions3D(options Options3D) (Options3D, error) {
	defaults := DefaultOptions3D()
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
			return Options3D{}, fmt.Errorf("meshbool: 3D option %s must not be negative", field.name)
		}
		if *field.value == 0 {
			*field.value = *field.valueDefault
		}
	}
	if options.PlanarOptions == (Options2D{}) {
		options.PlanarOptions = defaults.PlanarOptions
	}
	if err := options.PlanarOptions.Validate(); err != nil {
		return Options3D{}, err
	}
	return options, nil
}

// Validate checks that no option is negative. Zero values are valid and mean
// to use the corresponding default.
func (o Options3D) Validate() error {
	_, err := normalizeOptions3D(o)
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

// Union3D computes the closed-set union of zero or more triangle meshes using
// the supplied safety limits. Input meshes are not modified.
func Union3D(options Options3D, meshes ...*model3d.Mesh) (*model3d.Mesh, error) {
	options, err := normalizeOptions3D(options)
	if err != nil {
		return nil, err
	}
	return union3D(options, meshes...)
}

func union3D(options Options3D, meshes ...*model3d.Mesh) (*model3d.Mesh, error) {
	meshes = nonNilMeshes(meshes)
	if len(meshes) == 0 {
		return model3d.NewMesh(), nil
	}
	if err := checkInputMeshes(options, meshes); err != nil {
		return nil, err
	}
	result := meshes[0].DeepCopy()
	for _, mesh := range meshes[1:] {
		var err error
		result, err = booleanMeshPair(options, result, mesh, meshUnion)
		if err != nil {
			return nil, err
		}
	}
	return result, nil
}

// Intersection3D computes the intersection of zero or more triangle meshes
// using the supplied safety limits. With no meshes, it returns an empty mesh.
// Input meshes are not modified.
func Intersection3D(options Options3D, meshes ...*model3d.Mesh) (*model3d.Mesh, error) {
	options, err := normalizeOptions3D(options)
	if err != nil {
		return nil, err
	}
	return intersection3D(options, meshes...)
}

func intersection3D(options Options3D, meshes ...*model3d.Mesh) (*model3d.Mesh, error) {
	meshes = nonNilMeshes(meshes)
	if len(meshes) == 0 {
		return model3d.NewMesh(), nil
	}
	if err := checkInputMeshes(options, meshes); err != nil {
		return nil, err
	}
	result := meshes[0].DeepCopy()
	for _, mesh := range meshes[1:] {
		var err error
		result, err = booleanMeshPair(options, result, mesh, meshIntersection)
		if err != nil {
			return nil, err
		}
		if result.NumTriangles() == 0 {
			break
		}
	}
	return result, nil
}

// Difference3D subtracts every mesh in subtract from first using the supplied
// safety limits. Input meshes are not modified.
func Difference3D(options Options3D, first *model3d.Mesh, subtract ...*model3d.Mesh) (*model3d.Mesh, error) {
	options, err := normalizeOptions3D(options)
	if err != nil {
		return nil, err
	}
	return difference3D(options, first, subtract...)
}

func difference3D(options Options3D, first *model3d.Mesh, subtract ...*model3d.Mesh) (*model3d.Mesh, error) {
	if first == nil {
		return model3d.NewMesh(), nil
	}
	meshes := make([]*model3d.Mesh, 1, 1+len(subtract))
	meshes[0] = first
	meshes = append(meshes, nonNilMeshes(subtract)...)
	if err := checkInputMeshes(options, meshes); err != nil {
		return nil, err
	}
	result := first.DeepCopy()
	for _, mesh := range meshes[1:] {
		var err error
		result, err = booleanMeshPair(options, result, mesh, meshDifference)
		if err != nil {
			return nil, err
		}
		if result.NumTriangles() == 0 {
			break
		}
	}
	return result, nil
}

func checkComplexity(stage string, count, limit int) error {
	if count > limit {
		return &ComplexityError{Stage: stage, Limit: limit}
	}
	return nil
}

func checkInputMeshes(options Options3D, meshes []*model3d.Mesh) error {
	for _, mesh := range meshes {
		if err := checkComplexity("triangles in one input mesh", mesh.NumTriangles(), options.MaxInputTriangles); err != nil {
			return err
		}
	}
	return nil
}

type meshBooleanKind int

const (
	meshUnion meshBooleanKind = iota
	meshIntersection
	meshDifference
)

func booleanMeshPair(options Options3D, a, b *model3d.Mesh, kind meshBooleanKind) (*model3d.Mesh, error) {
	if err := checkInputMeshes(options, []*model3d.Mesh{a, b}); err != nil {
		return nil, err
	}
	if a.NumTriangles() == 0 || b.NumTriangles() == 0 {
		switch kind {
		case meshUnion:
			if a.NumTriangles() == 0 {
				return b.DeepCopy(), nil
			}
			return a.DeepCopy(), nil
		case meshIntersection:
			return model3d.NewMesh(), nil
		case meshDifference:
			return a.DeepCopy(), nil
		default:
			return nil, fmt.Errorf("meshbool: unknown boolean operation %d", kind)
		}
	}
	min := a.Min().Min(b.Min())
	max := a.Max().Max(b.Max())
	span := max.Sub(min).MaxCoord()
	center := min.Mid(max)
	if center.Abs().MaxCoord() > math.Max(span, 1)*1024 {
		localA := a.Translate(center.Scale(-1))
		localB := b.Translate(center.Scale(-1))
		result, err := booleanMeshPairLocal(options, localA, localB, kind)
		if err != nil {
			return nil, err
		}
		return result.Translate(center), nil
	}
	return booleanMeshPairLocal(options, a, b, kind)
}

func booleanMeshPairLocal(options Options3D, a, b *model3d.Mesh, kind meshBooleanKind) (*model3d.Mesh, error) {
	// The normal tolerance absorbs projection roundoff and is the stable choice
	// for ordinary meshes. In a near-tangent arrangement it can instead erase a
	// real microscopic surface cell. Retry topology failures with a tolerance
	// close to machine precision so that cell remains explicit.
	var lastErr error
	for _, relativeTolerance := range []float64{
		arithmeticToleranceRelative, fallbackToleranceRelative,
	} {
		result, err := booleanMeshPairLocalTolerance(options, a, b, kind, relativeTolerance)
		if err == nil {
			return result, nil
		}
		lastErr = err
		var topologyErr *TopologyError
		if !errors.As(err, &topologyErr) {
			return nil, err
		}
	}
	return nil, lastErr
}

func booleanMeshPairLocalTolerance(options Options3D, a, b *model3d.Mesh,
	kind meshBooleanKind, relativeTolerance float64) (*model3d.Mesh, error) {
	if err := checkComplexity("triangles in one input mesh", a.NumTriangles(), options.MaxInputTriangles); err != nil {
		return nil, err
	}
	if err := checkComplexity("triangles in one input mesh", b.NumTriangles(), options.MaxInputTriangles); err != nil {
		return nil, err
	}
	trianglesA, trianglesB := sortedTriangles(a), sortedTriangles(b)
	indexA, indexB := newTriangleIndex(trianglesA), newTriangleIndex(trianglesB)
	classifierA := newExactMeshClassifier(trianglesA, indexA)
	classifierB := newExactMeshClassifier(trianglesB, indexB)
	scale := meshScale([]*model3d.Mesh{a, b})
	tol := math.Max(scale*relativeTolerance, math.SmallestNonzeroFloat64*1024)
	candidatePairs := 0
	relationsA, relationsB, err := buildTriangleRelations(options, trianglesA, indexB, tol, &candidatePairs)
	if err != nil {
		return nil, err
	}
	var fragments []*polygon
	newFragments, err := splitAndClassifyMesh(
		options, trianglesA, relationsA, classifierA, classifierB, kind, tol, true)
	if err != nil {
		return nil, err
	}
	fragments = append(fragments, newFragments...)
	if err := checkComplexity("surface fragments", len(fragments), options.MaxTotalFragments); err != nil {
		return nil, err
	}
	newFragments, err = splitAndClassifyMesh(
		options, trianglesB, relationsB, classifierA, classifierB, kind, tol, false)
	if err != nil {
		return nil, err
	}
	fragments = append(fragments, newFragments...)
	if err := checkComplexity("surface fragments", len(fragments), options.MaxTotalFragments); err != nil {
		return nil, err
	}
	return polygonsMesh(options, fragments, tol)
}

type triangleRelation struct {
	cuts   []model3d.Segment
	nearby []*model3d.Triangle
}

func buildTriangleRelations(options Options3D, trianglesA []*model3d.Triangle,
	indexB *triangleIndex, tol float64, candidatePairs *int,
) (map[*model3d.Triangle]*triangleRelation, map[*model3d.Triangle]*triangleRelation, error) {
	resultA := map[*model3d.Triangle]*triangleRelation{}
	resultB := map[*model3d.Triangle]*triangleRelation{}
	intersector := newExactTriangleIntersector()
	var nearby []*model3d.Triangle
	for _, triangleA := range trianglesA {
		nearby = nearby[:0]
		indexB.query(triangleA.Min(), triangleA.Max(), &nearby)
		sort.Slice(nearby, func(i, j int) bool {
			keyI, _ := makeTriangleKey(*nearby[i])
			keyJ, _ := makeTriangleKey(*nearby[j])
			return triangleKeyLess(keyI, keyJ)
		})
		*candidatePairs += len(nearby)
		if err := checkComplexity("triangle candidate pairs", *candidatePairs,
			options.MaxTriangleCandidatePairs); err != nil {
			return nil, nil, err
		}
		relationA := &triangleRelation{}
		resultA[triangleA] = relationA
		for _, triangleB := range nearby {
			relationB := resultB[triangleB]
			if relationB == nil {
				relationB = &triangleRelation{}
				resultB[triangleB] = relationB
			}
			normalA, normalB := triangleA.Normal(), triangleB.Normal()
			if math.Abs(normalA.Dot(normalB)) >= 1-1e-12 &&
				math.Abs(normalA.Dot(triangleB[0].Sub(triangleA[0]))) <= tol {
				relationA.nearby = append(relationA.nearby, triangleB)
				relationB.nearby = append(relationB.nearby, triangleA)
			}
			cuts, err := intersector.intersection(triangleA, triangleB)
			if err != nil {
				return nil, nil, err
			}
			for _, cut := range cuts {
				relationA.cuts = append(relationA.cuts, cut)
				relationB.cuts = append(relationB.cuts, cut)
			}
		}
	}
	return resultA, resultB, nil
}

func splitAndClassifyMesh(options Options3D, triangles []*model3d.Triangle,
	relations map[*model3d.Triangle]*triangleRelation, classifierA, classifierB *exactMeshClassifier,
	kind meshBooleanKind, tol float64, sourceA bool) ([]*polygon, error) {
	var result []*polygon
	for _, tri := range triangles {
		if tri.Area() <= tol*tol {
			continue
		}
		normal := tri.Normal()
		u, v := planeBasis(normal)
		origin := tri[0]
		relation := relations[tri]
		if relation == nil {
			relation = &triangleRelation{}
		}
		project := func(point model3d.Coord3D) model2d.Coord {
			delta := point.Sub(origin)
			return model2d.XY(u.Dot(delta), v.Dot(delta))
		}
		type projectedNode struct {
			projected model2d.Coord
			original  model3d.Coord3D
		}
		nodes := make([]projectedNode, 0, 3+2*len(relation.cuts))
		for _, point := range tri {
			nodes = append(nodes, projectedNode{projected: project(point), original: point})
		}
		for _, cut := range relation.cuts {
			for _, point := range cut {
				nodes = append(nodes, projectedNode{projected: project(point), original: point})
			}
		}
		lift := func(point model2d.Coord) model3d.Coord3D {
			for _, node := range nodes {
				if point.Dist(node.projected) <= tol {
					return node.original
				}
			}
			return origin.Add(u.Scale(point.X)).Add(v.Scale(point.Y))
		}
		pieces := [][]model2d.Coord{{project(tri[0]), project(tri[1]), project(tri[2])}}
		var splitLines [][2]model2d.Coord
		for _, collision := range relation.cuts {
			splitLines = append(splitLines, [2]model2d.Coord{project(collision[0]), project(collision[1])})
		}
		projectedTri := []model2d.Coord{project(tri[0]), project(tri[1]), project(tri[2])}
		hasCoplanarOverlap := false
		for _, candidate := range relation.nearby {
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
			hasCoplanarOverlap = true
			for i, point := range projectedCandidate {
				splitLines = append(splitLines, [2]model2d.Coord{point, projectedCandidate[(i+1)%3]})
			}
		}
		if hasCoplanarOverlap {
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
						if err := checkComplexity("fragments for one triangle", len(next), options.MaxFragmentsPerTriangle); err != nil {
							return nil, err
						}
					}
					if len(backPiece) >= 3 {
						next = append(next, backPiece)
						if err := checkComplexity("fragments for one triangle", len(next), options.MaxFragmentsPerTriangle); err != nil {
							return nil, err
						}
					}
				}
				pieces = next
			}
		} else {
			if err := checkComplexity("fragments for one triangle", 1+2*len(splitLines),
				options.MaxFragmentsPerTriangle); err != nil {
				return nil, err
			}
			var err error
			pieces, err = constrainedTriangleFaces(projectedTri, splitLines, tol)
			if err != nil {
				return nil, err
			}
			if err := checkComplexity("fragments for one triangle", len(pieces), options.MaxFragmentsPerTriangle); err != nil {
				return nil, err
			}
		}
		for _, piece := range pieces {
			center2d := model2d.Coord{}
			vertices := make([]model3d.Coord3D, len(piece))
			for i, point := range piece {
				center2d = center2d.Add(point)
				vertices[i] = lift(point)
			}
			center2d = center2d.Scale(1 / float64(len(piece)))
			center := lift(center2d)
			var coplanarNormal *model3d.Coord3D
			for _, candidate := range relation.nearby {
				candidateNormal := candidate.Normal()
				if math.Abs(normal.Dot(candidateNormal)) < 1-1e-12 ||
					math.Abs(normal.Dot(candidate[0].Sub(tri[0]))) > tol {
					continue
				}
				projectedCandidate := []model2d.Coord{
					project(candidate[0]), project(candidate[1]), project(candidate[2]),
				}
				if pointInConvexPolygon2D(center2d, projectedCandidate, tol) {
					coplanarNormal = &candidateNormal
					break
				}
			}
			if coplanarNormal != nil {
				plus, minus, err := evalCoplanarSourceSides(kind, sourceA,
					normal.Dot(*coplanarNormal) >= 0)
				if err != nil {
					return nil, err
				}
				if plus == minus {
					continue
				}
				poly := newPolygon(vertices)
				if poly == nil {
					continue
				}
				if plus && !minus {
					poly.flip()
				}
				poly.coplanar = true
				result = append(result, poly)
				if err := checkComplexity("surface fragments", len(result), options.MaxTotalFragments); err != nil {
					return nil, err
				}
				continue
			}
			var insideOther bool
			var err error
			if sourceA {
				insideOther, err = classifierB.contains(center)
			} else {
				insideOther, err = classifierA.contains(center)
			}
			if err != nil {
				return nil, err
			}
			keep, flip, err := selectSourceSurface(kind, sourceA, insideOther)
			if err != nil {
				return nil, err
			}
			if !keep {
				continue
			}
			poly := newPolygon(vertices)
			if poly == nil {
				continue
			}
			if flip {
				poly.flip()
			}
			result = append(result, poly)
			if err := checkComplexity("surface fragments", len(result), options.MaxTotalFragments); err != nil {
				return nil, err
			}
		}
	}
	return result, nil
}

type indexedTriangle2D [3]int

func constrainedTriangleFaces(triangle []model2d.Coord, cuts [][2]model2d.Coord,
	tol float64) ([][]model2d.Coord, error) {
	points := append([]model2d.Coord(nil), triangle...)
	pointIndices := map[model2d.Coord]int{}
	for i, point := range points {
		pointIndices[point] = i
	}
	addPoint := func(point model2d.Coord) int {
		if index, ok := pointIndices[point]; ok {
			return index
		}
		// Projection introduces a final float64 rounding after the exact 3D
		// intersection. Canonicalize consistently with the output mesh so one
		// geometric node cannot become a microscopic, unenforceable constraint.
		for index, existing := range points {
			if existing.Dist(point) <= tol {
				return index
			}
		}
		index := len(points)
		points = append(points, point)
		pointIndices[point] = index
		return index
	}
	type constraint2D struct{ a, b int }
	constraints := []constraint2D{{0, 1}, {1, 2}, {2, 0}}
	for _, cut := range cuts {
		if cut[0].Dist(cut[1]) > tol {
			constraints = append(constraints, constraint2D{addPoint(cut[0]), addPoint(cut[1])})
		}
	}

	// Split every constraint at all input nodes on its interior. This gives the
	// edge-flip stage a proper PSLG with no vertex lying inside a constraint.
	var atomic []constraint2D
	for _, constraint := range constraints {
		start, end := points[constraint.a], points[constraint.b]
		delta := end.Sub(start)
		lengthSquared := delta.NormSquared()
		if lengthSquared == 0 {
			continue
		}
		type parameterizedPoint struct {
			index     int
			parameter float64
		}
		onSegment := []parameterizedPoint{{constraint.a, 0}, {constraint.b, 1}}
		for index, point := range points {
			if index == constraint.a || index == constraint.b {
				continue
			}
			parameter := point.Sub(start).Dot(delta) / lengthSquared
			if parameter > 0 && parameter < 1 &&
				start.Add(delta.Scale(parameter)).Dist(point) <= tol {
				onSegment = append(onSegment, parameterizedPoint{index, parameter})
			}
		}
		sort.Slice(onSegment, func(i, j int) bool {
			if onSegment[i].parameter != onSegment[j].parameter {
				return onSegment[i].parameter < onSegment[j].parameter
			}
			return onSegment[i].index < onSegment[j].index
		})
		for i := 0; i+1 < len(onSegment); i++ {
			if onSegment[i].index != onSegment[i+1].index {
				atomic = append(atomic, constraint2D{onSegment[i].index, onSegment[i+1].index})
			}
		}
	}

	triangles := []indexedTriangle2D{orientedTriangle2D(indexedTriangle2D{0, 1, 2}, points)}
	for pointIndex := 3; pointIndex < len(points); pointIndex++ {
		point := points[pointIndex]
		type edgeKey struct{ a, b int }
		edgeTriangles := map[edgeKey][]int{}
		for triangleIndex, tri := range triangles {
			for edge := 0; edge < 3; edge++ {
				a, b := tri[edge], tri[(edge+1)%3]
				if a > b {
					a, b = b, a
				}
				edgeTriangles[edgeKey{a, b}] = append(edgeTriangles[edgeKey{a, b}], triangleIndex)
			}
		}
		splitEdge := edgeKey{-1, -1}
		bestDistance := math.Inf(1)
		for edge := range edgeTriangles {
			start, end := points[edge.a], points[edge.b]
			delta := end.Sub(start)
			lengthSquared := delta.NormSquared()
			if lengthSquared == 0 {
				continue
			}
			parameter := point.Sub(start).Dot(delta) / lengthSquared
			if parameter <= 0 || parameter >= 1 {
				continue
			}
			distance := start.Add(delta.Scale(parameter)).Dist(point)
			if distance <= tol && distance < bestDistance {
				splitEdge, bestDistance = edge, distance
			}
		}
		if splitEdge.a >= 0 {
			incidentSet := map[int]bool{}
			for _, index := range edgeTriangles[splitEdge] {
				incidentSet[index] = true
			}
			var next []indexedTriangle2D
			for index, tri := range triangles {
				if !incidentSet[index] {
					next = append(next, tri)
					continue
				}
				other := triangleOtherVertex2D(tri, splitEdge.a, splitEdge.b)
				next = appendIndexedTriangle2D(next,
					indexedTriangle2D{splitEdge.a, pointIndex, other}, points, tol)
				next = appendIndexedTriangle2D(next,
					indexedTriangle2D{pointIndex, splitEdge.b, other}, points, tol)
			}
			triangles = next
			continue
		}
		containing := -1
		for index, tri := range triangles {
			if pointInIndexedTriangle2D(point, tri, points, tol) {
				containing = index
				break
			}
		}
		if containing < 0 {
			return nil, &TopologyError{Problem: "intersection node outside source triangle"}
		}
		old := triangles[containing]
		triangles = append(triangles[:containing], triangles[containing+1:]...)
		for edge := 0; edge < 3; edge++ {
			triangles = appendIndexedTriangle2D(triangles,
				indexedTriangle2D{old[edge], old[(edge+1)%3], pointIndex}, points, tol)
		}
	}

	protected := map[[2]int]bool{}
	for _, constraint := range atomic {
		target := orderedEdge2D(constraint.a, constraint.b)
		inserted := false
		for attempts := 0; attempts <= 3*len(triangles)+1; attempts++ {
			edgeTriangles := indexedTriangleEdges2D(triangles)
			if _, ok := edgeTriangles[target]; ok {
				inserted = true
				break
			}
			intersectionFn := segmentsProperlyIntersect2D
			var crossings [][2]int
			for edge, incident := range edgeTriangles {
				if len(incident) != 2 || protected[edge] {
					continue
				}
				if intersectionFn(points[target[0]], points[target[1]],
					points[edge[0]], points[edge[1]], tol) {
					crossings = append(crossings, edge)
				}
			}
			if len(crossings) == 0 {
				// A valid constraint can cross an existing edge arbitrarily close
				// to one of its endpoints. The tolerance-aware test deliberately
				// ignores such crossings, so retry with exact signs before falling
				// back to cavity reconstruction.
				intersectionFn = segmentsProperlyIntersect2DExact
				for edge, incident := range edgeTriangles {
					if len(incident) != 2 || protected[edge] {
						continue
					}
					if intersectionFn(points[target[0]], points[target[1]],
						points[edge[0]], points[edge[1]], tol) {
						crossings = append(crossings, edge)
					}
				}
			}
			sort.Slice(crossings, func(i, j int) bool {
				if crossings[i][0] != crossings[j][0] {
					return crossings[i][0] < crossings[j][0]
				}
				return crossings[i][1] < crossings[j][1]
			})
			flipped := false
			for _, crossing := range crossings {
				incident := edgeTriangles[crossing]
				first, second := triangles[incident[0]], triangles[incident[1]]
				oppositeA := triangleOtherVertex2D(first, crossing[0], crossing[1])
				oppositeB := triangleOtherVertex2D(second, crossing[0], crossing[1])
				if !intersectionFn(points[crossing[0]], points[crossing[1]],
					points[oppositeA], points[oppositeB], tol) {
					continue
				}
				if intersectionFn(points[target[0]], points[target[1]],
					points[oppositeA], points[oppositeB], tol) {
					continue
				}
				replacementA := orientedTriangle2D(
					indexedTriangle2D{oppositeA, oppositeB, crossing[0]}, points)
				replacementB := orientedTriangle2D(
					indexedTriangle2D{oppositeB, oppositeA, crossing[1]}, points)
				triangles[incident[0]], triangles[incident[1]] = replacementA, replacementB
				flipped = true
				break
			}
			if !flipped {
				break
			}
		}
		if !inserted {
			var err error
			triangles, err = insertConstraintCavity2D(triangles, target, protected, points, tol)
			if err != nil {
				return nil, err
			}
		}
		protected[target] = true
	}

	result := make([][]model2d.Coord, 0, len(triangles))
	for _, triangle := range triangles {
		result = append(result, []model2d.Coord{
			points[triangle[0]], points[triangle[1]], points[triangle[2]],
		})
	}
	return result, nil
}

func insertConstraintCavity2D(triangles []indexedTriangle2D, target [2]int,
	protected map[[2]int]bool, points []model2d.Coord, tol float64,
) ([]indexedTriangle2D, error) {
	edges := indexedTriangleEdges2D(triangles)
	removed := map[int]bool{}
	for _, intersectionFn := range []func(model2d.Coord, model2d.Coord,
		model2d.Coord, model2d.Coord, float64) bool{
		segmentsProperlyIntersect2D, segmentsProperlyIntersect2DExact,
	} {
		for edge, incident := range edges {
			if !intersectionFn(points[target[0]], points[target[1]],
				points[edge[0]], points[edge[1]], tol) {
				continue
			}
			if protected[edge] {
				return nil, &TopologyError{Problem: "intersecting triangle constraints"}
			}
			for _, triangleIndex := range incident {
				removed[triangleIndex] = true
			}
		}
		if len(removed) != 0 {
			break
		}
	}
	if len(removed) == 0 {
		return nil, &TopologyError{Problem: "could not enforce triangle intersection edge"}
	}

	boundaryCounts := map[[2]int]int{}
	for triangleIndex := range removed {
		triangle := triangles[triangleIndex]
		for edge := 0; edge < 3; edge++ {
			boundaryCounts[orderedEdge2D(triangle[edge], triangle[(edge+1)%3])]++
		}
	}
	adjacent := map[int][]int{}
	for edge, count := range boundaryCounts {
		if count == 1 {
			adjacent[edge[0]] = append(adjacent[edge[0]], edge[1])
			adjacent[edge[1]] = append(adjacent[edge[1]], edge[0])
		}
	}
	for vertex := range adjacent {
		sort.Ints(adjacent[vertex])
		if len(adjacent[vertex]) != 2 {
			return nil, &TopologyError{Problem: "non-simple triangle constraint cavity"}
		}
	}
	if len(adjacent[target[0]]) != 2 || len(adjacent[target[1]]) != 2 {
		return nil, &TopologyError{Problem: "triangle constraint endpoint outside cavity boundary"}
	}
	walkBoundary := func(first int) ([]int, error) {
		path := []int{target[0]}
		previous, current := target[0], first
		for len(path) <= len(adjacent)+1 {
			path = append(path, current)
			if current == target[1] {
				return path, nil
			}
			neighbors := adjacent[current]
			if len(neighbors) != 2 {
				break
			}
			next := neighbors[0]
			if next == previous {
				next = neighbors[1]
			}
			previous, current = current, next
		}
		return nil, &TopologyError{Problem: "open triangle constraint cavity boundary"}
	}
	firstPath, err := walkBoundary(adjacent[target[0]][0])
	if err != nil {
		return nil, err
	}
	secondPath, err := walkBoundary(adjacent[target[0]][1])
	if err != nil {
		return nil, err
	}

	result := make([]indexedTriangle2D, 0, len(triangles)+2)
	for triangleIndex, triangle := range triangles {
		if !removed[triangleIndex] {
			result = append(result, triangle)
		}
	}
	for _, polygon := range [][]int{firstPath, secondPath} {
		triangulated, err := triangulateIndexedPolygon2D(polygon, points, tol)
		if err != nil {
			return nil, err
		}
		result = append(result, triangulated...)
	}
	resultEdges := indexedTriangleEdges2D(result)
	if _, ok := resultEdges[target]; !ok {
		return nil, &TopologyError{Problem: "could not enforce triangle intersection edge"}
	}
	for edge := range protected {
		if _, ok := resultEdges[edge]; !ok {
			return nil, &TopologyError{Problem: "triangle constraint cavity removed a protected edge"}
		}
	}
	return result, nil
}

func triangulateIndexedPolygon2D(polygon []int, points []model2d.Coord,
	tol float64,
) ([]indexedTriangle2D, error) {
	polygon = append([]int(nil), polygon...)
	if len(polygon) < 3 {
		return nil, &TopologyError{Problem: "triangle constraint cavity has fewer than three vertices"}
	}
	signedAreaTwice := 0.0
	for i, vertex := range polygon {
		signedAreaTwice += cross2D(points[vertex], points[polygon[(i+1)%len(polygon)]])
	}
	orientation := 1.0
	if signedAreaTwice < 0 {
		orientation = -1
	}
	var result []indexedTriangle2D
	for len(polygon) > 3 {
		ear := -1
		for i, current := range polygon {
			previous := polygon[(i+len(polygon)-1)%len(polygon)]
			next := polygon[(i+1)%len(polygon)]
			areaTwice := cross2D(points[current].Sub(points[previous]),
				points[next].Sub(points[current])) * orientation
			if areaTwice <= 2*tol*tol {
				continue
			}
			candidate := orientedTriangle2D(indexedTriangle2D{previous, current, next}, points)
			containsVertex := false
			for _, vertex := range polygon {
				if vertex != previous && vertex != current && vertex != next &&
					pointInIndexedTriangle2D(points[vertex], candidate, points, tol) {
					containsVertex = true
					break
				}
			}
			if !containsVertex {
				ear = i
				result = append(result, candidate)
				break
			}
		}
		if ear < 0 {
			return nil, &TopologyError{Problem: "could not triangulate triangle constraint cavity"}
		}
		polygon = append(polygon[:ear], polygon[ear+1:]...)
	}
	result = appendIndexedTriangle2D(result,
		indexedTriangle2D{polygon[0], polygon[1], polygon[2]}, points, tol)
	if len(result) == 0 {
		return nil, &TopologyError{Problem: "degenerate triangle constraint cavity"}
	}
	return result, nil
}

func orderedEdge2D(a, b int) [2]int {
	if a > b {
		a, b = b, a
	}
	return [2]int{a, b}
}

func indexedTriangleEdges2D(triangles []indexedTriangle2D) map[[2]int][]int {
	result := map[[2]int][]int{}
	for triangleIndex, triangle := range triangles {
		for edge := 0; edge < 3; edge++ {
			key := orderedEdge2D(triangle[edge], triangle[(edge+1)%3])
			result[key] = append(result[key], triangleIndex)
		}
	}
	return result
}

func orientedTriangle2D(triangle indexedTriangle2D, points []model2d.Coord) indexedTriangle2D {
	if cross2D(points[triangle[1]].Sub(points[triangle[0]]),
		points[triangle[2]].Sub(points[triangle[0]])) < 0 {
		triangle[1], triangle[2] = triangle[2], triangle[1]
	}
	return triangle
}

func appendIndexedTriangle2D(triangles []indexedTriangle2D, triangle indexedTriangle2D,
	points []model2d.Coord, tol float64) []indexedTriangle2D {
	triangle = orientedTriangle2D(triangle, points)
	areaTwice := cross2D(points[triangle[1]].Sub(points[triangle[0]]),
		points[triangle[2]].Sub(points[triangle[0]]))
	if areaTwice > 2*tol*tol {
		return append(triangles, triangle)
	}
	return triangles
}

func triangleOtherVertex2D(triangle indexedTriangle2D, a, b int) int {
	for _, vertex := range triangle {
		if vertex != a && vertex != b {
			return vertex
		}
	}
	return -1
}

func pointInIndexedTriangle2D(point model2d.Coord, triangle indexedTriangle2D,
	points []model2d.Coord, tol float64) bool {
	for edge := 0; edge < 3; edge++ {
		start, end := points[triangle[edge]], points[triangle[(edge+1)%3]]
		direction := end.Sub(start)
		if cross2D(direction, point.Sub(start)) < -tol*math.Max(direction.Norm(), tol) {
			return false
		}
	}
	return true
}

func segmentsProperlyIntersect2D(a, b, c, d model2d.Coord, tol float64) bool {
	ab, cd := b.Sub(a), d.Sub(c)
	toleranceAB := tol * math.Max(ab.Norm(), tol)
	toleranceCD := tol * math.Max(cd.Norm(), tol)
	sideC, sideD := cross2D(ab, c.Sub(a)), cross2D(ab, d.Sub(a))
	sideA, sideB := cross2D(cd, a.Sub(c)), cross2D(cd, b.Sub(c))
	return ((sideC > toleranceAB && sideD < -toleranceAB) ||
		(sideC < -toleranceAB && sideD > toleranceAB)) &&
		((sideA > toleranceCD && sideB < -toleranceCD) ||
			(sideA < -toleranceCD && sideB > toleranceCD))
}

func segmentsProperlyIntersect2DExact(a, b, c, d model2d.Coord, _ float64) bool {
	sideC, sideD := exactOrient2DSign(a, b, c), exactOrient2DSign(a, b, d)
	sideA, sideB := exactOrient2DSign(c, d, a), exactOrient2DSign(c, d, b)
	return sideC != 0 && sideD != 0 && sideC != sideD &&
		sideA != 0 && sideB != 0 && sideA != sideB
}

func exactOrient2DSign(a, b, c model2d.Coord) int {
	ax, ay := new(big.Rat).SetFloat64(a.X), new(big.Rat).SetFloat64(a.Y)
	abx := new(big.Rat).Sub(new(big.Rat).SetFloat64(b.X), ax)
	aby := new(big.Rat).Sub(new(big.Rat).SetFloat64(b.Y), ay)
	acx := new(big.Rat).Sub(new(big.Rat).SetFloat64(c.X), ax)
	acy := new(big.Rat).Sub(new(big.Rat).SetFloat64(c.Y), ay)
	return new(big.Rat).Sub(
		new(big.Rat).Mul(abx, acy),
		new(big.Rat).Mul(aby, acx),
	).Sign()
}

func pointInConvexPolygon2D(point model2d.Coord, polygon []model2d.Coord, tol float64) bool {
	sign := 0
	for i, start := range polygon {
		edge := polygon[(i+1)%len(polygon)].Sub(start)
		side := cross2D(edge, point.Sub(start))
		determinantTolerance := tol * math.Max(edge.Norm(), tol)
		if math.Abs(side) <= determinantTolerance {
			continue
		}
		newSign := 1
		if side < 0 {
			newSign = -1
		}
		if sign != 0 && sign != newSign {
			return false
		}
		sign = newSign
	}
	return true
}

func evalCoplanarSourceSides(kind meshBooleanKind, sourceA, sameNormal bool) (bool, bool, error) {
	var plusA, minusA, plusB, minusB bool
	if sourceA {
		minusA = true
		if sameNormal {
			minusB = true
		} else {
			plusB = true
		}
	} else {
		minusB = true
		if sameNormal {
			minusA = true
		} else {
			plusA = true
		}
	}
	plus, err := evalMeshBoolean(kind, plusA, plusB)
	if err != nil {
		return false, false, err
	}
	minus, err := evalMeshBoolean(kind, minusA, minusB)
	return plus, minus, err
}

func selectSourceSurface(kind meshBooleanKind, sourceA, insideOther bool) (bool, bool, error) {
	switch kind {
	case meshUnion:
		return !insideOther, false, nil
	case meshIntersection:
		return insideOther, false, nil
	case meshDifference:
		if sourceA {
			return !insideOther, false, nil
		}
		return insideOther, true, nil
	default:
		return false, false, fmt.Errorf("meshbool: unknown boolean operation %d", kind)
	}
}

type keyedTriangle3D struct {
	triangle *model3d.Triangle
	key      triangleKey
}

type keyedTriangleSorter3D []keyedTriangle3D

func (k keyedTriangleSorter3D) Len() int           { return len(k) }
func (k keyedTriangleSorter3D) Less(i, j int) bool { return triangleKeyLess(k[i].key, k[j].key) }
func (k keyedTriangleSorter3D) Swap(i, j int)      { k[i], k[j] = k[j], k[i] }

func sortedTriangles(mesh *model3d.Mesh) []*model3d.Triangle {
	triangles := mesh.TriangleSlice()
	keyed := make([]keyedTriangle3D, len(triangles))
	for i, triangle := range triangles {
		keyed[i].triangle = triangle
		keyed[i].key, _ = makeTriangleKey(*triangle)
	}
	sort.Sort(keyedTriangleSorter3D(keyed))
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

func evalMeshBoolean(kind meshBooleanKind, inA, inB bool) (bool, error) {
	switch kind {
	case meshUnion:
		return inA || inB, nil
	case meshIntersection:
		return inA && inB, nil
	case meshDifference:
		return inA && !inB, nil
	default:
		return false, fmt.Errorf("meshbool: unknown boolean operation %d", kind)
	}
}

type polygon struct {
	vertices []model3d.Coord3D
	plane    plane
	coplanar bool
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

func polygonsMesh(options Options3D, polygons []*polygon, tol float64) (*model3d.Mesh, error) {
	var raw []model3d.Triangle
	var coplanar []*polygon
	for _, polygon := range polygons {
		if polygon.coplanar {
			coplanar = append(coplanar, polygon)
			continue
		}
		for i := 1; i+1 < len(polygon.vertices); i++ {
			triangle := model3d.Triangle{
				polygon.vertices[0], polygon.vertices[i], polygon.vertices[i+1],
			}
			if triangle.Area() > tol*tol {
				raw = append(raw, triangle)
			}
		}
		if err := checkComplexity("triangulated surface cells", len(raw),
			options.MaxOutputTriangles); err != nil {
			return nil, err
		}
	}
	if len(coplanar) != 0 {
		triangles, err := triangulateCoplanarPolygons(options, coplanar, tol)
		if err != nil {
			return nil, err
		}
		raw = append(raw, triangles...)
	}
	return finalizeTriangles(options, raw, tol)
}

func triangulateCoplanarPolygons(options Options3D, polygons []*polygon,
	tol float64) ([]model3d.Triangle, error) {
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
		positive, err := bool2d.Union(options.PlanarOptions.internal(), group.positive...)
		if err != nil {
			return nil, convertPlanarError(err)
		}
		negative, err := bool2d.Union(options.PlanarOptions.internal(), group.negative...)
		if err != nil {
			return nil, convertPlanarError(err)
		}
		positiveOnly, err := bool2d.Difference(options.PlanarOptions.internal(), positive, negative)
		if err != nil {
			return nil, convertPlanarError(err)
		}
		negativeOnly, err := bool2d.Difference(options.PlanarOptions.internal(), negative, positive)
		if err != nil {
			return nil, convertPlanarError(err)
		}
		if positiveOnly.NumSegments() != 0 {
			triangles, err := liftPlanarMesh(positiveOnly, group, true)
			if err != nil {
				return nil, err
			}
			raw = append(raw, triangles...)
			if err := checkComplexity("triangulated coplanar surfaces", len(raw), options.MaxOutputTriangles); err != nil {
				return nil, err
			}
		}
		if negativeOnly.NumSegments() != 0 {
			triangles, err := liftPlanarMesh(negativeOnly, group, false)
			if err != nil {
				return nil, err
			}
			raw = append(raw, triangles...)
			if err := checkComplexity("triangulated coplanar surfaces", len(raw), options.MaxOutputTriangles); err != nil {
				return nil, err
			}
		}
	}
	return raw, nil
}

func convertPlanarError(err error) error {
	switch failure := err.(type) {
	case *bool2d.ComplexityError:
		return &ComplexityError{Stage: "planar " + failure.Stage, Limit: failure.Limit}
	case *bool2d.TopologyError:
		return &TopologyError{Problem: "planar " + failure.Problem, Count: failure.Count}
	default:
		return fmt.Errorf("meshbool: planar operation: %w", err)
	}
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

func liftPlanarMesh(mesh *model2d.Mesh, group *coplanarGroup, positive bool) ([]model3d.Triangle, error) {
	u, v := planeBasis(group.normal)
	lift := func(c model2d.Coord) model3d.Coord3D {
		return u.Scale(c.X).Add(v.Scale(c.Y)).Add(group.normal.Scale(group.w))
	}
	triangles2d, err := triangulatePlanarMesh(mesh)
	if err != nil {
		return nil, err
	}
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
	return result, nil
}

func triangulatePlanarMesh(mesh *model2d.Mesh) ([][3]model2d.Coord, error) {
	if !mesh.Manifold() {
		return nil, &TopologyError{Problem: "non-manifold planar triangulation boundary"}
	}
	return model2d.TriangulateMesh(mesh), nil
}

func finalizeTriangles(options Options3D, raw []model3d.Triangle, tol float64) (*model3d.Mesh, error) {
	if err := checkComplexity("output triangles", len(raw), options.MaxOutputTriangles); err != nil {
		return nil, err
	}
	canon := newCoordCanonicalizer(tol)
	for i := range raw {
		for j := range raw[i] {
			raw[i][j] = canon.add(raw[i][j])
		}
	}
	index := newPointIndex(canon.points)
	conformed := make([]model3d.Triangle, 0, len(raw))
	var pointCandidates []model3d.Coord3D
	for _, tri := range raw {
		conformed = appendConformedTriangle(conformed, tri, index, tol, &pointCandidates)
		if err := checkComplexity("conforming output triangles", len(conformed), options.MaxOutputTriangles); err != nil {
			return nil, err
		}
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
	result := model3d.NewMesh()
	for _, balance := range balances {
		if balance.value == 0 {
			continue
		}
		tri := balance.positive
		if balance.value < 0 {
			tri = balance.negative
		}
		triCopy := tri
		result.Add(&triCopy)
	}
	result, err := separateContactEdges(options, result, tol)
	if err != nil {
		return nil, err
	}
	result = separateContactComponents(result, tol)
	result = separateSingularVertexFans(result, tol)
	return validateResultTopology(result)
}

func validateResultTopology(mesh *model3d.Mesh) (*model3d.Mesh, error) {
	if mesh.NumTriangles() == 0 {
		return mesh, nil
	}
	if mesh.NeedsRepair() {
		edgeCounts := map[model3d.Segment]int{}
		mesh.Iterate(func(triangle *model3d.Triangle) {
			for i := range triangle {
				edgeCounts[model3d.NewSegment(triangle[i], triangle[(i+1)%3])]++
			}
		})
		badEdges := 0
		for _, count := range edgeCounts {
			if count != 2 {
				badEdges++
			}
		}
		return nil, &TopologyError{Problem: "edges do not each have two incident triangles", Count: badEdges}
	}
	if singular := mesh.SingularVertices(); len(singular) != 0 {
		return nil, &TopologyError{Problem: "singular vertices", Count: len(singular)}
	}
	if !mesh.Orientable() {
		return nil, &TopologyError{Problem: "non-orientable surface"}
	}
	intersections, err := exactSelfIntersections(mesh)
	if err != nil {
		return nil, err
	}
	if intersections != 0 {
		return nil, &TopologyError{Problem: "self-intersections", Count: intersections}
	}
	return mesh, nil
}

func exactSelfIntersections(mesh *model3d.Mesh) (int, error) {
	// Use the same cached-bounds index as corefinement. Building model3d's
	// collider here duplicates a large BVH at the operation's peak memory use,
	// which is especially costly under WebAssembly. Approximate collisions are
	// still checked in both directions, then confirmed with exact predicates.
	triangles := sortedTriangles(mesh)
	spatialIndex := newTriangleIndex(triangles)
	intersector := newExactTriangleIntersector()
	count := 0
	var candidates []indexedTriangleCandidate3D
	for index, triangle := range triangles {
		candidates = candidates[:0]
		spatialIndex.queryIndexed(triangle.Min(), triangle.Max(), &candidates)
		for _, candidate := range candidates {
			if candidate.index <= index || trianglesShareEdge(triangle, candidate.triangle) {
				continue
			}
			if len(triangle.TriangleCollisions(candidate.triangle)) == 0 &&
				len(candidate.triangle.TriangleCollisions(triangle)) == 0 {
				continue
			}
			intersection, err := intersector.intersection(triangle, candidate.triangle)
			if err != nil {
				return 0, err
			}
			if len(intersection) != 0 {
				count++
			}
		}
	}
	return count, nil
}

func trianglesShareEdge(a, b *model3d.Triangle) bool {
	common := 0
	for _, pointA := range a {
		for _, pointB := range b {
			if pointA == pointB {
				common++
			}
		}
	}
	return common >= 2
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
func separateContactEdges(options Options3D, mesh *model3d.Mesh, tol float64) (*model3d.Mesh, error) {
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
		return mesh, nil
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
		if err := checkComplexity("triangles at one contact edge", len(incident), options.MaxContactEdgeTriangles); err != nil {
			return nil, err
		}
		groups, directions, err := pairContactEdgeTriangles(edge, incident)
		if err != nil {
			return nil, err
		}
		midpoint := edge.Mid()
		for groupIndex, group := range groups {
			shift := math.Min(tol*4, edge.Length()*0.25)
			for _, triangle := range group {
				other, err := contactEdgeOtherVertex(edge, triangle)
				if err != nil {
					return nil, err
				}
				shift = math.Min(shift, midpoint.Dist(other)*0.25)
			}
			newMidpoint := midpoint.Add(directions[groupIndex].Scale(shift))
			for _, triangle := range group {
				other, err := contactEdgeOtherVertex(edge, triangle)
				if err != nil {
					return nil, err
				}
				first := model3d.Triangle{other, edge[0], newMidpoint}
				second := model3d.Triangle{other, newMidpoint, edge[1]}
				shared := model3d.NewSegment(other, edge[0])
				firstOrientation, err := triangleSegmentOrientation(&first, shared)
				if err != nil {
					return nil, err
				}
				originalOrientation, err := triangleSegmentOrientation(triangle, shared)
				if err != nil {
					return nil, err
				}
				if firstOrientation != originalOrientation {
					first[0], first[1] = first[1], first[0]
					second[0], second[1] = second[1], second[0]
				}
				result.Remove(triangle)
				result.Add(&first)
				result.Add(&second)
			}
			if err := checkComplexity("edge-contact regularization triangles", result.NumTriangles(), options.MaxOutputTriangles); err != nil {
				return nil, err
			}
		}
	}
	return result, nil
}

func pairContactEdgeTriangles(edge model3d.Segment, triangles []*model3d.Triangle) ([][2]*model3d.Triangle, []model3d.Coord3D, error) {
	axis := edge[0].Sub(edge[1]).Normalize()
	basis1, basis2 := axis.OrthoBasis()
	midpoint := edge.Mid()
	angular := make([]contactEdgeTriangle, len(triangles))
	for i, triangle := range triangles {
		other, err := contactEdgeOtherVertex(edge, triangle)
		if err != nil {
			return nil, nil, err
		}
		triangleVector := other.Sub(midpoint).Normalize()
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
			firstOrientation, err := triangleSegmentOrientation(angular[i].triangle, edge)
			if err != nil {
				return nil, nil, err
			}
			secondOrientation, err := triangleSegmentOrientation(angular[j].triangle, edge)
			if err != nil {
				return nil, nil, err
			}
			if firstOrientation != secondOrientation {
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
	return groups, directions, nil
}

func contactEdgeOtherVertex(edge model3d.Segment, triangle *model3d.Triangle) (model3d.Coord3D, error) {
	for _, point := range triangle {
		if point != edge[0] && point != edge[1] {
			return point, nil
		}
	}
	return model3d.Coord3D{}, &TopologyError{Problem: "degenerate triangle on contact edge"}
}

func triangleSegmentOrientation(triangle *model3d.Triangle, segment model3d.Segment) (bool, error) {
	for i, point := range triangle {
		if point == segment[0] {
			return triangle[(i+2)%3] == segment[1], nil
		}
	}
	return false, &TopologyError{Problem: "segment is not in triangle"}
}

func separateContactComponents(mesh *model3d.Mesh, tol float64) *model3d.Mesh {
	triangles := sortedTriangles(mesh)
	if len(triangles) == 0 {
		return mesh
	}
	type incidentTriangles struct {
		first, second int
		count         int
	}
	edgeTriangles := map[model3d.Segment]incidentTriangles{}
	for i, tri := range triangles {
		for j := range tri {
			edge := model3d.NewSegment(tri[j], tri[(j+1)%3])
			incident := edgeTriangles[edge]
			if incident.count == 0 {
				incident.first = i
			} else if incident.count == 1 {
				incident.second = i
			}
			incident.count++
			edgeTriangles[edge] = incident
		}
	}
	adjacent := make([][3]int, len(triangles))
	adjacentCounts := make([]uint8, len(triangles))
	for _, incident := range edgeTriangles {
		if incident.count == 2 {
			adjacent[incident.first][adjacentCounts[incident.first]] = incident.second
			adjacentCounts[incident.first]++
			adjacent[incident.second][adjacentCounts[incident.second]] = incident.first
			adjacentCounts[incident.second]++
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
			for _, neighbor := range adjacent[index][:adjacentCounts[index]] {
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
	triangles   []boundedTriangle3D
	left, right *triangleIndex
}

type boundedTriangle3D struct {
	triangle *model3d.Triangle
	min, max model3d.Coord3D
	index    int
}

type boundedTriangleSorter3D struct {
	triangles []boundedTriangle3D
	axis      int
}

func (b boundedTriangleSorter3D) Len() int { return len(b.triangles) }
func (b boundedTriangleSorter3D) Less(i, j int) bool {
	centerI := coordAxis(b.triangles[i].min, b.axis) + coordAxis(b.triangles[i].max, b.axis)
	centerJ := coordAxis(b.triangles[j].min, b.axis) + coordAxis(b.triangles[j].max, b.axis)
	return centerI < centerJ
}
func (b boundedTriangleSorter3D) Swap(i, j int) {
	b.triangles[i], b.triangles[j] = b.triangles[j], b.triangles[i]
}

type indexedTriangleCandidate3D struct {
	triangle *model3d.Triangle
	index    int
}

func newTriangleIndex(triangles []*model3d.Triangle) *triangleIndex {
	if len(triangles) == 0 {
		return nil
	}
	bounded := make([]boundedTriangle3D, len(triangles))
	for i, triangle := range triangles {
		bounded[i] = boundedTriangle3D{
			triangle: triangle,
			min:      triangle.Min(),
			max:      triangle.Max(),
			index:    i,
		}
	}
	return buildTriangleIndex(bounded)
}

func buildTriangleIndex(triangles []boundedTriangle3D) *triangleIndex {
	node := &triangleIndex{min: triangles[0].min, max: triangles[0].max}
	for _, triangle := range triangles[1:] {
		node.min = node.min.Min(triangle.min)
		node.max = node.max.Max(triangle.max)
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
	sort.Sort(boundedTriangleSorter3D{triangles: triangles, axis: axis})
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
			if triangle.min.X <= max.X && triangle.max.X >= min.X &&
				triangle.min.Y <= max.Y && triangle.max.Y >= min.Y &&
				triangle.min.Z <= max.Z && triangle.max.Z >= min.Z {
				*result = append(*result, triangle.triangle)
			}
		}
		return
	}
	t.left.query(min, max, result)
	t.right.query(min, max, result)
}

func (t *triangleIndex) queryIndexed(min, max model3d.Coord3D,
	result *[]indexedTriangleCandidate3D,
) {
	if t == nil || t.min.X > max.X || t.max.X < min.X ||
		t.min.Y > max.Y || t.max.Y < min.Y || t.min.Z > max.Z || t.max.Z < min.Z {
		return
	}
	if t.left == nil {
		for _, triangle := range t.triangles {
			if triangle.min.X <= max.X && triangle.max.X >= min.X &&
				triangle.min.Y <= max.Y && triangle.max.Y >= min.Y &&
				triangle.min.Z <= max.Z && triangle.max.Z >= min.Z {
				*result = append(*result, indexedTriangleCandidate3D{
					triangle: triangle.triangle,
					index:    triangle.index,
				})
			}
		}
		return
	}
	t.left.queryIndexed(min, max, result)
	t.right.queryIndexed(min, max, result)
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

func appendConformedTriangle(result []model3d.Triangle, t model3d.Triangle,
	index *pointIndex, tol float64, candidates *[]model3d.Coord3D,
) []model3d.Triangle {
	min := t.Min().AddScalar(-tol)
	max := t.Max().AddScalar(tol)
	*candidates = (*candidates)[:0]
	index.query(min, max, candidates)
	normal := t.Normal()
	filtered := (*candidates)[:0]
	for _, point := range *candidates {
		if point == t[0] || point == t[1] || point == t[2] {
			continue
		}
		if math.Abs(normal.Dot(point.Sub(t[0]))) <= tol && pointInTriangle3D(t, point, tol) {
			filtered = append(filtered, point)
		}
	}
	if len(filtered) == 0 {
		return append(result, t)
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
	return append(result, triangles...)
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
	scale = math.Max(scale, ulp*16/arithmeticToleranceRelative)
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
