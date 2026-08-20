package meshbool

import (
	"fmt"
	"math"
	"math/big"
	"sort"

	"github.com/unixpickle/model3d/model3d"
)

// exactTriangleIntersector constructs triangle-intersection vertices from the
// exact rational values represented by the input float64 coordinates. The
// point cache is intentionally shared by the whole corefinement: neighboring
// triangle pairs which meet at the same geometric point consequently receive
// the same float64 vertex after the one final conversion.
type exactTriangleIntersector struct {
	triangles map[*model3d.Triangle]exactTriangle3D
	points    map[string]model3d.Coord3D
}

type exactMeshClassifier struct {
	triangles        map[*model3d.Triangle]exactTriangle3D
	index            *triangleIndex
	min, max         model3d.Coord3D
	directions       []exactRayDirection
	candidateScratch []*model3d.Triangle
}

type exactRayDirection struct {
	approximate model3d.Coord3D
	exact       exactCoord3D
	axis        int
}

func newExactMeshClassifier(triangles []*model3d.Triangle, index *triangleIndex) *exactMeshClassifier {
	classifier := &exactMeshClassifier{
		triangles: map[*model3d.Triangle]exactTriangle3D{},
		index:     index,
	}
	// Axis rays are cheapest. The oblique rays handle exact edge and vertex
	// hits without perturbing the query point or weakening exact predicates.
	for i, direction := range []model3d.Coord3D{
		model3d.X(1), model3d.Y(1), model3d.Z(1),
		model3d.XYZ(1, 1, 1), model3d.XYZ(1, 2, 4),
		model3d.XYZ(4, 2, 1), model3d.XYZ(2, 5, 3),
	} {
		axis := -1
		if i < 3 {
			axis = i
		}
		classifier.directions = append(classifier.directions, exactRayDirection{
			approximate: direction,
			exact:       exactCoordFromFinite(direction),
			axis:        axis,
		})
	}
	if index != nil {
		classifier.min, classifier.max = index.min, index.max
	}
	return classifier
}

func (e *exactMeshClassifier) contains(point model3d.Coord3D) (bool, error) {
	if e.index == nil || point.X < e.min.X || point.Y < e.min.Y || point.Z < e.min.Z ||
		point.X > e.max.X || point.Y > e.max.Y || point.Z > e.max.Z {
		return false, nil
	}
	if inside, conclusive := e.containsAxisFiltered(point); conclusive {
		return inside, nil
	}
	return e.containsExact(point)
}

func (e *exactMeshClassifier) containsExact(point model3d.Coord3D) (bool, error) {
	exactPoint, err := exactCoordFromFloat(point)
	if err != nil {
		return false, err
	}
	for _, direction := range e.directions {
		min, max := point, point
		if direction.approximate.X > 0 {
			max.X = e.max.X
		}
		if direction.approximate.Y > 0 {
			max.Y = e.max.Y
		}
		if direction.approximate.Z > 0 {
			max.Z = e.max.Z
		}
		e.candidateScratch = e.candidateScratch[:0]
		e.index.query(min, max, &e.candidateScratch)
		collisions := 0
		degenerate := false
		for _, candidate := range e.candidateScratch {
			triangle, err := e.triangle(candidate)
			if err != nil {
				return false, err
			}
			var denominatorSign int
			if direction.axis >= 0 {
				denominatorSign = filteredOrient2DSign(
					triangle[0], triangle[1], triangle[2], direction.axis)
			} else {
				denominatorSign = filteredNormalDotSign(triangle, direction.exact)
			}
			sideSign := filteredOrient3DSign(triangle[0], triangle[1], triangle[2], exactPoint)
			if denominatorSign == 0 {
				if sideSign == 0 {
					degenerate = true
					break
				}
				continue
			}
			if sideSign != 0 && sideSign == denominatorSign {
				continue
			}
			var denominator *big.Rat
			if direction.axis >= 0 {
				denominator = exactOrient2D(triangle[0], triangle[1], triangle[2], direction.axis)
			} else {
				denominator = exactNormalDot(triangle, direction.exact)
			}
			side := exactOrient3D(triangle[0], triangle[1], triangle[2], exactPoint)
			parameter := new(big.Rat).Quo(new(big.Rat).Neg(side), denominator)
			intersection := exactAddScaled(exactPoint, direction.exact, parameter)
			location := filteredPointTriangleLocation(intersection, triangle)
			if location == 0 {
				continue
			}
			if location < 0 || parameter.Sign() == 0 {
				degenerate = true
				break
			}
			collisions++
		}
		if !degenerate {
			return collisions%2 == 1, nil
		}
	}
	return false, fmt.Errorf("meshbool: could not classify a surface cell without a degenerate ray")
}

func (e *exactMeshClassifier) containsAxisFiltered(point model3d.Coord3D) (bool, bool) {
	query := boundedExactCoordFromFinite(point)
	for axis := 0; axis < 3; axis++ {
		min, max := point, point
		switch axis {
		case 0:
			max.X = e.max.X
		case 1:
			max.Y = e.max.Y
		default:
			max.Z = e.max.Z
		}
		e.candidateScratch = e.candidateScratch[:0]
		e.index.query(min, max, &e.candidateScratch)
		collisions := 0
		degenerate := false
		for _, candidate := range e.candidateScratch {
			triangle := boundedExactTriangleFromFinite(candidate)
			orientation, ok := intervalOrient2DSign(triangle[0], triangle[1], triangle[2], axis)
			if !ok {
				degenerate = true
				break
			}
			side, ok := intervalOrient3DSign(triangle[0], triangle[1], triangle[2], query)
			if !ok {
				degenerate = true
				break
			}
			// The ray-plane parameter is -side/orientation, so equal signs
			// place the intersection behind the ray origin.
			if side == orientation {
				continue
			}
			inside := true
			for i, start := range triangle {
				edgeSide, ok := intervalOrient2DSign(start, triangle[(i+1)%3], query, axis)
				if !ok {
					degenerate = true
					break
				}
				if edgeSide != orientation {
					inside = false
					break
				}
			}
			if degenerate {
				break
			}
			if inside {
				collisions++
			}
		}
		if !degenerate {
			return collisions%2 == 1, true
		}
	}
	return false, false
}

func (e *exactMeshClassifier) triangle(t *model3d.Triangle) (exactTriangle3D, error) {
	if triangle, ok := e.triangles[t]; ok {
		return triangle, nil
	}
	var triangle exactTriangle3D
	for i, point := range t {
		var err error
		triangle[i], err = exactCoordFromFloat(point)
		if err != nil {
			return exactTriangle3D{}, err
		}
	}
	e.triangles[t] = triangle
	return triangle, nil
}

type exactTriangle3D [3]exactCoord3D

type exactCoord3D struct {
	x, y, z *big.Rat
	bounds  [3]floatInterval
}

type floatInterval struct {
	min, max float64
}

func newExactTriangleIntersector() *exactTriangleIntersector {
	return &exactTriangleIntersector{
		triangles: map[*model3d.Triangle]exactTriangle3D{},
		points:    map[string]model3d.Coord3D{},
	}
}

func (e *exactTriangleIntersector) intersection(a, b *model3d.Triangle) ([]model3d.Segment, error) {
	exactA, err := e.triangle(a)
	if err != nil {
		return nil, err
	}
	exactB, err := e.triangle(b)
	if err != nil {
		return nil, err
	}
	if exactTrianglesCoplanar(exactA, exactB) {
		return nil, nil
	}

	points := map[string]exactCoord3D{}
	collectExactEdgeIntersections(exactA, exactB, points)
	collectExactEdgeIntersections(exactB, exactA, points)
	if len(points) < 2 {
		return nil, nil
	}
	ordered := make([]exactCoord3D, 0, len(points))
	for _, point := range points {
		ordered = append(ordered, point)
	}
	sort.Slice(ordered, func(i, j int) bool { return exactCoordLess(ordered[i], ordered[j]) })
	first, last := ordered[0], ordered[len(ordered)-1]
	if exactCoordEqual(first, last) {
		return nil, nil
	}
	firstFloat, err := e.floatPoint(first)
	if err != nil {
		return nil, err
	}
	lastFloat, err := e.floatPoint(last)
	if err != nil {
		return nil, err
	}
	if firstFloat == lastFloat {
		return nil, nil
	}
	return []model3d.Segment{{firstFloat, lastFloat}}, nil
}

func (e *exactTriangleIntersector) triangle(t *model3d.Triangle) (exactTriangle3D, error) {
	if result, ok := e.triangles[t]; ok {
		return result, nil
	}
	var result exactTriangle3D
	for i, point := range t {
		var err error
		result[i], err = exactCoordFromFloat(point)
		if err != nil {
			return exactTriangle3D{}, err
		}
	}
	e.triangles[t] = result
	return result, nil
}

func (e *exactTriangleIntersector) floatPoint(point exactCoord3D) (model3d.Coord3D, error) {
	key := exactCoordKey(point)
	if result, ok := e.points[key]; ok {
		return result, nil
	}
	x, _ := point.x.Float64()
	y, _ := point.y.Float64()
	z, _ := point.z.Float64()
	result := model3d.XYZ(x, y, z)
	e.points[key] = result
	return result, nil
}

func exactCoordFromFloat(point model3d.Coord3D) (exactCoord3D, error) {
	if math.IsNaN(point.X) || math.IsNaN(point.Y) || math.IsNaN(point.Z) ||
		math.IsInf(point.X, 0) || math.IsInf(point.Y, 0) || math.IsInf(point.Z, 0) {
		return exactCoord3D{}, fmt.Errorf("meshbool: input mesh contains a non-finite coordinate")
	}
	return exactCoordFromFinite(point), nil
}

func exactCoordFromFinite(point model3d.Coord3D) exactCoord3D {
	return exactCoord3D{
		x: new(big.Rat).SetFloat64(point.X),
		y: new(big.Rat).SetFloat64(point.Y),
		z: new(big.Rat).SetFloat64(point.Z),
		bounds: [3]floatInterval{
			{min: point.X, max: point.X},
			{min: point.Y, max: point.Y},
			{min: point.Z, max: point.Z},
		},
	}
}

func boundedExactCoordFromFinite(point model3d.Coord3D) exactCoord3D {
	return exactCoord3D{bounds: [3]floatInterval{
		{min: point.X, max: point.X},
		{min: point.Y, max: point.Y},
		{min: point.Z, max: point.Z},
	}}
}

func boundedExactTriangleFromFinite(triangle *model3d.Triangle) exactTriangle3D {
	return exactTriangle3D{
		boundedExactCoordFromFinite(triangle[0]),
		boundedExactCoordFromFinite(triangle[1]),
		boundedExactCoordFromFinite(triangle[2]),
	}
}

func exactTrianglesCoplanar(a, b exactTriangle3D) bool {
	for _, point := range a {
		if filteredOrient3DSign(b[0], b[1], b[2], point) != 0 {
			return false
		}
	}
	for _, point := range b {
		if filteredOrient3DSign(a[0], a[1], a[2], point) != 0 {
			return false
		}
	}
	return true
}

func collectExactEdgeIntersections(edges, face exactTriangle3D, result map[string]exactCoord3D) {
	for i, start := range edges {
		end := edges[(i+1)%3]
		startSign := filteredOrient3DSign(face[0], face[1], face[2], start)
		endSign := filteredOrient3DSign(face[0], face[1], face[2], end)
		if startSign != 0 && startSign == endSign {
			continue
		}
		if startSign == 0 {
			if exactPointInTriangle(start, face) {
				result[exactCoordKey(start)] = start
			}
		}
		if endSign == 0 {
			if exactPointInTriangle(end, face) {
				result[exactCoordKey(end)] = end
			}
		}
		if startSign == 0 || endSign == 0 {
			continue
		}
		startSide := exactOrient3D(face[0], face[1], face[2], start)
		endSide := exactOrient3D(face[0], face[1], face[2], end)
		parameter := new(big.Rat).Quo(startSide, new(big.Rat).Sub(startSide, endSide))
		point := exactInterpolate(start, end, parameter)
		if exactPointInTriangle(point, face) {
			result[exactCoordKey(point)] = point
		}
	}
}

func exactInterpolate(a, b exactCoord3D, t *big.Rat) exactCoord3D {
	interpolate := func(x, y *big.Rat) *big.Rat {
		return new(big.Rat).Add(x, new(big.Rat).Mul(t, new(big.Rat).Sub(y, x)))
	}
	result := exactCoord3D{
		x: interpolate(a.x, b.x),
		y: interpolate(a.y, b.y),
		z: interpolate(a.z, b.z),
	}
	result.bounds[0] = ratInterval(result.x)
	result.bounds[1] = ratInterval(result.y)
	result.bounds[2] = ratInterval(result.z)
	return result
}

func exactAddScaled(point, direction exactCoord3D, scale *big.Rat) exactCoord3D {
	addScaled := func(value, delta *big.Rat) *big.Rat {
		if delta.Sign() == 0 {
			return value
		}
		return new(big.Rat).Add(value, new(big.Rat).Mul(scale, delta))
	}
	result := exactCoord3D{
		x: addScaled(point.x, direction.x),
		y: addScaled(point.y, direction.y),
		z: addScaled(point.z, direction.z),
	}
	for axis, value := range []*big.Rat{result.x, result.y, result.z} {
		if exactCoordAxis(direction, axis).Sign() == 0 {
			result.bounds[axis] = point.bounds[axis]
		} else {
			result.bounds[axis] = ratInterval(value)
		}
	}
	return result
}

func ratInterval(value *big.Rat) floatInterval {
	approximation, exact := value.Float64()
	if exact {
		return floatInterval{min: approximation, max: approximation}
	}
	if math.IsInf(approximation, 0) {
		return floatInterval{min: math.Inf(-1), max: math.Inf(1)}
	}
	// Rat.Float64 rounds to the nearest float. Keeping both neighbors avoids
	// another rational comparison while still enclosing the exact value.
	return floatInterval{
		min: math.Nextafter(approximation, math.Inf(-1)),
		max: math.Nextafter(approximation, math.Inf(1)),
	}
}

func exactOrient3D(a, b, c, point exactCoord3D) *big.Rat {
	ab := exactSubtract(b, a)
	ac := exactSubtract(c, a)
	ap := exactSubtract(point, a)
	crossX := new(big.Rat).Sub(new(big.Rat).Mul(ab.y, ac.z), new(big.Rat).Mul(ab.z, ac.y))
	crossY := new(big.Rat).Sub(new(big.Rat).Mul(ab.z, ac.x), new(big.Rat).Mul(ab.x, ac.z))
	crossZ := new(big.Rat).Sub(new(big.Rat).Mul(ab.x, ac.y), new(big.Rat).Mul(ab.y, ac.x))
	return new(big.Rat).Add(
		new(big.Rat).Add(new(big.Rat).Mul(crossX, ap.x), new(big.Rat).Mul(crossY, ap.y)),
		new(big.Rat).Mul(crossZ, ap.z),
	)
}

// filteredOrient3DSign first evaluates the determinant with outward-rounded
// float64 intervals. A non-zero interval sign is therefore conclusive; only
// determinants whose interval contains zero pay for exact rational arithmetic.
func filteredOrient3DSign(a, b, c, point exactCoord3D) int {
	if sign, ok := intervalOrient3DSign(a, b, c, point); ok {
		return sign
	}
	return exactOrient3D(a, b, c, point).Sign()
}

func intervalOrient3DSign(a, b, c, point exactCoord3D) (int, bool) {
	ab := intervalCoordSubtract(b, a)
	ac := intervalCoordSubtract(c, a)
	ap := intervalCoordSubtract(point, a)
	crossX := intervalSubtract(intervalMultiply(ab[1], ac[2]), intervalMultiply(ab[2], ac[1]))
	crossY := intervalSubtract(intervalMultiply(ab[2], ac[0]), intervalMultiply(ab[0], ac[2]))
	crossZ := intervalSubtract(intervalMultiply(ab[0], ac[1]), intervalMultiply(ab[1], ac[0]))
	determinant := intervalAdd(
		intervalAdd(intervalMultiply(crossX, ap[0]), intervalMultiply(crossY, ap[1])),
		intervalMultiply(crossZ, ap[2]),
	)
	if sign := intervalSign(determinant); sign != 0 {
		return sign, true
	}
	return 0, false
}

func exactNormalDot(triangle exactTriangle3D, direction exactCoord3D) *big.Rat {
	normal := exactCross(exactSubtract(triangle[1], triangle[0]),
		exactSubtract(triangle[2], triangle[0]))
	return new(big.Rat).Add(
		new(big.Rat).Add(new(big.Rat).Mul(normal.x, direction.x),
			new(big.Rat).Mul(normal.y, direction.y)),
		new(big.Rat).Mul(normal.z, direction.z),
	)
}

func filteredNormalDotSign(triangle exactTriangle3D, direction exactCoord3D) int {
	ab := intervalCoordSubtract(triangle[1], triangle[0])
	ac := intervalCoordSubtract(triangle[2], triangle[0])
	crossX := intervalSubtract(intervalMultiply(ab[1], ac[2]), intervalMultiply(ab[2], ac[1]))
	crossY := intervalSubtract(intervalMultiply(ab[2], ac[0]), intervalMultiply(ab[0], ac[2]))
	crossZ := intervalSubtract(intervalMultiply(ab[0], ac[1]), intervalMultiply(ab[1], ac[0]))
	dot := intervalAdd(
		intervalAdd(intervalMultiply(crossX, direction.bounds[0]),
			intervalMultiply(crossY, direction.bounds[1])),
		intervalMultiply(crossZ, direction.bounds[2]),
	)
	if sign := intervalSign(dot); sign != 0 {
		return sign
	}
	return exactNormalDot(triangle, direction).Sign()
}

func exactSubtract(a, b exactCoord3D) exactCoord3D {
	return exactCoord3D{
		x: new(big.Rat).Sub(a.x, b.x),
		y: new(big.Rat).Sub(a.y, b.y),
		z: new(big.Rat).Sub(a.z, b.z),
	}
}

func exactPointInTriangle(point exactCoord3D, triangle exactTriangle3D) bool {
	return filteredPointTriangleLocation(point, triangle) != 0
}

func filteredPointTriangleLocation(point exactCoord3D, triangle exactTriangle3D) int {
	approxA, approxB, approxC := approximateCoord(triangle[0]), approximateCoord(triangle[1]), approximateCoord(triangle[2])
	normal := approxB.Sub(approxA).Cross(approxC.Sub(approxA))
	drop := 0
	if math.Abs(normal.Y) > math.Abs(normal.X) {
		drop = 1
	}
	if math.Abs(normal.Z) > math.Abs(coordAxis(normal, drop)) {
		drop = 2
	}
	for attempts := 0; attempts < 3; attempts++ {
		orientation := filteredOrient2DSign(triangle[0], triangle[1], triangle[2], drop)
		if orientation != 0 {
			boundary := false
			for i, start := range triangle {
				side := filteredOrient2DSign(start, triangle[(i+1)%3], point, drop)
				if side == 0 {
					boundary = true
				} else if side != orientation {
					return 0
				}
			}
			if boundary {
				return -1
			}
			return 1
		}
		drop = (drop + 1) % 3
	}
	return 0
}

// exactPointTriangleLocation returns 1 in the strict interior, -1 on the
// boundary, and 0 outside the triangle.
func exactPointTriangleLocation(point exactCoord3D, triangle exactTriangle3D) int {
	normal := exactCross(exactSubtract(triangle[1], triangle[0]), exactSubtract(triangle[2], triangle[0]))
	drop := 0
	if exactAbsCmp(normal.y, normal.x) > 0 {
		drop = 1
	}
	if exactAbsCmp(normal.z, exactCoordAxis(normal, drop)) > 0 {
		drop = 2
	}
	orientation := exactOrient2D(triangle[0], triangle[1], triangle[2], drop).Sign()
	if orientation == 0 {
		return 0
	}
	boundary := false
	for i, start := range triangle {
		side := exactOrient2D(start, triangle[(i+1)%3], point, drop).Sign()
		if side == 0 {
			boundary = true
		} else if side != orientation {
			return 0
		}
	}
	if boundary {
		return -1
	}
	return 1
}

func approximateCoord(point exactCoord3D) model3d.Coord3D {
	return model3d.XYZ(
		point.bounds[0].min/2+point.bounds[0].max/2,
		point.bounds[1].min/2+point.bounds[1].max/2,
		point.bounds[2].min/2+point.bounds[2].max/2,
	)
}

func exactCross(a, b exactCoord3D) exactCoord3D {
	return exactCoord3D{
		x: new(big.Rat).Sub(new(big.Rat).Mul(a.y, b.z), new(big.Rat).Mul(a.z, b.y)),
		y: new(big.Rat).Sub(new(big.Rat).Mul(a.z, b.x), new(big.Rat).Mul(a.x, b.z)),
		z: new(big.Rat).Sub(new(big.Rat).Mul(a.x, b.y), new(big.Rat).Mul(a.y, b.x)),
	}
}

func exactOrient2D(a, b, point exactCoord3D, drop int) *big.Rat {
	axis1, axis2 := (drop+1)%3, (drop+2)%3
	ab1 := new(big.Rat).Sub(exactCoordAxis(b, axis1), exactCoordAxis(a, axis1))
	ab2 := new(big.Rat).Sub(exactCoordAxis(b, axis2), exactCoordAxis(a, axis2))
	ap1 := new(big.Rat).Sub(exactCoordAxis(point, axis1), exactCoordAxis(a, axis1))
	ap2 := new(big.Rat).Sub(exactCoordAxis(point, axis2), exactCoordAxis(a, axis2))
	return new(big.Rat).Sub(new(big.Rat).Mul(ab1, ap2), new(big.Rat).Mul(ab2, ap1))
}

func filteredOrient2DSign(a, b, point exactCoord3D, drop int) int {
	if sign, ok := intervalOrient2DSign(a, b, point, drop); ok {
		return sign
	}
	return exactOrient2D(a, b, point, drop).Sign()
}

func intervalOrient2DSign(a, b, point exactCoord3D, drop int) (int, bool) {
	axis1, axis2 := (drop+1)%3, (drop+2)%3
	ab1 := intervalSubtract(b.bounds[axis1], a.bounds[axis1])
	ab2 := intervalSubtract(b.bounds[axis2], a.bounds[axis2])
	ap1 := intervalSubtract(point.bounds[axis1], a.bounds[axis1])
	ap2 := intervalSubtract(point.bounds[axis2], a.bounds[axis2])
	determinant := intervalSubtract(intervalMultiply(ab1, ap2), intervalMultiply(ab2, ap1))
	if sign := intervalSign(determinant); sign != 0 {
		return sign, true
	}
	return 0, false
}

func intervalCoordSubtract(a, b exactCoord3D) [3]floatInterval {
	return [3]floatInterval{
		intervalSubtract(a.bounds[0], b.bounds[0]),
		intervalSubtract(a.bounds[1], b.bounds[1]),
		intervalSubtract(a.bounds[2], b.bounds[2]),
	}
}

func intervalAdd(a, b floatInterval) floatInterval {
	return outwardInterval(a.min+b.min, a.max+b.max)
}

func intervalSubtract(a, b floatInterval) floatInterval {
	return outwardInterval(a.min-b.max, a.max-b.min)
}

func intervalMultiply(a, b floatInterval) floatInterval {
	products := [4]float64{a.min * b.min, a.min * b.max, a.max * b.min, a.max * b.max}
	minimum, maximum := products[0], products[0]
	for _, product := range products[1:] {
		if math.IsNaN(product) {
			return floatInterval{min: math.Inf(-1), max: math.Inf(1)}
		}
		minimum = math.Min(minimum, product)
		maximum = math.Max(maximum, product)
	}
	return outwardInterval(minimum, maximum)
}

func outwardInterval(minimum, maximum float64) floatInterval {
	if math.IsNaN(minimum) || math.IsNaN(maximum) {
		return floatInterval{min: math.Inf(-1), max: math.Inf(1)}
	}
	return floatInterval{
		min: math.Nextafter(minimum, math.Inf(-1)),
		max: math.Nextafter(maximum, math.Inf(1)),
	}
}

func intervalSign(value floatInterval) int {
	if value.min > 0 {
		return 1
	}
	if value.max < 0 {
		return -1
	}
	return 0
}

func exactCoordAxis(point exactCoord3D, axis int) *big.Rat {
	switch axis {
	case 0:
		return point.x
	case 1:
		return point.y
	default:
		return point.z
	}
}

func exactAbsCmp(a, b *big.Rat) int {
	absA := new(big.Rat).Abs(a)
	absB := new(big.Rat).Abs(b)
	return absA.Cmp(absB)
}

func exactCoordKey(point exactCoord3D) string {
	return point.x.RatString() + ";" + point.y.RatString() + ";" + point.z.RatString()
}

func exactCoordLess(a, b exactCoord3D) bool {
	if comparison := a.x.Cmp(b.x); comparison != 0 {
		return comparison < 0
	}
	if comparison := a.y.Cmp(b.y); comparison != 0 {
		return comparison < 0
	}
	return a.z.Cmp(b.z) < 0
}

func exactCoordEqual(a, b exactCoord3D) bool {
	return a.x.Cmp(b.x) == 0 && a.y.Cmp(b.y) == 0 && a.z.Cmp(b.z) == 0
}
