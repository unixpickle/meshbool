package meshbool

import (
	"fmt"
	"math"
	"math/rand"
	"os"
	"sort"
	"strconv"
	"testing"

	"github.com/unixpickle/meshbool/internal/bool2d"
	"github.com/unixpickle/model3d/model3d"
)

func mustUnion3D(t testing.TB, meshes ...*model3d.Mesh) *model3d.Mesh {
	t.Helper()
	result, err := Union3D(DefaultOptions3D(), meshes...)
	if err != nil {
		t.Fatalf("Union failed: %v", err)
	}
	return result
}

func mustIntersection3D(t testing.TB, meshes ...*model3d.Mesh) *model3d.Mesh {
	t.Helper()
	result, err := Intersection3D(DefaultOptions3D(), meshes...)
	if err != nil {
		t.Fatalf("Intersection failed: %v", err)
	}
	return result
}

func mustDifference3D(t testing.TB, first *model3d.Mesh, subtract ...*model3d.Mesh) *model3d.Mesh {
	t.Helper()
	result, err := Difference3D(DefaultOptions3D(), first, subtract...)
	if err != nil {
		t.Fatalf("Difference failed: %v", err)
	}
	return result
}

func TestBoxBooleans(t *testing.T) {
	a := model3d.NewMeshRect(model3d.XYZ(0, 0, 0), model3d.XYZ(2, 2, 2))
	b := model3d.NewMeshRect(model3d.XYZ(1, 1, 1), model3d.XYZ(3, 3, 3))
	tests := []struct {
		name   string
		mesh   *model3d.Mesh
		volume float64
	}{
		{"union", mustUnion3D(t, a, b), 15},
		{"intersection", mustIntersection3D(t, a, b), 1},
		{"difference", mustDifference3D(t, a, b), 7},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			assertValidMesh3D(t, test.mesh)
			if got := test.mesh.Volume(); math.Abs(got-test.volume) > 1e-7 {
				t.Fatalf("volume: got %g want %g", got, test.volume)
			}
		})
	}
}

func TestEqualCubesShiftedByTenPercent(t *testing.T) {
	const width = 2.0
	shift := width * 0.1
	a := model3d.NewMeshRect(model3d.Coord3D{}, model3d.Ones(width))
	bMin := model3d.X(shift)
	b := model3d.NewMeshRect(bMin, bMin.Add(model3d.Ones(width)))

	cubeVolume := width * width * width
	overlapVolume := (width - shift) * width * width
	tests := []struct {
		name string
		mesh *model3d.Mesh
		want float64
	}{
		{"union", mustUnion3D(t, a, b), 2*cubeVolume - overlapVolume},
		{"intersection", mustIntersection3D(t, a, b), overlapVolume},
		{"difference", mustDifference3D(t, a, b), cubeVolume - overlapVolume},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			assertValidMesh3D(t, test.mesh)
			if got := test.mesh.Volume(); math.Abs(got-test.want) > cubeVolume*1e-8 {
				t.Fatalf("volume: got %g want %g", got, test.want)
			}
		})
	}
}

func TestCoincidentBoxes(t *testing.T) {
	a := model3d.NewMeshRect(model3d.XYZ(-1, -1, -1), model3d.XYZ(1, 1, 1))
	b := a.DeepCopy()
	for name, result := range map[string]*model3d.Mesh{
		"union": mustUnion3D(t, a, b), "intersection": mustIntersection3D(t, a, b),
	} {
		t.Run(name, func(t *testing.T) {
			assertValidMesh3D(t, result)
			if math.Abs(result.Volume()-8) > 1e-8 {
				t.Fatalf("volume: got %g want 8", result.Volume())
			}
		})
	}
	if result := mustDifference3D(t, a, b); result.NumTriangles() != 0 {
		t.Fatalf("coincident difference has %d triangles", result.NumTriangles())
	}
}

func TestNestedBoxDifference(t *testing.T) {
	outer := model3d.NewMeshRect(model3d.XYZ(-2, -2, -2), model3d.XYZ(2, 2, 2))
	inner := model3d.NewMeshRect(model3d.XYZ(-1, -1, -1), model3d.XYZ(1, 1, 1))
	result := mustDifference3D(t, outer, inner)
	assertValidMesh3D(t, result)
	if math.Abs(result.Volume()-56) > 1e-7 {
		t.Fatalf("volume: got %g want 56", result.Volume())
	}
}

func TestPointTangentCavity3D(t *testing.T) {
	outer := model3d.NewMeshRect(model3d.Ones(-2), model3d.Ones(2))
	inner := octahedronMesh(model3d.Coord3D{}, 2)
	result := mustDifference3D(t, outer, inner)
	assertValidMesh3D(t, result)
	want := 64 - 4*math.Pow(2, 3)/3
	if got := result.Volume(); math.Abs(got-want) > 1e-6 {
		t.Fatalf("volume: got %g want %g", got, want)
	}
}

func TestFaceTouchingBoxes(t *testing.T) {
	a := model3d.NewMeshRect(model3d.XYZ(0, 0, 0), model3d.XYZ(1, 1, 1))
	b := model3d.NewMeshRect(model3d.XYZ(1, 0, 0), model3d.XYZ(2, 1, 1))
	result := mustUnion3D(t, a, b)
	assertValidMesh3D(t, result)
	if math.Abs(result.Volume()-2) > 1e-8 {
		t.Fatalf("volume: got %g want 2", result.Volume())
	}
	if result := mustIntersection3D(t, a, b); result.NumTriangles() != 0 {
		t.Fatalf("zero-volume intersection has %d triangles", result.NumTriangles())
	}
}

func TestSingularTouchRegularization3D(t *testing.T) {
	a := model3d.NewMeshRect(model3d.XYZ(0, 0, 0), model3d.XYZ(1, 1, 1))
	for name, other := range map[string]*model3d.Mesh{
		"edge":  model3d.NewMeshRect(model3d.XYZ(1, 1, 0), model3d.XYZ(2, 2, 1)),
		"point": model3d.NewMeshRect(model3d.XYZ(1, 1, 1), model3d.XYZ(2, 2, 2)),
	} {
		t.Run(name, func(t *testing.T) {
			result := mustUnion3D(t, a, other)
			assertValidMesh3D(t, result)
			if math.Abs(result.Volume()-2) > 1e-8 {
				t.Fatalf("volume: got %g want 2", result.Volume())
			}
		})
	}
}

func TestConnectedPointSelfContact3D(t *testing.T) {
	a := model3d.NewMeshRect(model3d.Coord3D{}, model3d.Ones(1))
	b := model3d.NewMeshRect(model3d.Ones(1), model3d.Ones(2))
	connector := mustUnion3D(t,
		model3d.NewMeshRect(model3d.XYZ(0.25, -1, 0.25), model3d.XYZ(0.75, 0.5, 0.75)),
		model3d.NewMeshRect(model3d.XYZ(0.25, -1, 0.25), model3d.XYZ(1.75, -0.5, 0.75)),
		model3d.NewMeshRect(model3d.XYZ(1.25, -1, 0.25), model3d.XYZ(1.75, 1.5, 1.5)),
	)
	result := mustUnion3D(t, mustUnion3D(t, a, connector), b)
	assertValidMesh3D(t, result)
}

func TestConnectedEdgeSelfContact3D(t *testing.T) {
	a := model3d.NewMeshRect(model3d.Coord3D{}, model3d.Ones(1))
	b := model3d.NewMeshRect(model3d.XY(1, 1), model3d.XYZ(2, 2, 1))
	connector := mustUnion3D(t,
		model3d.NewMeshRect(model3d.XYZ(0.25, -1, 0.25), model3d.XYZ(0.75, 0.5, 0.75)),
		model3d.NewMeshRect(model3d.XYZ(0.25, -1, 0.25), model3d.XYZ(1.75, -0.5, 0.75)),
		model3d.NewMeshRect(model3d.XYZ(1.25, -1, 0.25), model3d.XYZ(1.75, 1.5, 0.75)),
	)
	result := mustUnion3D(t, mustUnion3D(t, a, connector), b)
	assertValidMesh3D(t, result)
}

func TestLargeCoordinateOffset3D(t *testing.T) {
	base := model3d.Ones(1e12)
	a := model3d.NewMeshRect(base, base.Add(model3d.Ones(10)))
	b := model3d.NewMeshRect(base.Add(model3d.Ones(5)), base.Add(model3d.Ones(15)))
	result := mustUnion3D(t, a, b)
	assertValidMesh3D(t, result)
	volume := result.Translate(base.Scale(-1)).Volume()
	if math.Abs(volume-1875) > 1e-5 {
		t.Fatalf("volume at large offset: got %g want 1875", volume)
	}
}

func TestSmallCoordinateScale3D(t *testing.T) {
	scale := 1e-6
	a := model3d.NewMeshRect(model3d.Coord3D{}, model3d.Ones(2*scale))
	b := model3d.NewMeshRect(model3d.Ones(scale), model3d.Ones(3*scale))
	result := mustUnion3D(t, a, b)
	assertValidMesh3D(t, result)
	if volume := result.Volume(); math.Abs(volume-15*scale*scale*scale) > 1e-25 {
		t.Fatalf("small-scale volume: got %g want %g", volume, 15*scale*scale*scale)
	}
}

func TestThinOverlap3D(t *testing.T) {
	for _, thickness := range []float64{1e-7, 4e-8} {
		t.Run(fmt.Sprintf("thickness_%g", thickness), func(t *testing.T) {
			a := model3d.NewMeshRect(model3d.Coord3D{}, model3d.Ones(1))
			b := model3d.NewMeshRect(model3d.X(1-thickness), model3d.XYZ(2, 1, 1))
			probe := model3d.XYZ(1-thickness/2, 0.5, 0.5)
			if !a.Solid().Contains(probe) || !b.Solid().Contains(probe) {
				t.Fatal("model3d collider cannot classify the center of the thin overlap")
			}
			intersection := mustIntersection3D(t, a, b)
			assertValidMesh3D(t, intersection)
			if volume := intersection.Volume(); math.Abs(volume-thickness) > thickness*1e-4 {
				t.Fatalf("thin intersection volume: got %g want %g", volume, thickness)
			}
			difference := mustDifference3D(t, a, b)
			assertValidMesh3D(t, difference)
			if volume := difference.Volume(); math.Abs(volume-(1-thickness)) > 1e-11 {
				t.Fatalf("thin difference volume: got %g want %g", volume, 1-thickness)
			}
		})
	}
}

func TestBooleanIdentitiesIcospheres(t *testing.T) {
	rng := rand.New(rand.NewSource(123))
	trials := 2
	if value := os.Getenv("MESHBOOL_STRESS_TRIALS"); value != "" {
		parsed, err := strconv.Atoi(value)
		if err != nil || parsed < 1 || parsed > 10_000 {
			t.Fatalf("invalid MESHBOOL_STRESS_TRIALS %q", value)
		}
		trials = parsed
	}
	for trial := 0; trial < trials; trial++ {
		a := randomRaggedIcosphere(rng, model3d.X(-0.25))
		b := randomRaggedIcosphere(rng, model3d.X(0.25))
		assertValidMesh3D(t, a)
		assertValidMesh3D(t, b)
		u, i, d := mustUnion3D(t, a, b), mustIntersection3D(t, a, b), mustDifference3D(t, a, b)
		for name, result := range map[string]*model3d.Mesh{"union": u, "intersection": i, "difference": d} {
			t.Run(name, func(t *testing.T) { assertValidMesh3D(t, result) })
		}
		if got, want := u.Volume(), a.Volume()+b.Volume()-i.Volume(); math.Abs(got-want) > 1e-6 {
			t.Fatalf("trial %d: inclusion-exclusion got %g want %g", trial, got, want)
		}
		if got, want := d.Volume()+i.Volume(), a.Volume(); math.Abs(got-want) > 1e-6 {
			t.Fatalf("trial %d: partition got %g want %g", trial, got, want)
		}
	}
}

func randomRaggedIcosphere(rng *rand.Rand, center model3d.Coord3D) *model3d.Mesh {
	base := model3d.NewMeshIcosphere(model3d.Coord3D{}, 1, 1)
	var vertices []model3d.Coord3D
	base.IterateVertices(func(vertex model3d.Coord3D) {
		vertices = append(vertices, vertex)
	})
	sort.Slice(vertices, func(i, j int) bool { return coord3Less(vertices[i], vertices[j]) })
	radii := map[model3d.Coord3D]float64{}
	for _, vertex := range vertices {
		radii[vertex] = 0.55 + rng.Float64()*0.8
	}
	scale := model3d.XYZ(0.75+rng.Float64()*0.55, 0.75+rng.Float64()*0.55, 0.75+rng.Float64()*0.55)
	shearXY := (rng.Float64() - 0.5) * 0.3
	shearYZ := (rng.Float64() - 0.5) * 0.3
	shearZX := (rng.Float64() - 0.5) * 0.3
	return base.MapCoords(func(vertex model3d.Coord3D) model3d.Coord3D {
		vertex = vertex.Scale(radii[vertex])
		return model3d.XYZ(
			vertex.X*scale.X+vertex.Y*shearXY,
			vertex.Y*scale.Y+vertex.Z*shearYZ,
			vertex.Z*scale.Z+vertex.X*shearZX,
		).Add(center)
	})
}

func TestRandomBoxBooleans(t *testing.T) {
	rng := rand.New(rand.NewSource(9081726354))
	for trial := 0; trial < 40; trial++ {
		minA := model3d.NewCoord3DRandBounds(model3d.Ones(-1.5), model3d.Ones(0.5), rng)
		sizeA := model3d.NewCoord3DRandBounds(model3d.Ones(0.15), model3d.Ones(1.8), rng)
		minB := model3d.NewCoord3DRandBounds(model3d.Ones(-1.5), model3d.Ones(0.5), rng)
		sizeB := model3d.NewCoord3DRandBounds(model3d.Ones(0.15), model3d.Ones(1.8), rng)
		maxA, maxB := minA.Add(sizeA), minB.Add(sizeB)
		if trial%8 == 0 {
			// Force an exact face contact while retaining arbitrary overlap or
			// separation on the other two axes.
			minB.X = maxA.X
			maxB.X = minB.X + sizeB.X
		}
		a := model3d.NewMeshRect(minA, maxA)
		b := model3d.NewMeshRect(minB, maxB)
		intersectionVolume := axisBoxIntersectionVolume(minA, maxA, minB, maxB)
		tests := []struct {
			name string
			mesh *model3d.Mesh
			want float64
		}{
			{"union", mustUnion3D(t, a, b), sizeA.X*sizeA.Y*sizeA.Z + sizeB.X*sizeB.Y*sizeB.Z - intersectionVolume},
			{"intersection", mustIntersection3D(t, a, b), intersectionVolume},
			{"difference", mustDifference3D(t, a, b), sizeA.X*sizeA.Y*sizeA.Z - intersectionVolume},
		}
		for _, test := range tests {
			assertValidMesh3D(t, test.mesh)
			if got := test.mesh.Volume(); math.Abs(got-test.want) > 1e-7*math.Max(1, test.want) {
				t.Fatalf("trial %d %s volume: got %g want %g", trial, test.name, got, test.want)
			}
		}
	}
}

func axisBoxIntersectionVolume(minA, maxA, minB, maxB model3d.Coord3D) float64 {
	span := maxA.Min(maxB).Sub(minA.Max(minB))
	return math.Max(0, span.X) * math.Max(0, span.Y) * math.Max(0, span.Z)
}

func TestNaryOperations3D(t *testing.T) {
	a := model3d.NewMeshRect(model3d.XYZ(0, 0, 0), model3d.XYZ(4, 4, 4))
	b := model3d.NewMeshRect(model3d.XYZ(1, 1, 1), model3d.XYZ(2, 2, 2))
	c := model3d.NewMeshRect(model3d.XYZ(2.5, 2.5, 2.5), model3d.XYZ(3.5, 3.5, 3.5))
	difference := mustDifference3D(t, a, b, c)
	assertValidMesh3D(t, difference)
	if volume := difference.Volume(); math.Abs(volume-62) > 1e-8 {
		t.Fatalf("n-ary difference volume: got %g want 62", volume)
	}
	intersection := mustIntersection3D(t, a, model3d.NewMeshRect(model3d.Ones(1), model3d.Ones(5)),
		model3d.NewMeshRect(model3d.Ones(2), model3d.Ones(6)))
	assertValidMesh3D(t, intersection)
	if volume := intersection.Volume(); math.Abs(volume-8) > 1e-8 {
		t.Fatalf("n-ary intersection volume: got %g want 8", volume)
	}
}

func TestEmptyInputs3D(t *testing.T) {
	a := model3d.NewMeshRect(model3d.Coord3D{}, model3d.Ones(1))
	if mustUnion3D(t).NumTriangles() != 0 || mustIntersection3D(t).NumTriangles() != 0 ||
		mustDifference3D(t, nil, a).NumTriangles() != 0 {
		t.Fatal("zero-input operation was not empty")
	}
	result := mustDifference3D(t, a)
	assertValidMesh3D(t, result)
	if math.Abs(result.Volume()-1) > 1e-8 {
		t.Fatalf("identity difference volume: %g", result.Volume())
	}
}

func TestErrorReturningAPIs3D(t *testing.T) {
	a := model3d.NewMeshRect(model3d.Coord3D{}, model3d.Ones(1))
	b := model3d.NewMeshRect(model3d.Ones(0.5), model3d.Ones(1.5))
	for name, operation := range map[string]func() (*model3d.Mesh, error){
		"union":        func() (*model3d.Mesh, error) { return Union3D(DefaultOptions3D(), a, b) },
		"intersection": func() (*model3d.Mesh, error) { return Intersection3D(DefaultOptions3D(), a, b) },
		"difference":   func() (*model3d.Mesh, error) { return Difference3D(DefaultOptions3D(), a, b) },
	} {
		t.Run(name, func(t *testing.T) {
			result, err := operation()
			if err != nil {
				t.Fatal(err)
			}
			assertValidMesh3D(t, result)
		})
	}
}

func TestOptions3D(t *testing.T) {
	defaults := DefaultOptions3D()
	if defaults.MaxInputTriangles != 200_000 || defaults.MaxOutputTriangles != 200_000 ||
		defaults.MaxTotalFragments != 200_000 {
		t.Fatalf("unexpected triangle defaults: %#v", defaults)
	}
	a := model3d.NewMeshRect(model3d.Coord3D{}, model3d.Ones(1))
	b := model3d.NewMeshRect(model3d.Ones(0.5), model3d.Ones(1.5))
	result, err := Union3D(Options3D{}, a, b)
	if err != nil {
		t.Fatalf("zero options did not use defaults: %v", err)
	}
	assertValidMesh3D(t, result)

	limited := DefaultOptions3D()
	limited.MaxInputTriangles = a.NumTriangles() - 1
	result, err = Union3D(limited, a)
	if result != nil {
		t.Fatal("limited operation returned a mesh")
	}
	complexity, ok := err.(*ComplexityError)
	if !ok || complexity.Limit != limited.MaxInputTriangles {
		t.Fatalf("low input limit returned %#v", err)
	}

	invalid := DefaultOptions3D()
	invalid.MaxOutputTriangles = -1
	if result, err = Union3D(invalid, a); result != nil || err == nil {
		t.Fatalf("negative option returned result=%v err=%v", result, err)
	}

	planarLimited := DefaultOptions3D()
	planarLimited.PlanarOptions.MaxInputSegments = 1
	result, err = Union3D(planarLimited, a, b)
	complexity, ok = err.(*ComplexityError)
	if result != nil || !ok || complexity.Stage != "planar input segments" {
		t.Fatalf("planar limit was not propagated: result=%v err=%#v", result, err)
	}
}

func TestConvertPlanarComplexity(t *testing.T) {
	err := convertPlanarError(&bool2d.ComplexityError{Stage: "test arrangement", Limit: 123})
	complexity, ok := err.(*ComplexityError)
	if !ok {
		t.Fatalf("error type: got %T want *ComplexityError", err)
	}
	if complexity.Stage != "planar test arrangement" || complexity.Limit != 123 {
		t.Fatalf("unexpected converted error: %#v", complexity)
	}
}

func TestConvertPlanarTopology(t *testing.T) {
	err := convertPlanarError(&bool2d.TopologyError{Problem: "test topology", Count: 7})
	topology, ok := err.(*TopologyError)
	if !ok {
		t.Fatalf("error type: got %T want *TopologyError", err)
	}
	if topology.Problem != "planar test topology" || topology.Count != 7 {
		t.Fatalf("unexpected converted error: %#v", topology)
	}
}

func octahedronMesh(center model3d.Coord3D, radius float64) *model3d.Mesh {
	result := model3d.NewMesh()
	for _, sx := range []float64{-1, 1} {
		for _, sy := range []float64{-1, 1} {
			for _, sz := range []float64{-1, 1} {
				triangle := model3d.Triangle{
					center.Add(model3d.X(sx * radius)),
					center.Add(model3d.Y(sy * radius)),
					center.Add(model3d.Z(sz * radius)),
				}
				faceCenter := triangle[0].Add(triangle[1]).Add(triangle[2]).Scale(1.0 / 3)
				if triangle.Normal().Dot(faceCenter.Sub(center)) < 0 {
					triangle[0], triangle[1] = triangle[1], triangle[0]
				}
				result.Add(&triangle)
			}
		}
	}
	return result
}

func assertValidMesh3D(t *testing.T, mesh *model3d.Mesh) {
	t.Helper()
	if singular := mesh.SingularVertices(); len(singular) != 0 {
		t.Fatalf("mesh has %d singular vertices: %v (triangles=%d volume=%g)",
			len(singular), singular, mesh.NumTriangles(), mesh.Volume())
	}
	if mesh.NeedsRepair() {
		counts := map[model3d.Segment]int{}
		mesh.Iterate(func(tri *model3d.Triangle) {
			for i := range tri {
				counts[model3d.NewSegment(tri[i], tri[(i+1)%3])]++
			}
		})
		var bad []model3d.Segment
		badCount := 0
		for edge, count := range counts {
			if count != 2 {
				badCount++
				bad = append(bad, edge)
			}
		}
		if len(bad) > 8 {
			bad = bad[:8]
		}
		var pointsOnFirst []model3d.Coord3D
		if len(bad) != 0 {
			edge := bad[0]
			delta := edge[1].Sub(edge[0])
			lengthSq := delta.NormSquared()
			mesh.IterateVertices(func(point model3d.Coord3D) {
				param := point.Sub(edge[0]).Dot(delta) / lengthSq
				if param > 1e-10 && param < 1-1e-10 && edge[0].Add(delta.Scale(param)).Dist(point) < 1e-7 {
					pointsOnFirst = append(pointsOnFirst, point)
				}
			})
		}
		t.Fatalf("mesh needs repair: inconsistent=%d singular=%d triangles=%d bad_edge_count=%d bad_edges=%v points_on_first=%v",
			len(mesh.InconsistentEdges()), len(mesh.SingularVertices()), mesh.NumTriangles(), badCount, bad, pointsOnFirst)
	}
	if !mesh.Orientable() {
		t.Fatal("mesh is not orientable")
	}
	if intersections := mesh.SelfIntersections(); intersections != 0 {
		t.Fatalf("mesh has %d self-intersections", intersections)
	}
}
