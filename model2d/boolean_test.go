package model2d

import (
	"math"
	"math/rand"
	"testing"

	model "github.com/unixpickle/model3d/model2d"
)

func TestRectBooleans(t *testing.T) {
	a := model.NewMeshRect(model.XY(0, 0), model.XY(2, 2))
	b := model.NewMeshRect(model.XY(1, 1), model.XY(3, 3))
	tests := []struct {
		name string
		mesh *model.Mesh
		area float64
	}{
		{"union", Union(a, b), 7},
		{"intersection", Intersection(a, b), 1},
		{"difference", Difference(a, b), 3},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			assertValidMesh(t, test.mesh)
			if actual := test.mesh.Area(); math.Abs(actual-test.area) > 1e-8 {
				t.Fatalf("area: got %g want %g", actual, test.area)
			}
		})
	}
}

func TestCoincidentAndTouching(t *testing.T) {
	a := model.NewMeshRect(model.XY(0, 0), model.XY(1, 1))
	same := model.NewMeshRect(model.XY(0, 0), model.XY(1, 1))
	touchEdge := model.NewMeshRect(model.XY(1, 0), model.XY(2, 1))
	touchPoint := model.NewMeshRect(model.XY(1, 1), model.XY(2, 2))
	tests := []struct {
		name     string
		mesh     *model.Mesh
		area     float64
		segments int
		manifold bool
	}{
		{"same union", Union(a, same), 1, 4, true},
		{"same intersection", Intersection(a, same), 1, 4, true},
		{"same difference", Difference(a, same), 0, 0, true},
		{"edge union", Union(a, touchEdge), 2, 6, true},
		{"edge intersection", Intersection(a, touchEdge), 0, 0, true},
		{"edge difference", Difference(a, touchEdge), 1, 4, true},
		{"point union", Union(a, touchPoint), 2, 8, true},
		{"point intersection", Intersection(a, touchPoint), 0, 0, true},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			if test.manifold {
				assertValidMesh(t, test.mesh)
			} else if test.mesh.Manifold() {
				t.Fatal("expected exact point-contact result to be non-manifold")
			}
			if actual := test.mesh.Area(); math.Abs(actual-test.area) > 1e-8 {
				t.Fatalf("area: got %g want %g", actual, test.area)
			}
			if actual := test.mesh.NumSegments(); actual != test.segments {
				t.Fatalf("segments: got %d want %d", actual, test.segments)
			}
		})
	}
}

func TestNestedDifferenceCreatesHole(t *testing.T) {
	outer := model.NewMeshRect(model.XY(-2, -2), model.XY(2, 2))
	inner := model.NewMeshRect(model.XY(-1, -1), model.XY(1, 1))
	result := Difference(outer, inner)
	assertValidMesh(t, result)
	if actual := result.Area(); math.Abs(actual-12) > 1e-8 {
		t.Fatalf("area: got %g want 12", actual)
	}
	if result.NumSegments() != 8 {
		t.Fatalf("segments: got %d want 8", result.NumSegments())
	}
}

func TestPointTangentHole(t *testing.T) {
	outer := model.NewMeshRect(model.XY(0, 0), model.XY(4, 4))
	inner := model.NewMesh()
	points := []model.Coord{
		model.XY(2, 0), model.XY(1, 1), model.XY(2, 2), model.XY(3, 1),
	}
	for i, point := range points {
		inner.Add(&model.Segment{point, points[(i+1)%len(points)]})
	}
	result := Difference(outer, inner)
	assertValidMesh(t, result)
	if got := result.Area(); math.Abs(got-14) > 1e-8 {
		t.Fatalf("area: got %g want 14", got)
	}
}

func TestMultiplePointContacts(t *testing.T) {
	meshes := []*model.Mesh{
		model.NewMeshRect(model.XY(0, 0), model.XY(1, 1)),
		model.NewMeshRect(model.XY(1, 1), model.XY(2, 2)),
		model.NewMeshRect(model.XY(2, 2), model.XY(3, 3)),
	}
	result := Union(meshes...)
	assertValidMesh(t, result)
	if got := result.Area(); math.Abs(got-3) > 1e-8 {
		t.Fatalf("area: got %g want 3", got)
	}
}

func TestHighDegreePointContact(t *testing.T) {
	const count = 9
	const halfWidth = 0.12
	meshes := make([]*model.Mesh, count)
	for i := range meshes {
		angle := float64(i) * 2 * math.Pi / count
		points := [3]model.Coord{
			{},
			model.NewCoordPolar(angle+halfWidth, 1),
			model.NewCoordPolar(angle-halfWidth, 1),
		}
		meshes[i] = model.NewMesh()
		for j, point := range points {
			meshes[i].Add(&model.Segment{point, points[(j+1)%len(points)]})
		}
	}
	result := Union(meshes...)
	assertValidMesh(t, result)
	want := count * math.Sin(2*halfWidth) / 2
	if got := result.Area(); math.Abs(got-want) > 1e-7 {
		t.Fatalf("area: got %g want %g", got, want)
	}
}

func TestCollinearPartialOverlap(t *testing.T) {
	a := model.NewMeshRect(model.XY(0, 0), model.XY(3, 1))
	b := model.NewMeshRect(model.XY(1, 0), model.XY(2, 2))
	for name, result := range map[string]*model.Mesh{
		"union": Union(a, b), "intersection": Intersection(a, b), "difference": Difference(a, b),
	} {
		t.Run(name, func(t *testing.T) { assertValidMesh(t, result) })
	}
	if got := Union(a, b).Area(); math.Abs(got-4) > 1e-8 {
		t.Fatalf("union area: %g", got)
	}
	if got := Intersection(a, b).Area(); math.Abs(got-1) > 1e-8 {
		t.Fatalf("intersection area: %g", got)
	}
}

func TestLargeCoordinateOffset(t *testing.T) {
	base := 1e12
	a := model.NewMeshRect(model.XY(base, base), model.XY(base+10, base+10))
	b := model.NewMeshRect(model.XY(base+5, base+5), model.XY(base+15, base+15))
	result := Union(a, b)
	assertValidMesh(t, result)
	// Translate before measuring area to avoid cancellation in the model3d
	// helper itself becoming part of this boolean regression.
	area := result.Translate(model.XY(-base, -base)).Area()
	if math.Abs(area-175) > 1e-6 {
		t.Fatalf("area at large offset: got %g want 175", area)
	}
}

func TestSmallCoordinateScale(t *testing.T) {
	scale := 1e-9
	a := model.NewMeshRect(model.XY(0, 0), model.XY(2*scale, 2*scale))
	b := model.NewMeshRect(model.XY(scale, scale), model.XY(3*scale, 3*scale))
	result := Union(a, b)
	assertValidMesh(t, result)
	if area := result.Area(); math.Abs(area-7*scale*scale) > 1e-30 {
		t.Fatalf("small-scale area: got %g want %g", area, 7*scale*scale)
	}
}

func TestThinOverlap(t *testing.T) {
	thickness := 1e-7
	a := model.NewMeshRect(model.XY(0, 0), model.XY(1, 1))
	b := model.NewMeshRect(model.XY(1-thickness, 0), model.XY(2, 1))
	intersection := Intersection(a, b)
	assertValidMesh(t, intersection)
	if area := intersection.Area(); math.Abs(area-thickness) > 1e-12 {
		t.Fatalf("thin intersection area: got %g want %g", area, thickness)
	}
	difference := Difference(a, b)
	assertValidMesh(t, difference)
	if area := difference.Area(); math.Abs(area-(1-thickness)) > 1e-12 {
		t.Fatalf("thin difference area: got %g want %g", area, 1-thickness)
	}
}

func TestRandomRaggedPolygons(t *testing.T) {
	rng := rand.New(rand.NewSource(1337))
	for trial := 0; trial < 200; trial++ {
		a := randomRadialMesh(rng, model.XY(-0.2, 0), 31)
		b := randomRadialMesh(rng, model.XY(0.2, 0), 37)
		for name, result := range map[string]*model.Mesh{
			"union": Union(a, b), "intersection": Intersection(a, b), "difference": Difference(a, b),
		} {
			assertValidMesh(t, result)
			if math.IsNaN(result.Area()) || result.Area() < -1e-9 {
				t.Fatalf("trial %d %s: invalid area %g", trial, name, result.Area())
			}
		}
	}
}

func TestBooleanIdentitiesRandom(t *testing.T) {
	rng := rand.New(rand.NewSource(42))
	for trial := 0; trial < 100; trial++ {
		a := randomRadialMesh(rng, model.XY(-0.1, 0), 13+rng.Intn(20))
		b := randomRadialMesh(rng, model.XY(0.1, 0), 13+rng.Intn(20))
		u, i, d := Union(a, b), Intersection(a, b), Difference(a, b)
		wantUnion := a.Area() + b.Area() - i.Area()
		if math.Abs(u.Area()-wantUnion) > 1e-7 {
			t.Fatalf("trial %d: inclusion-exclusion: union=%g want=%g", trial, u.Area(), wantUnion)
		}
		if math.Abs(d.Area()+i.Area()-a.Area()) > 1e-7 {
			t.Fatalf("trial %d: partition: difference=%g intersection=%g a=%g", trial, d.Area(), i.Area(), a.Area())
		}
	}
}

func TestNilAndEmptyInputs(t *testing.T) {
	a := model.NewMeshRect(model.XY(0, 0), model.XY(1, 1))
	if Union().NumSegments() != 0 || Intersection().NumSegments() != 0 || Difference(nil, a).NumSegments() != 0 {
		t.Fatal("zero-input operation was not empty")
	}
	if got := Difference(a).Area(); math.Abs(got-1) > 1e-8 {
		t.Fatalf("identity difference area: %g", got)
	}
}

func TestCheckedAPIs(t *testing.T) {
	a := model.NewMeshRect(model.XY(0, 0), model.XY(1, 1))
	b := model.NewMeshRect(model.XY(0.5, 0.5), model.XY(1.5, 1.5))
	for name, operation := range map[string]func() (*model.Mesh, error){
		"union":        func() (*model.Mesh, error) { return UnionChecked(a, b) },
		"intersection": func() (*model.Mesh, error) { return IntersectionChecked(a, b) },
		"difference":   func() (*model.Mesh, error) { return DifferenceChecked(a, b) },
	} {
		t.Run(name, func(t *testing.T) {
			result, err := operation()
			if err != nil {
				t.Fatal(err)
			}
			assertValidMesh(t, result)
		})
	}
}

func TestOptions(t *testing.T) {
	defaults := DefaultOptions()
	if defaults.MaxInputSegments != 10_000 || defaults.MaxIntersectionCuts != 250_000 {
		t.Fatalf("unexpected defaults: %#v", defaults)
	}
	a := model.NewMeshRect(model.XY(0, 0), model.XY(1, 1))
	b := model.NewMeshRect(model.XY(0.5, 0.5), model.XY(1.5, 1.5))
	result, err := UnionCheckedWithOptions(Options{}, a, b)
	if err != nil {
		t.Fatalf("zero options did not use defaults: %v", err)
	}
	assertValidMesh(t, result)

	limited := DefaultOptions()
	limited.MaxInputSegments = a.NumSegments() - 1
	result, err = UnionCheckedWithOptions(limited, a)
	if result != nil {
		t.Fatal("limited operation returned a mesh")
	}
	complexity, ok := err.(*ComplexityError)
	if !ok || complexity.Limit != limited.MaxInputSegments {
		t.Fatalf("low input limit returned %#v", err)
	}

	invalid := DefaultOptions()
	invalid.MaxIntersectionPairs = -1
	if result, err = UnionCheckedWithOptions(invalid, a); result != nil || err == nil {
		t.Fatalf("negative option returned result=%v err=%v", result, err)
	}
}

func TestCheckedAPIRecoversTopology(t *testing.T) {
	want := &TopologyError{Problem: "test topology", Count: 3}
	var err error
	func() {
		defer recoverComplexity(&err)
		panic(want)
	}()
	if err != want {
		t.Fatalf("recovered error: got %#v want %#v", err, want)
	}
}

func TestNaryOperations(t *testing.T) {
	a := model.NewMeshRect(model.XY(0, 0), model.XY(4, 4))
	b := model.NewMeshRect(model.XY(1, 1), model.XY(2, 2))
	c := model.NewMeshRect(model.XY(2.5, 2.5), model.XY(3.5, 3.5))
	difference := Difference(a, b, c)
	assertValidMesh(t, difference)
	if area := difference.Area(); math.Abs(area-14) > 1e-8 {
		t.Fatalf("n-ary difference area: got %g want 14", area)
	}
	intersection := Intersection(a,
		model.NewMeshRect(model.XY(1, 1), model.XY(5, 5)),
		model.NewMeshRect(model.XY(2, 2), model.XY(6, 6)))
	assertValidMesh(t, intersection)
	if area := intersection.Area(); math.Abs(area-4) > 1e-8 {
		t.Fatalf("n-ary intersection area: got %g want 4", area)
	}
}

func assertValidMesh(t *testing.T, m *model.Mesh) {
	t.Helper()
	if !m.Manifold() {
		t.Fatalf("mesh is non-manifold: %v", m.InconsistentVertices())
	}
	if inconsistent := m.InconsistentVertices(); len(inconsistent) != 0 {
		t.Fatalf("mesh has inconsistent vertices: %v", inconsistent)
	}
	segments := m.SegmentSlice()
	for i, first := range segments {
		for _, second := range segments[i+1:] {
			shared, otherFirst, otherSecond, hasShared := sharedSegmentEndpoint(first, second)
			if !hasShared {
				if first.SegmentCollision(second) {
					t.Fatalf("mesh has crossing segments: %v and %v", *first, *second)
				}
				continue
			}
			v1, v2 := otherFirst.Sub(shared), otherSecond.Sub(shared)
			crossingTolerance := 1e-12 * math.Max(v1.Norm()*v2.Norm(), math.SmallestNonzeroFloat64)
			if math.Abs(cross(v1, v2)) <= crossingTolerance && v1.Dot(v2) > 0 {
				t.Fatalf("mesh has overlapping segments: %v and %v", *first, *second)
			}
		}
	}
}

func sharedSegmentEndpoint(first, second *model.Segment) (shared, otherFirst, otherSecond model.Coord, ok bool) {
	for i, point := range first {
		for j, other := range second {
			if point == other {
				return point, first[1-i], second[1-j], true
			}
		}
	}
	return model.Coord{}, model.Coord{}, model.Coord{}, false
}

func randomRadialMesh(rng *rand.Rand, center model.Coord, count int) *model.Mesh {
	points := make([]model.Coord, count)
	phase := rng.Float64() * math.Pi * 2
	for i := range points {
		theta := phase + float64(i)*2*math.Pi/float64(count)
		// Alternating and noisy radii create narrow, ragged spikes while the
		// angular ordering keeps each input simple.
		radius := 0.45 + rng.Float64()*0.55
		if i%2 == 0 {
			radius *= 0.55
		}
		points[i] = center.Add(model.NewCoordPolar(theta, radius))
	}
	mesh := model.NewMesh()
	for i := range points {
		// Descending angular order is clockwise.
		mesh.Add(&model.Segment{points[(i+1)%len(points)], points[i]})
	}
	return mesh
}
