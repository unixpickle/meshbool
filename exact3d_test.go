package meshbool

import (
	"math/big"
	"math/rand"
	"testing"

	"github.com/unixpickle/model3d/model3d"
)

func TestFilteredPredicatesMatchExact(t *testing.T) {
	rng := rand.New(rand.NewSource(0x51a7))
	for trial := 0; trial < 1000; trial++ {
		var points [4]exactCoord3D
		for i := range points {
			var err error
			points[i], err = exactCoordFromFloat(model3d.XYZ(
				rng.NormFloat64(), rng.NormFloat64(), rng.NormFloat64(),
			))
			if err != nil {
				t.Fatal(err)
			}
		}

		// Exercise both ordinary float input and rational points constructed
		// exactly on a triangle edge, where the filter must fall back.
		parameter := new(big.Rat).SetFrac64(int64(rng.Intn(2001)-500), 1000)
		if trial%2 == 0 {
			points[3] = exactInterpolate(points[0], points[1], parameter)
		}
		if got, want := filteredOrient3DSign(points[0], points[1], points[2], points[3]),
			exactOrient3D(points[0], points[1], points[2], points[3]).Sign(); got != want {
			t.Fatalf("trial %d: filtered 3D sign %d, exact sign %d", trial, got, want)
		}
		for axis := 0; axis < 3; axis++ {
			if got, want := filteredOrient2DSign(points[0], points[1], points[3], axis),
				exactOrient2D(points[0], points[1], points[3], axis).Sign(); got != want {
				t.Fatalf("trial %d axis %d: filtered 2D sign %d, exact sign %d", trial, axis, got, want)
			}
		}
		direction := exactCoordFromFinite(model3d.XYZ(
			float64(rng.Intn(5)+1), float64(rng.Intn(5)+1), float64(rng.Intn(5)+1),
		))
		triangle := exactTriangle3D{points[0], points[1], points[2]}
		if got, want := filteredNormalDotSign(triangle, direction),
			exactNormalDot(triangle, direction).Sign(); got != want {
			t.Fatalf("trial %d: filtered normal dot sign %d, exact sign %d", trial, got, want)
		}

		edgeA := exactInterpolate(points[0], points[1], parameter)
		edgeB := exactInterpolate(points[0], points[2], parameter)
		query := exactInterpolate(edgeA, edgeB,
			new(big.Rat).SetFrac64(int64(rng.Intn(2001)-500), 1000))
		if got, want := filteredPointTriangleLocation(query, triangle),
			exactPointTriangleLocation(query, triangle); got != want {
			t.Fatalf("trial %d: filtered location %d, exact location %d", trial, got, want)
		}
	}
}
