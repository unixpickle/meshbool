package bool2d

import (
	"testing"

	model "github.com/unixpickle/model3d/model2d"
)

func TestInvalidSegmentIntersectionNearParallel(t *testing.T) {
	// The float64 determinants for these ULP-separated segments cancel even
	// though their exact represented coordinates intersect.
	a := &model.Segment{
		model.XY(-5.030340259541398e19, -9.411951120872168e19),
		model.XY(6.695531074873695e20, 3.6445370774187606e20),
	}
	b := &model.Segment{
		model.XY(2.6369998227835457e20, 1.059113851519825e20),
		model.XY(8.925481662358787e20, 5.0650918750494956e20),
	}
	if !invalidSegmentIntersection(a, b) {
		t.Fatal("failed to detect exact near-parallel segment intersection")
	}
}
