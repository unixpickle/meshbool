package meshbool

import (
	"testing"

	"github.com/unixpickle/model3d/model3d"
)

func BenchmarkUnionBoxes(b *testing.B) {
	a := model3d.NewMeshRect(model3d.XYZ(0, 0, 0), model3d.XYZ(2, 2, 2))
	c := model3d.NewMeshRect(model3d.XYZ(1, 1, 1), model3d.XYZ(3, 3, 3))
	b.ReportAllocs()
	for i := 0; i < b.N; i++ {
		if _, err := Union(a, c); err != nil {
			b.Fatal(err)
		}
	}
}

func BenchmarkUnionIcospheresLevel1(b *testing.B) {
	a := model3d.NewMeshIcosphere(model3d.X(-0.25), 1, 1)
	c := model3d.NewMeshIcosphere(model3d.X(0.25), 1, 1)
	b.ReportAllocs()
	for i := 0; i < b.N; i++ {
		if _, err := Union(a, c); err != nil {
			b.Fatal(err)
		}
	}
}
