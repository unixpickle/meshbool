package model2d

import (
	"math"
	"testing"

	model "github.com/unixpickle/model3d/model2d"
)

func BenchmarkUnionRagged64(b *testing.B) {
	a := benchmarkRaggedMesh(model.X(-0.15), 64, 0)
	c := benchmarkRaggedMesh(model.X(0.15), 64, 0.17)
	b.ReportAllocs()
	for i := 0; i < b.N; i++ {
		Union(a, c)
	}
}

func benchmarkRaggedMesh(center model.Coord, count int, phase float64) *model.Mesh {
	points := make([]model.Coord, count)
	for i := range points {
		angle := phase + 2*math.Pi*float64(i)/float64(count)
		radius := 1.0
		if i%2 == 0 {
			radius = 0.57
		}
		points[i] = center.Add(model.NewCoordPolar(angle, radius))
	}
	result := model.NewMesh()
	for i := range points {
		result.Add(&model.Segment{points[(i+1)%count], points[i]})
	}
	return result
}
