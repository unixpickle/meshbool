package meshbool

import (
	"context"
	"fmt"
	"math"
	"math/rand"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"testing"
	"time"

	"github.com/unixpickle/model3d/model3d"
)

// TestOpenSCADDifferential is opt-in because invoking an external CAD kernel
// is much heavier than the ordinary bounded unit suite. The fixed inputs are
// nevertheless small (20 triangles each), and every invocation has a timeout.
func TestOpenSCADDifferential(t *testing.T) {
	if os.Getenv("MESHBOOL_OPENSCAD") != "1" {
		t.Skip("set MESHBOOL_OPENSCAD=1 to compare against OpenSCAD")
	}
	openscad, err := exec.LookPath("openscad")
	if err != nil {
		t.Skip("OpenSCAD is not installed")
	}
	rng := rand.New(rand.NewSource(7719283))
	a := randomRaggedIcosphere(rng, model3d.X(-0.22))
	b := randomRaggedIcosphere(rng, model3d.X(0.22))
	directory := t.TempDir()
	aPath, bPath := filepath.Join(directory, "a.stl"), filepath.Join(directory, "b.stl")
	if err := a.SaveGroupedSTL(aPath); err != nil {
		t.Fatal(err)
	}
	if err := b.SaveGroupedSTL(bPath); err != nil {
		t.Fatal(err)
	}
	tests := []struct {
		name       string
		expression string
		mesh       *model3d.Mesh
	}{
		{"union", "union", Union(a, b)},
		{"intersection", "intersection", Intersection(a, b)},
		{"difference", "difference", Difference(a, b)},
	}
	for _, test := range tests {
		t.Run(test.name, func(t *testing.T) {
			scriptPath := filepath.Join(directory, test.name+".scad")
			outputPath := filepath.Join(directory, test.name+".stl")
			script := fmt.Sprintf("render() %s() { import(%s); import(%s); }\n",
				test.expression, strconv.Quote(aPath), strconv.Quote(bPath))
			if err := os.WriteFile(scriptPath, []byte(script), 0o600); err != nil {
				t.Fatal(err)
			}
			ctx, cancel := context.WithTimeout(context.Background(), 20*time.Second)
			defer cancel()
			command := exec.CommandContext(ctx, openscad, "-o", outputPath, scriptPath)
			if output, err := command.CombinedOutput(); err != nil {
				if ctx.Err() != nil {
					t.Fatalf("OpenSCAD timed out: %v", ctx.Err())
				}
				t.Fatalf("OpenSCAD failed: %v\n%s", err, output)
			}
			oracle := readSTLMesh(t, outputPath).Repair(1e-6)
			assertValidMesh3D(t, oracle)
			want, got := oracle.Volume(), test.mesh.Volume()
			tolerance := 2e-4 * math.Max(1, math.Abs(want))
			if math.Abs(got-want) > tolerance {
				t.Fatalf("volume differs from OpenSCAD: got %g want %g (tolerance %g)", got, want, tolerance)
			}
		})
	}
}

func readSTLMesh(t *testing.T, path string) *model3d.Mesh {
	t.Helper()
	file, err := os.Open(path)
	if err != nil {
		t.Fatal(err)
	}
	defer file.Close()
	triangles, err := model3d.ReadSTL(file)
	if err != nil {
		t.Fatal(err)
	}
	return model3d.NewMeshTriangles(triangles)
}
