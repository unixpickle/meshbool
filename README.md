# meshbool

`meshbool` provides boolean operations for
[`github.com/unixpickle/model3d`](https://github.com/unixpickle/model3d) meshes.
It operates on the input facets rather than voxelizing them, so vertices and
flat surfaces are retained.

```go
import (
    "github.com/unixpickle/meshbool"
    "github.com/unixpickle/model3d/model2d"
    "github.com/unixpickle/model3d/model3d"
)

a := model3d.NewMeshRect(model3d.XYZ(0, 0, 0), model3d.XYZ(2, 2, 2))
b := model3d.NewMeshRect(model3d.XYZ(1, 1, 1), model3d.XYZ(3, 3, 3))
options := meshbool.DefaultOptions3D()

u, err := meshbool.Union3D(options, a, b)
if err != nil {
    return err
}
i, err := meshbool.Intersection3D(options, a, b)
if err != nil {
    return err
}
d, err := meshbool.Difference3D(options, a, b)
if err != nil {
    return err
}

a2D := model2d.NewMeshRect(model2d.XY(0, 0), model2d.XY(2, 2))
b2D := model2d.NewMeshRect(model2d.XY(1, 1), model2d.XY(3, 3))
u2D, err := meshbool.Union2D(meshbool.DefaultOptions2D(), a2D, b2D)
if err != nil {
    return err
}
```

The root package provides `Union2D`, `Intersection2D`, and `Difference2D` for
`*model2d.Mesh` values, and the corresponding `*3D` functions for
`*model3d.Mesh` values.

Inputs should be closed, consistently oriented, self-intersection-free manifold
meshes. The operations do not mutate them. Outputs are welded, conformingly
triangulated, consistently oriented, and regularized to remove surface
fragments that cannot bound a volume. Exact point-only contact is intrinsically
non-manifold, so both APIs separate contacting components by a few
scale-relative tolerance units.

## Resource limits

Intersection arrangements can have output size quadratic or worse in the input
size. Every operation stops at conservative internal expansion limits and
returns `*meshbool.ComplexityError` rather than risking unbounded memory growth.
Residual invalid-output conditions are returned as `*meshbool.TopologyError`.
Both dimensions use the same error types.

The implementation is single-threaded. This makes its cost predictable and
avoids multiplying peak memory use when several facets create dense
intersection arrangements.

The 3-D defaults allow 200,000 triangles in each input mesh and cap retained
surface fragments and output triangles at 200,000. Every guard can be tuned
through the options value:

```go
options := meshbool.DefaultOptions3D()
options.MaxInputTriangles = 500_000
options.MaxOutputTriangles = 500_000

result, err := meshbool.Union3D(options, a, b)
```

By default, an output that fails topology validation is discarded. Diagnostic
tools can opt to retain the completed candidate mesh while still receiving the
`*meshbool.TopologyError`:

```go
options := meshbool.DefaultOptions3D()
options.KeepInvalidOutput = true

result, err := meshbool.Union3D(options, a, b)
if err != nil && result != nil {
    // Inspect or export result, but do not treat it as a valid solid.
}
```

This option applies only after a complete candidate reaches final topology
validation. Complexity limits and earlier construction failures still return a
nil mesh.

Zero-valued option fields inherit defaults; negative values are rejected. The
3-D `PlanarOptions` field configures the 2-D work used to merge coplanar
surfaces. Direct planar operations use `Options2D` and `DefaultOptions2D`.

The ordinary test suite deliberately keeps the expensive 3-D randomized test
bounded. Larger deterministic campaigns are opt-in, for example:

```sh
MESHBOOL_STRESS_TRIALS=100 go test -run TestBooleanIdentitiesIcospheres
```

Choose that value according to the machine's available time and memory; the
error-returning APIs and internal expansion limits still apply.

If OpenSCAD is installed, a small fixed-seed differential check is available
separately. Each external render has a 20-second timeout:

```sh
MESHBOOL_OPENSCAD=1 go test -run TestOpenSCADDifferential
```
