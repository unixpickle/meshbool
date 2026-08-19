# meshbool

`meshbool` provides boolean operations for
[`github.com/unixpickle/model3d`](https://github.com/unixpickle/model3d) meshes.
It operates on the input facets rather than voxelizing them, so vertices and
flat surfaces are retained.

```go
import (
    "github.com/unixpickle/meshbool"
    "github.com/unixpickle/model3d/model3d"
)

a := model3d.NewMeshRect(model3d.XYZ(0, 0, 0), model3d.XYZ(2, 2, 2))
b := model3d.NewMeshRect(model3d.XYZ(1, 1, 1), model3d.XYZ(3, 3, 3))

u := meshbool.Union(a, b)
i := meshbool.Intersection(a, b)
d := meshbool.Difference(a, b)
```

The corresponding planar API is in `github.com/unixpickle/meshbool/bool2d`
and has the same `Union`, `Intersection`, and `Difference` names for
`*model2d.Mesh` values.

Inputs should be closed, consistently oriented manifold meshes. The operations
do not mutate them. Outputs are welded, conformingly triangulated, consistently
oriented, and regularized to remove surface fragments that cannot bound a
volume. Exact point-only contact is intrinsically non-manifold, so both APIs
separate contacting components by a few scale-relative tolerance units.

## Resource limits

Intersection arrangements can have output size quadratic or worse in the input
size. The convenience functions stop at conservative internal expansion limits
and panic with `*meshbool.ComplexityError` rather than risking unbounded memory
growth. Servers and other long-running programs should use `UnionChecked`,
`IntersectionChecked`, and `DifferenceChecked`. They return complexity failures
and any residual invalid-output condition (`*meshbool.TopologyError`) as errors
instead of returning a bad mesh. The planar package provides the same checked
variants and error types.

The implementation is single-threaded. This makes its cost predictable and
avoids multiplying peak memory use when several facets create dense
intersection arrangements.

The 3-D defaults allow 200,000 triangles in each input mesh and cap retained
surface fragments and output triangles at 200,000. Every guard can be tuned
without changing the convenience APIs:

```go
options := meshbool.DefaultOptions()
options.MaxInputTriangles = 500_000
options.MaxOutputTriangles = 500_000

result, err := meshbool.UnionCheckedWithOptions(options, a, b)
```

Zero-valued option fields inherit defaults; negative values are rejected. The
3-D `PlanarOptions` field configures the 2-D work used to merge coplanar
surfaces. The planar package also exposes its own `DefaultOptions` and
`*WithOptions` functions.

The ordinary test suite deliberately keeps the expensive 3-D randomized test
bounded. Larger deterministic campaigns are opt-in, for example:

```sh
MESHBOOL_STRESS_TRIALS=100 go test -run TestBooleanIdentitiesIcospheres
```

Choose that value according to the machine's available time and memory; the
checked APIs and internal expansion limits still apply.

If OpenSCAD is installed, a small fixed-seed differential check is available
separately. Each external render has a 20-second timeout:

```sh
MESHBOOL_OPENSCAD=1 go test -run TestOpenSCADDifferential
```
