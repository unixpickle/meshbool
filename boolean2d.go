package meshbool

import (
	"fmt"

	"github.com/unixpickle/meshbool/internal/bool2d"
	"github.com/unixpickle/model3d/model2d"
)

// Options2D configures safety limits for 2-D boolean operations. A zero field
// uses the corresponding value from DefaultOptions2D. Negative values are
// invalid.
type Options2D struct {
	MaxInputSegments     int
	MaxIntersectionCuts  int
	MaxIntersectionPairs int
	MaxContactDegree     int
}

// DefaultOptions2D returns the standard limits for 2-D operations.
func DefaultOptions2D() Options2D {
	defaults := bool2d.DefaultOptions()
	return Options2D{
		MaxInputSegments:     defaults.MaxInputSegments,
		MaxIntersectionCuts:  defaults.MaxIntersectionCuts,
		MaxIntersectionPairs: defaults.MaxIntersectionPairs,
		MaxContactDegree:     defaults.MaxContactDegree,
	}
}

// Validate checks that no option is negative. Zero values are valid and mean
// to use the corresponding default.
func (o Options2D) Validate() error {
	fields := []struct {
		name  string
		value int
	}{
		{"MaxInputSegments", o.MaxInputSegments},
		{"MaxIntersectionCuts", o.MaxIntersectionCuts},
		{"MaxIntersectionPairs", o.MaxIntersectionPairs},
		{"MaxContactDegree", o.MaxContactDegree},
	}
	for _, field := range fields {
		if field.value < 0 {
			return fmt.Errorf("meshbool: 2D option %s must not be negative", field.name)
		}
	}
	return nil
}

func (o Options2D) internal() bool2d.Options {
	return bool2d.Options{
		MaxInputSegments:     o.MaxInputSegments,
		MaxIntersectionCuts:  o.MaxIntersectionCuts,
		MaxIntersectionPairs: o.MaxIntersectionPairs,
		MaxContactDegree:     o.MaxContactDegree,
	}
}

// Union2D computes the union of zero or more planar meshes using the supplied
// safety limits. Input meshes are not modified.
func Union2D(options Options2D, meshes ...*model2d.Mesh) (*model2d.Mesh, error) {
	if err := options.Validate(); err != nil {
		return nil, err
	}
	result, err := bool2d.Union(options.internal(), meshes...)
	return result, convert2DError(err)
}

// Intersection2D computes the intersection of zero or more planar meshes
// using the supplied safety limits. With no meshes, it returns an empty mesh.
// Input meshes are not modified.
func Intersection2D(options Options2D, meshes ...*model2d.Mesh) (*model2d.Mesh, error) {
	if err := options.Validate(); err != nil {
		return nil, err
	}
	result, err := bool2d.Intersection(options.internal(), meshes...)
	return result, convert2DError(err)
}

// Difference2D subtracts every mesh in subtract from first using the supplied
// safety limits. Input meshes are not modified.
func Difference2D(options Options2D, first *model2d.Mesh, subtract ...*model2d.Mesh) (*model2d.Mesh, error) {
	if err := options.Validate(); err != nil {
		return nil, err
	}
	result, err := bool2d.Difference(options.internal(), first, subtract...)
	return result, convert2DError(err)
}

func convert2DError(err error) error {
	switch failure := err.(type) {
	case nil:
		return nil
	case *bool2d.ComplexityError:
		return &ComplexityError{Stage: "2D " + failure.Stage, Limit: failure.Limit}
	case *bool2d.TopologyError:
		return &TopologyError{Problem: "2D " + failure.Problem, Count: failure.Count}
	default:
		return fmt.Errorf("meshbool: 2D operation: %w", err)
	}
}
