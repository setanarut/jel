package jel

import (
	"math"
)

// Matrix3x3 represents a 2D affine transformation matrix.
//
// The underlying structure is optimized so that its zero-value inherently
// represents the identity matrix. To achieve this, the main diagonal components
type Matrix3x3 struct{ a1, b, c, d1, tx, ty float64 }

// Clear resets the matrix to the identity transformation (zero-value).
func (g *Matrix3x3) Clear() {
	*g = Matrix3x3{}
}

// Apply transforms the given 2D vector 'v' by multiplying it with the matrix.
func (g Matrix3x3) Apply(v Vec2) Vec2 {
	return Vec2{
		X: (g.a1+1)*v.X + g.b*v.Y + g.tx,
		Y: g.c*v.X + (g.d1+1)*v.Y + g.ty,
	}
}

// Scale applies a scaling transformation to the matrix.
// This effectively pre-multiplies the current matrix by a scaling matrix.
func (g *Matrix3x3) Scale(x, y float64) {
	a := (g.a1 + 1) * x
	b := g.b * x
	tx := g.tx * x
	c := g.c * y
	d := (g.d1 + 1) * y
	ty := g.ty * y

	g.a1 = a - 1
	g.b = b
	g.c = c
	g.d1 = d - 1
	g.tx = tx
	g.ty = ty
}

// Translate applies a translation offset to the matrix.
func (g *Matrix3x3) Translate(tx, ty float64) {
	g.tx += tx
	g.ty += ty
}

// Rotate applies a counter-clockwise rotation (in radians) to the matrix.
// This effectively pre-multiplies the current matrix by a rotation matrix.
func (g *Matrix3x3) Rotate(theta float64) {
	if theta == 0 {
		return
	}

	sin, cos := math.Sincos(theta)

	a := cos*(g.a1+1) - sin*g.c
	b := cos*g.b - sin*(g.d1+1)
	tx := cos*g.tx - sin*g.ty
	c := sin*(g.a1+1) + cos*g.c
	d := sin*g.b + cos*(g.d1+1)
	ty := sin*g.tx + cos*g.ty

	g.a1 = a - 1
	g.b = b
	g.c = c
	g.d1 = d - 1
	g.tx = tx
	g.ty = ty
}

// NewMatrix3x3 returns a matrix that combines scale, rotation, and translation.
// Transformation order: scale -> rotate -> translate.
func NewMatrix3x3(scale Vec2, angle float64, pos Vec2) Matrix3x3 {
	// Fast path: no rotation
	if angle == 0 {
		// Fast path: no scale either (only translation)
		if scale.X == 1 && scale.Y == 1 {
			return Matrix3x3{
				tx: pos.X,
				ty: pos.Y,
			}
		}
		return Matrix3x3{
			a1: scale.X - 1,
			d1: scale.Y - 1,
			tx: pos.X,
			ty: pos.Y,
		}
	}

	sin, cos := math.Sincos(angle)

	// Fast path: unit scale (only rotation + translation)
	if scale.X == 1 && scale.Y == 1 {
		return Matrix3x3{
			a1: cos - 1,
			b:  -sin,
			c:  sin,
			d1: cos - 1,
			tx: pos.X,
			ty: pos.Y,
		}
	}

	// Full transformation: scale * rotation * translation
	return Matrix3x3{
		a1: scale.X*cos - 1,
		b:  -scale.Y * sin,
		c:  scale.X * sin,
		d1: scale.Y*cos - 1,
		tx: pos.X,
		ty: pos.Y,
	}
}
