package jel

import (
	"math"

	"github.com/setanarut/v"
)

// Matrix3x3 represents a 2D affine transformation matrix.
//
// The underlying structure is optimized so that its zero-value inherently
// represents the identity matrix. To achieve this, the main diagonal components
type Matrix3x3 struct{ a1, b, c, d1, tx, ty float64 }

// Clear resets the matrix to the identity transformation (zero-value).
func (m *Matrix3x3) Clear() {
	*m = Matrix3x3{}
}

// Apply transforms the given 2D vector 'a' by multiplying it with the matrix.
func (m Matrix3x3) Apply(a v.Vec) v.Vec {
	return v.Vec{
		X: (m.a1+1)*a.X + m.b*a.Y + m.tx,
		Y: m.c*a.X + (m.d1+1)*a.Y + m.ty,
	}
}

// Scale applies a scaling transformation to the matrix.
// This effectively pre-multiplies the current matrix by a scaling matrix.
func (m *Matrix3x3) Scale(x, y float64) {
	a := (m.a1 + 1) * x
	b := m.b * x
	tx := m.tx * x
	c := m.c * y
	d := (m.d1 + 1) * y
	ty := m.ty * y

	m.a1 = a - 1
	m.b = b
	m.c = c
	m.d1 = d - 1
	m.tx = tx
	m.ty = ty
}

// Translate applies a translation offset to the matrix.
func (m *Matrix3x3) Translate(tx, ty float64) {
	m.tx += tx
	m.ty += ty
}

// Rotate applies a counter-clockwise rotation (in radians) to the matrix.
// This effectively pre-multiplies the current matrix by a rotation matrix.
func (m *Matrix3x3) Rotate(theta float64) {
	if theta == 0 {
		return
	}

	sin, cos := math.Sincos(theta)

	a := cos*(m.a1+1) - sin*m.c
	b := cos*m.b - sin*(m.d1+1)
	tx := cos*m.tx - sin*m.ty
	c := sin*(m.a1+1) + cos*m.c
	d := sin*m.b + cos*(m.d1+1)
	ty := sin*m.tx + cos*m.ty

	m.a1 = a - 1
	m.b = b
	m.c = c
	m.d1 = d - 1
	m.tx = tx
	m.ty = ty
}

// NewMatrix3x3 returns a matrix that combines scale, rotation, and translation.
// Transformation order: scale -> rotate -> translate.
func NewMatrix3x3(scale v.Vec, angle float64, pos v.Vec) Matrix3x3 {
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
