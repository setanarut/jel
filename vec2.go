package jel

import (
	"fmt"
	"math"
)

const epsilon = 1e-8

// +Y coordinate system (Y increases downward).
// Angles are measured clockwise from the positive X axis.

var (
	// One is a vector with all components set to 1.
	One = Vec2{1, 1}
	// Left unit vector.
	Left = Vec2{-1, 0}
	// Right unit vector.
	Right = Vec2{1, 0}
	// Up unit vector (points in the -Y direction).
	Up = Vec2{0, -1}
	// Down unit vector (points in the +Y direction).
	Down = Vec2{0, 1}
)

type Vec2 struct {
	X, Y float64
}

// Add returns this + a.
func (v Vec2) Add(a Vec2) Vec2 {
	return Vec2{v.X + a.X, v.Y + a.Y}
}

// Sub returns this - a.
func (v Vec2) Sub(a Vec2) Vec2 {
	return Vec2{v.X - a.X, v.Y - a.Y}
}

// Div divides this vector by a component‑wise.
func (v Vec2) Div(a Vec2) Vec2 {
	return Vec2{v.X / a.X, v.Y / a.Y}
}

// DivS divides this vector by scalar s.
func (v Vec2) DivS(s float64) Vec2 {
	return Vec2{v.X / s, v.Y / s}
}

// Mul returns this * a component‑wise.
func (v Vec2) Mul(a Vec2) Vec2 {
	return Vec2{v.X * a.X, v.Y * a.Y}
}

// Scale scales this vector by scalar s.
func (v Vec2) Scale(s float64) Vec2 {
	return Vec2{v.X * s, v.Y * s}
}

// Unit returns a normalized copy of this vector.
// Uses epsilon = 1e‑8 for numerical stability:
//   - vectors with squared length < 1e‑8 are considered zero and left unchanged
//   - vectors with squared length within 1e‑8 of 1.0 are considered already normalized
func (v Vec2) Unit() Vec2 {
	sl := v.MagSq()
	if sl < epsilon {
		return v
	}
	if math.Abs(sl-1) < epsilon {
		return v
	}
	return v.Scale(1.0 / math.Sqrt(sl))
}

// Abs returns the absolute value of each component.
func (v Vec2) Abs() Vec2 {
	return Vec2{math.Abs(v.X), math.Abs(v.Y)}
}

// AbsX returns the absolute value of X.
func (v Vec2) AbsX() float64 {
	return math.Abs(v.X)
}

// AbsY returns the absolute value of Y.
func (v Vec2) AbsY() float64 {
	return math.Abs(v.Y)
}

// Neg returns the negated vector.
func (v Vec2) Neg() Vec2 {
	return Vec2{-v.X, -v.Y}
}

// NegX negates the X component.
func (v Vec2) NegX() Vec2 {
	return Vec2{-v.X, v.Y}
}

// NegY negates the Y component.
func (v Vec2) NegY() Vec2 {
	return Vec2{v.X, -v.Y}
}

// +Y coordinate system (Y increases downward).
// Angles are measured clockwise from positive X.

// Perp returns the perpendicular vector rotated 90° clockwise.
// In +Y-down system: (x,y) -> (y,-x)
// This is the "right-hand" normal for CCW winding order.
func (v Vec2) Perp() Vec2 {
	return Vec2{v.Y, -v.X}
}

// PerpNeg returns the perpendicular vector rotated 90° counter-clockwise.
// In +Y-down system: (x,y) -> (-y,x)
func (v Vec2) PerpNeg() Vec2 {
	return Vec2{-v.Y, v.X}
}

// Dot returns the dot product.
func (v Vec2) Dot(other Vec2) float64 {
	return v.X*other.X + v.Y*other.Y
}

// Cross returns the 2D analog of the cross product (z‑component magnitude).
func (v Vec2) Cross(other Vec2) float64 {
	return v.X*other.Y - v.Y*other.X
}

// Project returns the projection of v onto other.
func (v Vec2) Project(other Vec2) Vec2 {
	return other.Scale(v.Dot(other) / other.Dot(other))
}

// Angle returns the direction angle of v in radians,
// measured clockwise from the positive X axis.
func (v Vec2) Angle() float64 {
	return math.Atan2(v.Y, v.X)
}

// Rotate rotates v by the given angle (radians) clockwise.
// Positive angle = clockwise rotation, negative = counter‑clockwise.
func (v Vec2) Rotate(angle float64) Vec2 {
	// This is the standard CCW rotation matrix, which corresponds to
	// clockwise rotation in the +Y‑down system.
	return Vec2{
		X: v.X*math.Cos(angle) - v.Y*math.Sin(angle),
		Y: v.X*math.Sin(angle) + v.Y*math.Cos(angle),
	}
}

// Mag returns the magnitude (length) of v.
func (v Vec2) Mag() float64 {
	return math.Hypot(v.X, v.Y)
}

// SetMag returns a copy of v with its magnitude set to m.
func (v Vec2) SetMag(m float64) Vec2 {
	if mag := v.Mag(); mag != 0 {
		return v.Scale(m / mag)
	}
	return v
}

// MagSq returns the squared magnitude (length) of v.
func (v Vec2) MagSq() float64 {
	return v.X*v.X + v.Y*v.Y
}

// Slerp performs spherical linear interpolation between v and to
// with weight in [0,1]. It respects the clockwise angle convention.
func (v Vec2) Slerp(to Vec2, weight float64) Vec2 {
	startLengthSq := v.MagSq()
	endLengthSq := to.MagSq()
	if startLengthSq == 0.0 || endLengthSq == 0.0 {
		return v.Lerp(to, weight)
	}
	startLength := math.Sqrt(startLengthSq)
	resultLength := (1-weight)*startLength + weight*math.Sqrt(endLengthSq)
	angle := v.AngleTo(to) // now returns clockwise angle difference
	return v.Rotate(angle * weight).Scale(resultLength / startLength)
}

// AngleTo returns the angle (radians) to rotate v by — using Rotate —
// to align it with other. Positive = clockwise on screen (+Y-down),
// matching Ebitengine's GeoM.Rotate convention.
func (v Vec2) AngleTo(other Vec2) float64 {
	return math.Atan2(v.Cross(other), v.Dot(other))
}

// Limit clamps the magnitude of v to max.
func (v Vec2) Limit(max float64) Vec2 {
	if sl := v.MagSq(); sl > max*max {
		return v.Scale(max / math.Sqrt(sl))
	}
	return v
}

// Lerp linearly interpolates between v and other.
func (v Vec2) Lerp(other Vec2, t float64) Vec2 {
	return v.Scale(1.0 - t).Add(other.Scale(t))
}

// IsZero returns true if v is the zero vector.
func (v Vec2) IsZero() bool {
	return v == Vec2{}
}

// Dist returns the Euclidean distance between v and other.
func (v Vec2) Dist(other Vec2) float64 {
	return math.Hypot(v.X-other.X, v.Y-other.Y)
}

// DistSq returns the squared distance between v and other.
func (v Vec2) DistSq(other Vec2) float64 {
	return v.Sub(other).MagSq()
}

// Round returns a vector with each component rounded to the nearest integer,
// rounding half away from zero.
func (v Vec2) Round() Vec2 {
	return Vec2{math.Round(v.X), math.Round(v.Y)}
}

// Floor returns a vector with each component rounded down.
func (v Vec2) Floor() Vec2 {
	return Vec2{math.Floor(v.X), math.Floor(v.Y)}
}

// Ceil returns a vector with each component rounded up.
func (v Vec2) Ceil() Vec2 {
	return Vec2{math.Ceil(v.X), math.Ceil(v.Y)}
}

// FromAngle creates a unit vector from the given angle (clockwise from +X).
func FromAngle(angle float64) Vec2 {
	return Vec2{math.Cos(angle), math.Sin(angle)}
}

// EqualsPr returns true if v and other are practically equal within delta.
func (v Vec2) EqualsPr(other Vec2, allowedDelta float64) bool {
	return (math.Abs(v.X-other.X) <= allowedDelta) &&
		(math.Abs(v.Y-other.Y) <= allowedDelta)
}

// Equals checks exact equality (caution with floating point values).
func (v Vec2) Equals(other Vec2) bool {
	return v.X == other.X && v.Y == other.Y
}

// Reflect returns the reflection of v over the given normal (which should be unit).
func (v Vec2) Reflect(normal Vec2) Vec2 {
	return v.Sub(normal.Scale(2 * v.Dot(normal)))
}

// String returns a string representation of v.
func (v Vec2) String() string {
	return fmt.Sprintf("(%.1f, %.1f)", v.X, v.Y)
}
