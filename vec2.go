package jel

import (
	"fmt"
	"math"
)

const epsilonUnit float64 = 1e-8

// Vec2One is a [Vec2] with both components set to 1. Do not modify.
var Vec2One = Vec2{X: 1, Y: 1}

// Vec2 represents a 2D vector with X and Y components.
type Vec2 struct {
	X, Y float64
}

// Add returns this + a
func (v Vec2) Add(a Vec2) Vec2 {
	return Vec2{v.X + a.X, v.Y + a.Y}
}

// Add adds n to v.X
func (v Vec2) AddX(n float64) Vec2 {
	v.X += n
	return v
}

// Add adds n to v.Y
func (v Vec2) AddY(n float64) Vec2 {
	v.Y += n
	return v
}

// Sub returns this - a
func (v Vec2) Sub(a Vec2) Vec2 {
	return Vec2{v.X - a.X, v.Y - a.Y}
}

// Div divides this vector by a.
func (v Vec2) Div(a Vec2) Vec2 {
	return Vec2{v.X / a.X, v.Y / a.Y}
}

// DivS divides this vector by scalar value s.
func (v Vec2) DivS(s float64) Vec2 {
	return Vec2{v.X / s, v.Y / s}
}

// Mul returns this * a
func (v Vec2) Mul(a Vec2) Vec2 {
	return Vec2{v.X * a.X, v.Y * a.Y}
}

// Scale scales vector
func (v Vec2) Scale(s float64) Vec2 {
	return Vec2{v.X * s, v.Y * s}
}

// Unit returns a normalized copy of this vector (unit vector).
func (v Vec2) Unit() Vec2 {
	sl := v.MagSq()
	if sl < epsilonUnit {
		return v
	}

	if math.Abs(sl-1) < epsilonUnit {
		return v
	}

	return v.Scale(1.0 / math.Sqrt(sl))
}

// Abs returns the absolute value of vector.
func (v Vec2) Abs() Vec2 {
	return Vec2{math.Abs(v.X), math.Abs(v.Y)}
}

// AbsX returns the absolute X value of vector.
func (v Vec2) AbsX() float64 {
	return math.Abs(v.X)
}

// AbsY returns the absolute Y value of vector.
func (v Vec2) AbsY() float64 {
	return math.Abs(v.Y)
}

// Neg negates a vector.
func (v Vec2) Neg() Vec2 {
	return Vec2{-v.X, -v.Y}
}

// NegY negates X.
func (v Vec2) NegX() Vec2 {
	return Vec2{-v.X, v.Y}
}

// NegY negates Y.
func (v Vec2) NegY() Vec2 {
	return Vec2{v.X, -v.Y}
}

// Perp returns the perpendicular vector rotated 90 degrees counter-clockwise.
func (v Vec2) Perp() Vec2 {
	return Vec2{-v.Y, v.X}
}

// Dot returns dot product
func (v Vec2) Dot(other Vec2) float64 {
	return v.X*other.X + v.Y*other.Y
}

// Cross calculates the 2D vector cross product analog.
// The cross product of 2D vectors results in a 3D vector with only a z component.
// This function returns the magnitude of the z value.
func (v Vec2) Cross(other Vec2) float64 {
	return v.X*other.Y - v.Y*other.X
}

// Returns the vector projection onto other.
func (v Vec2) Project(other Vec2) Vec2 {
	return other.Scale(v.Dot(other) / other.Dot(other))
}

// Angle returns the angular direction v is pointing in (in radians).
func (v Vec2) Angle() float64 {
	return math.Atan2(v.Y, v.X)
}

// Rotate a vector by an angle in radians
func (v Vec2) Rotate(angle float64) Vec2 {
	return Vec2{
		X: v.X*math.Cos(angle) - v.Y*math.Sin(angle),
		Y: v.X*math.Sin(angle) + v.Y*math.Cos(angle),
	}
}

// Mag returns the magnitude (length) of the vector.
func (v Vec2) Mag() float64 {
	return math.Hypot(v.X, v.Y)
}

// SetMag sets the magnitude (length) of the vector.
func (v Vec2) SetMag(m float64) Vec2 {
	if mag := v.Mag(); mag != 0 {
		return v.Scale(m / mag)
	}
	return v
}

// MagSq returns the magnitude (length) of the vector, squared.
//
// This method is often used to improve performance since, unlike Mag(),
// it does not require a Sqrt() operation.
func (v Vec2) MagSq() float64 {
	return v.X*v.X + v.Y*v.Y
}

// Slerp performs spherical linear interpolation between two vectors with given weight value in [0,1] range, returning interpolated vector
func (v Vec2) Slerp(to Vec2, weight float64) Vec2 {
	startLengthSq := v.MagSq()
	endLengthSq := to.MagSq()
	if startLengthSq == 0.0 || endLengthSq == 0.0 {
		return v.Lerp(to, weight)
	}
	startLength := math.Sqrt(startLengthSq)
	resultLength := (1-weight)*startLength + weight*math.Sqrt(endLengthSq)
	angle := v.AngleTo(to)
	return v.Rotate(angle * weight).Scale(resultLength / startLength)
}

// AngleTo returns the angle to the given vector, in radians.
func (v Vec2) AngleTo(other Vec2) float64 {
	return math.Atan2(v.Cross(other), v.Dot(other))
}

// Limits a vector's magnitude to a maximum value.
func (v Vec2) Limit(max float64) Vec2 {
	if v.Dot(v) > max*max {
		return v.Unit().Scale(max)
	}
	return v
}

// Lerp linearly interpolates between this and other vector.
func (v Vec2) Lerp(other Vec2, t float64) Vec2 {
	return v.Scale(1.0 - t).Add(other.Scale(t))
}

// IsZero returns true if vector is zero vector
func (v Vec2) IsZero() bool {
	return v == Vec2{}
}

// Dist returns distance between v and other.
func (v Vec2) Dist(other Vec2) float64 {
	return math.Hypot(v.X-other.X, v.Y-other.Y)
}

// DistSq returns the squared distance between this and other.
//
// Faster than v.Dist() when you only need to compare distances.
func (v Vec2) DistSq(other Vec2) float64 {
	return v.Sub(other).MagSq()
}

// Round returns the nearest integer Vector, rounding half away from zero.
func (v Vec2) Round() Vec2 {
	return Vec2{math.Round(v.X), math.Round(v.Y)}
}

// Floor returns vector with all components rounded down (towards negative infinity).
func (v Vec2) Floor() Vec2 {
	return Vec2{math.Floor(v.X), math.Floor(v.Y)}
}

// Ceil returns vector with all components rounded up (towards positive infinity).
func (v Vec2) Ceil() Vec2 {
	return Vec2{math.Ceil(v.X), math.Ceil(v.Y)}
}

// FromAngle makes a new 2D unit vector from an angle
func FromAngle(angle float64) Vec2 {
	return Vec2{math.Cos(angle), math.Sin(angle)}
}

// EqualsP returns they are practically equal with each other within a delta tolerance.
func (v Vec2) EqualsPr(other Vec2, allowedDelta float64) bool {
	return (math.Abs(v.X-other.X) <= allowedDelta) &&
		(math.Abs(v.Y-other.Y) <= allowedDelta)
}

// Equals checks if two vectors are equal. (Be careful when comparing floating point numbers!)
func (v Vec2) Equals(other Vec2) bool {
	return v.X == other.X && v.Y == other.Y
}

// Reflect returns the reflection of the vector v over the given normal.
// normal should be a normalized (unit) vector.
func (v Vec2) Reflect(normal Vec2) Vec2 {
	return v.Sub(normal.Scale(2 * v.Dot(normal)))
}

// String returns string representation of this vector.
func (v Vec2) String() string {
	return fmt.Sprintf("(%.1f, %.1f)", v.X, v.Y)
}

// Min returns the component-wise minimum of two vectors.
func (v Vec2) Min(other Vec2) Vec2 {
	return Vec2{X: min(v.X, other.X), Y: min(v.Y, other.Y)}
}

// Max returns the component-wise maximum of two vectors.
func (v Vec2) Max(other Vec2) Vec2 {
	return Vec2{X: max(v.X, other.X), Y: max(v.Y, other.Y)}
}
