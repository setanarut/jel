// Jel is a 2D soft-body physics library for games.
package jel

import (
	"math"
)

const (

	// Pi is a convenience re-export of math.Pi.
	Pi float64 = math.Pi

	// Tau (τ) is two times pi , representing a full circle in radians. https://oeis.org/A019692
	Tau = 6.2831853071795864769252867665590057683943387987502

	// epsilonSpring is the minimum distance between two anchor points below which
	// a spring force is not computed, to avoid division by (near) zero.
	epsilonSpring float64 = 5e-07
)

// Infinity represents positive infinity, commonly used to mark a PointMass
// as having infinite mass (i.e. immovable / static).
var Infinity = math.Inf(1)

// Bitmask is a 64-bit flag set, typically used for collision layers/masks.
type Bitmask uint64

// SetOn sets the bit corresponding to the given 1-based index.
// Indices less than 1 are clamped to bit 0.
func (b *Bitmask) SetOn(index int) {
	*b |= 1 << max(0, index-1)
}

// CollisionInfo describes a single detected collision between a point mass
// on BodyA and either a point mass or an edge on BodyB.
type CollisionInfo struct {
	// BodyA is the body owning the colliding point mass (index BodyApm).
	BodyA *Body
	// BodyApm is the index of the colliding point mass on BodyA.
	BodyApm int
	// BodyB is the other body involved in the collision.
	BodyB *Body
	// BodyBpmA is the index of the first point mass of the edge on BodyB
	// involved in the collision, or -1 if the collision is against a single
	// point mass rather than an edge.
	BodyBpmA int
	// BodyBpmB is the index of the second point mass of the edge on BodyB
	// involved in the collision, or -1 if not applicable.
	BodyBpmB int
	// HitPt is the world-space point at which the collision occurred.
	HitPt Vec2
	// EdgeD is the interpolation factor (0-1) along the BodyB edge
	// (BodyBpmA -> BodyBpmB) at which the hit point lies.
	EdgeD float64
	// Normal is the collision normal, typically pointing from BodyB toward BodyA.
	Normal Vec2
	// Penetration is the overlap depth between the colliding shapes.
	Penetration float64
}

// NewCollisionInfo creates a CollisionInfo representing a collision between a
// point mass on bodyA and bodyB as a whole (not a specific edge). BodyBpmA
// and BodyBpmB are set to -1 to indicate no edge is involved.
func NewCollisionInfo(bodyA *Body, bodyApm int, bodyB *Body) CollisionInfo {
	return CollisionInfo{
		BodyA:    bodyA,
		BodyApm:  bodyApm,
		BodyB:    bodyB,
		BodyBpmA: -1,
		BodyBpmB: -1,
	}
}

// NewCollisionInfoWithEdge creates a CollisionInfo representing a collision
// between a point mass on bodyA and a specific edge on bodyB, defined by the
// point mass indices bodyBpmA and bodyBpmB.
func NewCollisionInfoWithEdge(bodyA *Body, bodyApm int, bodyB *Body, bodyBpmA, bodyBpmB int) CollisionInfo {
	return CollisionInfo{
		BodyA:    bodyA,
		BodyApm:  bodyApm,
		BodyB:    bodyB,
		BodyBpmA: bodyBpmA,
		BodyBpmB: bodyBpmB,
	}
}

// Resolver is implemented by anything that can advance its own state by a
// fixed time step, such as a physics world or constraint solver.
type Resolver interface {
	// Resolve advances the implementation's state by dt seconds.
	Resolve(dt float64)
}

// CollisionObserver receives notifications about collisions detected during
// a simulation step, allowing external code to react to them (e.g. play a
// sound, apply damage, trigger gameplay logic).
type CollisionObserver interface {
	// BodiesDidCollide is called once per step with all collisions detected
	// during that step.
	BodiesDidCollide(infos []CollisionInfo)
	// BodyCollision is called for an individual collision, allowing the
	// observer to react only when the penetration exceeds penetrationThreshold.
	BodyCollision(info CollisionInfo, penetrationThreshold float64)
}

// UnimplementedCollisionObserver is a no-op CollisionObserver that can be
// embedded to satisfy the CollisionObserver interface without implementing
// every method.
type UnimplementedCollisionObserver struct{}

// BodiesDidCollide is a no-op implementation of CollisionObserver.
func (u *UnimplementedCollisionObserver) BodiesDidCollide(infos []CollisionInfo) {}

// BodyCollision is a no-op implementation of CollisionObserver.
func (u *UnimplementedCollisionObserver) BodyCollision(info CollisionInfo, penetrationThreshold float64) {
}

// polygonAreaFromPointMasses computes the signed area of the polygon formed
// by the positions of points, using the shoelace formula. The sign indicates
// winding order (positive for one orientation, negative for the other).
// Returns 0 for an empty slice.
func polygonAreaFromPointMasses(points []*PointMass) float64 {
	if len(points) == 0 {
		return 0
	}
	v2 := points[len(points)-1].Position
	var area float64 = 0
	for _, p := range points {
		area -= v2.Cross(p.Position)
		v2 = p.Position
	}
	return area / 2
}

// LineIntersectResult holds the outcome of a successful LineIntersect call.
type LineIntersectResult struct {
	// HitPt is the world-space point where the two line segments intersect.
	HitPt Vec2
	// Ua is the interpolation factor (0-1) along lineA at which the
	// intersection occurs.
	Ua float64
	// Ub is the interpolation factor (0-1) along lineB at which the
	// intersection occurs.
	Ub float64
}

// LineIntersect computes the intersection point of two finite line segments,
// lineA and lineB. It returns the intersection details and true if the
// segments intersect within their bounds; otherwise it returns a zero value
// and false (including the degenerate case where the segments are parallel
// or collinear).
func LineIntersect(aStart, aEnd, bStart, bEnd Vec2) (LineIntersectResult, bool) {
	r := aEnd.Sub(aStart)
	s := bEnd.Sub(bStart)
	denom := r.Cross(s)
	if math.Abs(denom) < math.SmallestNonzeroFloat64 {
		return LineIntersectResult{}, false
	}
	w := aStart.Sub(bStart)
	ua := s.Cross(w) / denom
	if ua < 0 || ua > 1 {
		return LineIntersectResult{}, false
	}
	ub := r.Cross(w) / denom
	if ub < 0 || ub > 1 {
		return LineIntersectResult{}, false
	}
	hitPt := aStart.Add(r.Scale(ua))
	return LineIntersectResult{HitPt: hitPt, Ua: ua, Ub: ub}, true
}

// MaterialPair represents information about the collision response behavior between two bodies
type MaterialPair struct {
	//  Whether the collision between the two bodies should happen
	Collide bool
	// The elasticity of the point mass when bouncing off the bodies
	Elasticity float64
	// The relative friction between the two bodies
	Friction float64
	// A function to call and utilize as a collision filter when figuring out
	// whether the two bodies should collide
	CollisionFilterFunc func(info CollisionInfo, normalVelocity float64) bool
}

// DefaultMaterialPair returns default MaterialPair
//
//	Friction:        0.3
//	Elasticity:      0.2
func DefaultMaterialPair() MaterialPair {
	return MaterialPair{
		Collide:             true,
		Friction:            0.3,
		Elasticity:          0.2,
		CollisionFilterFunc: DefaultCollisionFilterFunc,
	}
}

// DefaultCollisionFilterFunc is the default collision filter. It always
// returns true, so all collisions passed through it are approved.
func DefaultCollisionFilterFunc(info CollisionInfo, normalVelocity float64) bool {
	return true
}

// PointMass represents a single simulated particle with mass, position,
// velocity, and any accumulated force to apply on the next integration step.
// Point masses are the fundamental building blocks that soft/rigid Bodies.
type PointMass struct {
	// Mass is the mass of the point. Use Infinity (or math.Inf(1)) to mark
	// the point as immovable/static; Integrate becomes a no-op in that case.
	Mass float64
	// Position is the current world-space position of the point.
	Position Vec2
	// Velocity is the current velocity of the point.
	Velocity Vec2
	// Force is the force currently accumulated on the point, to be applied
	// during the next call to Integrate. It is reset to zero after each
	// integration step.
	Force Vec2
	// Normal is an optional surface normal associated with the point,
	// typically used for collision response.
	Normal Vec2
}

// NewPointMass creates a new PointMass with the given mass and initial
// position. Velocity, Force, and Normal are left at their zero values.
func NewPointMass(mass float64, position Vec2) *PointMass {
	return &PointMass{
		Mass:     mass,
		Position: position,
	}
}

// Integrate advances the point mass's velocity and position by elapsed
// seconds using simple semi-implicit (symplectic) Euler integration:
// the accumulated Force is converted to acceleration (Force / Mass),
// applied to Velocity, and Velocity is then applied to Position. The
// accumulated Force is cleared afterward.
//
// If Mass is infinite or NaN (i.e. the point is static/immovable), this
// is a no-op and the point's position and velocity are left unchanged.
func (p *PointMass) Integrate(elapsed float64) {
	if math.IsInf(p.Mass, 0) || math.IsNaN(p.Mass) {
		return
	}
	elapsedMass := elapsed / p.Mass
	p.Velocity = p.Velocity.Add(p.Force.Scale(elapsedMass))
	p.Position = p.Position.Add(p.Velocity.Scale(elapsed))
	p.Force = Vec2{}
}

// ApplyForce accumulates force into the point mass's current Force, to be
// applied on the next call to Integrate.
func (p *PointMass) ApplyForce(force Vec2) {
	p.Force = p.Force.Add(force)
}

// AveragePointMassPosition returns the centroid (mean position) of pointMasses.
// Returns the zero Vec2 if pointMasses is empty.
func AveragePointMassPosition(pointMasses []*PointMass) (centroid Vec2) {
	if len(pointMasses) == 0 {
		return centroid
	}
	for i := range pointMasses {
		centroid = centroid.Add(pointMasses[i].Position)
	}
	return centroid.DivS(float64(len(pointMasses)))
}

// AveragePointMassVelocity returns the mean velocity across pointMasses.
// Returns the zero Vec2 if pointMasses is empty.
func AveragePointMassVelocity(pointMasses []*PointMass) (average Vec2) {
	if len(pointMasses) == 0 {
		return average
	}
	for i := range pointMasses {
		average = average.Add(pointMasses[i].Velocity)
	}
	return average.DivS(float64(len(pointMasses)))
}

// AverageVec2 returns the mean of the given vectors.
// Returns the zero Vec2 if vectors is empty.
func AverageVec2(vectors []Vec2) (average Vec2) {
	if len(vectors) == 0 {
		return Vec2{}
	}
	for i := range vectors {
		average = average.Add(vectors[i])
	}
	return average.DivS(float64(len(vectors)))
}
