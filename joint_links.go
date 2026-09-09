package jel

import "math"

// Interface to be implemented by objects that specify the way a joint links with a body
type JointLink interface {
	// Gets the body that this joint link is linked to.
	Body() *Body
	// Gets the position, in world coordinates, at which this joint links with
	// the underlying body
	Position() Vec2
	// Gets the velocity of the object this joint links to
	Velocity() Vec2
	// Gets the total mass of the subject of this joint link
	Mass() float64
	// Gets a value specifying whether the object referenced by this
	// JointLinkType is static
	IsStatic() bool
	// Applies a given force to the subject of this joint link.
	ApplyForce(force Vec2)
	// Applies a direct positional translation of this joint link by a given offset.
	Translate(offset Vec2)
	// AddVelocity adds a velocity delta to the subject(s) of this joint
	// link, distributing it the same way ApplyForce does (e.g. across
	// multiple point masses for Edge/Shape links).
	AddVelocity(velocity Vec2)
}

type baseJointLink struct {
	body *Body
}

func (b *baseJointLink) Body() *Body {
	return b.body
}

// ############ BodyJointLink ##############

// BodyJointLink represents a joint link that links to a while body
type BodyJointLink struct {
	baseJointLink
}

// NewBodyJointLink inits a new body joint link with the specified parameter
func NewBodyJointLink(body *Body) *BodyJointLink {
	return &BodyJointLink{
		body: body,
	}
}

// Position returns the position, in world coordinates,
// at which this joint links with the underlying [Body].
func (b *BodyJointLink) Position() Vec2 {
	return b.body.DerivedPos
}

// Velocity returns the velocity of the object this joint links to
func (b *BodyJointLink) Velocity() Vec2 {
	return b.body.DerivedVel
}

// Mass calculates and returns the total mass of the subject of this joint link
func (b *BodyJointLink) Mass() (totalMass float64) {
	for _, pm := range b.body.PointMasses {
		totalMass += pm.Mass
	}
	return
}

// IsStatic returns a value specifying whether
// the object referenced by this JointLinkType is static
func (b *BodyJointLink) IsStatic() bool {
	return b.body.IsStatic || b.body.IsPinned
}

// ApplyForce applies a given force to the subject of this joint link
func (b *BodyJointLink) ApplyForce(force Vec2) {
	b.body.ApplyGlobalForce(force)
}

// Translate applies a direct positional translation of this joint link
func (b *BodyJointLink) Translate(offset Vec2) {
	for i := range b.body.PointMasses {
		b.body.TranslatePointAt(offset, i)
	}
}

// AddVelocity adds a velocity delta to every point mass of the body.
func (b *BodyJointLink) AddVelocity(velocity Vec2) {
	b.body.AddVelocity(velocity)
}

// ############ PointJointLink ##############

// Represents a joint link that links directly to a point mass of a body
type PointJointLink struct {
	// The point mass this joint is linked to
	pointMass int
	// Gets the body that this joint link is linked to
	body *Body

	baseJointLink
}

// NewPointJointLink returns new NewPointJointLink
func NewPointJointLink(body *Body, pointMassIndex int) *PointJointLink {
	return &PointJointLink{
		body:      body,
		pointMass: pointMassIndex,
	}
}

// Gets the position, in world coordinates, at which this joint links with
// the underlying body
func (p *PointJointLink) Position() Vec2 {
	return p.body.PointMasses[p.pointMass].Position
}

// Velocity returns the velocity of the object this joint links to
func (p *PointJointLink) Velocity() Vec2 {
	return p.body.PointMasses[p.pointMass].Velocity
}

// Gets the total mass of the subject of this joint link
func (p *PointJointLink) Mass() (totalMass float64) {
	return p.body.PointMasses[p.pointMass].Mass
}

// IsStatic returns a value specifying whether
// the object referenced by this JointLinkType is static
func (p *PointJointLink) IsStatic() bool {
	return p.body.PointMasses[p.pointMass].Mass == Infinity
}

// Applies a given force to the subject of this joint link
func (p *PointJointLink) ApplyForce(force Vec2) {
	p.body.ApplyForceToPointAt(force, p.pointMass)
}

// Applies a direct positional translation of this joint link by a given offset
func (p *PointJointLink) Translate(offset Vec2) {
	p.body.TranslatePointAt(offset, p.pointMass)
}

// AddVelocity adds a velocity delta to the linked point mass.
func (p *PointJointLink) AddVelocity(velocity Vec2) {
	p.body.AddVelocityToPointAt(velocity, p.pointMass)
}

// ############ EdgeJointLink ##############

// Represents a joint link that links to an edge of a body
type EdgeJointLink struct {
	// The first point mass this joint is linked to
	pointMass1 int
	// The second point mass this joint is linked to
	pointMass2 int
	// A [0-1] ratio defining the point along the edge; 0.5 is the middle, while 0 or 1 acts like a [PointJointLink].
	EdgeRatio float64

	baseJointLink
}

// NewEdgeJointLink returns new edge joint link with the specified parameters.
//
//   - edgeIndex: the index of the first point mass on the edge, the second is the next one with wrap around
//   - edgeRatio: A [0, 1] ratio defining the point along the edge; 0.5 is the middle, while 0 or 1 acts like a [PointJointLink]. See [EdgeJointLink.EdgeRatio]
func NewEdgeJointLink(body *Body, edgeIndex int, edgeRatio ...float64) *EdgeJointLink {
	ratio := 0.5
	if len(edgeRatio) > 0 {
		ratio = edgeRatio[0]
	}
	return &EdgeJointLink{
		pointMass1: edgeIndex % len(body.PointMasses),
		pointMass2: (edgeIndex + 1) % len(body.PointMasses),
		EdgeRatio:  ratio,
		body:       body,
	}
}

// Gets the position, in world coordinates, at which this joint links with
// the underlying body
func (e *EdgeJointLink) Position() Vec2 {
	pos1 := e.body.PointMasses[e.pointMass1].Position
	pos2 := e.body.PointMasses[e.pointMass2].Position
	return pos1.Lerp(pos2, e.EdgeRatio)
}

// Velocity returns the velocity of the object this joint links to
func (e *EdgeJointLink) Velocity() Vec2 {
	vel1 := e.body.PointMasses[e.pointMass1].Velocity
	vel2 := e.body.PointMasses[e.pointMass2].Velocity
	return vel1.Lerp(vel2, e.EdgeRatio)
}

// Gets the total mass of the subject of this joint link
func (e *EdgeJointLink) Mass() (totalMass float64) {
	mass1 := e.body.PointMasses[e.pointMass1].Mass
	mass2 := e.body.PointMasses[e.pointMass2].Mass
	return mass1*(1-e.EdgeRatio) + mass2*(e.EdgeRatio)
}

// IsStatic returns a value specifying whether
// the object referenced by this JointLinkType is static
func (e *EdgeJointLink) IsStatic() bool {
	inf1 := e.body.PointMasses[e.pointMass1].Mass == Infinity
	inf2 := e.body.PointMasses[e.pointMass2].Mass == Infinity
	return e.body.IsStatic || (inf1 && inf2)
}

// Applies a given force to the subject of this joint link
func (e *EdgeJointLink) ApplyForce(force Vec2) {
	e.body.ApplyForceToPointAt(force.Scale(1-e.EdgeRatio), e.pointMass1)
	e.body.ApplyForceToPointAt(force.Scale(e.EdgeRatio), e.pointMass2)
}

// Applies a direct positional translation of this joint link by a given offset
func (e *EdgeJointLink) Translate(offset Vec2) {
	e.body.TranslatePointAt(offset, e.pointMass1)
	e.body.TranslatePointAt(offset, e.pointMass2)
}

// AddVelocity adds a velocity delta to both endpoints of the edge,
// weighted by EdgeRatio — mirrors ApplyForce's distribution.
func (e *EdgeJointLink) AddVelocity(velocity Vec2) {
	e.body.AddVelocityToPointAt(velocity.Scale(1-e.EdgeRatio), e.pointMass1)
	e.body.AddVelocityToPointAt(velocity.Scale(e.EdgeRatio), e.pointMass2)
}

// ############ ShapeJointLink ##############

// Represents a joint link that links to multiple point masses of a body
type ShapeJointLink struct {
	// The indices of this shape joint link
	Indexes []int
	/// The Offset to apply to the position of this shape joint, in body
	/// coordinates
	Offset Vec2

	baseJointLink
}

// ShapeJointLink returns new ShapeJointLink. See [ShapeJointLink]
func NewShapeJointLink(body *Body, pointMassIndexes []int) *ShapeJointLink {
	return &ShapeJointLink{
		body:    body,
		Indexes: pointMassIndexes,
	}
}

// Position returns the position, in world coordinates, at which this joint
// links with the underlying body — the centroid of the indexed point masses,
// plus Offset rotated into world space by the shape's current deformation
// angle (so the offset point follows the body's rotation, not just its
// centroid).
func (s *ShapeJointLink) Position() Vec2 {
	return s.centroid().Add(s.offsetPosition())
}

// Velocity returns the velocity of the object this joint links to — the
// average velocity of the indexed point masses (centroid velocity), plus
// the rotational contribution from Offset, computed as ω × r using the
// body's current angular velocity. This keeps Velocity() consistent with
// the offset point returned by Position(): both describe the same
// physical point on the rotating body, not just the centroid.
func (s *ShapeJointLink) Velocity() (vel Vec2) {
	for _, i := range s.Indexes {
		vel = vel.Add(s.body.PointMasses[i].Velocity)
	}
	centroidVel := vel.DivS(float64(len(s.Indexes)))

	offset := s.offsetPosition()
	if offset.IsZero() {
		return centroidVel
	}

	// 2D cross product ω × r = r.Perp() * ω
	rotational := offset.Perp().Scale(s.body.DerivedOmega)
	return centroidVel.Add(rotational)
}

// Gets the total mass of the subject of this joint link
func (s *ShapeJointLink) Mass() (mass float64) {
	for _, i := range s.Indexes {
		mass += s.body.PointMasses[i].Mass
	}
	return
}

// IsStatic returns a value specifying whether
// the object referenced by this JointLinkType is static
func (s *ShapeJointLink) IsStatic() bool {
	for _, i := range s.Indexes {
		if s.body.PointMasses[i].Mass == Infinity {
			return true
		}
	}
	return false
}

// Applies a given force to the subject of this joint link
func (s *ShapeJointLink) ApplyForce(force Vec2) {
	torqueF := s.offsetPosition().Dot(force.Perp())
	centroid := s.centroid() // eski Position() davranışı, offsetsiz
	for _, i := range s.Indexes {
		pm := s.body.PointMasses[i]
		tempR := pm.Position.Sub(centroid).Add(s.offsetPosition()).Perp()
		s.body.ApplyForceToPointAt(force.Add(tempR.Scale(torqueF)), i)
	}
}

// centroid returns the unweighted average position of the indexed point
// masses, without Offset applied — used internally as the rotation
// reference point for torque calculations.
func (s *ShapeJointLink) centroid() (pos Vec2) {
	for _, i := range s.Indexes {
		pos = pos.Add(s.body.PointMasses[i].Position)
	}
	return pos.DivS(float64(len(s.Indexes)))
}

// Applies a direct positional translation of this joint link by a given offset
func (s *ShapeJointLink) Translate(offset Vec2) {
	for _, i := range s.Indexes {
		s.body.TranslatePointAt(offset, i)
	}
}

// AddVelocity adds a velocity delta at the offset point, distributing it
// across the indexed point masses the same way ApplyForce distributes a
// force — including the rotational component so a velocity change at an
// offset point produces angular motion around the shape's centroid, not
// just a uniform translation.
func (s *ShapeJointLink) AddVelocity(velocity Vec2) {
	torqueV := s.offsetPosition().Dot(velocity.Perp())
	centroid := s.centroid()
	n := float64(len(s.Indexes))
	for _, i := range s.Indexes {
		pm := s.body.PointMasses[i]
		tempR := pm.Position.Sub(centroid).Add(s.offsetPosition()).Perp()
		// velocity/n: mirrors how ApplyForce splits `force` (not force/n)
		// across n points relying on each point's own force accumulation;
		// for velocity we divide by n since AddVelocityToPointAt sets an
		// additive delta per point directly (no mass-based redistribution
		// happens later, unlike forces which integrate through mass).
		s.body.AddVelocityToPointAt(velocity.Add(tempR.Scale(torqueV)).DivS(n), i)
	}
}

// SubDerivedAngle returns the average angle of the vertices of this ShapeJointLink, based
// on the body's original shape's vertices.
//
// This represents the local deformation angle of this specific point mass
// subset, which may differ from the body's overall derivedAngle in a
// soft body where different regions can rotate independently.
func (s *ShapeJointLink) SubDerivedAngle() float64 {
	var angle, originalAngle float64
	originalSign := 1
	first := true
	for _, i := range s.Indexes {
		pm := s.body.PointMasses[i]
		base := s.body.BaseShape[i]
		baseNorm := base.Unit()
		curNorm := pm.Position.Sub(s.body.DerivedPos).Unit()
		thisAngle := math.Atan2(baseNorm.X*curNorm.Y-baseNorm.Y*curNorm.X, baseNorm.Dot(curNorm))

		if first {
			if thisAngle >= 0.0 {
				originalSign = 1
			} else {
				originalSign = -1
			}
			originalAngle = thisAngle

			first = false
		} else {
			diff := thisAngle - originalAngle
			thisSign := 1
			if thisAngle < 0.0 {
				thisSign = -1
			}

			if math.Abs(diff) > Pi && (thisSign != originalSign) {
				if thisSign == -1 {
					thisAngle = Pi + (Pi + thisAngle)
				} else {
					thisAngle = (Pi - thisAngle) - Pi
				}
			}
		}

		angle += thisAngle
	}

	angle /= float64(len(s.Indexes))

	return angle
}

func (s *ShapeJointLink) offsetPosition() (pos Vec2) {
	if s.Offset.IsZero() {
		return s.Offset
	}
	return s.Offset.Rotate(s.SubDerivedAngle())
}
