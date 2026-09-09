package jel

// Joint is implemented by every joint type (BodyJoint, SpringBodyJoint, ...).
// This is what allows the World to store and resolve different joint kinds
// polymorphically.
type Joint interface {
	Resolve(dt float64)
	LinkA() JointLink
	LinkB() JointLink
	CollisionsAllowed() bool
}

// Base class for joints which unites two separate bodies
type bodyJoint struct {
	BodyLinkA       JointLink
	BodyLinkB       JointLink
	AllowCollisions bool
	Enabled         bool
}

func (j *bodyJoint) LinkA() JointLink        { return j.BodyLinkA }
func (j *bodyJoint) LinkB() JointLink        { return j.BodyLinkB }
func (j *bodyJoint) CollisionsAllowed() bool { return j.AllowCollisions }

// SpringJoint represents a joint that links two [JointLink]'s with spring forces
type SpringJoint struct {
	bodyJoint
	Spring
}

func NewSpringJoint(a, b JointLink, coefficient, damping float64, distance ...RestDistance) *SpringJoint {
	var restDistance RestDistance
	if len(distance) == 0 {
		restDistance = NewFixedRestDistance(a.Position().Dist(b.Position()))
	} else {
		restDistance = distance[0]
	}

	return &SpringJoint{
		BodyLinkA:    a,
		BodyLinkB:    b,
		Enabled:      true,
		RestDistance: restDistance,
		Stiffness:    coefficient,
		Damping:      damping,
		Type:         JointSpring,
	}
}

// SetPlasticity enables plasticity for this joint, using the joint's
// current rest distance as the reference point for the plasticity limit.
// Pass nil to disable plasticity.
func (j *SpringJoint) SetPlasticity(p *SpringPlasticity) {
	if p != nil {
		p.initialRestDistance = j.RestDistance
	}
	j.Plasticity = p
}

// Resolve resolves this joint
//
// dt is the delta time to update the resolve on
func (j *SpringJoint) Resolve(dt float64) {
	if !j.Enabled {
		return
	}
	static1 := j.BodyLinkA.IsStatic()
	static2 := j.BodyLinkB.IsStatic()
	if static1 && static2 {
		return
	}
	pos1 := j.BodyLinkA.Position()
	pos2 := j.BodyLinkB.Position()
	dist := pos1.Dist(pos2)
	if j.RestDistance.InRange(dist) {
		return
	}
	targetDist := j.RestDistance.Clamp(dist)
	force := CalcSpringForce(
		pos1, j.BodyLinkA.Velocity(),
		pos2, j.BodyLinkB.Velocity(),
		targetDist,
		j.Stiffness,
		j.Damping,
	)

	j.BodyLinkA.ApplyForce(force)
	j.BodyLinkB.ApplyForce(force.Neg())

	if j.Plasticity != nil {
		j.RestDistance = CalcPlasticity(dist, j.RestDistance, j.Plasticity)
	}
}

// PinJoint rigidly binds two JointLinks to occupy the same world-space
// point, using velocity impulses (not forces or direct position snapping)
// to resolve separation. Both bodies remain free to rotate/deform around
// the shared point — only the point itself is prevented from separating.
//
// This is a fully rigid weld: there is no softness factor and no
// rest-distance range. Separation is closed completely every Resolve
// call, both in position and in relative velocity along the separation
// axis, so no residual gap is left to accumulate step to step.
type PinJoint struct {
	bodyJoint

	// MaxCorrection clamps the magnitude of the per-step position
	// correction (and matching velocity impulse) applied to close the
	// separation, to avoid explosive corrections after a large
	// disturbance (e.g. body teleported, high dt spike). Zero means
	// unclamped.
	MaxCorrection float64
}

// NewPinJoint creates a joint that rigidly locks link a and link b to the
// same world point. There is no distance parameter (unlike
// NewSpringJoint) and no softness parameter (unlike a Baumgarte-style
// pin) — the joint always fully closes separation to zero every step,
// behaving like a weld.
func NewPinJoint(a, b JointLink, maxCorrection float64) *PinJoint {
	return &PinJoint{
		BodyLinkA:       a,
		BodyLinkB:       b,
		Enabled:         true,
		AllowCollisions: false,
		MaxCorrection:   maxCorrection,
	}
}

// Resolve resolves this joint by rigidly closing the separation between
// the two links: position is corrected fully (not fractionally) and the
// full relative velocity component along the separation axis is
// cancelled, not just the diverging part. Called once per physics step,
// same as SpringJoint.Resolve.
func (j *PinJoint) Resolve(dt float64) {
	if !j.Enabled || dt <= 0 {
		return
	}
	static1 := j.BodyLinkA.IsStatic()
	static2 := j.BodyLinkB.IsStatic()
	if static1 && static2 {
		return
	}

	posA := j.BodyLinkA.Position()
	posB := j.BodyLinkB.Position()
	separation := posB.Sub(posA)
	if separation.IsZero() {
		return
	}

	// Fully close the gap — no softness fraction, unlike a Baumgarte
	// style correction. This is what makes the joint behave like a
	// weld instead of a spring.
	posCorrection := separation
	if j.MaxCorrection > 0 {
		if mag := posCorrection.Mag(); mag > j.MaxCorrection {
			posCorrection = posCorrection.Scale(j.MaxCorrection / mag)
		}
	}

	dir := separation.Unit()
	relVel := j.BodyLinkB.Velocity().Sub(j.BodyLinkA.Velocity())
	velAlongDir := relVel.Dot(dir)
	// Cancel the FULL relative velocity along the axis, both signs —
	// not just the diverging component. A one-sided check (only
	// correcting when velAlongDir is opening) leaves a residual give
	// that reads as "esneme"; cancelling both directions removes it.
	impulse := dir.Scale(velAlongDir)

	switch {
	case static1:
		j.BodyLinkB.Translate(posCorrection.Neg())
		j.BodyLinkB.AddVelocity(impulse.Neg())
	case static2:
		j.BodyLinkA.Translate(posCorrection)
		j.BodyLinkA.AddVelocity(impulse)
	default:
		massA := j.BodyLinkA.Mass()
		massB := j.BodyLinkB.Mass()
		total := massA + massB
		if total <= 0 {
			return
		}
		ratioA := massB / total
		ratioB := massA / total

		j.BodyLinkA.Translate(posCorrection.Scale(ratioA))
		j.BodyLinkB.Translate(posCorrection.Neg().Scale(ratioB))

		j.BodyLinkA.AddVelocity(impulse.Scale(ratioA))
		j.BodyLinkB.AddVelocity(impulse.Neg().Scale(ratioB))
	}
}
