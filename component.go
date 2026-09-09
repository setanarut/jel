package jel

// Component represents something that can be attached to a Body to affect
// its physical behavior.
type Component interface {
	// Prepare is called once after the component is added to a body.
	Prepare(body *Body)
	// AccumulateInternalForces adds shape-preserving internal forces to
	// each PointMass.Force in the body. worldRelaxing indicates whether the
	// world is currently in its relaxation phase.
	AccumulateInternalForces(body *Body, worldRelaxing bool)
	// AccumulateExternalForces adds external forces (e.g. gravity) to
	// each PointMass.Force in the body.
	AccumulateExternalForces(body *Body, world *World)

	Disabled() bool
}

// baseComponent provides no‑op Component methods for embedding.
type baseComponent struct {
	// Is this component disabled
	Off bool
}

func (b *baseComponent) Prepare(body *Body)                                      {}
func (b *baseComponent) AccumulateInternalForces(body *Body, worldRelaxing bool) {}
func (b *baseComponent) AccumulateExternalForces(body *Body, world *World)       {}
func (b *baseComponent) Disabled() bool {
	return b.Off
}
