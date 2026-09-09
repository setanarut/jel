package jel

// PressureComponent simulates internal gas pressure that pushes a body's
// edges outward, keeping soft-body shapes inflated (like a balloon).
type PressureComponent struct {
	// The current volume (area) enclosed by the body's shape
	Volume float64
	// The gas pressure constant used to push outward on the body's edges
	GasPressure float64
	baseComponent
}

// NewPressureComponent returns a new [PressureComponent] that can be added to
// a body to simulate outward gas pressure against its edges.
func NewPressureComponent(gasPressure float64) *PressureComponent {
	return &PressureComponent{GasPressure: gasPressure}
}

// AccumulateInternalForces recomputes the body's enclosed volume (area) and
// applies an outward force along each point's normal proportional to the
// gas pressure and inversely proportional to the current volume, so the
// body resists being squeezed.
func (p *PressureComponent) AccumulateInternalForces(body *Body, relaxing bool) {
	if len(body.PointMasses) < 1 {
		p.Volume = 0
		return
	}
	p.Volume = max(0.5, polygonAreaFromPointMasses(body.PointMasses))
	invVolume := 1.0 / p.Volume
	for _, e := range body.Edges {
		pressureV := invVolume * e.Length * p.GasPressure
		forceStart := body.PointMasses[e.StartPointIndex].Normal.Scale(pressureV)
		body.ApplyForceToPointAt(forceStart, e.StartPointIndex)
		forceEnd := body.PointMasses[e.EndPointIndex].Normal.Scale(pressureV)
		body.ApplyForceToPointAt(forceEnd, e.EndPointIndex)
	}
}
