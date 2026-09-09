package jel

// SpringComponent adds spring-based physics to a body's point masses.
//
// By default it places a spring between every pair of consecutive point masses
// along the body's outline (i.e. one spring per edge, wrapping around to
// close the shape), plus any extra springs added manually via
// [SpringComponent.AddExtraSpring]. Each spring pulls its two endpoints back toward a rest
// distance whenever they drift apart or are pushed together, which is what
// keeps neighboring points from separating or overlapping.
type SpringComponent struct {
	// The number of default edge springs built between consecutive point masses
	EdgeSpringsCount int
	// All springs belonging to this component (edge springs first, then extra springs) See [SpringComponent.AddExtraSpring]
	Springs []*InternalSpring
	// The stiffness constant
	DefaultStiffness float64
	// The damping constant
	DefaultDamping float64
	baseComponent
}

// Returns default SpringComponent
func DefaultSpringComponent() *SpringComponent {
	return &SpringComponent{DefaultStiffness: 50, DefaultDamping: 2}
}

// NewSpringComponent returns new [SpringComponent].
func NewSpringComponent(stiffness, damping float64) *SpringComponent {
	return &SpringComponent{DefaultStiffness: stiffness, DefaultDamping: damping}
}

// Prepare rebuilds all internal springs for the body, discarding any
// previously added springs and re-creating the default edge springs.
func (s *SpringComponent) Prepare(body *Body) {
	if s.EdgeSpringsCount == 0 {
		s.prepareEdgeSprings(body)
	}

}

// AccumulateInternalForces applies each spring's force between its two
// point masses, clamping the rest distance to the spring's actual current
// distance when applicable, and updates spring plasticity (permanent
// deformation) when not relaxing.
// AccumulateInternalForces applies each spring's force between its two
// point masses, clamping the rest distance to the spring's actual current
// distance when applicable, and updates spring plasticity (permanent
// deformation) when not relaxing.
func (s *SpringComponent) AccumulateInternalForces(body *Body, relaxing bool) {
	for _, sp := range s.Springs {
		p1 := body.PointMasses[sp.PointMassA]
		p2 := body.PointMasses[sp.PointMassB]

		actDist := p1.Position.Dist(p2.Position)
		inRange := sp.RestDistance.InRange(actDist)

		// Aralık içindeyse hedef mesafe olarak mevcut mesafenin kendisini
		// veriyoruz: bu, CalcSpringForce içindeki yay terimini (dist*stiffness)
		// otomatik olarak sıfırlar ama damping terimini (totalRelVel*damping)
		// aktif bırakır. Böylece nokta kütleleri "gevşek" bölgede yay
		// kuvveti hissetmez, ama hâlâ sönümlenir — sınırda ani kuvvet
		// açılıp kapanmasından kaynaklanan sekme/titreme (jitter) önlenir.
		targetDist := actDist
		if !inRange {
			targetDist = sp.RestDistance.Clamp(actDist)
		}

		force := CalcSpringForce(
			p1.Position, p1.Velocity,
			p2.Position, p2.Velocity,
			targetDist,
			sp.Stiffness, sp.Damping,
		)

		body.ApplyForceToPointAt(force, sp.PointMassA)
		body.ApplyForceToPointAt(force.Neg(), sp.PointMassB)

		if !relaxing && !inRange && sp.Plasticity != nil {
			sp.UpdatePlasticity(actDist)
		}
	}
}

// AddExtraSpring adds an internal spring to this body.
// plasticity is an optional argument and may be nil.
func (s *SpringComponent) AddExtraSpring(body *Body, pointA, pointB int, stiffness, damping float64, plasticity *SpringPlasticity) *InternalSpring {
	pA := body.PointMasses[pointA]
	pB := body.PointMasses[pointB]
	dist := NewFixedRestDistance(pA.Position.Dist(pB.Position))
	return s.addSpringWithDist(pointA, pointB, stiffness, damping, dist, plasticity, ExtraSpring)
}

// addSpringWithDist adds an internal spring to this body using an
// explicit RestDistance rather than deriving it from the points' current
// separation. plasticity is an optional argument and may be nil.
func (s *SpringComponent) addSpringWithDist(pointA, pointB int, stiffness, damping float64, dist RestDistance, plasticity *SpringPlasticity, st SpringType) *InternalSpring {
	spring := NewInternalSpring(pointA, pointB, dist, stiffness, damping, plasticity)
	spring.Type = st
	s.Springs = append(s.Springs, spring)
	return spring
}

// ClearAllSprings removes all springs (including any added via
// [SpringComponent.AddExtraSpring]
func (s *SpringComponent) ClearAllSprings(body *Body) {
	s.Springs = nil

}

// prepareEdgeSprings creates one edge spring between each consecutive
// pair of point masses (wrapping around to close the shape), using the
// [SpringComponent.defaultStiffness] and [SpringComponent.defaultDamping] constants.
func (s *SpringComponent) prepareEdgeSprings(body *Body) {
	s.EdgeSpringsCount = len(body.PointMasses)
	for i := range s.EdgeSpringsCount {
		pointB := (i + 1) % s.EdgeSpringsCount
		pA := body.PointMasses[i]
		pB := body.PointMasses[pointB]
		s.addSpringWithDist(
			i,
			pointB,
			s.DefaultStiffness,
			s.DefaultDamping,
			NewFixedRestDistance(pA.Position.Dist(pB.Position)),
			nil,
			EdgeSpring,
		)
	}
}

// SetAllEdges sets the stiffness and damping of every edge springs
func (s *SpringComponent) SetAllEdges(stiffness, damping float64) {
	for i := range s.EdgeSpringsCount {
		s.Springs[i].Stiffness = stiffness
		s.Springs[i].Damping = damping
	}
}

// SetAll sets the stiffness and damping of all springs (edge + extra)
func (s *SpringComponent) SetAll(stiffness, damping float64) {
	for i := range s.Springs {
		s.Springs[i].Stiffness = stiffness
		s.Springs[i].Damping = damping
	}
}

// ApplyDefaults sets the [SpringComponent.DefaultStiffness] and [SpringComponent.DefaultDamping] values ​​for all springs.
func (s *SpringComponent) ApplyDefaults() {
	s.SetAll(s.DefaultStiffness, s.DefaultDamping)
}

// SetExtraAt sets the stiffness and damping of an extra (non-edge) spring.
// relativeIndex is the spring's position among the extra springs only
// (0 = first extra spring added after the edge springs), not its index
// in the full Springs slice. See [SpringComponent.AddExtraSpring]
func (s *SpringComponent) SetExtraAt(relativeIndex int, stiffness, damping float64) {
	index := s.EdgeSpringsCount + relativeIndex
	s.Springs[index].Stiffness = stiffness
	s.Springs[index].Damping = damping
}

func (s *SpringComponent) SetSpringPlasticity(relativeIndex int, plasticity *SpringPlasticity) {
	index := s.EdgeSpringsCount + relativeIndex
	if plasticity != nil {
		plasticity.initialRestDistance = s.Springs[index].RestDistance
	}
	s.Springs[index].Plasticity = plasticity
}
