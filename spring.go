package jel

// SpringType identifies what a spring connection represents within the
// simulation, distinguishing default perimeter springs, extra internal
// springs, and springs used by joints between separate bodies.
type SpringType uint8

const (
	// EdgeSpring is a default spring between two consecutive point masses
	// along a body's outline (perimeter), automatically created to keep
	// the shape's outline together.
	EdgeSpring SpringType = iota
	// ExtraSpring is an extra spring added between two point masses
	// that are not necessarily adjacent on the outline, used to connect
	// edges to each other and help the body resist internal deformation
	// (e.g. shape-holding cross braces). Added via [SpringComponent.AddExtraSpring].
	ExtraSpring
	// JointSpring is the spring used by a [SpringBodyJoint], connecting two
	// joint links — typically belonging to two separate bodies — rather
	// than two point masses within the same body.
	JointSpring
)

// Spring represents an spring and keeps points close together.
type Spring struct {
	// Rest distance of the spring, or the distance the spring tries to maintain.
	RestDistance RestDistance
	// Stiffness is the spring stiffness.
	Stiffness float64
	// Damping is the spring damping.
	Damping float64
	// Plasticity specifies the plasticity properties of this spring.
	// If nil, plasticity is disabled and spring never deforms permanently.
	Plasticity *SpringPlasticity
	// Spring type
	Type SpringType
}

// CalcSpringTension computes the normalized tension (-1..1) of a spring
// based on the current distance between its two endpoints (a, b) relative
// to its rest distance range.
//
// If dist falls within [RestDistance.MinDist], [RestDistance.MaxDist] the spring is considered
// relaxed and tension is 0. Otherwise, the ratio is computed against the
// nearest bound ([RestDistance.MaxDist] when stretched, [RestDistance.MinDist] when compressed),
// clamped so that a ratio of ±30% or more maps to ±1. A positive value
// indicates stretching, a negative value indicates compression.
func CalcSpringTension(a, b Vec2, rd RestDistance) float64 {
	dist := a.Dist(b)

	var ratio float64
	switch {
	case dist > rd.MaxDist():
		if bound := rd.MaxDist(); bound > 0.001 {
			ratio = (dist - bound) / bound
		}
	case dist < rd.MinDist():
		if bound := rd.MinDist(); bound > 0.001 {
			ratio = (dist - bound) / bound
		}
	default:
		ratio = 0
	}

	ratio = max(-0.3, min(0.3, ratio))
	return ratio / 0.3
}

// StiffnessDampingFrom returns (stiffness, damping) for a spring,
// given only a mass and a softness value in [0, 1].
//
// - softness = 0 -> rigid (stiff, snaps back fast, no wobble)
//
// - softness = 1 -> ideal loose/jelly (soft, wobbles, settles slowly)
//
// Same softness value gives the same "feel" regardless of mass.
func StiffnessDampingFrom(mass, softness float64) (stiffness, damping float64) {
	if mass <= 0 || mass == Infinity {
		return 0, 0
	}
	t := max(0, min(1, softness))

	const (
		minFreq = 4.0  // softness=1
		maxFreq = 18.0 // softness=0
		minZeta = 0.15 // softness=1
		maxZeta = 1.1  // softness=0
	)

	freq := maxFreq + t*(minFreq-maxFreq)
	zeta := maxZeta + t*(minZeta-maxZeta)

	stiffness = freq * freq * mass
	damping = zeta * 2 * mass * freq
	return stiffness, damping
}

type InternalSpring struct {
	Spring
	PointMassA int
	PointMassB int
}

// NewSpring creates a new spring.
func NewInternalSpring(pmA int, pmB int, distance RestDistance, stiffness float64, damping float64, plasticity *SpringPlasticity) *InternalSpring {
	s := &InternalSpring{
		PointMassA:   pmA,
		PointMassB:   pmB,
		RestDistance: distance,
		Stiffness:    stiffness,
		Damping:      damping,
		Plasticity:   plasticity,
		Type:         EdgeSpring,
	}

	return s
}

// UpdatePlasticity updates the plasticity settings of this spring.
// Does nothing, if plasticity is not configured.
func (s *Spring) UpdatePlasticity(distance float64) {
	if s.Plasticity == nil {
		return
	}
	s.RestDistance = CalcPlasticity(distance, s.RestDistance, s.Plasticity)
}

// SpringPlasticity specifies plasticity properties of a spring.
//
// Plasticity permanently affects a spring's rest length by modifying it
// when its length is stretched beyond a certain limit.
type SpringPlasticity struct {
	// YieldRatio is the ratio (of resting distance vs actual length) before plasticity starts
	// to change the resting length of the spring, deforming it permanently.
	YieldRatio float64

	//if L/R > Y or R/L < 1/Y, the resting length will be updated
	//to be R += P * (L - R - (Y * R)) (for L > R) or R -= P * (R - (Y * R) - L)
	//(for L < R).
	//e.g. given a spring with a resting distance R = 10, a plasticity
	//rate P = 0.5, a yield ratio of Y = 0.3, and an actual length L = 15,
	//the resulting R would be R = 10 + (0.5 * (15 - 10 - (0.3 * 10))) ->
	//R = 10 + 1 -> R = 11.

	// Rate is the plasticity rate for the spring.
	// When the rest distance of a spring goes past its yield limit, the
	// resting distance of the spring is stretched so it deforms 'plastically'
	// by adapting the resting length to be the resulting factor between the
	// rest length and the actual length, times this rate.
	Rate float64

	//If the rest length of a spring goes R > IL * limit (with IL being
	//the initial length), or R < IL / limit, the plasticity does not
	//take effect.

	// Limit is a factor limit at which the plasticity stops affecting the rest length
	// of the spring beyond its initial rest length.
	Limit float64

	// initialRestDistance is the spring's rest distance at the moment plasticity
	// was configured (or the spring was created with plasticity), ignoring any
	// deformations since. Used as the reference point for the Limit calculation.
	initialRestDistance RestDistance
}

// DefaultSpringPlasticity returns a SpringPlasticity with default values.
func DefaultSpringPlasticity() *SpringPlasticity {
	return &SpringPlasticity{
		YieldRatio: 0.3,
		Rate:       0.5,
		Limit:      2.0,
	}
}

// NewSpringPlasticity creates a new SpringPlasticity with the given values.
func NewSpringPlasticity(yieldRatio float64, rate float64, limit float64) *SpringPlasticity {
	return &SpringPlasticity{
		YieldRatio: yieldRatio,
		Rate:       rate,
		Limit:      limit,
	}
}

// CalcPlasticity calculates a new resting distance based on provided plasticity parameters.
// The resulting resting distance is returned by the function.
//
// - Parameters:
//   - dist: The current distance of the spring
//   - rd: The resting distance for the spring
//
// - Returns: The new rest distance to the spring, after plasticity is applied.
func CalcPlasticity(dist float64, rd RestDistance, sp *SpringPlasticity) RestDistance {
	if rd.InRange(dist) {
		return rd
	}
	var out = rd
	if dist > rd.MaxDist() {
		dstr := sp.YieldRatio * rd.MaxDist()
		if dist > rd.MaxDist()+dstr {
			out.SetMaxDist(out.MaxDist() + sp.Rate*(dist-out.MaxDist()-dstr))
			if out.MaxDist() > sp.initialRestDistance.MaxDist()*sp.Limit {
				out.SetMaxDist(sp.initialRestDistance.MaxDist() * sp.Limit)
			}
		}
	} else if dist < rd.MinDist() {
		dst := sp.YieldRatio * rd.MinDist()
		if dist < rd.MinDist()-dst {
			out.SetMinDist(out.MinDist() - sp.Rate*(out.MinDist()-dst-dist))
			if out.MinDist() < sp.initialRestDistance.MinDist()/sp.Limit {
				out.SetMinDist(sp.initialRestDistance.MinDist() / sp.Limit)
			}
		}
	}
	return out
}

// CalcSpringForce computes the force exerted by a damped spring
// connecting two points (posA, velA) and (posB, velB), given the spring's
// rest length (distance), stiffness, and damping factor.
// The returned force is directed along the axis between the two points and
// should be applied to posA (and its negation to posB). Returns the zero
// Vec2 if the two points are closer than epsilonSpring, to avoid dividing
// by a near-zero distance.
func CalcSpringForce(
	posA, velA, posB, velB Vec2,
	distance, stiffness, damping float64,
) Vec2 {
	var dist = posA.Dist(posB)
	if dist <= epsilonSpring {
		return Vec2{}
	}
	BtoA := posA.Sub(posB).DivS(dist)
	dist = distance - dist
	relVel := velA.Sub(velB)
	totalRelVel := relVel.Dot(BtoA)
	return BtoA.Scale((dist * stiffness) - (totalRelVel * damping))
}
