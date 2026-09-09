package jel

import "math"

// ShapeMatchComponent pulls a body's point masses back toward its original
// base shape using spring forces, so the body tends to retain its shape
// after deformation. It can target the full shape or a subset of points.
type ShapeMatchComponent struct {
	// The spring stiffness for all points used to pull points back toward the base shape
	Stiffness float64
	// The spring damping for all points used to pull points back toward the base shape
	Damping float64
	// The point mass indices to match against; empty means match the full shape
	TargetIndices []int
	baseComponent
}

// DefaultShapeMatchComponent returns a full-shape ShapeMatchComponent with default spring constants.
func DefaultShapeMatchComponent() *ShapeMatchComponent {
	return NewShapeMatchComponent(200, 10, nil)
}

// NewShapeMatchComponent returns a new ShapeMatchComponent that pulls a body
// (or a subset of its points) back toward its original base shape using a
// spring force. Pass nil (or an empty slice) for indices to match the full
// shape, or a list of point mass indices to match only that subset.
func NewShapeMatchComponent(stiffness, damping float64, indices []int) *ShapeMatchComponent {
	return &ShapeMatchComponent{
		Stiffness:     stiffness,
		Damping:       damping,
		TargetIndices: indices,
	}
}

// SetTarget changes what the component matches against: pass nil (or an
// empty slice) to match the full shape, or a list of point mass indices to
// match only that subset.
func (s *ShapeMatchComponent) SetTarget(indices []int) {
	s.TargetIndices = indices
}

// AccumulateInternalForces applies shape-matching spring forces if shape
// matching is enabled and the spring constant is positive.
func (s *ShapeMatchComponent) AccumulateInternalForces(body *Body, relaxing bool) {
	if s.Stiffness <= 0 {
		return
	}

	if len(s.TargetIndices) == 0 {
		matrix := NewMatrix3x3(body.Scale, body.DerivedAngle, body.DerivedPos)
		body.BaseShape.TransformByMatrixToTarget(body.GlobalShape, matrix)
		for i, global := range body.GlobalShape {
			if i >= len(body.PointMasses) {
				break
			}
			s.applySpringForce(body, i, global)
		}
		return
	}

	pos, angle := s.deriveSubsetPositionAngle(body, s.TargetIndices)
	matrix := NewMatrix3x3(body.Scale, angle, pos)
	for _, i := range s.TargetIndices {
		if i >= len(body.PointMasses) {
			continue
		}
		global := matrix.Apply(body.BaseShape[i])
		s.applySpringForce(body, i, global)
	}
}

// applySpringForce computes and applies the zero-rest-length spring +
// damping force pulling point mass i toward its derived global shape
// position (global).
func (s *ShapeMatchComponent) applySpringForce(body *Body, i int, global Vec2) {
	p := body.PointMasses[i]

	velB := Vec2{}
	if !body.IsKinematic {
		radiusVec := global.Sub(body.DerivedPos)
		tangentVel := Vec2{
			X: -body.DerivedOmega * radiusVec.Y,
			Y: body.DerivedOmega * radiusVec.X,
		}
		velB = body.DerivedVel.Add(tangentVel)
	}

	// Zero rest length that pulls the particle to
	//  a derived global shape point
	displacement := global.Sub(p.Position)
	springForce := displacement.Scale(s.Stiffness)
	relVel := velB.Sub(p.Velocity)
	dampForce := relVel.Scale(s.Damping)
	force := springForce.Add(dampForce)

	body.ApplyForceToPointAt(force, i)
}

// deriveSubsetPositionAngle computes the reference position and average
// rotation angle for a subset of point masses, by comparing each point's
// current position (relative to the body's mean position) against its
// corresponding base shape vertex, and averaging the resulting angles
// (handling wraparound across the +/-Pi boundary).
func (s *ShapeMatchComponent) deriveSubsetPositionAngle(body *Body, indices []int) (Vec2, float64) {
	meanPos := body.DerivedPos
	if body.IsPinned {
		meanPos = AveragePointMassPosition(body.PointMasses)
	}
	var angle float64 = 0
	originalSign := 1
	var originalAngle float64 = 0
	c := len(body.PointMasses)
	first := true
	for _, index := range indices {
		base := body.BaseShape[index]
		pm := body.PointMasses[index]
		baseNorm := base.Unit()
		curNorm := pm.Position.Sub(meanPos).Unit()
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
	angle /= float64(c)
	return meanPos, angle
}
