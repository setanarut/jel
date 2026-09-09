package jel

// StickyRayComponent sticks a body to whatever other body its point masses'
// rays hit. For each point mass, a short ray is cast outward along that
// point's normal; if it hits another body, the closest point on that body's
// nearest edge is found, and the originating point mass is pulled toward
// that point with a spring force (like tape/velcro between two soft bodies).
// The target is not the middle of the edge — it's whichever point along the
// edge sits closest to where the ray actually hit.
type StickyRayComponent struct {
	baseComponent
	// IgnoreJoinedBodies skips raycasting against bodies connected to this
	// one via a Joint, avoiding redundant sticky forces on an existing joint.
	IgnoreJoinedBodies bool
	// RayLength is the length of the ray cast out from each point mass
	RayLength float64
	// Stiffness is the stiffness of the spring pulling toward the hit edge
	Stiffness float64
	// Damping is the damping applied to the spring force
	Damping float64
}

// NewStickyRayComponent creates a new StickyRayComponent See [StickyRayComponent] for info
func NewStickyRayComponent(rayLength, stiffness, damping float64, ignoreJoinedBodies bool) *StickyRayComponent {
	return &StickyRayComponent{
		IgnoreJoinedBodies: ignoreJoinedBodies,
		RayLength:          rayLength,
		Stiffness:          stiffness,
		Damping:            damping,
	}
}

// DefaultStickyRayComponent returns a StickyRayComponent with default settings
func DefaultStickyRayComponent() *StickyRayComponent {
	return &StickyRayComponent{
		IgnoreJoinedBodies: true,
		RayLength:          1,
		Stiffness:          100,
		Damping:            10,
	}
}

// AccumulateExternalForces applies sticky rays to other bodies
func (s *StickyRayComponent) AccumulateExternalForces(body *Body, world *World) {
	for i, point := range body.PointMasses {
		vertex := point.Position
		normal := point.Normal

		// Cast the ray slightly behind the vertex (along the inverse normal) out to RayLength
		start := vertex.Sub(normal.Scale(0.1))
		end := vertex.Add(normal.Scale(s.RayLength))

		// Skip self and, optionally, bodies already joined to this one
		pt, hitBody := world.RayCast(start, end, 0, func(otherBody *Body) bool {
			if otherBody == body {
				return true
			}
			if s.IgnoreJoinedBodies && world.AreBodiesJoined(body, otherBody) {
				return true
			}
			return false
		})

		if hitBody == nil {
			continue
		}

		// Find the closest edge on the hit body to attach the spring to
		edgePosition, edgeRatio, edgePoint1, edgePoint2, ok := hitBody.ClosestEdge(pt, Infinity)
		if !ok {
			continue
		}

		// Weaken the spring the farther the hit point is from the ray origin
		normalizedDistance := min(1, max(0, pt.Dist(start)/s.RayLength))
		strength := min(0.9, 1-normalizedDistance)

		// Interpolate the edge's velocity between its two endpoints
		edgeVelocity := hitBody.PointMasses[edgePoint1].
			Velocity.
			Lerp(hitBody.PointMasses[edgePoint2].Velocity, edgeRatio)

		// Compute the spring force pulling the vertex toward the edge
		force := CalcSpringForce(
			vertex,
			point.Velocity,
			edgePosition,
			edgeVelocity,
			0,
			s.Stiffness*strength,
			s.Damping*strength,
		)

		// Apply the force to this body's point and the opposing force
		// distributed across the hit edge's two endpoints (weighted by edgeRatio)
		body.ApplyForceToPointAt(force, i)
		hitBody.ApplyForceToPointAt(force.Scale(-(1 - edgeRatio)), edgePoint1)
		hitBody.ApplyForceToPointAt(force.Scale(-edgeRatio), edgePoint2)
	}
}
