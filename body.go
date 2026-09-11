package jel

import (
	"math"
	"slices"
	"sort"
)

// Represents a soft body on the [World]
type Body struct {
	// The base shape for the body with local vertices
	BaseShape Shape
	// The global shape for the body - rotated and translated around the world
	GlobalShape Shape
	// Point masses for the body.
	PointMasses []*PointMass
	// Edges on the body.
	Edges []*BodyEdge
	// edgeTree accelerates closest-edge queries made during narrow-phase
	// collision detection. Its leaves reference edgeTreeIndices.
	edgeTree        []edgeTreeNode
	edgeTreeIndices []int
	// Body joints this body participates in
	Joints []Joint
	// Body components for this body object
	Components []Component
	// The scale for this body's shape
	Scale Vec2
	// The velocity damping to apply to the body. Values closer to 0
	// decelerate faster, values closer to 1 decelerate slower.
	//
	// 1 never decelerates. Values outside the range [0, 1] inclusive may
	// introduce instability. Default is 0.999
	VelDamping float64
	// The axis-aligned bounding box for this body's point masses.
	//
	// Will be slightly expanded to include the velocity of the points, in case
	// this body is kinematic, so it won't always match exactly the position of
	// the point masses.
	AABB AABB
	// The index of the material in the world material slice to use for this
	// body.
	Material int
	// Whether this body is static.
	IsStatic bool
	// Whether this body is kinematic. When true, [Body.DerivePositionAndAngle]
	// skips recomputing [Body.DerivedPos], [Body.DerivedVel], [Body.DerivedAngle],
	// and [Body.DerivedOmega] from the point masses each frame. Instead, the
	// body's transform is driven externally via [Body.SetScaleAnglePosition],
	// which sets [Body.DerivedPos]/[Body.DerivedAngle] directly and pushes
	// that transform onto the point masses (overwriting their positions to
	// match the new base-shape placement). In other words, the usual
	// point-masses -> derived-transform flow is reversed: for kinematic
	// bodies, the derived transform drives the point masses instead of being
	// computed from them.
	//
	// [Body.SetScaleAnglePosition] does not update [Body.DerivedVel],
	// [Body.DerivedOmega], or the point masses' [PointMass.Velocity] — it
	// only overwrites [PointMass.Position]. So a kinematic body's
	// velocity/angular-velocity fields go stale (or stay zero) unless the
	// caller updates them separately. Consumers should not assume these
	// fields reflect the body's actual motion when IsKinematic is true (e.g.
	// [ShapeMatchComponent.Damping] term ignores them in this case).
	IsKinematic bool
	// Whether this body is pinned - pinned bodies rotate around their axis,
	// but try to remain in place, like a kinematic body.
	IsPinned bool
	// Whether the body is able to rotate while moving. Default is true
	FreeRotate bool
	// The collision bitmask for this body.
	Bitmask Bitmask
	// The X-axis bitmask for the body - used for collision filtering.
	BitmaskX Bitmask
	// The Y-axis bitmask for the body - used for collision filtering.
	BitmaskY Bitmask
	// The derived center position of this body - in world coordinates
	DerivedPos Vec2
	// The derived velocity of this body - in world coordinates. The derivation
	DerivedVel Vec2
	// The derived rotation of the body, in radians
	DerivedAngle float64
	// Omega (ω) is the relative angular speed of the body, in radians/s
	DerivedOmega float64
	// Custom user data attached to the body
	UserData any
	// Whether this body's bitmaskX & bitmaskY are stale.
	// Used to avoid recalculating bitmasks
	bitmasksStale bool
	// Utilize to calculate the omega for the body
	lastAngle float64
}

// NewBody returns a new [Body] from shape.
//
//	To ensure a stable physics simulation, the shape is automatically re-centered and inverted if it is not CCW.
//
//	 - pos: initial position in world coordinates.
//	 - angle:  refers to the initial rotation angle from the center in radians (clockwise).
//	 - mass is the individual mass of each point within [Body.PointMasses].
//	   See [Body.SetMassesFromSlice],  [Body.SetMassAll], [Body.SetMassByIndex]
//	 - world: The world this body will be added to (optional). See [World.AddBody] and [World.AddBodies].
func NewBody(shape Shape, pos Vec2, angle, mass float64, world ...*World) *Body {
	b := &Body{
		VelDamping:    0.999,
		FreeRotate:    true,
		Bitmask:       ^Bitmask(0),
		bitmasksStale: true,
		DerivedPos:    pos,
		DerivedAngle:  angle,
		lastAngle:     angle,
		Scale:         Vec2One,
	}
	b.SetShape(shape)
	b.SetMassAll(mass)
	b.UpdateAABB(0, true)

	if len(world) > 0 && world[0] != nil {
		world[0].AddBody(b)
	}

	return b
}

// NewStaticBody returns a new static [Body].
func NewStaticBody(shape Shape, pos Vec2, angle float64, world ...*World) *Body {
	b := NewBody(shape, pos, angle, Infinity, nil)
	if len(world) > 0 && world[0] != nil {
		world[0].AddBody(b)
	}
	return b
}

// SetShape sets the body's shape to a new Shape.
// If the vertex count differs from the current shape, existing [Body.PointMasses]
// are replaced with new ones (mass set to zero). Otherwise, the shape is
// updated without affecting existing [Body.PointMasses].
//
// To ensure a stable physics simulation, the shape is automatically re-centered and inverted if it is not CCW.
func (b *Body) SetShape(shape Shape) {
	if !shape.IsCCW() {
		shape.Reverse()
	}
	shape.Recenter()
	b.BaseShape = shape
	b.GlobalShape = make(Shape, len(shape))
	matrix := NewMatrix3x3(b.Scale, b.DerivedAngle, b.DerivedPos)
	b.BaseShape.TransformByMatrixToTarget(b.GlobalShape, matrix)

	if len(b.BaseShape) != len(b.PointMasses) {
		b.PointMasses = make([]*PointMass, 0, len(b.BaseShape))
		for i := range b.BaseShape {
			b.PointMasses = append(b.PointMasses, NewPointMass(0.0, b.GlobalShape[i]))
		}
		c := len(b.PointMasses)
		b.Edges = make([]*BodyEdge, c)
		for i := range c {
			j := (i + 1) % c
			b.Edges[i] = NewBodyEdge(i, i, j, b.PointMasses[i].Position, b.PointMasses[j].Position)
		}
	}

	for _, c := range b.Components {
		c.Prepare(b)
	}
	b.updateEdges()
	b.bitmasksStale = true
}

// PointMass returns the point at the given index.
func (b *Body) PointMass(pointMassIndex int) *PointMass {
	return b.PointMasses[pointMassIndex]
}

// Adds a body component to this body.
func (b *Body) AddComponent(comp Component) {
	b.Components = append(b.Components, comp)
	comp.Prepare(b)
}

// Removes a component from this body.
func (b *Body) RemoveComponent(comp Component) {
	b.Components = slices.DeleteFunc(b.Components, func(c Component) bool {
		return c == comp
	})
}

// This function should add all internal forces to the Force member
// variable of each PointMass in the body.
//
// These should be forces that try to maintain the shape of the body.
func (b *Body) AccumulateInternalForces(relaxing bool) {
	for _, component := range b.Components {
		if !component.Disabled() {
			component.AccumulateInternalForces(b, relaxing)
		}
	}
}

// This function should add all external forces to the Force member
// variable of each PointMass in the body.
//
// These are external forces acting on the PointMasses, such as gravity,
// etc.
func (b *Body) AccumulateExternalForces(world *World) {
	for _, component := range b.Components {
		if !component.Disabled() {
			component.AccumulateExternalForces(b, world)
		}
	}
}

// Updates the edges and normals of this body.
func (b *Body) updateEdgesAndNormals() {
	b.updateEdges()
	b.updateNormals()
}

func (b *Body) updateEdges() {
	for _, edge := range b.Edges {
		start := b.PointMasses[edge.StartPointIndex].Position
		end := b.PointMasses[edge.EndPointIndex].Position
		difference := end.Sub(start)
		lengthSquared := difference.MagSq()
		length := math.Hypot(difference.X, difference.Y)

		edge.Start = start
		edge.End = end
		edge.Difference = difference
		if lengthSquared >= epsilonUnit && math.Abs(lengthSquared-1) >= epsilonUnit {
			edge.Difference = difference.Scale(1 / length)
		}
		edge.Normal = edge.Difference.Perp()
		edge.Length = length
		edge.LengthSquared = length * length
	}
	b.rebuildEdgeTree()
}

const edgeTreeLeafSize = 4

type edgeTreeNode struct {
	bounds      AABB
	start, end  int
	left, right int
}

// rebuildEdgeTree builds a bounding-volume hierarchy over the body's current
// edges. It is rebuilt alongside edge data, so collision queries can prune
// edges that cannot improve the current closest distance.
func (b *Body) rebuildEdgeTree() {
	if len(b.Edges) == 0 {
		b.edgeTree = b.edgeTree[:0]
		b.edgeTreeIndices = b.edgeTreeIndices[:0]
		return
	}
	if cap(b.edgeTreeIndices) < len(b.Edges) {
		b.edgeTreeIndices = make([]int, len(b.Edges))
	} else {
		b.edgeTreeIndices = b.edgeTreeIndices[:len(b.Edges)]
	}
	for i := range b.edgeTreeIndices {
		b.edgeTreeIndices[i] = i
	}
	b.edgeTree = b.edgeTree[:0]

	var build func(start, end int) int
	build = func(start, end int) int {
		nodeIndex := len(b.edgeTree)
		b.edgeTree = append(b.edgeTree, edgeTreeNode{left: -1, right: -1})
		bounds := b.edgeBounds(b.edgeTreeIndices[start])
		for _, edgeIndex := range b.edgeTreeIndices[start+1 : end] {
			bounds.ExpandToInclude(b.edgeBounds(edgeIndex).Min)
			bounds.ExpandToInclude(b.edgeBounds(edgeIndex).Max)
		}
		if end-start <= edgeTreeLeafSize {
			b.edgeTree[nodeIndex] = edgeTreeNode{bounds: bounds, start: start, end: end, left: -1, right: -1}
			return nodeIndex
		}
		width := bounds.Max.X - bounds.Min.X
		height := bounds.Max.Y - bounds.Min.Y
		sort.Slice(b.edgeTreeIndices[start:end], func(i, j int) bool {
			first := b.edgeBounds(b.edgeTreeIndices[start+i])
			second := b.edgeBounds(b.edgeTreeIndices[start+j])
			if width >= height {
				return first.midX() < second.midX()
			}
			return (first.Min.Y+first.Max.Y)/2 < (second.Min.Y+second.Max.Y)/2
		})
		middle := start + (end-start)/2
		left := build(start, middle)
		right := build(middle, end)
		b.edgeTree[nodeIndex] = edgeTreeNode{bounds: bounds, left: left, right: right}
		return nodeIndex
	}
	build(0, len(b.edgeTreeIndices))
}

func (b *Body) edgeBounds(edgeIndex int) AABB {
	edge := b.Edges[edgeIndex]
	return NewAABB(edge.Start.Min(edge.End), edge.Start.Max(edge.End))
}

// closestCollisionEdges finds the closest edges whose normals face away from
// and toward pointNormal. It uses the edge tree to avoid scanning edges whose
// bounds cannot beat either current closest distance.
func (b *Body) closestCollisionEdges(pt, pointNormal Vec2) (away, same CollisionInfo, foundAway bool) {
	closestAway, closestSame := Infinity, Infinity
	away.BodyBpmA, away.BodyBpmB = -1, -1
	same.BodyBpmA, same.BodyBpmB = -1, -1
	if len(b.edgeTree) == 0 {
		return away, same, false
	}
	var visit func(int)
	visit = func(nodeIndex int) {
		node := b.edgeTree[nodeIndex]
		nodeDistance := pointAABBDistanceSq(pt, node.bounds)
		if nodeDistance >= closestAway && nodeDistance >= closestSame {
			return
		}
		if node.left < 0 {
			for _, edgeIndex := range b.edgeTreeIndices[node.start:node.end] {
				hitPt, normal, edgeD, distance := b.ClosestPointOnEdgeSq(pt, edgeIndex)
				if pointNormal.Dot(normal) <= 0 {
					if distance < closestAway {
						closestAway = distance
						away = CollisionInfo{BodyBpmA: edgeIndex, BodyBpmB: (edgeIndex + 1) % len(b.PointMasses), EdgeD: edgeD, HitPt: hitPt, Normal: normal, Penetration: distance}
						foundAway = true
					}
				} else if distance < closestSame {
					closestSame = distance
					same = CollisionInfo{BodyBpmA: edgeIndex, BodyBpmB: (edgeIndex + 1) % len(b.PointMasses), EdgeD: edgeD, HitPt: hitPt, Normal: normal, Penetration: distance}
				}
			}
			return
		}
		leftDistance := pointAABBDistanceSq(pt, b.edgeTree[node.left].bounds)
		rightDistance := pointAABBDistanceSq(pt, b.edgeTree[node.right].bounds)
		if leftDistance <= rightDistance {
			visit(node.left)
			visit(node.right)
		} else {
			visit(node.right)
			visit(node.left)
		}
	}
	visit(0)
	return away, same, foundAway
}

func pointAABBDistanceSq(point Vec2, box AABB) float64 {
	dx := max(box.Min.X-point.X, 0, point.X-box.Max.X)
	dy := max(box.Min.Y-point.Y, 0, point.Y-box.Max.Y)
	return dx*dx + dy*dy
}

// Updates the point normals of the body.
func (b *Body) updateNormals() {
	if len(b.Edges) == 0 {
		return
	}
	prev := b.Edges[len(b.Edges)-1]
	for i, curEdge := range b.Edges {
		edge1N := prev.Difference
		edge2N := curEdge.Difference
		sum := edge1N.Add(edge2N)
		if sum == (Vec2{}) {
			b.PointMasses[i].Normal = edge1N
		} else {
			b.PointMasses[i].Normal = sum.Perp().Unit()
		}
		prev = curEdge
	}
}

// UpdateAABB updates the body's AABB with velocity padding for the given
// timestep. Called automatically by [World.Update]. Use forceUpdate to
// force update even for static bodies.
func (b *Body) UpdateAABB(elapsed float64, forceUpdate bool) {
	if b.IsStatic && !forceUpdate {
		return
	}
	b.AABB.Clear()
	for _, point := range b.PointMasses {
		b.AABB.ExpandToInclude(point.Position)
		if !b.IsStatic {
			b.AABB.ExpandToInclude(point.Position.Add(point.Velocity.Scale(elapsed)))
		}
	}
	b.bitmasksStale = true
}

// SetMassAll sets the mass for all [Body.PointMasses] elements in the body.
func (b *Body) SetMassAll(mass float64) {
	for i := range b.PointMasses {
		b.PointMasses[i].Mass = mass
	}
	b.IsStatic = math.IsInf(mass, 1)
}

// SetMassByIndex sets the mass of the point mass at the given index.
//
// If any point mass has an infinite mass, the body is marked as static.
func (b *Body) SetMassByIndex(pointMassIndex int, mass float64) {
	b.PointMasses[pointMassIndex].Mass = mass
	b.IsStatic = false
	for _, pm := range b.PointMasses {
		if math.IsInf(pm.Mass, 0) {
			b.IsStatic = true
			break
		}
	}
}

// SetMassesFromSlice sets the masses of the point masses from the given
// slice, up to the smaller of the two lengths.
//
// If any point mass has an infinite mass, the
// body is marked as static.
func (b *Body) SetMassesFromSlice(masses []float64) {
	n := min(len(b.PointMasses), len(masses))
	for i := range n {
		b.PointMasses[i].Mass = masses[i]
	}
	b.IsStatic = false
	for _, pm := range b.PointMasses {
		if math.IsInf(pm.Mass, 0) {
			b.IsStatic = true
			break
		}
	}
}

// Sets the scale, angle and position of the body manually.
//
// Setting the position and angle resets the current shape to the original
// base shape of the object.
func (b *Body) SetScaleAnglePosition(scale Vec2, angle float64, pos Vec2) {
	matrix := NewMatrix3x3(scale, angle, pos)
	b.BaseShape.TransformByMatrixToTarget(b.GlobalShape, matrix)
	for i := range b.PointMasses {
		b.PointMasses[i].Position = b.GlobalShape[i]
	}
	b.updateEdges()
	b.DerivedPos = pos
	b.DerivedAngle = angle
	// Forcefully update the AABB when changing shapes
	if b.IsStatic {
		b.UpdateAABB(0, true)
	}
	b.bitmasksStale = true
}

// DerivePositionAndAngle derives the global position and angle of this body,
// based on the average of all the points.
//
// This updates the [Body.derivedPos], [Body.derivedAngle], and [Body.derivedVel]
// fields.
//
// This is called by [World.Update], so usually a user does not need to call
// this. Instead you can just use the [Body.DerivedPos], [Body.DerivedAngle],
// [Body.DerivedVel], and [Body.DerivedOmega] getter methods.
func (b *Body) DerivePositionAndAngle(elapsed float64) {
	if b.IsStatic || b.IsKinematic {
		return
	}

	currentDerivedPosition := AveragePointMassPosition(b.PointMasses)
	if !b.IsPinned {
		b.DerivedPos = currentDerivedPosition
		b.DerivedVel = AveragePointMassVelocity(b.PointMasses)
	}

	if !b.FreeRotate {
		return
	}

	meanPos := b.DerivedPos
	if b.IsPinned {
		meanPos = currentDerivedPosition
	}

	var angle float64 = 0
	var originalAngle float64 = 0
	c := len(b.PointMasses)
	baseVerts := b.BaseShape

	for idx := range c {
		base := baseVerts[idx]
		pm := b.PointMasses[idx]
		cur := pm.Position.Sub(meanPos)
		cross := base.X*cur.Y - base.Y*cur.X
		dot := base.X*cur.X + base.Y*cur.Y
		thisAngle := math.Atan2(cross, dot)

		if idx == 0 {
			originalAngle = thisAngle
		} else {
			diff := thisAngle - originalAngle
			if diff > Pi {
				thisAngle -= Tau
			} else if diff < -Pi {
				thisAngle += Tau
			}
		}
		angle += thisAngle
	}

	angle /= float64(c)
	b.DerivedAngle = angle

	angleChange := b.DerivedAngle - b.lastAngle
	if angleChange > Pi {
		angleChange -= Tau
	} else if angleChange < -Pi {
		angleChange += Tau
	}

	b.DerivedOmega = angleChange / elapsed
	b.lastAngle = b.DerivedAngle
}

// Integrates the point masses for this Body.
// Ignored, if body is static.
func (b *Body) Integrate(elapsed float64) {
	if b.IsStatic {
		return
	}
	for i := range b.PointMasses {
		b.PointMasses[i].Integrate(elapsed)
	}
	b.bitmasksStale = true
}

// Applies the velocity damping to the point masses.
// Ignored, if body is static.
func (b *Body) DampenVelocity(elapsed float64) {
	if b.IsStatic {
		return
	}
	for i := range b.PointMasses {
		pointMass := b.PointMasses[i]
		delta := pointMass.Velocity.Sub(pointMass.Velocity.Scale(b.VelDamping)).Scale(-(elapsed * 200))
		b.AddVelocityToPointAt(delta, i)
	}
}

// Applies a rotational clockwise torque of a given force on this body.
// Ignored, if body is static.
func (b *Body) ApplyTorque(force float64) {
	if b.IsStatic {
		return
	}
	for _, pm := range b.PointMasses {
		pm.ApplyForce(pm.Position.Sub(b.DerivedPos).Unit().Perp().Scale(force))
	}
}

// Sets the angular velocity for this body.
// Ignored, if body is static.
//
// The method keeps the average velocity of the point masses the same during the procedure.
func (b *Body) SetAngularVelocity(vel float64) {
	if b.IsStatic {
		return
	}
	for i, pm := range b.PointMasses {
		diff := pm.Position.Sub(b.DerivedPos).Unit().Perp()
		b.SetPointVelocityAt(b.DerivedVel.Add(diff.Scale(vel)), i)
	}
}

// Accumulates the angular velocity for this body
func (b *Body) AddAngularVelocity(vel float64) {
	if b.IsStatic {
		return
	}
	for i, pm := range b.PointMasses {
		diff := pm.Position.Sub(b.DerivedPos).Unit().Perp()
		b.AddVelocityToPointAt(diff.Scale(vel), i)
	}
}

// Returns whether a global point is inside this body.
func (b *Body) Contains(pt Vec2) bool {
	if !b.AABB.Contains(pt) {
		return false
	}
	var endPt Vec2
	inside := false
	if pt.X < b.AABB.midX() {
		endPt = Vec2{X: b.AABB.Min.X - 0.1, Y: pt.Y}
		for _, e := range b.Edges {
			edgeSt := e.Start
			edgeEnd := e.End
			if edgeSt.X > pt.X && edgeEnd.X > pt.X {
				continue
			}
			if (edgeSt.Y <= pt.Y && edgeEnd.Y > pt.Y) || (edgeSt.Y > pt.Y && edgeEnd.Y <= pt.Y) {
				slope := (edgeEnd.X - edgeSt.X) / (edgeEnd.Y - edgeSt.Y)
				hitX := edgeSt.X + ((pt.Y - edgeSt.Y) * slope)
				if hitX <= pt.X && hitX >= endPt.X {
					inside = !inside
				}
			}
		}
	} else {
		endPt = Vec2{X: b.AABB.Max.X + 0.1, Y: pt.Y}
		for _, e := range b.Edges {
			edgeSt := e.Start
			edgeEnd := e.End
			if edgeSt.X < pt.X && edgeEnd.X < pt.X {
				continue
			}
			if (edgeSt.Y <= pt.Y && edgeEnd.Y > pt.Y) || (edgeSt.Y > pt.Y && edgeEnd.Y <= pt.Y) {
				slope := (edgeEnd.X - edgeSt.X) / (edgeEnd.Y - edgeSt.Y)
				hitX := edgeSt.X + ((pt.Y - edgeSt.Y) * slope)
				if hitX >= pt.X && hitX <= endPt.X {
					inside = !inside
				}
			}
		}
	}
	return inside
}

// Returns whether the given line consisting of two points intersects this body.
func (b *Body) IntersectsLine(start, end Vec2) bool {
	if !b.AABB.Intersects(NewAABB(start.Min(end), start.Max(end))) {
		return false
	}
	for _, edge := range b.Edges {
		if _, ok := LineIntersect(start, end, edge.Start, edge.End); ok {
			return true
		}
	}
	return false
}

// Tests a ray starting and ending at a given interval, returning the point
// at which the ray intersects this body the closest to `start`.
//
// If the ray does not crosses this body, `nil` is returned, instead.
func (b *Body) Raycast(start, end Vec2) (closestHit Vec2, ok bool) {
	if !b.AABB.Intersects(NewAABBOf(start, end)) {
		return Vec2{}, false
	}
	var p1, p2 Vec2
	var hasHit bool
	for _, e := range b.Edges {
		p1 = e.Start
		p2 = e.End
		rayEnd := end
		if hasHit {
			rayEnd = closestHit
		}
		if result, ok := LineIntersect(start, rayEnd, p1, p2); ok {
			closestHit = result.HitPt
			hasHit = true
		}
	}
	if !hasHit {
		return Vec2{}, false
	}
	return closestHit, true
}

// ClosestPointOnEdgeSq finds the closest point on a specific edge of the body
// to the given global point.
//
// Precondition: len([Body.PointMasses]) > 0.
//
// Parameters:
//   - pt: The point in world coordinates to find the closest edge point to
//   - edgeNum: The index of the edge to search
//
// Returns:
//   - hitPoint: The closest point on the edge to the given point
//   - normal: A unit vector representing the normal of the edge
//   - edgeD: The ratio along the edge where the point was found, in range [0, 1]
//   - distance: The squared distance to the closest edge point
func (b *Body) ClosestPointOnEdgeSq(pt Vec2, edgeNum int) (hitPoint, normal Vec2, edgeD, distance float64) {
	edge := b.Edges[edgeNum]
	ptA := edge.Start
	ptB := edge.End
	toP := pt.Sub(ptA)
	normal = edge.Normal
	x := toP.Dot(edge.Difference)
	if x <= 0.0 {
		distance = pt.DistSq(ptA)
		hitPoint = ptA
		edgeD = 0
	} else if x >= edge.Length {
		distance = pt.DistSq(ptB)
		hitPoint = ptB
		edgeD = 1
	} else {
		pd := toP.Dot(edge.Normal)
		distance = pd * pd
		hitPoint = ptA.Add(edge.Difference.Scale(x))
		edgeD = x / edge.Length
	}
	return hitPoint, normal, edgeD, distance
}

// Given a global point, finds the closest point on an edge of a specified
// index, returning the distance to the edge found.
//
// Precondition: len([Body.PointMasses]) > 0.
//
// Parameters:
//   - pt: The point to get the closest edge of, in world coordinates
//   - edgeNum: The index of the edge to search
//
// Returns:
//   - hitPoint: The closest point in the edge to the global point provided
//   - normal: A unit vector containing information about the normal of the edge found
//   - edgeD: The ratio of the edge where the point was grabbed, [0-1] inclusive
//   - distance: The distance to the closest edge found
func (b *Body) ClosestPointOnEdge(pt Vec2, edgeNum int) (hitPoint, normal Vec2, edgeD, distance float64) {
	hitPoint, normal, edgeD, sqDist := b.ClosestPointOnEdgeSq(pt, edgeNum)
	return hitPoint, normal, edgeD, math.Sqrt(sqDist)
}

// ClosestPoint finds the closest point on any edge of the body to the
// given global point. It returns detailed information about the edge,
// including its endpoints, normal, and the ratio along the edge.
//
// Precondition: len([Body.PointMasses]) > 0.
//
// Parameters:
//   - pt: The point in world coordinates to find the closest point to
//
// Returns:
//   - hitPoint: The closest point on the body's surface
//   - normal: The unit normal vector of the edge at the closest point
//   - pointA: The index of the first endpoint of the edge
//   - pointB: The index of the second endpoint of the edge
//   - edgeD: The ratio along the edge [0,1] where the closest point lies
//   - distance: The Euclidean distance from pt to the closest point
func (b *Body) ClosestPoint(pt Vec2) (hitPoint, normal Vec2, pointA, pointB int, edgeD, distance float64) {
	pointA = -1
	pointB = -1
	var edgeDVal float64 = 0
	normal = Vec2{}
	hitPoint = Vec2{}
	closestD := Infinity
	c := len(b.PointMasses)
	for i := range c {
		tempHit, tempNorm, tempEdgeD, dist := b.ClosestPointOnEdgeSq(pt, i)
		if dist < closestD {
			closestD = dist
			pointA = i
			pointB = (i + 1) % c
			edgeDVal = tempEdgeD
			normal = tempNorm
			hitPoint = tempHit
		}
	}
	return hitPoint, normal, pointA, pointB, edgeDVal, math.Sqrt(closestD)
}

// ClosestEdge finds the point on any of the body's edges that is closest to
// point. point must be given in world coordinates.
//
// tolerance limits how far an edge may be from point to be considered: any
// edge whose closest point is farther than tolerance is ignored. Pass
// math.Inf(1) to consider every edge regardless of distance (this mirrors
// the original default behavior). Passing 0 means no edge will ever be
// accepted, since a distance can never be strictly less than 0 — this is
// rarely what you want, so double-check the value before hardcoding 0.
//
// Returns:
//   - edgePosition: the closest point on the edge to point
//   - edgeRatio: where along the edge that point falls, in [0, 1], with 0
//     at edgePoint1 and 1 at edgePoint2
//   - edgePoint1: index of the first point mass forming the edge
//   - edgePoint2: index of the second point mass forming the edge
//   - ok: false if the body has no edges or point masses, or if no edge
//     was found within tolerance — in that case the other return values
//     are zero values and should not be used
func (b *Body) ClosestEdge(point Vec2, tolerance float64) (edgePosition Vec2, edgeRatio float64, edgePoint1, edgePoint2 int, ok bool) {
	if len(b.Edges) == 0 || len(b.PointMasses) == 0 {
		return Vec2{}, 0, 0, 0, false
	}
	found := false
	closestP1 := 0
	closestP2 := 0
	edgePosition = Vec2{}
	edgeRatio = 0
	closestD := Infinity
	for _, edge := range b.Edges {
		pm := b.PointMasses[edge.StartPointIndex]
		// clamped
		aDotB := max(0, min(edge.Length, point.Sub(pm.Position).Dot(edge.Difference)))
		d := edge.Difference.Scale(aDotB)
		dis := point.Sub(pm.Position.Add(d))
		curD := dis.Mag()
		if curD < closestD && curD < tolerance {
			found = true
			closestP1 = edge.StartPointIndex
			closestP2 = edge.EndPointIndex
			edgePosition = pm.Position.Add(d)
			edgeRatio = aDotB / edge.Length
			closestD = curD
		}
	}
	if found {
		return edgePosition, edgeRatio, closestP1, closestP2, true
	}
	return Vec2{}, 0, 0, 0, false
}

// Find the closest PointMass index in this body, given a global point
func (b *Body) ClosestPointMass(pos Vec2) (point int, distance float64) {
	closestSQD := math.MaxFloat64
	closest := -1
	for i, point := range b.PointMasses {
		thisD := pos.DistSq(point.Position)
		if thisD < closestSQD {
			closestSQD = thisD
			closest = i
		}
	}
	return closest, math.Sqrt(closestSQD)
}

// ApplyForce applies a force to the body at `pt` in world coordinates.
// If `pt` is not at the center (`derivedPos`), torque is applied causing spin.
// Ignored if the body is static.
//
// Parameters:
//   - force: The force vector to apply
//   - pt: The world position where the force is applied. Use `derivedPos` for center.
func (b *Body) ApplyForceAtGlobalPoint(force Vec2, pt Vec2) {
	if b.IsStatic {
		return
	}
	torqueF := b.DerivedPos.Sub(pt).Dot(force.Perp())
	for i := range b.PointMasses {
		point := b.PointMasses[i]
		tempR := point.Position.Sub(pt).Perp()
		b.ApplyForceToPointAt(force.Add(tempR.Scale(torqueF)), i)
	}
}

// ApplyGlobalForce applies the same force to every point mass directly.
// No torque is calculated, so the body translates without rotating.
// Ignored if the body is static.
func (b *Body) ApplyGlobalForce(force Vec2) {
	if b.IsStatic {
		return
	}
	for i := range b.PointMasses {
		b.PointMasses[i].ApplyForce(force)
	}

}

// Adds a velocity vector to all the point masses in this body.
// Does nothing, if body is static.
func (b *Body) AddVelocity(velocity Vec2) {
	if b.IsStatic {
		return
	}
	for i := range b.PointMasses {
		b.AddVelocityToPointAt(velocity, i)
	}
}

// SetAverageVelocity modifies the average velocity of all point masses to a
// certain value.
//
// The method keeps the individual difference of velocity between the point
// masses and the average body velocity while making the operation.
//
// Does nothing if the body is static.
//
// Parameters:
//   - velocity: The velocity to set. Set to Zero to reset average velocity of
//     the body to 0.
func (b *Body) SetAverageVelocity(velocity Vec2) {
	if b.IsStatic {
		return
	}
	for i, pointMass := range b.PointMasses {
		diff := pointMass.Velocity.Sub(b.DerivedVel)
		b.SetPointVelocityAt(velocity.Add(diff), i)
	}
}
func (b *Body) Reset() {
	if b.IsStatic {
		return
	}
	for i, pm := range b.PointMasses {
		pm.Velocity = Vec2{}
		pm.Position = b.GlobalShape[i]
		pm.Mass = 1.0
		pm.Force = Vec2{}
	}
}

// Applies a relative velocity change to a single point mass at the given index..
func (b *Body) ApplyForceToPointAt(force Vec2, pointMassIndex int) {
	b.PointMasses[pointMassIndex].ApplyForce(force)
}

// Adds velocity to the current velocity of a single point mass.
func (b *Body) AddVelocityToPointAt(velocity Vec2, pointMassIndex int) {
	b.PointMasses[pointMassIndex].Velocity = b.PointMasses[pointMassIndex].Velocity.Add(velocity)
}

// Sets the absolute velocity of a single point mass.
func (b *Body) SetPointVelocityAt(velocity Vec2, pointMassIndex int) {
	b.PointMasses[pointMassIndex].Velocity = velocity
}

// Sets the absolute position of a single point mass.
func (b *Body) SetPointPositionAt(position Vec2, pointMassIndex int) {
	b.PointMasses[pointMassIndex].Position = position
	b.bitmasksStale = true
}

// Translates [PointMass.Position] at index i
func (b *Body) TranslatePointAt(offset Vec2, i int) {
	b.PointMasses[i].Position = b.PointMasses[i].Position.Add(offset)
	b.bitmasksStale = true
}

// GetComponent returns the first [Component] attached to this [Body] matching type T, or nil if not found.
//
// Example:
//
//	if springComp := body.GetComponent[*jel.SpringComponent](); springComp != nil {
//		fmt.Println(springComp.EdgeSpringsCount)
//	}
func (b *Body) GetComponent[T Component]() (component T) {
	for _, comp := range b.Components {
		if c, ok := comp.(T); ok {
			return c
		}
	}
	return
}

// BodyEdge contains information about the edge of a body.
type BodyEdge struct {
	// EdgeIndex is the index of the edge on the body.
	EdgeIndex int
	// StartPointIndex is the index of the start point mass of this edge on the
	// [Body]'s [Body.PointMasses] slice.
	StartPointIndex int
	// EndPointIndex is the index of the end point mass of this edge on the
	// body's [Body.PointMasses] slice.
	EndPointIndex int
	// Start is the start position of the edge.
	Start Vec2
	// End is the end position of the edge.
	End Vec2
	// Normal is the normal for the edge.
	Normal Vec2
	// Difference is the difference between the start and end points, normalized.
	Difference Vec2
	// Length is the edge's length.
	Length float64
	// LengthSquared is the edge's length, squared.
	LengthSquared float64
}

// NewBodyEdge creates and initializes a new [BodyEdge] with the given index,
// start point index, end point index, and start/end vectors.
//
// The [BodyEdge.Difference], [BodyEdge.Normal], [BodyEdge.Length] and
// [BodyEdge.LengthSquared] fields are automatically initialized from these
// values.
func NewBodyEdge(edgeIndex, startPointIndex, endPointIndex int, start, end Vec2) *BodyEdge {
	e := &BodyEdge{
		EdgeIndex:       edgeIndex,
		StartPointIndex: startPointIndex,
		EndPointIndex:   endPointIndex,
		Start:           start,
		End:             end,
	}
	e.Difference = end.Sub(start).Unit()
	e.Normal = e.Difference.Perp()
	e.Length = start.Dist(end)
	e.LengthSquared = e.Length * e.Length
	return e
}
