package jel

import (
	"math"
	"slices"
	"sort"
)

type World struct {
	// The bodies contained within this world
	Bodies []*Body
	// The joints contained within this world
	Joints []Joint
	// MaterialPairs is a 2D lookup table used to resolve collisions between different materials.
	// It is accessed using material IDs (e.g., MaterialPairs[idA][idB]) to determine
	// shared physical properties like friction, bounciness, and custom collision rules.
	MaterialPairs [][]MaterialPair
	// The default material pair for newly created materials
	DefaultMatPair MaterialPair
	// The object to report collisions to
	CollisionObserver CollisionObserver
	// The threshold at which penetrations are ignored, since they are far too
	// deep to be resolved without applying unreasonable forces that will
	// destabilize the simulation. Default is 0.3
	PenetrationThreshold float64

	worldLimits          AABB
	worldSize            Vec2
	worldGridStep        Vec2
	worldGridSubdivision int
	subdivVec            Vec2
	// Inverse of [World.worldGridStep] , for multiplication over coordinates when
	// projecting AABBs into the world grid.
	invWorldGridStep Vec2
	// Whether the world is currently in a relaxation pass.
	//
	// See [World.RelaxWorld] and [World.RelaxBodies]
	relaxing      bool
	materialCount int
	collisionList []CollisionInfo
	// broadPhaseCells maps spatial-grid cells to the indices of bodies whose
	// AABBs overlap that cell. broadPhasePairs and broadPhasePairKeys are
	// reusable scratch space used to generate unique candidate pairs.
	broadPhaseCells     map[uint64][]int
	broadPhasePairs     map[uint64]struct{}
	broadPhasePairKeys  []uint64
	broadPhaseOversized []int
}

// SetWorldLimits sets the boundaries of the simulation world.
// The world is divided into a grid for broad-phase collision detection,
// with the grid step size calculated based on the world size and subdivision count.
// The world is divided into worldGridSubdivision x worldGridSubdivision cells
// (default is 64x64 = 4096 cells).
//
// Parameters:
//   - min: The minimum corner of the world bounds.
//   - max: The maximum corner of the world bounds.
func (w *World) SetWorldLimits(min, max Vec2) {
	w.worldLimits, w.worldSize = NewAABB(min, max), max.Sub(min)
	// Divide the world into (by default) 4096 boxes (64 x 64) for broad-phase collision detection
	w.worldGridStep = w.worldSize.DivS(float64(w.worldGridSubdivision))
	w.invWorldGridStep = Vec2{X: 1 / w.worldGridStep.X, Y: 1 / w.worldGridStep.Y}
}

// Returns world limits
func (w *World) WorldLimits() AABB {
	return w.worldLimits
}

// Is world in relaxing phase?
func (w *World) IsRelaxing() bool {
	return w.relaxing
}

func (w *World) setWorldGridSubdivision(v int) {
	w.worldGridSubdivision = v
	w.subdivVec = Vec2{float64(v), float64(v)}
}

// NewWorld inits an returns empty world
func NewWorld() *World {
	w := &World{}
	w.setWorldGridSubdivision(64)
	w.subdivVec = Vec2{X: float64(w.worldGridSubdivision), Y: float64(w.worldGridSubdivision)}
	w.Reset()
	return w
}

// Reset resets the world's contents to their initial state and readies it to be loaded again.
func (w *World) Reset() {
	// Remove all joints - this is needed to avoid retain cycles
	for _, joint := range w.Joints {
		w.RemoveJoint(joint)
	}
	// Reset bodies
	for _, body := range w.Bodies {
		body.Joints = nil
	}
	w.Bodies = nil
	w.collisionList = nil
	w.broadPhaseCells = nil
	w.broadPhasePairs = nil
	w.broadPhasePairKeys = nil
	w.broadPhaseOversized = nil
	w.DefaultMatPair = DefaultMaterialPair()
	w.materialCount = 1
	w.MaterialPairs = [][]MaterialPair{{w.DefaultMatPair}}
	w.PenetrationThreshold = 0.3
	w.SetWorldLimits(Vec2{X: -20.0, Y: -20.0}, Vec2{X: 20.0, Y: 20.0})
}

// ---------- MATERIALS ---------- //

// Adds a new material to the world. All previous material data is kept intact.
func (w *World) AddMaterial() int {
	old := w.MaterialPairs
	w.materialCount++
	w.MaterialPairs = nil
	// replace old data.
	for i := 0; i < w.materialCount; i++ {
		w.MaterialPairs = append(w.MaterialPairs, nil)
		for j := 0; j < w.materialCount; j++ {
			if i < w.materialCount-1 && j < w.materialCount-1 {
				w.MaterialPairs[i] = append(w.MaterialPairs[i], old[i][j])
			} else {
				w.MaterialPairs[i] = append(w.MaterialPairs[i], w.DefaultMatPair)
			}
		}
	}
	return w.materialCount - 1
}

// Enables or disables collision between 2 materials.
func (w *World) SetMaterialPairCollide(a, b int, collide bool) {
	if a >= 0 && a < w.materialCount && b >= 0 && b < w.materialCount {
		w.MaterialPairs[a][b].Collide = collide
		w.MaterialPairs[b][a].Collide = collide
	}
}

// Sets the collision response variables for a pair of materials.
func (w *World) SetMaterialPairData(a, b int, friction, elasticity float64) {
	if a >= 0 && a < w.materialCount && b >= 0 && b < w.materialCount {
		w.MaterialPairs[a][b].Friction = friction
		w.MaterialPairs[a][b].Elasticity = elasticity
		w.MaterialPairs[b][a].Friction = friction
		w.MaterialPairs[b][a].Elasticity = elasticity
	}
}

// Sets a user function to call when 2 bodies of the given materials collide.
func (w *World) SetMaterialPairFilterCallback(a, b int, filter func(CollisionInfo, float64) bool) {
	if a >= 0 && a < w.materialCount && b >= 0 && b < w.materialCount {
		w.MaterialPairs[a][b].CollisionFilterFunc = filter
		w.MaterialPairs[b][a].CollisionFilterFunc = filter
	}
}

// AddBody adds a [Body] to the [World].
func (w *World) AddBody(body *Body) {
	if !containsBody(w.Bodies, body) {
		w.Bodies = append(w.Bodies, body)
	}
}

// AddBodies adds bodies to the world.
func (w *World) AddBodies(bodies ...*Body) {
	for _, body := range bodies {
		w.AddBody(body)
	}
}

// Removes a body from the world. Call this outside of an update to remove the body.
func (w *World) RemoveBody(body *Body) {
	idx := slices.Index(w.Bodies, body)
	if idx != -1 {
		w.Bodies = slices.Delete(w.Bodies, idx, idx+1)
	}
}

// Adds a joint to the world. Joints call this automatically during their initialization
func (w *World) AddJoint(joint Joint) {
	if !containsJoint(w.Joints, joint) {
		w.Joints = append(w.Joints, joint)
		joint.LinkA().Body().Joints = append(joint.LinkA().Body().Joints, joint)
		joint.LinkB().Body().Joints = append(joint.LinkB().Body().Joints, joint)
	}
}

// Removes a joint from the world
func (w *World) RemoveJoint(joint Joint) {
	joint.LinkA().Body().Joints = removeJoint(joint.LinkA().Body().Joints, joint)
	joint.LinkB().Body().Joints = removeJoint(joint.LinkB().Body().Joints, joint)
	w.Joints = removeJoint(w.Joints, joint)
}

// Returns `true` if the two given bodies are joined to one another.
// SONRA
func (w *World) AreBodiesJoined(body1, body2 *Body) bool {
	for _, j := range body1.Joints {
		if j.LinkA().Body() == body2 || j.LinkB().Body() == body2 {
			return true
		}
	}
	return false
}

// Finds the closest PointMass in the world to a given point
func (w *World) ClosestPointMass(pt Vec2, ignoreFunction func(*Body, int) bool) (*Body, int, bool) {
	var retBody *Body
	var retIdx int
	found := false
	closestD := math.MaxFloat64
	for _, body := range w.Bodies {
		pm, dist := body.ClosestPointMass(pt)
		if ignoreFunction != nil && ignoreFunction(body, pm) {
			continue
		}
		if dist < closestD {
			closestD = dist
			retBody = body
			retIdx = pm
			found = true
		}
	}
	return retBody, retIdx, found
}

// ClosestPoint returns the closest body and the nearest point on its surface
// to the given position. The hit point always lies on the body's boundary.
// Bodies can be excluded using ignoreFunction.
func (w *World) ClosestPoint(pt Vec2, ignoreFunction func(*Body) bool) (closestBody *Body, closestHitPoint Vec2, found bool) {
	for _, body := range w.Bodies {
		if ignoreFunction != nil && ignoreFunction(body) {
			continue
		}
		hitPoint, _, _, _, _, distance := body.ClosestPoint(pt)
		if !found || distance < closestHitPoint.Dist(pt) {
			closestBody = body
			closestHitPoint = hitPoint
			found = true
		}
	}
	return
}

// Given a global point, returns a body (if any) that contains this point.
// Useful for picking objects with a cursor, etc.
func (w *World) BodyUnder(pt Vec2, bitmask Bitmask) *Body {
	for _, body := range w.Bodies {
		if (bitmask == 0 || (body.Bitmask&bitmask) != 0) && body.Contains(pt) {
			return body
		}
	}
	return nil
}

// Given a global point, returns all bodies that contain this point.
// Useful for picking objects with a cursor, etc.
func (w *World) BodiesUnder(pt Vec2, bitmask Bitmask) []*Body {
	var result []*Body
	for _, b := range w.Bodies {
		if (bitmask == 0 || (b.Bitmask&bitmask) != 0) && b.Contains(pt) {
			result = append(result, b)
		}
	}
	return result
}

// Returns a vector of bodies intersecting with the given line.
func (w *World) BodiesIntersectingLine(start, end Vec2, bitmask Bitmask) []*Body {
	var result []*Body
	for _, body := range w.Bodies {
		w.updateBodyBitmask(body)
		if (bitmask == 0 || (body.Bitmask&bitmask) != 0) && body.IntersectsLine(start, end) {
			result = append(result, body)
		}
	}
	return result
}

// BodiesIntersectingShape returns all bodies that overlap a given shape
// at a specified point in world coordinates.
//
// This method is optimized for zero-allocation performance. It utilizes pre-allocated
// slices for both the result set and the vertex transformations to prevent heap
// allocations and reduce Garbage Collector overhead during the game loop.
//
// Parameters:
//   - Shape: A shape that represents the segments to query.
//     Must contain at least 2 vertices.
//   - worldPos: The location in world coordinates to apply to the shape
//     when performing the query.
//   - ignoreTest: An optional function applied to every body intersecting
//     the shape to filter out results. If the function returns true, the body
//     is ignored. Defaults to nil.
//   - outResults: A pre-allocated slice where the intersecting bodies will be
//     appended. To reuse this slice across frames, it should be cleared beforehand
//     (e.g., results = results[:0]).
//   - tempBuffer: A pre-allocated slice used internally to store the transformed
//     vertices of the shape. Its capacity must be greater than or equal
//     to the number of vertices in the shape.
//
// Returns:
//   - A slice containing all bodies that intersect with the shape. This is
//     the populated 'outResults' slice. If the shape contains less than 2
//     points, the unmodified 'outResults' slice is returned.
func (w *World) BodiesIntersectingShape(
	shape Shape,
	worldPos Vec2,
	ignoreTest func(*Body) bool,
	outResults []*Body,
	tempBuffer Shape,
) []*Body {
	if len(shape) < 2 {
		return outResults
	}

	if cap(tempBuffer) < len(shape) {
		tempBuffer = make(Shape, len(shape))
	}
	tempBuffer = tempBuffer[:len(shape)]

	shape.TranslateVerticesToTarget(tempBuffer, worldPos)

	shapeAABB := NewAABBFromPoints(tempBuffer)
	shapeBitmask := w.bitmask(shapeAABB)

	for _, body := range w.Bodies {
		w.updateBodyBitmask(body)
		if !w.bitmasksIntersect(shapeBitmask, bitmaskPair{body.BitmaskX, body.BitmaskY}) {
			continue
		}
		if !shapeAABB.Intersects(body.AABB) {
			continue
		}

		last := tempBuffer[len(tempBuffer)-1]
		for _, point := range tempBuffer {
			if body.IntersectsLine(last, point) {
				if ignoreTest != nil && ignoreTest(body) {
					break
				}
				outResults = append(outResults, body)
				break
			}
			last = point
		}
	}
	return outResults
}

// Casts a ray between the given points and returns the first body it comes
// in contact with.
//
// Parameters:
//   - start: The start point to cast the ray from, in world coordinates
//   - end: The end point to end the ray cast at, in world coordinates
//   - bitmask: An optional collision bitmask that filters the bodies to
//     collide using a bitwise AND (&) operation.
//     If the value specified is 0, collision filtering is ignored and all
//     bodies are considered for collision.
//   - ignoreTest: Optional function that will be called for each body along
//     the way (not guaranteed to execute in order of farthest to closest body)
//     that tests whether the body should be ignored during ray casting.
//
// Returns:
//   - An optional tuple containing the farthest point reached by the ray,
//     and a Body value specifying the body that was closest to the ray,
//     if it hit any body, or nil if it hit nothing.
func (w *World) RayCast(start, end Vec2, bitmask Bitmask, ignoreTest func(*Body) bool) (retPt Vec2, body *Body) {
	aabb := NewAABBFromPoints([]Vec2{start, end})
	aabbBitmask := w.bitmask(aabb)
	closestDistSq := math.MaxFloat64
	for _, b := range w.Bodies {
		if bitmask != 0 && (b.Bitmask&bitmask) == 0 {
			continue
		}
		w.updateBodyBitmask(b)
		if !w.bitmasksIntersect(aabbBitmask, bitmaskPair{b.BitmaskX, b.BitmaskY}) {
			continue
		}
		if !b.AABB.Intersects(aabb) {
			continue
		}
		if ignoreTest != nil && ignoreTest(b) {
			continue
		}
		ret, hit := b.Raycast(start, end)
		if !hit {
			continue
		}
		// Only keep this hit if it's actually closer to the ray origin
		// than the best one found so far - otherwise a later body in
		// w.Bodies can overwrite a closer hit from an earlier one.
		distSq := ret.DistSq(start)
		if distSq >= closestDistSq {
			continue
		}
		closestDistSq = distSq
		retPt = ret
		body = b
		aabb = NewAABBFromPoints([]Vec2{start, ret})
		aabbBitmask = w.bitmask(aabb)
	}
	return
}

// Updates the world by a specific timestep.
// This method performs body point mass force/velocity/position simulation,
// and collision detection & resolving.
//
// Parameters:
//   - elapsed: The elapsed time to update by, usually in 1/60ths of a second.
func (w *World) Update(elapsed float64) {
	w.update(elapsed, w.Bodies, w.Joints)
}

// Internal updating method
func (w *World) update(elapsed float64, bodies []*Body, joints []Joint) {
	// Update the bodies
	for _, body := range bodies {
		body.DerivePositionAndAngle(elapsed)
		// Only update edge and normals pre-accumulation if the body has
		// components - only components really use this information.
		if len(body.Components) > 0 {
			body.updateEdgesAndNormals()
			body.AccumulateExternalForces(w)
			body.AccumulateInternalForces(w.relaxing)
		} else {
			// We need these for the collision detection
			body.updateNormals()
		}
		body.Integrate(elapsed)
		// Static bodies retain their AABB and spatial bitmask until their shape or
		// transform changes, where those mutators explicitly force an update.
		body.UpdateAABB(elapsed, false)
		w.updateBodyBitmask(body)
	}
	// Update the joints
	for _, joint := range joints {
		joint.Resolve(elapsed)
	}
	for _, pair := range w.broadPhaseCandidates(bodies) {
		body1 := bodies[uint32(pair>>32)]
		body2 := bodies[uint32(pair)]
		w.collidePair(body1, body2)
	}
	if !w.relaxing { // Disabled during relaxation
		// Notify collisions that will happen
		if w.CollisionObserver != nil {
			w.CollisionObserver.BodiesDidCollide(w.collisionList)
		}
	}
	w.handleCollisions()
	for _, body := range bodies {
		body.DampenVelocity(elapsed)
	}
}

// broadPhaseCandidates returns all unique body-index pairs that share at least
// one spatial-grid cell. Sorting preserves the previous stable body-pair order,
// which is important because collision resolution mutates body positions.
func (w *World) broadPhaseCandidates(bodies []*Body) []uint64 {
	if w.broadPhaseCells == nil {
		w.broadPhaseCells = make(map[uint64][]int)
		w.broadPhasePairs = make(map[uint64]struct{})
	}
	for key, indices := range w.broadPhaseCells {
		w.broadPhaseCells[key] = indices[:0]
	}
	clear(w.broadPhasePairs)
	w.broadPhasePairKeys = w.broadPhasePairKeys[:0]
	w.broadPhaseOversized = w.broadPhaseOversized[:0]

	for i, body := range bodies {
		minX, minY, maxX, maxY, ok := w.gridCellRange(body.AABB)
		if !ok {
			continue
		}
		if (maxX-minX+1)*(maxY-minY+1) > broadPhaseOversizedCellThreshold {
			w.broadPhaseOversized = append(w.broadPhaseOversized, i)
			continue
		}
		for y := minY; y <= maxY; y++ {
			for x := minX; x <= maxX; x++ {
				key := uint64(uint32(y))<<32 | uint64(uint32(x))
				w.broadPhaseCells[key] = append(w.broadPhaseCells[key], i)
			}
		}
	}

	for _, indices := range w.broadPhaseCells {
		for i, first := range indices {
			for _, second := range indices[i+1:] {
				w.addBroadPhasePair(first, second)
			}
		}
	}
	// Large AABBs are deliberately not inserted into every cell they cover.
	// Instead, test each once against all bodies. This avoids enumerating the
	// same large-body pair once per shared grid cell.
	for _, first := range w.broadPhaseOversized {
		for second := range bodies {
			if first == second || !bodies[first].AABB.Intersects(bodies[second].AABB) {
				continue
			}
			w.addBroadPhasePair(first, second)
		}
	}
	sort.Slice(w.broadPhasePairKeys, func(i, j int) bool {
		return w.broadPhasePairKeys[i] < w.broadPhasePairKeys[j]
	})
	return w.broadPhasePairKeys
}

const broadPhaseOversizedCellThreshold = 16

func (w *World) addBroadPhasePair(first, second int) {
	if first > second {
		first, second = second, first
	}
	pair := uint64(uint32(first))<<32 | uint64(uint32(second))
	if _, exists := w.broadPhasePairs[pair]; exists {
		return
	}
	w.broadPhasePairs[pair] = struct{}{}
	w.broadPhasePairKeys = append(w.broadPhasePairKeys, pair)
}

func (w *World) gridCellRange(aabb AABB) (minX, minY, maxX, maxY int, ok bool) {
	if !aabb.Valid || w.worldGridSubdivision <= 0 ||
		math.IsNaN(aabb.Min.X) || math.IsNaN(aabb.Min.Y) ||
		math.IsNaN(aabb.Max.X) || math.IsNaN(aabb.Max.Y) {
		return 0, 0, 0, 0, false
	}
	toCell := func(value, min, inverseStep float64) int {
		return int(math.Floor((value - min) * inverseStep))
	}
	limit := w.worldGridSubdivision - 1
	minX = min(limit, max(0, toCell(aabb.Min.X, w.worldLimits.Min.X, w.invWorldGridStep.X)))
	minY = min(limit, max(0, toCell(aabb.Min.Y, w.worldLimits.Min.Y, w.invWorldGridStep.Y)))
	maxX = min(limit, max(0, toCell(aabb.Max.X, w.worldLimits.Min.X, w.invWorldGridStep.X)))
	maxY = min(limit, max(0, toCell(aabb.Max.Y, w.worldLimits.Min.Y, w.invWorldGridStep.Y)))
	return minX, minY, maxX, maxY, true
}

func (w *World) collidePair(body1, body2 *Body) {
	// bitmask filtering
	if (body1.Bitmask & body2.Bitmask) == 0 {
		return
	}
	// Another early-out: both bodies are static, or their spatial bitmasks do
	// not intersect. The grid already reduces candidates, but these checks are
	// retained for compatibility and cheap rejection.
	if (body1.IsStatic && body2.IsStatic) ||
		!w.bitmasksIntersect(bitmaskPair{body1.BitmaskX, body1.BitmaskY}, bitmaskPair{body2.BitmaskX, body2.BitmaskY}) {
		return
	}
	if !body1.AABB.Intersects(body2.AABB) || !w.MaterialPairs[body1.Material][body2.Material].Collide {
		return
	}
	for _, jt := range body1.Joints {
		if (jt.LinkA().Body() == body1 && jt.LinkB().Body() == body2) ||
			(jt.LinkB().Body() == body1 && jt.LinkA().Body() == body2) {
			if !jt.CollisionsAllowed() {
				return
			}
		}
	}
	w.bodyCollide(body1, body2)
	w.bodyCollide(body2, body1)
}

// Checks collision between two bodies, and store the collision information if they do
func (w *World) bodyCollide(bA, bB *Body) {
	for i, pmA := range bA.PointMasses {
		pt := pmA.Position
		if !bB.Contains(pt) {
			continue
		}
		infoAway, infoSame, found := bB.closestCollisionEdges(pt, pmA.Normal)
		infoAway.BodyA, infoAway.BodyApm, infoAway.BodyB = bA, i, bB
		infoSame.BodyA, infoSame.BodyApm, infoSame.BodyB = bA, i, bB
		if found && infoAway.Penetration > w.PenetrationThreshold && infoSame.Penetration < infoAway.Penetration {
			infoSame.Penetration = math.Sqrt(infoSame.Penetration)
			w.collisionList = append(w.collisionList, infoSame)
		} else {
			infoAway.Penetration = math.Sqrt(infoAway.Penetration)
			w.collisionList = append(w.collisionList, infoAway)
		}
	}
}
func (w *World) handleCollisions() {
	for _, info := range w.collisionList {
		bodyA := info.BodyA
		bodyB := info.BodyB
		A := bodyA.PointMasses[info.BodyApm]
		B1 := bodyB.PointMasses[info.BodyBpmA]
		B2 := bodyB.PointMasses[info.BodyBpmB]
		bVel := B1.Velocity.Add(B2.Velocity).DivS(2)
		relVel := A.Velocity.Sub(bVel)
		relDot := relVel.Dot(info.Normal)
		material := w.MaterialPairs[bodyA.Material][bodyB.Material]
		if !material.CollisionFilterFunc(info, relDot) {
			continue
		}
		if info.Penetration > w.PenetrationThreshold {
			if w.CollisionObserver != nil {
				w.CollisionObserver.BodyCollision(info, w.PenetrationThreshold)
			}
			continue
		}
		b1inf := 1.0 - info.EdgeD
		b2inf := info.EdgeD
		b2MassSum := B1.Mass + B2.Mass
		massSum := A.Mass + b2MassSum
		var Amove float64
		var Bmove float64
		if math.IsInf(A.Mass, 1) {
			Amove = 0
			Bmove = info.Penetration + 0.001
		} else if math.IsInf(b2MassSum, 1) {
			Amove = info.Penetration + 0.001
			Bmove = 0
		} else {
			Amove = info.Penetration * (b2MassSum / massSum)
			Bmove = info.Penetration * (A.Mass / massSum)
		}
		if !math.IsInf(A.Mass, 0) {
			bodyA.SetPointPositionAt(A.Position.Add(info.Normal.Scale(Amove)), info.BodyApm)
		}
		if !math.IsInf(B1.Mass, 0) {
			bodyB.SetPointPositionAt(B1.Position.Sub(info.Normal.Scale(Bmove*b1inf)), info.BodyBpmA)
		}
		if !math.IsInf(B2.Mass, 0) {
			bodyB.SetPointPositionAt(B2.Position.Sub(info.Normal.Scale(Bmove*b2inf)), info.BodyBpmB)
		}
		if relDot <= 0.0001 && (!math.IsInf(A.Mass, 1) || !math.IsInf(b2MassSum, 1)) {
			var AinvMass float64
			if math.IsInf(A.Mass, 1) {
				AinvMass = 0
			} else {
				AinvMass = 1.0 / A.Mass
			}
			var BinvMass float64
			if math.IsInf(b2MassSum, 1) {
				BinvMass = 0
			} else {
				BinvMass = 1.0 / b2MassSum
			}
			jDenom := AinvMass + BinvMass
			elas := 1 + material.Elasticity
			j := -(relVel.Scale(elas).Dot(info.Normal)) / jDenom
			tangent := info.Normal.Perp()
			friction := material.Friction
			f := (relVel.Dot(tangent)) * friction / jDenom
			if !math.IsInf(A.Mass, 0) {
				bodyA.AddVelocityToPointAt(
					info.Normal.Scale(j/A.Mass).Sub(tangent.Scale(f/A.Mass)),
					info.BodyApm,
				)
			}
			if !math.IsInf(b2MassSum, 0) {
				jComp := info.Normal.Scale(j).DivS(b2MassSum)
				fComp := tangent.Scale(f * b2MassSum)
				bodyB.AddVelocityToPointAt(jComp.Scale(b1inf).Sub(fComp.Scale(b1inf)).Neg(), info.BodyBpmA)
				bodyB.AddVelocityToPointAt(jComp.Scale(b2inf).Sub(fComp.Scale(b2inf)).Neg(), info.BodyBpmB)
			}
		}
	}
	w.collisionList = w.collisionList[:0]
}

func (w *World) bitmasksIntersect(b1, b2 bitmaskPair) bool {
	return ((b1.X & b2.X) != 0) && ((b1.Y & b2.Y) != 0)
}

type bitmaskPair struct {
	X, Y Bitmask
}

func (w *World) updateBodyBitmask(body *Body) {
	if !body.bitmasksStale {
		return
	}
	pair := w.bitmask(body.AABB)
	body.BitmaskX, body.BitmaskY = pair.X, pair.Y
	body.bitmasksStale = false
}

// bitmask maps an AABB to a pair of X/Y spatial grid bitmasks.
func (w *World) bitmask(aabb AABB) bitmaskPair {
	if math.IsNaN(aabb.Min.X) || math.IsNaN(aabb.Min.Y) ||
		math.IsNaN(aabb.Max.X) || math.IsNaN(aabb.Max.Y) {
		return bitmaskPair{0, 0}
	}
	minVec := aabb.Min.Sub(w.worldLimits.Min).Mul(w.invWorldGridStep).Min(w.subdivVec).Max(Vec2{})
	maxVec := aabb.Max.Sub(w.worldLimits.Min).Mul(w.invWorldGridStep).Min(w.subdivVec).Max(Vec2{})

	if math.IsNaN(minVec.X) || math.IsNaN(minVec.Y) ||
		math.IsNaN(maxVec.X) || math.IsNaN(maxVec.Y) {
		return bitmaskPair{0, 0}
	}
	minShiftX := ^uint(0) >> uint(max(0, 64-int(maxVec.X)))
	maxShiftX := ^uint(0) << uint(max(0, int(minVec.X)))
	bitmaskX := Bitmask(minShiftX & maxShiftX)
	bitmaskX.SetOn(int(minVec.X))
	bitmaskX.SetOn(int(maxVec.X))
	minShiftY := ^uint(0) >> uint(max(0, 64-int(maxVec.Y)))
	maxShiftY := ^uint(0) << uint(max(0, int(minVec.Y)))
	bitmaskY := Bitmask(minShiftY & maxShiftY)
	bitmaskY.SetOn(int(minVec.Y))
	bitmaskY.SetOn(int(maxVec.Y))
	return bitmaskPair{bitmaskX, bitmaskY}
}

// Relaxes all bodies in this simulation so they match a more approximate rest shape
// once simulation starts.
//
// This will move or change the position of each body after iterations are done.
// It performs collisions and joint resolving, and resets the velocities to 0 before finishing.
//
// All body joints, velocities, and components are executed, except those with relaxable == false.
//
// Parameters:
//   - iterations: The number of iterations of relaxation to apply.
//   - timestep:   The timestep (in seconds) of each iteration.
//
// Precondition: iterations > 0.
func (w *World) RelaxWorld(timestep float64, iterations int) {
	w.relaxing = true
	for i := 0; i <= iterations; i++ {
		w.Update(timestep)
	}
	w.relaxing = false
	for _, body := range w.Bodies {
		for i := 0; i < len(body.PointMasses); i++ {
			body.SetPointVelocityAt(Vec2{}, i)
		}
	}
}

// Relaxes a list of bodies in this simulation so they match a more approximate
// rest shape once simulation starts.
//
// This will move or change the position of each body after iterations are done.
// It performs collisions and joint resolving of only the bodies or joints that
// are related to the bodies array, and resets the velocities to 0 before finishing.
//
// Only body joints that involve bodies contained within the passed body list
// are executed. Joints that involve a body within this list and another body
// that is not in the list are not resolved during relaxation.
//
// All body joints, velocities, and components are executed, except those with
// relaxable == false.
//
// Parameters:
//   - bodies:    The list of bodies to relax.
//   - iterations: The number of iterations of relaxation to apply.
//   - timestep:  The timestep (in seconds) of each iteration.
//
// Precondition: iterations > 0.
func (w *World) RelaxBodies(bodies []*Body, timestep float64, iterations int) {
	w.relaxing = true
	var joints []Joint
	var existingJoints []Joint
	for _, b := range bodies {
		for _, j := range b.Joints {
			if containsBody(bodies, j.LinkA().Body()) && containsBody(bodies, j.LinkB().Body()) {
				existingJoints = append(existingJoints, j)
			}
		}
	}
	for _, joint := range existingJoints {
		if !containsJoint(joints, joint) {
			joints = append(joints, joint)
		}
	}
	for i := 0; i <= iterations; i++ {
		w.update(timestep, bodies, joints)
	}
	w.relaxing = false
	for _, body := range bodies {
		for i := 0; i < len(body.PointMasses); i++ {
			body.SetPointVelocityAt(Vec2{}, i)
		}
	}
}

func containsBody(bodies []*Body, b *Body) bool {
	return slices.Contains(bodies, b)
}

func containsJoint(joints []Joint, j Joint) bool {
	return slices.Contains(joints, j)
}
func removeJoint(joints []Joint, j Joint) []Joint {
	for i, x := range joints {
		if x == j {
			return append(joints[:i], joints[i+1:]...)
		}
	}
	return joints
}
