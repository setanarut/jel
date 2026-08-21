package jel

import (
	"math"
)

const (
	epsilonDistance float64 = 0.001
	penetrationSlop float64 = 0.001
)

var infinity float64 = math.Inf(1)

type Body interface {
	GetBaseBody() *BaseBody
	GetAABB() *AABB
	IsStatic() bool
	IsKinematic() bool
	GetClosestPointOnEdgeSquared(pt Vec2, edgeNum int) (dist, edgeD float64, hitPt, normal Vec2)
	GetClosestPointMass(pos Vec2) (closest int, dist float64)
	ContainstPoint(pt Vec2) bool
	GetPointMasses() []PointMass
	derivePositionAndAngle(elapsed float64)
	applyGravity(force Vec2)
	accumulateInternalForces(delta float64)
	integrate(elapsed float64)
	dampenVelocity()
	updateAABB(elapsed float64, forceUpdate bool)
	updateEdgeInfo()
	bitmaskX() *Bitmask
	bitmaskY() *Bitmask
	material() int
}

type Bitmask uint

func (b *Bitmask) SetOn(bit int) {
	shift := max(0, bit-1)
	*b |= (1 << shift)
}
func (b *Bitmask) Clear() { *b = 0 }

type PointMass struct {
	Pos   Vec2
	Vel   Vec2
	Force Vec2
	Mass  float64
}

func NewPointMass(mass float64, pos Vec2) PointMass {
	return PointMass{Mass: mass, Pos: pos}
}
func (p *PointMass) IntegrateForce(elapsed float64) {
	if p.Mass != infinity {
		elapMass := elapsed / p.Mass
		p.Vel = p.Vel.Add(p.Force.Scale(elapMass))
		p.Pos = p.Pos.Add(p.Vel.Scale(elapsed))
	}
	p.Force = Vec2{}
}

type EdgeInfo struct {
	Direction Vec2
	Length    float64
}
type Polygon struct {
	Vertices []Vec2
}

func NewPolygon(verts []Vec2) *Polygon {
	s := &Polygon{Vertices: make([]Vec2, 0)}
	if verts != nil {
		s.Vertices = append(s.Vertices, verts...)
	}
	s.ReCenter()
	return s
}
func (s *Polygon) Begin() {
	s.Vertices = s.Vertices[:0]
}
func (s *Polygon) AddVertex(vert Vec2) int {
	s.Vertices = append(s.Vertices, vert)
	return len(s.Vertices)
}
func (s *Polygon) ReCenter() {
	center := AverageVec2(s.Vertices)
	for i := range s.Vertices {
		s.Vertices[i] = s.Vertices[i].Sub(center)
	}
}
func AverageVec2(points []Vec2) Vec2 {
	if len(points) == 0 {
		return Vec2{}
	}
	sum := Vec2{}
	for i := range points {
		sum = sum.Add(points[i])
	}
	return sum.DivS(float64(len(points)))
}
func (s *Polygon) TransformVertices(worldPos Vec2, angle float64, localScale Vec2, outList []Vec2) {
	sin, cos := math.Sincos(angle)
	sx, sy := localScale.X, localScale.Y
	m00 := cos * sx
	m01 := -sin * sy
	m10 := sin * sx
	m11 := cos * sy
	for i := range s.Vertices {
		outList[i].X = s.Vertices[i].X*m00 + s.Vertices[i].Y*m01 + worldPos.X
		outList[i].Y = s.Vertices[i].X*m10 + s.Vertices[i].Y*m11 + worldPos.Y
	}
}

type BaseBody struct {
	BaseShape     *Polygon
	GlobalShape   []Vec2
	PointMasses   []PointMass
	EdgeInfo      []EdgeInfo
	Scale         Vec2
	DerivedPos    Vec2
	DerivedVel    Vec2
	DerivedAngle  float64
	DerivedOmega  float64
	LastAngle     float64
	AABB          *AABB
	MaterialID    int
	Static        bool
	Kinematic     bool
	IsPinned      bool
	IgnoreGravity bool
	VelDamping    float64
	UserData      any
	maskX         *Bitmask
	maskY         *Bitmask
	TotalMass     float64
}

func (b *BaseBody) GetBaseBody() *BaseBody {
	return b
}
func NewStaticBody(w *World, shape *Polygon, pos Vec2, angle float64, scale Vec2) *BaseBody {
	b := NewBaseBody(w, shape, infinity, pos, angle, scale, false)
	b.Static = true
	b.SetShape(shape, infinity)
	b.updateAABB(0, true)
	if w != nil {
		w.AddBody(b)
		w.updateBodyBitmask(b)
	}
	b.updateEdgeInfo()
	return b
}
func NewBaseBody(w *World, shape *Polygon, masses float64, pos Vec2, angle float64, scale Vec2, kinematic bool) *BaseBody {
	b := &BaseBody{
		AABB:         &AABB{Validity: true},
		DerivedPos:   pos,
		DerivedAngle: angle,
		LastAngle:    angle,
		Scale:        scale,
		Kinematic:    kinematic,
		VelDamping:   0.99,
		maskX:        new(Bitmask),
		maskY:        new(Bitmask),
		PointMasses:  make([]PointMass, 0),
	}
	b.SetShape(shape, masses)
	b.updateAABB(0, true)
	if w != nil {
		w.AddBody(b)
		w.updateBodyBitmask(b)
	}
	b.updateEdgeInfo()
	return b
}
func (b *BaseBody) GetPointMasses() []PointMass {
	return b.PointMasses
}
func (b *BaseBody) SetShape(shape *Polygon, masses float64) {
	b.BaseShape = shape
	if len(b.BaseShape.Vertices) != len(b.PointMasses) {
		b.PointMasses = make([]PointMass, 0)
		b.GlobalShape = make([]Vec2, len(b.BaseShape.Vertices))
		b.BaseShape.TransformVertices(b.DerivedPos, b.DerivedAngle, b.Scale, b.GlobalShape)
		totalMass := 0.0
		for i := range b.BaseShape.Vertices {
			pm := NewPointMass(masses, b.GlobalShape[i])
			b.PointMasses = append(b.PointMasses, pm)
			totalMass += pm.Mass
		}
		b.TotalMass = totalMass
		b.EdgeInfo = make([]EdgeInfo, len(b.PointMasses))
		b.updateEdgeInfo()
	}
}
func (b *BaseBody) SetPositionAngleScale(pos Vec2, angle float64, scale Vec2) {
	b.BaseShape.TransformVertices(pos, angle, scale, b.GlobalShape)
	for i := range b.PointMasses {
		b.PointMasses[i].Pos = b.GlobalShape[i]
	}
	b.DerivedPos = pos
	b.DerivedAngle = angle
}
func (b *BaseBody) derivePositionAndAngle(elapsed float64) {
	l := float64(len(b.PointMasses))
	revLength := 1.0 / l
	if !b.IsPinned {
		center := Vec2{}
		vel := Vec2{}
		for i := range b.PointMasses {
			center = center.Add(b.PointMasses[i].Pos)
			vel = vel.Add(b.PointMasses[i].Vel)
		}
		b.DerivedPos = center.Scale(revLength)
		b.DerivedVel = vel.Scale(revLength)
	}
	var angle float64
	originalSign := 1
	var originalAngle float64
	for i := range b.PointMasses {
		baseNorm := b.BaseShape.Vertices[i].Unit()
		curNorm := b.PointMasses[i].Pos.Sub(b.DerivedPos).Unit()
		dot := baseNorm.Dot(curNorm)
		dot = max(min(dot, 1.0), -1.0)
		thisAngle := math.Acos(dot)
		if curNorm.Dot(baseNorm.PerpNeg()) <= 0.0 {
			thisAngle = -thisAngle
		}
		if i == 0 {
			if thisAngle >= 0 {
				originalSign = 1
			} else {
				originalSign = -1
			}
			originalAngle = thisAngle
		} else {
			diff := thisAngle - originalAngle
			thisSign := 1
			if thisAngle < 0 {
				thisSign = -1
			}
			absDiff := math.Abs(diff)
			if absDiff > math.Pi && thisSign != originalSign {
				if thisSign == -1 {
					thisAngle = math.Pi + (math.Pi + thisAngle)
				} else {
					thisAngle = (math.Pi - thisAngle) - math.Pi
				}
			}
		}
		angle += thisAngle
	}
	angle /= l
	b.DerivedAngle = angle
	angleChange := b.DerivedAngle - b.LastAngle
	absAngleChange := math.Abs(angleChange)
	if absAngleChange >= math.Pi {
		if angleChange < 0 {
			angleChange += math.Pi * 2
		} else {
			angleChange -= math.Pi * 2
		}
	}
	b.DerivedOmega = angleChange / elapsed
	b.LastAngle = b.DerivedAngle
}

func (b *BaseBody) updateEdgeInfo() {
	pointMasses := b.PointMasses
	l := len(pointMasses)
	if len(b.EdgeInfo) != l {
		b.EdgeInfo = make([]EdgeInfo, l)
	}
	for i := range b.EdgeInfo {
		nextIdx := i + 1
		if nextIdx == l {
			nextIdx = 0
		}
		e := pointMasses[nextIdx].Pos.Sub(pointMasses[i].Pos)
		b.EdgeInfo[i] = EdgeInfo{
			Direction: e.Unit(),
			Length:    e.Mag(),
		}
	}
}
func (b *BaseBody) accumulateInternalForces(delta float64) {}
func (b *BaseBody) applyGravity(gravity Vec2) {
	if b.IgnoreGravity {
		return
	}
	for i := range b.PointMasses {
		pm := &b.PointMasses[i]
		pm.Force = pm.Force.Add(gravity.Scale(pm.Mass))
	}
}
func (b *BaseBody) integrate(elapsed float64) {
	for i := range b.PointMasses {
		b.PointMasses[i].IntegrateForce(elapsed)
	}
}
func (b *BaseBody) dampenVelocity() {
	for i := range b.PointMasses {
		pm := &b.PointMasses[i]
		pm.Vel = pm.Vel.Scale(b.VelDamping)
	}
}
func (b *BaseBody) updateAABB(elapsed float64, forceUpdate bool) {
	if !b.Static || forceUpdate {
		b.AABB.Clear()
		for i := range b.PointMasses {
			pm := &b.PointMasses[i]
			b.AABB.ExpandToIncludePos(pm.Pos)
			if !b.Static {
				nextPos := pm.Pos.Add(pm.Vel.Scale(elapsed))
				b.AABB.ExpandToIncludePos(nextPos)
			}
		}
	}
}
func (b *BaseBody) ContainstPoint(pt Vec2) bool {
	endPtX := b.AABB.Max.X + 0.1
	inside := false
	edgeSt := b.PointMasses[0].Pos
	var edgeEnd Vec2
	c := len(b.PointMasses)
	for i := range c {
		if i < c-1 {
			edgeEnd = b.PointMasses[i+1].Pos
		} else {
			edgeEnd = b.PointMasses[0].Pos
		}
		if (edgeSt.Y <= pt.Y && edgeEnd.Y > pt.Y) || (edgeSt.Y > pt.Y && edgeEnd.Y <= pt.Y) {
			slope := (edgeEnd.X - edgeSt.X) / (edgeEnd.Y - edgeSt.Y)
			hitX := edgeSt.X + ((pt.Y - edgeSt.Y) * slope)
			if hitX >= pt.X && hitX <= endPtX {
				inside = !inside
			}
		}
		edgeSt = edgeEnd
	}
	return inside
}
func (b *BaseBody) GetClosestPointOnEdgeSquared(pt Vec2, edgeNum int) (dist, edgeD float64, hitPt, normal Vec2) {
	ptA := b.PointMasses[edgeNum].Pos
	var ptB Vec2
	if edgeNum < len(b.PointMasses)-1 {
		ptB = b.PointMasses[edgeNum+1].Pos
	} else {
		ptB = b.PointMasses[0].Pos
	}
	toP := pt.Sub(ptA)
	e := b.EdgeInfo[edgeNum].Direction
	edgeLength := b.EdgeInfo[edgeNum].Length
	n := e.PerpNeg()
	x := toP.Dot(e)
	switch {
	case x <= 0.0:
		dist = pt.Sub(ptA).MagSq()
		hitPt = ptA
		edgeD = 0
	case x >= edgeLength:
		dist = pt.Sub(ptB).MagSq()
		hitPt = ptB
		edgeD = 1
	default:
		crossZ := toP.X*e.Y - toP.Y*e.X
		dist = crossZ * crossZ
		hitPt = Vec2{X: ptA.X + e.X*x, Y: ptA.Y + e.Y*x}
		edgeD = x / edgeLength
	}
	normal = n
	return
}
func (b *BaseBody) GetClosestPointMass(pos Vec2) (closest int, dist float64) {
	closestSQD := 100000.0
	closest = -1
	for i, pm := range b.PointMasses {
		thisD := pos.Sub(pm.Pos).MagSq()
		if thisD < closestSQD {
			closestSQD = thisD
			closest = i
		}
	}
	dist = math.Sqrt(closestSQD)
	return
}
func (b *BaseBody) GetAABB() *AABB     { return b.AABB }
func (b *BaseBody) IsStatic() bool     { return b.Static }
func (b *BaseBody) IsKinematic() bool  { return b.Kinematic }
func (b *BaseBody) bitmaskX() *Bitmask { return b.maskX }
func (b *BaseBody) bitmaskY() *Bitmask { return b.maskY }
func (b *BaseBody) material() int      { return b.MaterialID }

type InternalSpring struct {
	SpringMat
	RestLength float64
	IndexA     int
	IndexB     int
}
type SpringMat struct {
	Stiffness float64
	Damping   float64
}
type SpringBody struct {
	*BaseBody
	SpringBodyOptions
	Springs []InternalSpring
}
type SpringBodyOptions struct {
	SpringMat
	MassPerPoint        float64
	ShapeMatchStiffness float64
	ShapeMatchDamping   float64
	ShapeMatching       bool
}

func NewSpringBody(
	w *World,
	shape *Polygon,
	opts SpringBodyOptions,
	pos Vec2,
	angle float64,
	scale Vec2,
) *SpringBody {
	base := NewBaseBody(nil, shape, opts.MassPerPoint, pos, angle, scale, false)
	sb := &SpringBody{
		BaseBody:          base,
		Springs:           make([]InternalSpring, 0),
		SpringBodyOptions: opts,
	}
	sb.SetPositionAngleScale(pos, angle, scale)
	sb.buildDefaultSprings()
	if w != nil {
		w.AddBody(sb)
		w.updateBodyBitmask(sb)
	}
	return sb
}
func (s *SpringBody) AddInternalSpring(pointA, pointB int, mat SpringMat) {
	sp := InternalSpring{
		SpringMat:  mat,
		RestLength: s.PointMasses[pointB].Pos.Dist(s.PointMasses[pointA].Pos),
		IndexA:     pointA,
		IndexB:     pointB,
	}
	s.Springs = append(s.Springs, sp)
}
func (s *SpringBody) buildDefaultSprings() {
	s.Springs = make([]InternalSpring, 0)
	for i := 0; i < len(s.PointMasses); i++ {
		if i < len(s.PointMasses)-1 {
			s.AddInternalSpring(i, i+1, s.SpringMat)
		} else {
			s.AddInternalSpring(i, 0, s.SpringMat)
		}
	}
}
func (s *SpringBody) accumulateInternalForces(delta float64) {
	if s.Static || s.Kinematic {
		return
	}
	s.BaseBody.accumulateInternalForces(delta)

	var forceOut Vec2

	// Mevcut spring kuvvetleri
	for i := range s.Springs {
		spr := &s.Springs[i]
		a := &s.PointMasses[spr.IndexA]
		b := &s.PointMasses[spr.IndexB]
		forceOut = CalculateSpringForce(
			a.Pos, a.Vel,
			b.Pos, b.Vel,
			spr.RestLength,
			spr.Stiffness,
			spr.Damping,
		)
		a.Force = a.Force.Add(forceOut)
		b.Force = b.Force.Sub(forceOut)
	}

	// Shape matching (C# mantığı ile)
	if s.ShapeMatching && s.ShapeMatchStiffness > 0 {
		// Global şekli güncelle
		s.BaseShape.TransformVertices(s.DerivedPos, s.DerivedAngle, s.Scale, s.GlobalShape)

		for i := range s.PointMasses {
			p := &s.PointMasses[i]
			targetPos := s.GlobalShape[i]

			// Hedef hız = kütle merkezi hızı + dönme katkısı (ω × r)
			r := targetPos.Sub(s.DerivedPos)
			rotVel := Vec2{
				X: -s.DerivedOmega * r.Y,
				Y: s.DerivedOmega * r.X,
			}
			targetVel := s.DerivedVel.Add(rotVel)

			if !s.Kinematic {
				forceOut = CalculateSpringForce(
					p.Pos, p.Vel,
					targetPos, targetVel,
					0.0,
					s.ShapeMatchStiffness,
					s.ShapeMatchDamping,
				)
			} else {
				forceOut = CalculateSpringForce(
					p.Pos, p.Vel,
					targetPos, Vec2{},
					0.0,
					s.ShapeMatchStiffness,
					s.ShapeMatchDamping,
				)
			}
			p.Force = p.Force.Add(forceOut)
		}
	}
}

type PressureBody struct {
	*SpringBody
	GasPressure    float64
	Volume         float64
	NormalList     []Vec2
	EdgeLengthList []float64
}
type PressureBodyOptions struct {
	SpringBodyOptions
	GasPressure float64
}

func NewPressureBody(
	w *World,
	shape *Polygon,
	opts PressureBodyOptions,
	pos Vec2,
	angle float64,
	scale Vec2,
) *PressureBody {
	sb := NewSpringBody(nil, shape, opts.SpringBodyOptions, pos, angle, scale)
	pb := &PressureBody{
		SpringBody:     sb,
		GasPressure:    opts.GasPressure,
		NormalList:     make([]Vec2, len(sb.PointMasses)),
		EdgeLengthList: make([]float64, len(sb.PointMasses)),
	}
	if w != nil {
		w.AddBody(pb)
		w.updateBodyBitmask(sb)
	}
	return pb
}
func (p *PressureBody) accumulateInternalForces(e float64) {
	p.SpringBody.accumulateInternalForces(e)
	p.Volume = 0
	var edge1, edge2, norm Vec2
	l := len(p.PointMasses)
	for i := range l {
		prev := i - 1
		if i == 0 {
			prev = l - 1
		}
		next := (i + 1) % l
		edge1 = p.PointMasses[i].Pos.Sub(p.PointMasses[prev].Pos).PerpNeg()
		edge2 = p.PointMasses[next].Pos.Sub(p.PointMasses[i].Pos).PerpNeg()
		norm = edge1.Add(edge2).Unit()
		edgeLength := p.EdgeInfo[i].Length
		p.NormalList[i] = norm
		p.EdgeLengthList[i] = edgeLength
		xdist := math.Abs(p.PointMasses[i].Pos.X - p.PointMasses[next].Pos.X)
		absNormX := math.Abs(norm.X)
		volumeProduct := xdist * absNormX * edgeLength
		p.Volume += 0.5 * volumeProduct
	}
	invVolume := 1.0 / p.Volume
	for i := range l {
		j := i + 1
		if i == l-1 {
			j = 0
		}
		pressureV := invVolume * p.EdgeLengthList[i] * p.GasPressure
		p.PointMasses[i].Force = p.PointMasses[i].Force.Add(p.NormalList[i].Scale(pressureV))
		p.PointMasses[j].Force = p.PointMasses[j].Force.Add(p.NormalList[j].Scale(pressureV))
	}
}

type WheelJoint struct {
	BodyCar     Body
	BodyWheel   Body
	CarPoints   []int
	WheelPoints []int
	MotorTorque float64
}

func NewWheelJoint(car, wheel Body, carPts, wheelPts []int) *WheelJoint {
	return &WheelJoint{
		BodyCar:     car,
		BodyWheel:   wheel,
		CarPoints:   carPts,
		WheelPoints: wheelPts,
	}
}
func (j *WheelJoint) ApplyMotor() {
	if j.MotorTorque == 0 {
		return
	}

	wheelPMs := j.BodyWheel.GetPointMasses()
	wheelBase := j.BodyWheel.GetBaseBody()
	carPMs := j.BodyCar.GetPointMasses()
	carBase := j.BodyCar.GetBaseBody()

	// 1. TEKERLEĞE (WHEEL) TORK UYGULAMA KISMI
	if j.WheelPoints == nil {
		anchorWheel := wheelBase.DerivedPos
		torquePerPoint := j.MotorTorque / float64(len(wheelPMs))
		for i := range wheelPMs {
			pm := &wheelPMs[i]
			dir := pm.Pos.Sub(anchorWheel)
			r := dir.Mag()
			if r > epsilonDistance {
				tangent := dir.PerpNeg().DivS(r)
				forceMag := torquePerPoint / r
				pm.Force = pm.Force.Add(tangent.Scale(forceMag))
			}
		}
	} else if len(j.WheelPoints) > 0 {
		anchorWheel := Vec2{}
		for _, idx := range j.WheelPoints {
			anchorWheel = anchorWheel.Add(wheelPMs[idx].Pos)
		}
		anchorWheel = anchorWheel.DivS(float64(len(j.WheelPoints)))
		torquePerPoint := j.MotorTorque / float64(len(j.WheelPoints))
		for _, idx := range j.WheelPoints {
			pm := &wheelPMs[idx]
			dir := pm.Pos.Sub(anchorWheel)
			r := dir.Mag()
			if r > epsilonDistance {
				tangent := dir.PerpNeg().DivS(r)
				forceMag := torquePerPoint / r
				pm.Force = pm.Force.Add(tangent.Scale(forceMag))
			}
		}
	}

	// 2. ARABAYA (CAR) TERS TEPKİ TORKU UYGULAMA KISMI (DÜZELTİLDİ)
	// Motorun ters tepkisi arabanın sadece bağlanan dingil noktalarına (CarPoints) değil,
	// tüm gövdesine ve ana ağırlık merkezine (DerivedPos) göre uygulanmalıdır.
	// Bu, 'r' değerinin sıfıra yaklaşıp patlama yapmasını engeller.
	anchorCar := carBase.DerivedPos
	reactionTorquePerPoint := -j.MotorTorque / float64(len(carPMs))

	for i := range carPMs {
		pm := &carPMs[i]
		dir := pm.Pos.Sub(anchorCar)
		r := dir.Mag()
		if r > epsilonDistance {
			tangent := dir.PerpNeg().DivS(r)
			forceMag := reactionTorquePerPoint / r
			pm.Force = pm.Force.Add(tangent.Scale(forceMag))
		}
	}
}
func (j *WheelJoint) SolveConstraint() {
	carBase := j.BodyCar.GetBaseBody()
	wheelBase := j.BodyWheel.GetBaseBody()
	carPMs := j.BodyCar.GetPointMasses()
	wheelPMs := j.BodyWheel.GetPointMasses()
	var anchorCar, velCar Vec2
	var massCar float64
	if len(j.CarPoints) == 0 {
		anchorCar = carBase.DerivedPos
		velCar = carBase.DerivedVel
		massCar = carBase.TotalMass
	} else {
		carPtCount := float64(len(j.CarPoints))
		for _, idx := range j.CarPoints {
			anchorCar = anchorCar.Add(carPMs[idx].Pos)
			velCar = velCar.Add(carPMs[idx].Vel)
			massCar += carPMs[idx].Mass
		}
		anchorCar = anchorCar.DivS(carPtCount)
		velCar = velCar.DivS(carPtCount)
	}
	var anchorWheel, velWheel Vec2
	var massWheel float64
	if len(j.WheelPoints) == 0 {
		anchorWheel = wheelBase.DerivedPos
		velWheel = wheelBase.DerivedVel
		massWheel = wheelBase.TotalMass
	} else {
		wheelPtCount := float64(len(j.WheelPoints))
		for _, idx := range j.WheelPoints {
			anchorWheel = anchorWheel.Add(wheelPMs[idx].Pos)
			velWheel = velWheel.Add(wheelPMs[idx].Vel)
			massWheel += wheelPMs[idx].Mass
		}
		anchorWheel = anchorWheel.DivS(wheelPtCount)
		velWheel = velWheel.DivS(wheelPtCount)
	}
	var carRatio, wheelRatio float64
	if massCar == infinity && massWheel == infinity {
		return
	} else if massCar == infinity {
		carRatio = 0.0
		wheelRatio = 1.0
	} else if massWheel == infinity {
		carRatio = 1.0
		wheelRatio = 0.0
	} else {
		totalMass := massCar + massWheel
		carRatio = massWheel / totalMass
		wheelRatio = massCar / totalMass
	}
	posDelta := anchorWheel.Sub(anchorCar)
	carMove := posDelta.Scale(carRatio)
	wheelMove := posDelta.Scale(-wheelRatio)
	if len(j.CarPoints) == 0 {
		for idx := range carPMs {
			carPMs[idx].Pos = carPMs[idx].Pos.Add(carMove)
		}
	} else {
		for _, idx := range j.CarPoints {
			carPMs[idx].Pos = carPMs[idx].Pos.Add(carMove)
		}
	}
	if len(j.WheelPoints) == 0 {
		for idx := range wheelPMs {
			wheelPMs[idx].Pos = wheelPMs[idx].Pos.Add(wheelMove)
		}
	} else {
		for _, idx := range j.WheelPoints {
			wheelPMs[idx].Pos = wheelPMs[idx].Pos.Add(wheelMove)
		}
	}
	velDelta := velWheel.Sub(velCar)
	carVelChange := velDelta.Scale(carRatio)
	wheelVelChange := velDelta.Scale(-wheelRatio)
	if len(j.CarPoints) == 0 {
		for idx := range carPMs {
			carPMs[idx].Vel = carPMs[idx].Vel.Add(carVelChange)
		}
	} else {
		for _, idx := range j.CarPoints {
			carPMs[idx].Vel = carPMs[idx].Vel.Add(carVelChange)
		}
	}
	if len(j.WheelPoints) == 0 {
		for idx := range wheelPMs {
			wheelPMs[idx].Vel = wheelPMs[idx].Vel.Add(wheelVelChange)
		}
	} else {
		for _, idx := range j.WheelPoints {
			wheelPMs[idx].Vel = wheelPMs[idx].Vel.Add(wheelVelChange)
		}
	}
}

type CollisionFilterFunc func(bodyA Body, pmA *PointMass, bodyB Body, pmB1 *PointMass, pmB2 *PointMass, hitPt Vec2, normSpeed float64) bool
type Material struct {
	Elasticity      float64
	Friction        float64
	CollisionFilter CollisionFilterFunc
}
type World struct {
	Gravity               Vec2
	Bodies                []Body
	Joints                []*WheelJoint
	Materials             []Material
	CollisionList         []BodyCollisionInfo
	MaxPositionCorrection float64
	PenetrationThreshold  float64
	Iterations            int
	size                  Vec2
	worldLimits           *AABB
	worldGridStep         Vec2
}

func (w *World) CalculateCorrectionAndThreshold(minObjectSize float64) {
	w.PenetrationThreshold = minObjectSize * 0.01
	w.MaxPositionCorrection = minObjectSize * 0.20
}

func NewWorld() *World {
	w := &World{
		Gravity:       Vec2{0, 9.80665},
		Bodies:        make([]Body, 0),
		Joints:        make([]*WheelJoint, 0),
		CollisionList: make([]BodyCollisionInfo, 0),
		Iterations:    6,
		Materials:     make([]Material, 0),
	}
	w.SetWorldLimits(Vec2{-20.0, -20.0}, Vec2{20.0, 20.0})
	w.AddMaterial(0.3, 0.2, CollideFunc)
	w.CalculateCorrectionAndThreshold(0.3)
	return w
}
func (w *World) SetWorldLimits(min, max Vec2) {
	w.worldLimits = NewAABB(min, max)
	w.size = max.Sub(min)
	w.worldGridStep = w.size.DivS(32)
}
func (w *World) AddMaterial(f, e float64, collisionFilter CollisionFilterFunc) int {
	w.Materials = append(w.Materials, Material{
		Friction:        f,
		Elasticity:      e,
		CollisionFilter: collisionFilter,
	})
	return len(w.Materials) - 1
}
func (w *World) AddBody(b Body) {
	w.Bodies = append(w.Bodies, b)
}
func (w *World) RemoveBody(b Body) {
	for i, body := range w.Bodies {
		if body == b {
			w.Bodies = append(w.Bodies[:i], w.Bodies[i+1:]...)
			break
		}
	}
}
func (w *World) AddWheelJoint(j *WheelJoint) {
	w.Joints = append(w.Joints, j)
}
func (w *World) Update(delta float64) {
	subStepDelta := delta / float64(w.Iterations)
	bodies := w.Bodies
	lenBodies := len(bodies)
	for _, b := range bodies {
		if b.IsKinematic() {
			b.updateAABB(subStepDelta, false)
			w.updateBodyBitmask(b)
			b.updateEdgeInfo()
		}
	}
	for range w.Iterations {
		for _, j := range w.Joints {
			j.ApplyMotor()
		}
		for _, b := range bodies {
			if b.IsStatic() || b.IsKinematic() {
				continue
			}
			b.derivePositionAndAngle(subStepDelta)
			b.applyGravity(w.Gravity)
			b.accumulateInternalForces(delta)
			b.integrate(subStepDelta)
			b.updateAABB(subStepDelta, false)
			w.updateBodyBitmask(b)
			b.updateEdgeInfo()
		}
		for i := range lenBodies {
			body1 := bodies[i]
			bmX1 := *body1.bitmaskX()
			bmY1 := *body1.bitmaskY()
			aabb1 := body1.GetAABB()
			for j := i + 1; j < lenBodies; j++ {
				body2 := bodies[j]
				if body1.IsStatic() && body2.IsStatic() {
					continue
				}
				if (bmX1&*body2.bitmaskX()) == 0 || (bmY1&*body2.bitmaskY()) == 0 {
					continue
				}
				if !aabb1.Intersects(body2.GetAABB()) {
					continue
				}
				w.bodyCollide(body1, body2)
			}
		}
		w.handleCollisions()
		for _, j := range w.Joints {
			j.SolveConstraint()
		}
	}
	for _, b := range bodies {
		if !b.IsStatic() && !b.IsKinematic() {
			b.dampenVelocity()
		}
	}
}
func (w *World) updateBodyBitmask(body Body) {
	box := body.GetAABB()
	revDividerX := 1.0 / w.worldGridStep.X
	revDividerY := 1.0 / w.worldGridStep.Y
	minX := max(0, min(32, int((box.Min.X-w.worldLimits.Min.X)*revDividerX)))
	maxX := max(0, min(32, int((box.Max.X-w.worldLimits.Min.X)*revDividerX)))
	minY := max(0, min(32, int((box.Min.Y-w.worldLimits.Min.Y)*revDividerY)))
	maxY := max(0, min(32, int((box.Max.Y-w.worldLimits.Min.Y)*revDividerY)))
	body.bitmaskX().Clear()
	for i := minX; i <= maxX; i++ {
		body.bitmaskX().SetOn(i)
	}
	body.bitmaskY().Clear()
	for i := minY; i <= maxY; i++ {
		body.bitmaskY().SetOn(i)
	}
}

type BodyCollisionInfo struct {
	BodyA       Body
	BodyB       Body
	AIdx        int
	B1Idx       int
	B2Idx       int
	HitPt       Vec2
	Normal      Vec2
	EdgeD       float64
	Penetration float64
}

func (w *World) bodyCollide(bA, bB Body) {
	ptsA := bA.GetPointMasses()
	ptsB := bB.GetPointMasses()
	bApCount := len(ptsA)
	bBpCount := len(ptsB)
	boxB := bB.GetAABB()
	for i := range ptsA {
		pt := ptsA[i].Pos
		if !boxB.Contains(pt) || !bB.ContainstPoint(pt) {
			continue
		}
		prevPt := i - 1
		if prevPt < 0 {
			prevPt = bApCount - 1
		}
		nextPt := i + 1
		if nextPt >= bApCount {
			nextPt = 0
		}
		prevPos := ptsA[prevPt].Pos
		nextPos := ptsA[nextPt].Pos
		ptNorm := pt.Sub(prevPos).Add(nextPos.Sub(pt)).PerpNeg()
		closestAway := infinity
		closestSame := infinity
		var awayJ, sameJ int
		var awayHit, sameHit, awayNorm, sameNorm Vec2
		var awayEdgeD, sameEdgeD float64
		found := false
		for j := range bBpCount {
			dist, edgeD, hitPt, norm := bB.GetClosestPointOnEdgeSquared(pt, j)
			dot := ptNorm.Dot(norm)
			if dot <= 0.0 {
				if dist < closestAway {
					closestAway = dist
					awayJ = j
					awayHit = hitPt
					awayNorm = norm
					awayEdgeD = edgeD
					found = true
				}
			} else {
				if dist < closestSame {
					closestSame = dist
					sameJ = j
					sameHit = hitPt
					sameNorm = norm
					sameEdgeD = edgeD
				}
			}
		}
		targetJ := awayJ
		targetHit := awayHit
		targetNorm := awayNorm
		targetEdgeD := awayEdgeD
		targetPen := closestAway
		if found && closestAway > w.PenetrationThreshold && closestSame < closestAway {
			targetJ = sameJ
			targetHit = sameHit
			targetNorm = sameNorm
			targetEdgeD = sameEdgeD
			targetPen = closestSame
		}
		nextJ := targetJ + 1
		if nextJ >= bBpCount {
			nextJ = 0
		}
		w.CollisionList = append(w.CollisionList, BodyCollisionInfo{
			BodyA:       bA,
			BodyB:       bB,
			AIdx:        i,
			B1Idx:       targetJ,
			B2Idx:       nextJ,
			HitPt:       targetHit,
			Normal:      targetNorm,
			EdgeD:       targetEdgeD,
			Penetration: math.Sqrt(targetPen),
		})
	}
}
func (w *World) handleCollisions() {
	const baumgarte = 0.2
	for i := range w.CollisionList {
		info := &w.CollisionList[i]
		ptsA := info.BodyA.GetPointMasses()
		ptsB := info.BodyB.GetPointMasses()
		A := &ptsA[info.AIdx]
		B1 := &ptsB[info.B1Idx]
		B2 := &ptsB[info.B2Idx]
		bVel := B1.Vel.Add(B2.Vel).Scale(0.5)
		relVel := A.Vel.Sub(bVel)
		relDot := relVel.Dot(info.Normal)
		matA := w.Materials[info.BodyA.material()]
		matB := w.Materials[info.BodyB.material()]
		if !matA.CollisionFilter(info.BodyA, A, info.BodyB, B1, B2, info.HitPt, relDot) ||
			!matB.CollisionFilter(info.BodyA, A, info.BodyB, B1, B2, info.HitPt, relDot) {
			continue
		}
		correctionMag := max(info.Penetration-penetrationSlop, 0.0) * baumgarte
		correctionMag = min(correctionMag, w.MaxPositionCorrection)

		b1inf := 1.0 - info.EdgeD
		b2inf := info.EdgeD
		b2MassSum := B1.Mass + B2.Mass
		if infinity == B1.Mass || infinity == B2.Mass {
			b2MassSum = infinity
		}
		massSum := A.Mass + b2MassSum
		var Amove, Bmove float64
		if A.Mass == infinity {
			Amove = 0.0
			Bmove = correctionMag
		} else if b2MassSum == infinity {
			Amove = correctionMag
			Bmove = 0.0
		} else {
			revMassSum := 1.0 / massSum
			Amove = correctionMag * (b2MassSum * revMassSum)
			Bmove = correctionMag * (A.Mass * revMassSum)
		}
		B1move := Bmove * b1inf
		B2move := Bmove * b2inf
		AinvMass := 1.0 / A.Mass
		if infinity == A.Mass {
			AinvMass = 0
		}
		BinvMass := 1.0 / b2MassSum
		if infinity == b2MassSum {
			BinvMass = 0
		}
		jDenom := AinvMass + BinvMass
		avgElasticity := (matA.Elasticity + matB.Elasticity) * 0.5
		elas := 1.0 + avgElasticity
		numV := relVel.Scale(elas)
		revJDenom := 1.0 / jDenom
		j := -numV.Dot(info.Normal) * revJDenom
		infoNormal := info.Normal
		if infinity != A.Mass {
			A.Pos = A.Pos.Add(infoNormal.Scale(Amove))
		}
		if infinity != B1.Mass {
			B1.Pos = B1.Pos.Sub(infoNormal.Scale(B1move))
		}
		if infinity != B2.Mass {
			B2.Pos = B2.Pos.Sub(infoNormal.Scale(B2move))
		}
		tangent := info.Normal.PerpNeg()
		friction := (matA.Friction + matB.Friction) * 0.5
		f := (relVel.Dot(tangent) * friction) * revJDenom
		if relDot <= epsilonDistance {
			if infinity != A.Mass {
				revAMass := 1.0 / A.Mass
				jMult := j * revAMass
				fMult := f * revAMass
				A.Vel = A.Vel.Add(infoNormal.Scale(jMult)).Sub(tangent.Scale(fMult))
			}
			if infinity != b2MassSum {
				jMult := j / b2MassSum
				fMult := f / b2MassSum
				B1.Vel = B1.Vel.Sub(infoNormal.Scale(jMult * b1inf)).Add(tangent.Scale(fMult * b1inf))
				B2.Vel = B2.Vel.Sub(infoNormal.Scale(jMult * b2inf)).Add(tangent.Scale(fMult * b2inf))
			}
		}
	}
	w.CollisionList = w.CollisionList[:0]
}
func CollideFunc(A Body, pmA *PointMass, B Body, pmB1 *PointMass, pmB2 *PointMass, hitPt Vec2, normSpeed float64) bool {
	return true
}
func IgnoreCollisionFunc(A Body, pmA *PointMass, B Body, pmB1 *PointMass, pmB2 *PointMass, hitPt Vec2, normSpeed float64) bool {
	return false
}
func CalculateSpringForce(posA, velA, posB, velB Vec2, springD, springK, damping float64) (forceOut Vec2) {
	bToA := posA.Sub(posB)
	if bToA.MagSq() < epsilon {
		return
	}
	dist := bToA.Mag()
	dir := bToA.DivS(dist)
	forceMag := ((springD - dist) * springK) - (velA.Sub(velB).Dot(dir) * damping)
	return dir.Scale(forceMag)
}
