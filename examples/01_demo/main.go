package main

import (
	"log"
	"math"
	"math/rand/v2"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"

	"github.com/setanarut/jel"
	"github.com/setanarut/jel/examples/common"
	"github.com/setanarut/jel/presets"
)

const (
	timeStep = 1.0 / 60.0
)

func NewGame(ppm float64) *Game {
	g := &Game{
		world: jel.NewWorld(),
	}
	g.PPM = ppm
	g.ScreenSize = jel.Vec2{854, 480}
	g.WorldSize = g.screenToWorld(g.ScreenSize)
	g.renderer = common.NewRenderer(ppm)
	g.renderer.PixelsPerMeter = ppm
	g.world.SetWorldLimits(jel.Vec2{}, g.WorldSize)
	g.world.Iterations = 8
	g.Initalize()
	return g
}

func (g *Game) Initalize() {
	// g.renderer.ShowSpring = true
	// g.renderer.ShowPointMassIndices = true
	// g.renderer.ShowPointMassDots = true
	// g.renderer.ShowStrokeGlobalShape = true
	g.renderer.ShowFillPointMasses = true
	g.renderer.ShowStrokePointMasses = true
	g.renderer.So.Width = 3

	// Materyaller
	matFloor := g.world.AddMaterial(1, 0., jel.CollideFunc)

	matCar := g.world.AddMaterial(1, 0, func(bodyA jel.Body, pmA *jel.PointMass, bodyB jel.Body, pmB1 *jel.PointMass, pmB2 *jel.PointMass, hitPt jel.Vec2, normSpeed float64) bool {
		if bodyB.GetBaseBody().UserData == "tekerlek" {
			return false
		}
		return true
	})

	// Kutu: 30 cm = 0.3 metre
	carSize := jel.Vec2{2, 0.5}
	car := g.makeBox(jel.Vec2{2, 2}, carSize)
	car.AddInternalSpring(0, 4, presets.Jelly)
	car.AddInternalSpring(6, 2, presets.Jelly)
	// car.AddInternalSpring(1, 5, presets.Jelly)
	// car.IsPinned = true
	car.GetBaseBody().UserData = "araba"
	car.BaseBody.MaterialID = matCar

	matWheel := g.world.AddMaterial(1, 0.3, func(bodyA jel.Body, pmA *jel.PointMass, bodyB jel.Body, pmB1 *jel.PointMass, pmB2 *jel.PointMass, hitPt jel.Vec2, normSpeed float64) bool {
		if bodyB.GetBaseBody().UserData == "araba" {
			return false
		}
		return true
	})
	// Tekerlek: 20 cm çap = yarıçap 0.1 metre
	wheelRadius := 0.1
	wheel := g.makeWheel(jel.Vec2{2, 1.85}, wheelRadius, 12)
	wheel.BaseBody.MaterialID = matWheel
	wheel.GetBaseBody().UserData = "tekerlek"
	// Tekerlek: 20 cm çap = yarıçap 0.1 metre
	wheel2 := g.makeWheel(jel.Vec2{2, 1.85}, wheelRadius, 12)
	wheel2.BaseBody.MaterialID = matWheel
	wheel2.GetBaseBody().UserData = "tekerlek"

	// Tekerleği arabanın altına bağla
	wj := jel.NewWheelJoint(car, wheel, []int{0, 2, 3}, nil)
	wj2 := jel.NewWheelJoint(car, wheel2, []int{3, 4, 6}, nil)
	wj.MotorTorque = 5
	wj2.MotorTorque = 5
	g.world.AddWheelJoint(wj)
	g.world.AddWheelJoint(wj2)

	g.makeWalls(matFloor)
}

type Game struct {
	ScreenSize jel.Vec2
	WorldSize  jel.Vec2
	world      *jel.World
	renderer   *common.Renderer
	isDragging bool
	dragBody   jel.Body
	dragPoint  int
	targetPos  jel.Vec2
	prevTarget jel.Vec2
	grabOffset jel.Vec2
	PPM        float64
}

func (g *Game) HandleMouseDragForcePoint() {
	g.targetPos = g.screenToWorld(CursorPositionVec2())

	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		maxDist := 30.0 / g.PPM
		g.dragBody, g.dragPoint = getClosestPointMass(g.world, g.targetPos, maxDist)
		if g.dragBody != nil {
			g.isDragging = true
		}
	}

	if inpututil.IsMouseButtonJustReleased(ebiten.MouseButtonLeft) {
		g.isDragging = false
		g.dragBody = nil
	}

	if !g.isDragging || g.dragBody == nil {
		return
	}

	const stiffness = 200.0
	const damping = 20.0

	pm := &g.dragBody.GetPointMasses()[g.dragPoint]
	displacement := g.targetPos.Sub(pm.Pos)
	springForce := displacement.Scale(stiffness)
	dampingForce := pm.Vel.Scale(-damping)
	pm.Force = pm.Force.Add(springForce.Add(dampingForce))
}

func (g *Game) handleMouseDragAABBBaseShape() {
	g.targetPos = g.screenToWorld(CursorPositionVec2())

	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		for _, b := range g.world.Bodies {
			if b.GetAABB().Contains(g.targetPos) {
				g.dragBody = b
				g.isDragging = true
				g.prevTarget = g.targetPos
				body := b.GetBaseBody()
				g.grabOffset = body.DerivedPos.Sub(g.targetPos)
				break
			}
		}
	}

	if inpututil.IsMouseButtonJustReleased(ebiten.MouseButtonLeft) {
		g.isDragging = false
		g.dragBody = nil
	}

	if !g.isDragging || g.dragBody == nil {
		return
	}

	desiredPos := g.targetPos.Add(g.grabOffset)
	mouseVel := g.targetPos.Sub(g.prevTarget).DivS(timeStep)

	body := g.dragBody.GetBaseBody()
	body.SetPositionAngleScale(desiredPos, body.DerivedAngle, body.Scale)

	for i := range body.PointMasses {
		body.PointMasses[i].Vel = mouseVel
	}

	g.prevTarget = g.targetPos
}

func (g *Game) Update() error {
	g.world.Update(timeStep)
	g.HandleMouseDragForcePoint()
	// g.handleMouseDragAABBBaseShape()
	return nil
}

func (g *Game) Draw(screen *ebiten.Image) {
	g.renderer.Draw(screen, g.world)
}

func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return int(g.ScreenSize.X), int(g.ScreenSize.Y)
}

func getClosestPointMass(world *jel.World, cursor jel.Vec2, maxDist float64) (jel.Body, int) {
	var closestBody jel.Body
	closestIndex, minDist := -1, maxDist

	for _, b := range world.Bodies {
		if idx, dist := b.GetClosestPointMass(cursor); dist >= 0 && dist < minDist {
			closestBody, closestIndex, minDist = b, idx, dist
		}
	}
	return closestBody, closestIndex
}

func main() {
	game := NewGame(100)
	ebiten.SetWindowTitle("jell Physics")
	ebiten.SetWindowSize(int(game.ScreenSize.X), int(game.ScreenSize.Y))
	if err := ebiten.RunGame(game); err != nil {
		log.Fatal(err)
	}
}

func (g *Game) worldToScreen(v jel.Vec2) jel.Vec2 {
	return v.Scale(g.PPM)
}

func (g *Game) screenToWorld(v jel.Vec2) jel.Vec2 {
	return v.DivS(g.PPM)
}

func (g *Game) makeWalls(materialID int) {
	thickness := 0.5
	margin := 4 / g.PPM

	hw := g.WorldSize.X / 2.0
	ht := thickness / 2.0

	horizontalVerts := []jel.Vec2{
		{X: -hw, Y: -ht},
		{X: -hw, Y: ht},
		{X: hw, Y: ht},
		{X: hw, Y: -ht},
	}

	hh := g.WorldSize.Y / 2.0
	verticalVerts := []jel.Vec2{
		{X: -ht, Y: -hh},
		{X: -ht, Y: hh},
		{X: ht, Y: hh},
		{X: ht, Y: -hh},
	}

	bottomPos := jel.Vec2{X: g.WorldSize.X / 2.0, Y: g.WorldSize.Y - margin + ht}
	bottom := jel.NewStaticBody(g.world, jel.NewPolygon(horizontalVerts), bottomPos, 0, jel.One)
	bottom.MaterialID = materialID
	bottom.UserData = "zemin"

	topPos := jel.Vec2{X: g.WorldSize.X / 2.0, Y: margin - ht}
	top := jel.NewStaticBody(g.world, jel.NewPolygon(horizontalVerts), topPos, 0, jel.One)
	top.UserData = "zemin"
	top.MaterialID = materialID

	leftPos := jel.Vec2{X: margin - ht, Y: g.WorldSize.Y / 2.0}
	left := jel.NewStaticBody(g.world, jel.NewPolygon(verticalVerts), leftPos, 0, jel.One)
	left.UserData = "zemin"
	left.MaterialID = materialID

	rightPos := jel.Vec2{X: g.WorldSize.X - margin + ht, Y: g.WorldSize.Y / 2.0}
	right := jel.NewStaticBody(g.world, jel.NewPolygon(verticalVerts), rightPos, 0, jel.One)
	right.UserData = "zemin"
	right.MaterialID = materialID
}

func (g *Game) makeBox(pos jel.Vec2, size jel.Vec2) *jel.SpringBody {
	// verts := []jel.Vec2{
	// 	{X: 0, Y: 0},
	// 	{X: 0, Y: size.Y},
	// 	{X: size.X, Y: size.Y},
	// 	{X: size.X, Y: 0},
	// }

	verts := []jel.Vec2{
		{X: 0, Y: 0},               // sol alt
		{X: 0, Y: size.Y / 2},      // sol orta
		{X: 0, Y: size.Y},          // sol üst
		{X: size.X / 2, Y: size.Y}, // üst orta
		{X: size.X, Y: size.Y},     // sağ üst
		{X: size.X, Y: size.Y / 2}, // sağ orta
		{X: size.X, Y: 0},          // sağ alt
		{X: size.X / 2, Y: 0},      // alt orta
	}

	cs := jel.NewPolygon(verts)
	sb := jel.NewSpringBody(g.world, cs, presets.Jell(), pos, 0, jel.One)
	return sb
}

func (g *Game) makeWheel(pos jel.Vec2, r float64, n int) *jel.PressureBody {
	ballVerts := make([]jel.Vec2, 0, n)
	for i := range n {
		rad := -float64(i) * (2 * math.Pi / float64(n))
		ballVerts = append(ballVerts, jel.Vec2{
			X: math.Cos(rad) * r,
			Y: math.Sin(rad) * r,
		})
	}
	ballShape := jel.NewPolygon(ballVerts)
	b := jel.NewPressureBody(
		g.world,
		ballShape,
		presets.CarTire(),
		pos,
		0,
		jel.One,
	)
	return b
}

func randPos(bounds jel.Vec2, margin float64) jel.Vec2 {
	return jel.Vec2{
		X: margin + rand.Float64()*(bounds.X-2*margin),
		Y: margin + rand.Float64()*(bounds.Y-2*margin),
	}
}

func CursorPositionVec2() jel.Vec2 {
	x, y := ebiten.CursorPosition()
	return jel.Vec2{X: float64(x), Y: float64(y)}
}

func (g *Game) makeBoxGrid(startPixel jel.Vec2, cols, rows int, boxW, boxH, gap float64) {
	for row := range rows {
		for col := range cols {
			x := startPixel.X + float64(col)*(boxW+gap)
			y := startPixel.Y + float64(row)*(boxH+gap)
			g.makeBox(jel.Vec2{X: x, Y: y}, jel.Vec2{boxW, boxH})
		}
	}
}
