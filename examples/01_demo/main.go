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
	g.renderer.ShowSpring = true
	// g.renderer.ShowFillPointMasses = true
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

	// Araba Gövdesini Oluştur
	car := g.makeCarChassis(jel.Vec2{X: 4, Y: 2})
	car.GetBaseBody().UserData = "araba"
	car.BaseBody.MaterialID = matCar

	matWheel := g.world.AddMaterial(1, 0, func(bodyA jel.Body, pmA *jel.PointMass, bodyB jel.Body, pmB1 *jel.PointMass, pmB2 *jel.PointMass, hitPt jel.Vec2, normSpeed float64) bool {
		if bodyB.GetBaseBody().UserData == "araba" {
			return false
		}
		if bodyB.GetBaseBody().UserData == "tekerlek" {
			return false
		}
		return true
	})

	// Tekerlekler: 20 cm çap = yarıçap 0.1 metre, biraz büyütmek istersen 0.15 yapabilirsin
	wheelRadius := 0.15
	wheel := g.makeWheel(jel.Vec2{X: 3.8, Y: 2.5}, wheelRadius, 16)
	wheel.BaseBody.MaterialID = matWheel
	wheel.GetBaseBody().UserData = "tekerlek"

	wheel2 := g.makeWheel(jel.Vec2{X: 5.2, Y: 2.5}, wheelRadius, 16)
	wheel2.BaseBody.MaterialID = matWheel
	wheel2.GetBaseBody().UserData = "tekerlek"

	// Tekerlekleri arabanın altındaki spesifik noktalara bağla
	// Arka tekerlek (indeks 2 ve 3)
	wj := jel.NewWheelJoint(car, wheel, []int{2, 3}, nil)
	// Ön tekerlek (indeks 4 ve 5)
	wj2 := jel.NewWheelJoint(car, wheel2, []int{4, 5}, nil)

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

	const stiffness = 500.0
	const damping = 20.0

	pm := &g.dragBody.GetPointMasses()[g.dragPoint]
	displacement := g.targetPos.Sub(pm.Pos)
	springForce := displacement.Scale(stiffness)
	dampingForce := pm.Vel.Scale(-damping)
	pm.Force = pm.Force.Add(springForce.Add(dampingForce)).Scale(10)
}

func (g *Game) Update() error {
	g.world.Update(timeStep)
	g.HandleMouseDragForcePoint()

	// KLAVYE İLE GAZ KONTROLÜ
	torque := 0.0
	if ebiten.IsKeyPressed(ebiten.KeyRight) {
		torque = 6.0 // İleri gitmek için tork
	} else if ebiten.IsKeyPressed(ebiten.KeyLeft) {
		torque = -6.0 // Geri gitmek / Fren için tork
	}

	// Torku world içindeki WheelJoint'lere uygula
	if len(g.world.Joints) >= 2 {
		g.world.Joints[0].MotorTorque = torque // Arka tekerlek
		g.world.Joints[1].MotorTorque = torque // Ön tekerlek (4 çeker yapmak istersen. Sadece arkadan itiş için 1. indexi 0 bırakabilirsin)
	}

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
	ebiten.SetWindowTitle("jell Physics - Car Demo")
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
	bottom := jel.NewStaticBody(g.world, jel.NewPolygon(horizontalVerts), bottomPos, 0, jel.Vec2{X: 1, Y: 1})
	bottom.MaterialID = materialID
	bottom.UserData = "zemin"

	topPos := jel.Vec2{X: g.WorldSize.X / 2.0, Y: margin - ht}
	top := jel.NewStaticBody(g.world, jel.NewPolygon(horizontalVerts), topPos, 0, jel.Vec2{X: 1, Y: 1})
	top.UserData = "zemin"
	top.MaterialID = materialID

	leftPos := jel.Vec2{X: margin - ht, Y: g.WorldSize.Y / 2.0}
	left := jel.NewStaticBody(g.world, jel.NewPolygon(verticalVerts), leftPos, 0, jel.Vec2{X: 1, Y: 1})
	left.UserData = "zemin"
	left.MaterialID = materialID

	rightPos := jel.Vec2{X: g.WorldSize.X - margin + ht, Y: g.WorldSize.Y / 2.0}
	right := jel.NewStaticBody(g.world, jel.NewPolygon(verticalVerts), rightPos, 0, jel.Vec2{X: 1, Y: 1})
	right.UserData = "zemin"
	right.MaterialID = materialID
}

func (g *Game) makeCarChassis(pos jel.Vec2) *jel.SpringBody {
	width := 2.4
	height := 0.6

	verts := []jel.Vec2{
		{X: 0, Y: 0},                 // 0: Sol Üst (Tavan)
		{X: 0, Y: height},            // 1: Sol Alt (Tampon)
		{X: width * 0.20, Y: height}, // 2: Arka Tekerlek Bağlantı - Arka
		{X: width * 0.40, Y: height}, // 3: Arka Tekerlek Bağlantı - Ön
		{X: width * 0.60, Y: height}, // 4: Ön Tekerlek Bağlantı - Arka
		{X: width * 0.80, Y: height}, // 5: Ön Tekerlek Bağlantı - Ön
		{X: width, Y: height},        // 6: Sağ Alt (Tampon)
		{X: width, Y: height * 0.4},  // 7: Kaput
		{X: width * 0.7, Y: 0},       // 8: Tavan Ön Cam Birleşimi
	}

	opts := jel.SpringBodyOptions{
		SpringMat: jel.SpringMat{
			Stiffness: 800.0,
			// Titremeyi kesmek için sönümleme artırıldı
			Damping: 60.0,
		},
		MassPerPoint: 3.0,

		ShapeMatching: true,
		// Aşırı yüksek stiffness sapıtmaya yol açar, dengeledik
		ShapeMatchStiffness: 1200.0,
		// Şekli korurken oluşan mikro titreşimleri emmesi için yüksek sönümleme
		ShapeMatchDamping: 80.0,
	}

	cs := jel.NewPolygon(verts)
	sb := jel.NewSpringBody(g.world, cs, opts, pos, 0, jel.Vec2{X: 1, Y: 1})

	// Kafes (Truss) Sistemi: 5 Adet İç Çapraz Yay (Cross-Bracing)
	// Bu yaylar, dış kenarların içe çökmesini fiziksel olarak kilitler.
	sb.AddInternalSpring(0, 6, opts.SpringMat) // Sol üstten -> Sağ alta (Ana Çapraz)
	sb.AddInternalSpring(1, 8, opts.SpringMat) // Sol alttan -> Ön cama (İkinci Ana Çapraz)
	sb.AddInternalSpring(0, 4, opts.SpringMat) // Sol üstten -> Ön tekerlek arkasına
	sb.AddInternalSpring(8, 2, opts.SpringMat) // Ön camdan -> Arka tekerlek arkasına
	sb.AddInternalSpring(1, 7, opts.SpringMat) // Sol alttan -> Kaputa

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
		jel.Vec2{X: 1, Y: 1},
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
