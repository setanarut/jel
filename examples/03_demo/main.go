package main

import (
	"log"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/inpututil"

	"github.com/setanarut/jel"
	"github.com/setanarut/jel/examples/common"
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
	g.world.Iterations = 10
	g.Initalize()
	return g
}

func (g *Game) Initalize() {
	// g.renderer.ShowSpring = true
	// g.renderer.ShowFillPointMasses = true
	g.renderer.ShowStrokePointMasses = true
	g.renderer.ShowPointMassDots = true
	g.world.CalculateCorrectionAndThreshold(0.5)

	g.renderer.So.Width = 3
	g.makeStar(jel.Vec2{X: 4, Y: 2}, 10, 0.5)
	g.makeStar(jel.Vec2{X: 5, Y: 2}, 10, 0.5)
	// b.Static = true

	g.makeWalls(g.world.AddMaterial(1, 0, jel.CollideFunc))
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

	const stiffness = 100.0
	const damping = 10.0

	pm := &g.dragBody.GetPointMasses()[g.dragPoint]
	displacement := g.targetPos.Sub(pm.Pos)
	springForce := displacement.Scale(stiffness)
	dampingForce := pm.Vel.Scale(-damping)
	pm.Force = pm.Force.Add(springForce.Add(dampingForce)).Scale(10)
}

func (g *Game) Update() error {
	g.HandleMouseDragForcePoint()
	g.world.Update(timeStep)
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

func (g *Game) makeStar(pos jel.Vec2, n int, outerRadius float64) *jel.SpringBody {
	const tips = 5 // sabit: her zaman 5 uçlu yıldız
	const innerRatio = 0.9
	innerRadius := outerRadius * innerRatio

	if n < tips*2 {
		n = tips * 2
	}

	// Önce 5 uçlu yıldızın 10 temel köşesini (5 uç + 5 çentik) hesapla.
	baseCount := tips * 2
	base := make([]jel.Vec2, baseCount)
	angleStep := math.Pi / float64(tips)
	startAngle := -math.Pi / 2 // ilk uç yukarı baksın

	for i := 0; i < baseCount; i++ {
		angle := startAngle + float64(i)*angleStep
		r := outerRadius
		if i%2 == 1 {
			r = innerRadius
		}
		base[i] = jel.Vec2{
			X: r * math.Cos(angle),
			Y: r * math.Sin(angle),
		}
	}

	// n, toplam nokta sayısı: temel 10 köşeyi n noktaya böl,
	// kenarlar boyunca eşit aralıklarla ara noktalar (interpolasyon) ekle.
	verts := make([]jel.Vec2, 0, n)
	for i := 0; i < n; i++ {
		// base üzerinde hangi kenarda olduğumuzu bul (kesirli index)
		t := float64(i) / float64(n) * float64(baseCount)
		idx := int(math.Floor(t))
		frac := t - float64(idx)

		a := base[idx%baseCount]
		b := base[(idx+1)%baseCount]

		verts = append(verts, jel.Vec2{
			X: a.X + (b.X-a.X)*frac,
			Y: a.Y + (b.Y-a.Y)*frac,
		})
	}

	opts := jel.SpringBodyOptions{
		SpringMat: jel.SpringMat{
			Stiffness: 400.0,
			Damping:   40.0,
		},
		MassPerPoint: 0.2,

		ShapeMatching:       true,
		ShapeMatchStiffness: 400.0,
		ShapeMatchDamping:   40.0,
	}

	cs := jel.NewPolygon(verts)
	sb := jel.NewSpringBody(g.world, cs, opts, pos, 0, jel.Vec2{X: 1, Y: 1})

	return sb
}

func CursorPositionVec2() jel.Vec2 {
	x, y := ebiten.CursorPosition()
	return jel.Vec2{X: float64(x), Y: float64(y)}
}
