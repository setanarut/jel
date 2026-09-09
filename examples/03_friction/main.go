package main

import (
	"log"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/setanarut/jel"
	"github.com/setanarut/jel/examples/renderer"
)

type Vec2 = jel.Vec2

const (
	fixedTimeStep         = 1.0 / 60.0
	Ppm           float64 = 20.0
)

func NewGame(ppm float64) *Game {
	g := &Game{
		world: jel.NewWorld(),
	}
	g.rnd = renderer.NewJelEbitenRenderer(ppm)
	g.ScreenSize = jel.Vec2{854, 480}
	g.WorldSize = g.rnd.ScreenToWorld(g.ScreenSize)
	g.Initalize()
	return g
}
func (g *Game) Initalize() {
	g.world.PenetrationThreshold = 1
	center := g.rnd.ScreenToWorld(g.ScreenSize.DivS(2))
	g.rnd.ShowFillPointMasses = true
	g.material = g.world.AddMaterial()
	lowFriction := g.world.AddMaterial()
	mediumFriction := g.world.AddMaterial()
	highFriction := g.world.AddMaterial()
	g.world.SetMaterialPairData(g.material, lowFriction, 0.05, 0.0)
	g.world.SetMaterialPairData(g.material, mediumFriction, 0.35, 0.0)
	g.world.SetMaterialPairData(g.material, highFriction, 0.85, 0.0)
	g.makeWalls(g.material)
	angle := 20.0 * math.Pi / 180.0
	rampWidth := 9.0
	rampThickness := 0.45
	rampY := center.Y + 1.5
	ramp1X := center.X - 11.5
	ramp2X := center.X
	ramp3X := center.X + 11.5
	g.createFrictionRamp(Vec2{ramp1X, rampY}, rampWidth, rampThickness, angle, lowFriction)
	g.createFrictionRamp(Vec2{ramp2X, rampY}, rampWidth, rampThickness, angle, mediumFriction)
	g.createFrictionRamp(Vec2{ramp3X, rampY}, rampWidth, rampThickness, angle, highFriction)
	g.createSlidingBox(ramp1X, rampY, rampWidth, angle, 0.55)
	g.createSlidingBox(ramp2X, rampY, rampWidth, angle, 0.55)
	g.createSlidingBox(ramp3X, rampY, rampWidth, angle, 0.55)
}

type Game struct {
	ScreenSize jel.Vec2
	WorldSize  jel.Vec2
	world      *jel.World
	rnd        *renderer.JelEbitenRenderer
	material   int
}

func (g *Game) Update() error {
	g.world.Update(fixedTimeStep)
	g.rnd.HandleMouseDragPoint(g.world)
	return nil
}
func (g *Game) Draw(screen *ebiten.Image) {
	g.rnd.Draw(screen, g.world)
}
func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return int(g.ScreenSize.X), int(g.ScreenSize.Y)
}
func (g *Game) makeWalls(materialID int) {
	w, h := g.WorldSize.X, g.WorldSize.Y
	thickness := 80.0
	ht := thickness / 2.0
	overlap := 0.5
	extend := 200.0
	horizontalShape := jel.Rectangle(w+2*extend, thickness)
	bottom := jel.NewBody(horizontalShape, Vec2{w / 2.0, h + ht - overlap}, 0, jel.Infinity)
	bottom.AddComponent(jel.DefaultSpringComponent())
	bottom.Material = materialID
	top := jel.NewBody(horizontalShape, Vec2{w / 2.0, -ht + overlap}, 0, jel.Infinity)
	top.AddComponent(jel.DefaultSpringComponent())
	top.Material = materialID
	verticalShape := jel.Rectangle(thickness, h+2*extend)
	left := jel.NewBody(verticalShape, Vec2{-ht + overlap, h / 2.0}, 0, jel.Infinity)
	left.AddComponent(jel.DefaultSpringComponent())
	left.Material = materialID
	right := jel.NewBody(verticalShape, Vec2{w + ht - overlap, h / 2.0}, 0, jel.Infinity)
	right.AddComponent(jel.DefaultSpringComponent())
	right.Material = materialID
	g.world.AddBodies(bottom, top, left, right)
}
func (g *Game) createFrictionRamp(pos Vec2, w, h, angle float64, materialID int) *jel.Body {
	shape := jel.Rectangle(w, h)
	body := jel.NewBody(shape, pos, angle, jel.Infinity, g.world)
	body.AddComponent(jel.DefaultSpringComponent())
	body.Material = materialID
	return body
}
func (g *Game) createSlidingBox(rampX, rampY, rampWidth, angle, mass float64) *jel.Body {
	shape := jel.Rectangle(2, 2)
	localX := -rampWidth * 0.32
	localY := -(0.5 + 0.5 + 0.08)
	c := math.Cos(angle)
	s := math.Sin(angle)
	worldOffset := jel.Vec2{
		X: localX*c - localY*s,
		Y: localX*s + localY*c,
	}
	pos := Vec2{rampX + worldOffset.X, rampY + worldOffset.Y}
	body := jel.NewBody(shape, pos, angle, mass, g.world)
	body.AddComponent(jel.NewGravityComponent(0, 9.8, true))
	body.AddComponent(jel.NewShapeMatchComponent(200, 5, nil))
	body.Material = g.material
	return body
}
func main() {
	game := NewGame(Ppm)
	ebiten.SetWindowTitle("jel Physics")
	ebiten.SetWindowSize(int(game.ScreenSize.X), int(game.ScreenSize.Y))
	if err := ebiten.RunGameWithOptions(game, &ebiten.RunGameOptions{
		DisableHiDPI: true,
	}); err != nil {
		log.Fatal(err)
	}
}
