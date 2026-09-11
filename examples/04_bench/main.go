// Command 04_bench runs a dense soft-body scene for profiling collision work.
package main

import (
	"log"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/setanarut/jel"
	"github.com/setanarut/jel/examples/renderer"
)

const (
	fixedTimeStep  = 1.0 / 60.0
	pixelsPerMeter = 30.0
	ballCount      = 500
	ballSides      = 12
	ballRadius     = 0.18
)

type game struct {
	world        *jel.World
	renderer     *renderer.JelEbitenRenderer
	screenSize   jel.Vec2
	worldSize    jel.Vec2
	ballMaterial int
}

func newGame() *game {
	g := &game{
		world:      jel.NewWorld(),
		renderer:   renderer.NewJelEbitenRenderer(pixelsPerMeter),
		screenSize: jel.Vec2{X: 900, Y: 600},
	}
	g.worldSize = g.renderer.ScreenToWorld(g.screenSize)
	g.ballMaterial = g.world.AddMaterial()
	g.world.SetMaterialPairData(g.ballMaterial, 0, 0.35, 0.15)
	g.renderer.SetAll(false)
	g.renderer.ShowFillPointMasses = true
	g.renderer.ShowStrokeGlobalShape = true
	g.makeWalls()
	g.addTestBalls()
	return g
}

// makeTestBall constructs one of the twelve-sided soft balls used by the
// benchmark scene. Callers provide a pre-spaced position so creation itself
// does not introduce initial overlap work.
func (g *game) makeTestBall(position jel.Vec2) *jel.Body {
	body := jel.NewBody(jel.RegularPolygon(ballRadius, ballSides), position, 0, 0.25, g.world)
	body.Material = g.ballMaterial
	body.AddComponent(jel.NewSpringComponent(150, 4))
	body.AddComponent(jel.NewShapeMatchComponent(350, 12, nil))
	body.AddComponent(jel.NewGravityComponent(0, 9.8, true))
	return body
}

func (g *game) addTestBalls() {
	const columns = 25
	spacing := ballRadius * 2.5
	start := jel.Vec2{
		X: (g.worldSize.X - float64(columns-1)*spacing) / 2,
		Y: ballRadius * 2.5,
	}
	for index := 0; index < ballCount; index++ {
		column := index % columns
		row := index / columns
		position := start.Add(jel.Vec2{X: float64(column) * spacing, Y: float64(row) * spacing})
		g.makeTestBall(position)
	}
}

func (g *game) makeWalls() {
	const thickness = 0.25
	halfThickness := thickness / 2
	wallMaterial := 0
	newWall := func(shape jel.Shape, position jel.Vec2) {
		wall := jel.NewStaticBody(shape, position, 0, g.world)
		wall.Material = wallMaterial
	}

	newWall(jel.Rectangle(g.worldSize.X+thickness*2, thickness), jel.Vec2{X: g.worldSize.X / 2, Y: -halfThickness})
	newWall(jel.Rectangle(g.worldSize.X+thickness*2, thickness), jel.Vec2{X: g.worldSize.X / 2, Y: g.worldSize.Y + halfThickness})
	newWall(jel.Rectangle(thickness, g.worldSize.Y), jel.Vec2{X: -halfThickness, Y: g.worldSize.Y / 2})
	newWall(jel.Rectangle(thickness, g.worldSize.Y), jel.Vec2{X: g.worldSize.X + halfThickness, Y: g.worldSize.Y / 2})
}

func (g *game) Update() error {
	g.world.Update(fixedTimeStep)
	g.renderer.HandleMouseDragPoint(g.world)
	return nil
}

func (g *game) Draw(screen *ebiten.Image) {
	g.renderer.Draw(screen, g.world)
}

func (g *game) Layout(_, _ int) (int, int) {
	return int(g.screenSize.X), int(g.screenSize.Y)
}

func main() {
	g := newGame()
	ebiten.SetWindowTitle("jel collision benchmark: 500 balls")
	ebiten.SetWindowSize(int(g.screenSize.X), int(g.screenSize.Y))
	if err := ebiten.RunGameWithOptions(g, &ebiten.RunGameOptions{DisableHiDPI: true}); err != nil {
		log.Fatal(err)
	}
}
