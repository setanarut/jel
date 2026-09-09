package main

import (
	"log"

	_ "github.com/ebitengine/debugui"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/setanarut/jel"
	"github.com/setanarut/jel/examples"
	"github.com/setanarut/jel/examples/renderer"
)

var rgb = renderer.Rgb

type SceneJ struct {
	examples.BaseScene
}

func main() {
	game := examples.NewGame(100)
	game.Scene = &SceneJ{}
	game.MakeRoom()
	game.Body = makeJel(2, game.Center, game.World)
	game.World.SetMaterialPairData(game.BodyMaterial, game.WallMaterial, 1., 1.)
	game.Renderer.Colors.FillPointMasses = rgb(255, 143, 195)
	game.Renderer.Colors.EdgeSpring = rgb(0, 0, 0)
	game.Renderer.Colors.BodyEdge = rgb(0, 0, 0)
	game.Renderer.LineThickness = 3
	game.Renderer.SetAll(false)
	game.Renderer.ShowFillPointMasses = true
	game.Renderer.ShowEdgeSprings = true
	game.Renderer.Antialias = true
	ebiten.SetWindowTitle("SceneJ")
	if err := ebiten.RunGameWithOptions(game, &ebiten.RunGameOptions{
		DisableHiDPI: true,
	}); err != nil {
		log.Fatal(err)
	}
}

func makeJel(h float64, center jel.Vec2, world *jel.World) *jel.Body {
	center.AddX(0.5)

	s := jel.ShapeFromSVGPolygonPoints(examples.J)
	s.FitHeight(h)
	jb := jel.NewBody(s, center.AddX(-1), 0, 1, world)
	jb.AddComponent(jel.NewSpringComponent(200, 10))
	jb.AddComponent(jel.NewShapeMatchComponent(200, 10, nil))
	// jb.AddComponent(jel.NewPressureComponent(30))
	jb.AddComponent(jel.NewGravityComponent(0, 0, true))
	// jb.IsPinned = true

	s = jel.ShapeFromSVGPolygonPoints(examples.E)
	s.FitHeight(h)
	eb := jel.NewBody(s, center, 0, 1, world)
	eb.AddComponent(jel.NewSpringComponent(200, 10))
	eb.AddComponent(jel.NewShapeMatchComponent(200, 10, nil))
	// eb.AddComponent(jel.NewPressureComponent(30))
	eb.AddComponent(jel.NewGravityComponent(0, 0, true))
	// eb.IsPinned = true
	// eb.FreeRotate = false

	s = jel.ShapeFromSVGPolygonPoints(examples.L)
	s.FitHeight(h)
	lb := jel.NewBody(s, center.AddX(1), 0, 1, world)
	lb.AddComponent(jel.NewSpringComponent(200, 10))
	lb.AddComponent(jel.NewShapeMatchComponent(200, 10, nil))
	// lb.AddComponent(jel.NewPressureComponent(30))
	lb.AddComponent(jel.NewGravityComponent(0, 0, true))
	// lb.IsPinned = true

	link := jel.NewBodyJointLink(jb)
	link2 := jel.NewBodyJointLink(eb)

	world.AddJoint(jel.NewSpringJoint(link, link2, 59, 10, jel.NewFixedRestDistance(2)))

	jb.UserData = rgb(255, 143, 195)
	eb.UserData = rgb(255, 143, 195)
	lb.UserData = rgb(255, 143, 195)

	return jb

}
