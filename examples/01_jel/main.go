package main

import (
	"log"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/setanarut/jel"
	"github.com/setanarut/jel/examples"
	"github.com/setanarut/jel/examples/renderer"
	"github.com/setanarut/v"
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

func makeJel(h float64, center v.Vec, world *jel.World) *jel.Body {
	center.AddX(0.5)

	s := jel.ShapeFromSVGPolygonPoints(examples.J)
	s.SetHeight(h)
	jb := jel.NewBody(s, center.AddX(-1), 0, 1, world)
	jb.AddComponent(jel.NewSpringComponent(200, 10))
	jb.AddComponent(jel.NewShapeMatchComponent(200, 10, nil))
	jb.AddComponent(jel.NewGravityComponent(0, 2, true))

	s = jel.ShapeFromSVGPolygonPoints(examples.E)
	s.SetHeight(h)
	eb := jel.NewBody(s, center, 0, 1, world)
	eb.AddComponent(jel.NewSpringComponent(200, 10))
	eb.AddComponent(jel.NewShapeMatchComponent(200, 10, nil))
	eb.AddComponent(jel.NewGravityComponent(0, 2, true))

	s = jel.ShapeFromSVGPolygonPoints(examples.L)
	s.SetHeight(h)
	lb := jel.NewBody(s, center.AddX(1), 0, 1, world)
	lb.AddComponent(jel.NewSpringComponent(200, 10))
	lb.AddComponent(jel.NewShapeMatchComponent(200, 10, nil))
	lb.AddComponent(jel.NewGravityComponent(0, 2, true))

	link := jel.NewBodyJointLink(jb)
	link2 := jel.NewBodyJointLink(eb)

	world.AddJoint(jel.NewSpringJoint(link, link2, 59, 10, jel.NewFixedRestDistance(2)))

	// If UserData is a color.Color, the renderer uses it.
	jb.UserData = rgb(244, 75, 75)
	eb.UserData = rgb(25, 237, 36)
	lb.UserData = rgb(82, 75, 231)

	return jb

}
