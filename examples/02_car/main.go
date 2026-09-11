package main

import (
	"log"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/setanarut/jel"
	"github.com/setanarut/jel/examples"
	"github.com/setanarut/jel/examples/renderer"
)

var rgb = renderer.Rgb

type SceneCar struct {
	examples.BaseScene
	pinnedWheels []*jel.Body
	tire1        *jel.Body
	tire2        *jel.Body
	carBody      *jel.Body
	world        *jel.World
	renderer     *renderer.JelEbitenRenderer
}

func (g *SceneCar) Update() {
	if g.pinnedWheels != nil {
		for _, body := range g.pinnedWheels {
			body.SetAngularVelocity(8)
		}
	}
	if ebiten.IsKeyPressed(ebiten.KeySpace) {
		g.carBody.AddVelocity(jel.FromAngle(g.carBody.DerivedAngle).Scale(0.4))
		g.tire1.ApplyTorque(4)
		g.tire2.ApplyTorque(4)
	}
}

func main() {

	game := examples.NewGame(60)
	t1, t2, car := makeCar(game.BodyMaterial, game.Center, game.World)
	wheelMaterial := game.World.AddMaterial()
	scene := &SceneCar{
		pinnedWheels: makePinnedWheels(wheelMaterial, game.Center, game.World),
		tire1:        t1,
		tire2:        t2,
		carBody:      car,
		renderer:     game.Renderer,
		world:        game.World,
	}
	game.World.AddBody(makePoly(car.Material, 12, game.Center.AddY(-1.5), 0.5, 1.2, game.World))
	game.World.AddBody(makePoly(car.Material, 12, game.Center.AddY(-2), 0.5, 1.2, game.World))

	game.World.SetMaterialPairCollide(wheelMaterial, game.WallMaterial, false)
	game.World.SetMaterialPairCollide(wheelMaterial, wheelMaterial, false)
	game.World.SetMaterialPairData(game.BodyMaterial, wheelMaterial, 1.0, 0.5)
	game.Body = car
	game.Scene = scene

	game.Renderer.SetAll(false)
	game.Renderer.ShowEdgeSprings = true
	game.Renderer.ShowExtraSprings = true
	game.Renderer.ShowFillPointMasses = true
	game.Renderer.ShowJoints = true
	game.Renderer.Colors.EdgeSpring = rgb(0, 0, 0)
	game.Renderer.LineThickness = 1.5

	ebiten.SetWindowTitle("SceneJ")
	if err := ebiten.RunGameWithOptions(game, &ebiten.RunGameOptions{
		DisableHiDPI: true,
	}); err != nil {
		log.Fatal(err)
	}
}
func makePinnedWheel(mat int, pos jel.Vec2, r, mass float64, world *jel.World) *jel.Body {
	ball := jel.RegularPolygon(r, 12)
	body := jel.NewBody(ball, pos, 0, mass, world)
	body.AddComponent(jel.NewShapeMatchComponent(500, 10, nil))
	body.AddComponent(jel.NewSpringComponent(1, 1))
	body.IsPinned = true
	body.Material = mat
	return body
}
func makePoly(mat, n int, pos jel.Vec2, r, mass float64, world *jel.World) *jel.Body {
	ball := jel.RegularPolygon(r, n)
	body := jel.NewBody(ball, pos, 0, mass, world)
	body.UserData = rgb(22, 207, 99)
	body.AddComponent(jel.NewShapeMatchComponent(200, 10, nil))
	body.AddComponent(jel.NewSpringComponent(100, 10))
	body.AddComponent(jel.NewGravityComponent(0, 1, true))
	body.Material = mat
	return body
}

func makePinnedWheels(mat int, center jel.Vec2, world *jel.World) []*jel.Body {
	clockRadius := 6.0
	balls := 6
	circleRadius := clockRadius * math.Sin(math.Pi/float64(balls))
	margin := 1.0
	circleRadius *= margin
	pinnedWheels := make([]*jel.Body, int(balls))
	for h := range balls {
		angle := math.Pi/2 - float64(h)*(2*math.Pi/float64(balls))
		x := center.X + clockRadius*math.Cos(angle)
		y := center.Y + clockRadius*math.Sin(angle)
		pinnedWheels[h] = makePinnedWheel(mat, jel.Vec2{x, y}, circleRadius, 3, world)

	}
	return pinnedWheels
}

func makeCar(mat int, pos jel.Vec2, world *jel.World) (tire1, tire2, carBody *jel.Body) {
	const carBodyMass = 0.6
	const tireMass = 0.5

	gravity := 4.8

	carShape := jel.ShapeFromSVGPolygonPoints(examples.Car)
	carShape.Scale(0.05, 0.05)
	carShape.Recenter()
	carBody = jel.NewBody(carShape, pos, 0, carBodyMass, world)
	s, d := 50.0, 5.0
	scomp := jel.NewSpringComponent(s, d)
	scomp.AddExtraSpring(carBody, 2, 4, s, d, nil)
	// scomp.AddExtraSpring(carBody, 1, 10, s, d, nil)
	scomp.AddExtraSpring(carBody, 1, 3, s, d, nil)
	scomp.AddExtraSpring(carBody, 1, 4, s, d, nil)
	scomp.AddExtraSpring(carBody, 0, 4, s, d, nil)
	scomp.AddExtraSpring(carBody, 0, 5, s, d, nil)
	scomp.AddExtraSpring(carBody, 4, 11, s, d, nil)
	scomp.AddExtraSpring(carBody, 4, 11, s, d, nil)
	scomp.AddExtraSpring(carBody, 5, 10, s, d, nil)
	scomp.AddExtraSpring(carBody, 5, 11, s, d, nil)
	scomp.AddExtraSpring(carBody, 6, 11, s, d, nil)
	scomp.AddExtraSpring(carBody, 6, 8, s, d, nil)
	scomp.AddExtraSpring(carBody, 6, 9, s, d, nil)
	scomp.AddExtraSpring(carBody, 6, 10, s, d, nil)
	scomp.AddExtraSpring(carBody, 7, 9, s, d, nil)

	carBody.AddComponent(scomp)
	carBody.AddComponent(jel.NewShapeMatchComponent(100, 5, nil))
	carBody.AddComponent(jel.NewGravityComponent(0, gravity, true))

	tireRadius := 0.4
	tireShapeJoingLink1 := jel.NewShapeJointLink(carBody, []int{1, 3, 4, 5})
	tireShapeJoingLink1.Offset.Y += 0.4
	wpos1 := tireShapeJoingLink1.Position()
	c1 := jel.RegularPolygon(tireRadius, 12)
	tire1 = jel.NewBody(c1, jel.Vec2{wpos1.X, wpos1.Y}, 0, tireMass, world)
	tire1.AddComponent(jel.NewSpringComponent(600, 5))
	tire1.AddComponent(jel.DefaultShapeMatchComponent())
	tire1.AddComponent(jel.NewPressureComponent(120))
	tire1.AddComponent(jel.NewGravityComponent(0, gravity, true))
	bj1 := jel.NewBodyJointLink(tire1)
	tirePinJoint1 := jel.NewPinJoint(tireShapeJoingLink1, bj1, 0)
	tirePinJoint1.AllowCollisions = false
	world.AddJoint(tirePinJoint1)
	tireShapeJoingLink2 := jel.NewShapeJointLink(carBody, []int{6, 7, 8, 10})
	tireShapeJoingLink2.Offset.Y += 0.4
	wpos2 := tireShapeJoingLink2.Position()
	c2 := jel.RegularPolygon(tireRadius, 12)
	tire2 = jel.NewBody(c2, jel.Vec2{wpos2.X, wpos2.Y}, 0, tireMass, world)
	tire2.AddComponent(jel.NewSpringComponent(600, 5))
	tire2.AddComponent(jel.DefaultShapeMatchComponent())
	tire2.AddComponent(jel.NewPressureComponent(120))
	tire2.AddComponent(jel.NewGravityComponent(0, gravity, true))
	bj2 := jel.NewBodyJointLink(tire2)
	tirePinJoint2 := jel.NewPinJoint(tireShapeJoingLink2, bj2, 0)
	tirePinJoint2.AllowCollisions = false
	world.AddJoint(tirePinJoint2)

	tire1.Material = mat
	tire2.Material = mat
	carBody.Material = mat

	tire1.UserData = rgb(0, 133, 235)
	tire2.UserData = rgb(0, 133, 235)
	carBody.UserData = rgb(255, 150, 65)
	return
}

func makeJel(h float64, center jel.Vec2, world *jel.World) *jel.Body {

	s := jel.ShapeFromSVGPolygonPoints(examples.J)
	s.FitHeight(h)
	jb := jel.NewBody(s, center.AddX(-1), 0, 1, world)
	jb.AddComponent(jel.NewSpringComponent(200, 10))
	jb.AddComponent(jel.NewShapeMatchComponent(200, 10, nil))
	// jb.AddComponent(jel.NewPressureComponent(30))
	jb.AddComponent(jel.NewGravityComponent(0, 9, true))
	// jb.IsPinned = true

	s = jel.ShapeFromSVGPolygonPoints(examples.E)
	s.FitHeight(h)
	eb := jel.NewBody(s, center, 0, 1, world)
	// eb.AddComponent(jel.NewSpringComponent(200, 10))
	eb.AddComponent(jel.NewShapeMatchComponent(200, 10, nil))
	// eb.AddComponent(jel.NewPressureComponent(30))
	eb.AddComponent(jel.NewGravityComponent(0, 9, true))
	// eb.IsPinned = true
	// eb.FreeRotate = false

	s = jel.ShapeFromSVGPolygonPoints(examples.L)
	s.FitHeight(h)
	lb := jel.NewBody(s, center.AddX(1), 0, 1, world)
	// lb.AddComponent(jel.NewSpringComponent(200, 10))
	lb.AddComponent(jel.NewShapeMatchComponent(200, 10, nil))
	// lb.AddComponent(jel.NewPressureComponent(30))
	lb.AddComponent(jel.NewGravityComponent(0, 9, true))
	// lb.IsPinned = true

	link := jel.NewBodyJointLink(jb)
	link2 := jel.NewBodyJointLink(eb)

	world.AddJoint(jel.NewSpringJoint(link, link2, 59, 10, jel.NewFixedRestDistance(2)))

	jb.UserData = rgb(255, 143, 195)
	eb.UserData = rgb(255, 143, 195)
	lb.UserData = rgb(255, 143, 195)

	return jb

}
