package examples

import (
	"image"

	"github.com/ebitengine/debugui"
	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/setanarut/jel"
	"github.com/setanarut/jel/examples/renderer"
)

type Scene interface {
	Update()
	Draw(screen *ebiten.Image)
}

type BaseScene struct {
}

func (s *BaseScene) Update() {

}
func (s *BaseScene) Draw(screen *ebiten.Image) {

}

const (
	fixedTimeStep         = 1.0 / 60.0
	Ppm           float64 = 80.0
)

type Game struct {
	Renderer         *renderer.JelEbitenRenderer
	World            *jel.World
	Body             *jel.Body
	PPM              float64
	ScreenSize       jel.Vec2
	WorldSize        jel.Vec2
	Center           jel.Vec2
	Debugui          debugui.DebugUI
	BodyPanelPos     image.Rectangle
	RendererPanelPos image.Rectangle

	mass float64

	WallMaterial int
	BodyMaterial int
	Scene        Scene
}

func NewGame(ppm float64) *Game {
	g := &Game{
		World: jel.NewWorld(),
	}
	g.Renderer = renderer.NewJelEbitenRenderer(ppm)
	g.PPM = ppm
	g.Renderer.PixelsPerMeter = ppm
	g.ScreenSize = jel.Vec2{900, 480}
	g.WorldSize = g.Renderer.ScreenToWorld(g.ScreenSize)
	ebiten.SetWindowSize(int(g.ScreenSize.X), int(g.ScreenSize.Y))
	g.Initalize()
	return g
}
func (g *Game) Initalize() {
	g.mass = 1.
	g.BodyPanelPos = RectFromXYWH(0, 0, 250, 450)
	g.RendererPanelPos = RectFromXYWH(900-150, 0, 150, 480)
	g.Center = g.Renderer.ScreenToWorld(g.ScreenSize.DivS(2))
	g.Renderer.SetAll(false)
	g.Renderer.ShowEdgeSprings = true
	g.Renderer.ShowExtraSprings = true
	g.Renderer.ShowEdgeSpringTensions = true
	g.Renderer.ShowExtraSpringTensions = true
	g.Renderer.ShowStrokeGlobalShape = true
	// g.renderer.ShowBodyEdges = true
	g.Renderer.ShowPointMassDots = true
	g.Renderer.ShowJoints = true

	// Materials
	g.BodyMaterial = g.World.AddMaterial()
	g.WallMaterial = g.World.AddMaterial()
	g.World.SetMaterialPairData(g.BodyMaterial, g.WallMaterial, 1.0, 0.5)
}

func (g *Game) MakeRoom() {
	g.MakeWalls(g.WallMaterial, float64(g.BodyPanelPos.Dx())+3, float64(g.RendererPanelPos.Dx())+3, 4, 4)
}

func (g *Game) Update() error {
	if _, err := g.Debugui.Update(func(ctx *debugui.Context) error {
		ctx.Window("Renderer Settings", g.RendererPanelPos, func(layout debugui.ContainerLayout) {
			ctx.Checkbox(&g.Renderer.ShowAABB, "AABB")
			ctx.Checkbox(&g.Renderer.ShowFillPointMasses, "FillPointMasses")
			ctx.Checkbox(&g.Renderer.ShowPointMassDots, "PointMassDots")
			ctx.Checkbox(&g.Renderer.ShowFillGlobalShape, "FillGlobalShape")
			ctx.Checkbox(&g.Renderer.ShowStrokeGlobalShape, "StrokeGlobalShape")
			ctx.Checkbox(&g.Renderer.ShowEdgeSprings, "EdgeSprings")
			ctx.Checkbox(&g.Renderer.ShowExtraSprings, "ExtraSprings")
			ctx.Checkbox(&g.Renderer.ShowEdgeSpringTensions, "EdgeSpringTensions")
			ctx.Checkbox(&g.Renderer.ShowExtraSpringTensions, "ExtraSpringTensions")
			ctx.Checkbox(&g.Renderer.ShowPointMassIndices, "PointMassIndices")
			ctx.Checkbox(&g.Renderer.ShowJoints, "Joints")
			ctx.Checkbox(&g.Renderer.ShowBodyEdges, "BodyEdges")
			ctx.Checkbox(&g.Renderer.Antialias, "Antialias")
			ctx.Text("Fill method")
			ctx.Checkbox(&g.Renderer.UseVectorPackage, "UseVectorPackage")
			// ctx.Checkbox(&g.Renderer.UseTriangleFan, "UseTriangleFan")
			e := ctx.Button("Enable All")
			e.On(func() {
				g.Renderer.SetAll(true)
			})
			e2 := ctx.Button("Disable All")
			e2.On(func() {
				g.Renderer.SetAll(false)
			})
		})
		ctx.Window("Body Settings", g.BodyPanelPos, func(layout debugui.ContainerLayout) {
			if g.Body == nil {
				return
			}
			e := ctx.Button("Reset")
			e.On(func() {
				g.Body.Reset()
				g.Body.SetScaleAnglePosition(jel.Vec2One, 0, g.Center)
			})
			ctx.Text("Masses")
			esmass := ctx.SliderF(&g.mass, 0, 20, 0.1, 2)
			esmass.On(func() {
				g.Body.SetMassAll(g.mass)
			})
			ctx.Text("Velocity Damping")
			ctx.SliderF(&g.Body.VelDamping, 0, 1, 0.01, 3)
			compSMC := g.Body.GetComponent[*jel.ShapeMatchComponent]()
			if compSMC != nil {
				ctx.Header("ShapeMatchComponent", true, func() {
					ctx.Checkbox(&compSMC.Off, "Disabled")
					ctx.Text("Stiffness")
					ctx.SliderF(&compSMC.Stiffness, 0, 2000, 1., 3)
					ctx.Text("Damping")
					ctx.SliderF(&compSMC.Damping, 0, 200, 1., 3)
				})
			}
			compSC := g.Body.GetComponent[*jel.SpringComponent]()
			if compSC != nil {
				ctx.Header("SpringComponent", true, func() {
					ctx.Checkbox(&compSC.Off, "Disabled")
					ctx.Text("Stiffness")
					ctx.SliderF(&compSC.DefaultStiffness, 0, 2000, 1., 3)
					ctx.Text("Damping")
					es := ctx.SliderF(&compSC.DefaultDamping, 0, 100, 0.1, 3)
					es.On(func() {
						compSC.ApplyDefaults()
					})
				})
			}
			compG := g.Body.GetComponent[*jel.GravityComponent]()
			if compG != nil {
				ctx.Header("GravityComponent", true, func() {
					ctx.Checkbox(&compG.Off, "Disabled")
					ctx.NumberFieldF(&compG.Gravity.X, 1, 1)
					ctx.NumberFieldF(&compG.Gravity.Y, 1, 1)
				})
			}
			compP := g.Body.GetComponent[*jel.PressureComponent]()
			if compP != nil {
				ctx.Header("PressureComponent", true, func() {
					ctx.Checkbox(&compP.Off, "Disabled")
					ctx.Text("Pressure")
					ctx.SliderF(&compP.GasPressure, 0, 200, 1., 2)
				})
			}
		})
		return nil
	}); err != nil {
		return err
	}
	g.World.Update(fixedTimeStep)
	g.Renderer.HandleMouseDragPoint(g.World)
	g.Scene.Update()
	return nil
}
func (g *Game) Draw(screen *ebiten.Image) {
	g.Renderer.Draw(screen, g.World)
	g.Debugui.Draw(screen)
	ebitenutil.DebugPrintAt(screen, "You can drag the mass points with the cursor.", 400, 10)
	g.Scene.Draw(screen)
}
func (g *Game) Layout(outsideWidth, outsideHeight int) (int, int) {
	return int(g.ScreenSize.X), int(g.ScreenSize.Y)
}

func RectFromXYWH(x, y, w, h int) image.Rectangle {
	return image.Rectangle{image.Point{x, y}, image.Point{x + w, y + h}}
}
func (g *Game) MakeWalls(materialID int, l, r, b, t float64) {
	w, h := g.WorldSize.X, g.WorldSize.Y
	ppm := g.Renderer.PixelsPerMeter
	l /= ppm
	r /= ppm
	b /= ppm
	t /= ppm
	const thickness = 30.0
	ht := thickness / 2
	horizontalShape := jel.Rectangle(w-l-r, thickness)
	verticalShape := jel.Rectangle(thickness, h-t-b)
	bottom := jel.NewBody(
		horizontalShape,
		jel.Vec2{l + (w-l-r)/2, h - b + ht},
		0,
		jel.Infinity,
	)
	bottom.Material = materialID
	top := jel.NewBody(
		horizontalShape,
		jel.Vec2{l + (w-l-r)/2, t - ht},
		0,
		jel.Infinity,
	)
	top.Material = materialID
	left := jel.NewBody(
		verticalShape,
		jel.Vec2{l - ht, t + (h-t-b)/2},
		0,
		jel.Infinity,
	)
	left.Material = materialID
	right := jel.NewBody(
		verticalShape,
		jel.Vec2{w - r + ht, t + (h-t-b)/2},
		0,
		jel.Infinity,
	)
	right.Material = materialID
	g.World.AddBodies(bottom, top, left, right)
}
