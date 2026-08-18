package common

import (
	"fmt"
	"image/color"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/vector"
	"github.com/setanarut/jel"
)

func init() {
	ebiten.SetScreenClearedEveryFrame(false)
}

type Renderer struct {
	ShowAABB                bool
	ShowFillPointMasses     bool
	ShowStrokePointMasses   bool
	ShowPointMassDots       bool
	ShowFillGlobalShape     bool
	ShowStrokeGlobalShape   bool
	ShowSpring              bool
	ShowStatic              bool
	ShowSpringTensionColors bool
	ShowPointMassIndices    bool

	Antialias      bool
	DrawOffset     jel.Vec2
	PixelsPerMeter float64

	path   vector.Path
	So     vector.StrokeOptions
	SoAABB vector.StrokeOptions
	Dpo    vector.DrawPathOptions
	Fo     vector.FillOptions
}

func NewRenderer(pixelsPerMeter float64) *Renderer {
	return &Renderer{
		Antialias: true,
		path:      vector.Path{},
		So: vector.StrokeOptions{
			Width:      1,
			LineCap:    0,
			LineJoin:   vector.LineJoinMiter,
			MiterLimit: 0,
		},
		SoAABB:         vector.StrokeOptions{Width: 1},
		Dpo:            vector.DrawPathOptions{},
		Fo:             vector.FillOptions{},
		PixelsPerMeter: pixelsPerMeter,
	}
}

var (
	colorBackground        = rgb(47, 91, 46)
	colorAABB              = rgb(190, 254, 254)
	colorFillPointMasses   = rgb(155, 157, 117)
	colorStrokePointMasses = rgb(255, 255, 255)
	colorPointMassDots     = rgb(255, 255, 255)
	colorFillGlobalShape   = rgb(255, 0, 64)
	colorStrokeGlobalShape = rgb(255, 0, 64)
	colorSpring            = rgb(0, 0, 0)
	colorStatic            = rgb(161, 161, 161)
)

func rgb(r, g, b uint8) color.RGBA {
	return color.RGBA{r, g, b, 255}
}

func (r *Renderer) Draw(screen *ebiten.Image, world *jel.World) {
	screen.Fill(colorBackground)

	for _, body := range world.Bodies {
		if r.ShowAABB {
			r.strokeAABB(screen, body)
		}

		if r.ShowFillPointMasses || r.ShowStrokePointMasses {
			r.drawPointMasses(screen, body)
		}

		if r.ShowFillGlobalShape || r.ShowStrokeGlobalShape {
			r.drawGlobalShape(screen, body)
		}

		if r.ShowPointMassDots {
			r.fillPointMassDots(screen, body)
		}

		if r.ShowSpring {
			r.drawSprings(screen, body)
		}

		if r.ShowPointMassIndices {
			r.drawPointMassIndices(screen, body)
		}
	}

	ebitenutil.DebugPrintAt(
		screen,
		fmt.Sprintf("FPS: %v\nTPS: %v", ebiten.ActualFPS(), ebiten.ActualTPS()), 20, 20,
	)
}

// drawPointMasses builds a single path from point masses and draws fill/stroke.
func (r *Renderer) drawPointMasses(screen *ebiten.Image, body jel.Body) {
	if !r.ShowFillPointMasses && !r.ShowStrokePointMasses {
		return
	}

	pts := body.GetPointMasses()
	if len(pts) < 3 {
		return
	}

	r.path.Reset()
	for i, pm := range pts {
		pos := r.WorldToScreen(pm.Pos).Add(r.DrawOffset)
		if i == 0 {
			r.path.MoveTo(float32(pos.X), float32(pos.Y))
		} else {
			r.path.LineTo(float32(pos.X), float32(pos.Y))
		}
	}
	r.path.Close()

	if r.ShowFillPointMasses {
		clr := r.getBodyColor(body, colorFillPointMasses)
		r.Dpo.ColorScale.Reset()
		r.Dpo.ColorScale.ScaleWithColor(clr)
		vector.FillPath(screen, &r.path, &r.Fo, &r.Dpo)
	}

	if r.ShowStrokePointMasses {
		clr := r.getBodyColor(body, colorStrokePointMasses)
		r.Dpo.ColorScale.Reset()
		r.Dpo.ColorScale.ScaleWithColor(clr)
		vector.StrokePath(screen, &r.path, &r.So, &r.Dpo)
	}
}

// drawGlobalShape builds a single path from global shape and draws fill/stroke.
func (r *Renderer) drawGlobalShape(screen *ebiten.Image, body jel.Body) {
	if !r.ShowFillGlobalShape && !r.ShowStrokeGlobalShape {
		return
	}

	base := body.GetBaseBody()
	if base == nil || base.GlobalShape == nil || len(base.GlobalShape) < 3 {
		return
	}

	r.path.Reset()
	for i, v := range base.GlobalShape {
		pos := r.WorldToScreen(v).Add(r.DrawOffset)
		if i == 0 {
			r.path.MoveTo(float32(pos.X), float32(pos.Y))
		} else {
			r.path.LineTo(float32(pos.X), float32(pos.Y))
		}
	}
	r.path.Close()

	if r.ShowFillGlobalShape {
		clr := r.getBodyColor(body, colorFillGlobalShape)
		r.Dpo.ColorScale.Reset()
		r.Dpo.ColorScale.ScaleWithColor(clr)
		vector.FillPath(screen, &r.path, &r.Fo, &r.Dpo)
	}

	if r.ShowStrokeGlobalShape {
		clr := r.getBodyColor(body, colorStrokeGlobalShape)
		r.Dpo.ColorScale.Reset()
		r.Dpo.ColorScale.ScaleWithColor(clr)
		vector.StrokePath(screen, &r.path, &r.So, &r.Dpo)
	}
}

// fillPointMassDots draws small circles at each point mass.
func (r *Renderer) fillPointMassDots(screen *ebiten.Image, body jel.Body) {
	for _, pm := range body.GetPointMasses() {
		pos := r.WorldToScreen(pm.Pos).Add(r.DrawOffset)
		vector.FillCircle(screen, float32(pos.X), float32(pos.Y), r.So.Width*1.5, colorPointMassDots, r.Antialias)
	}
}

// drawPointMassIndices prints indices above each point mass.
func (r *Renderer) drawPointMassIndices(screen *ebiten.Image, body jel.Body) {
	for i, pm := range body.GetPointMasses() {
		pos := r.WorldToScreen(pm.Pos).Add(r.DrawOffset)
		ebitenutil.DebugPrintAt(screen, fmt.Sprintf("%d", i), int(pos.X), int(pos.Y+float64(r.So.Width)))
	}
}

// drawSprings draws internal springs with optional tension coloring.
func (r *Renderer) drawSprings(screen *ebiten.Image, body jel.Body) {
	var sb *jel.SpringBody
	switch v := body.(type) {
	case *jel.SpringBody:
		sb = v
	case *jel.PressureBody:
		sb = v.SpringBody
	default:
		return
	}
	if sb == nil {
		return
	}

	pts := sb.PointMasses
	for i := range sb.Springs {
		spr := &sb.Springs[i]
		a := &pts[spr.IndexA]
		b := &pts[spr.IndexB]

		var clr color.RGBA = colorSpring
		if r.ShowSpringTensionColors {
			forceVec := jel.CalculateSpringForce(
				a.Pos, a.Vel,
				b.Pos, b.Vel,
				spr.RestLength, spr.Stiffness, spr.Damping,
			)
			dist := a.Pos.Sub(b.Pos).Mag()
			scalar := forceVec.Mag()
			if dist < spr.RestLength {
				scalar = -scalar
			}
			mass := max(min(a.Mass, b.Mass), 1e-9)
			maxForce := max(2*math.Sqrt(spr.Stiffness*mass), 1e-9)
			ratio := scalar / maxForce
			clr = tensionColor(ratio)
		}

		aScreen := r.WorldToScreen(a.Pos).Add(r.DrawOffset)
		bScreen := r.WorldToScreen(b.Pos).Add(r.DrawOffset)
		vector.StrokeLine(
			screen,
			float32(aScreen.X), float32(aScreen.Y),
			float32(bScreen.X), float32(bScreen.Y),
			r.So.Width,
			clr,
			r.Antialias,
		)
	}
}

// strokeAABB draws the AABB outline.
func (r *Renderer) strokeAABB(screen *ebiten.Image, body jel.Body) {
	corners := r.getAABBCorners(body.GetAABB())
	r.path.Reset()
	for i, corner := range corners {
		pos := r.WorldToScreen(corner).Add(r.DrawOffset)
		if i == 0 {
			r.path.MoveTo(float32(pos.X), float32(pos.Y))
		} else {
			r.path.LineTo(float32(pos.X), float32(pos.Y))
		}
	}
	r.path.Close()
	r.Dpo.ColorScale.Reset()
	r.Dpo.ColorScale.ScaleWithColor(colorAABB)
	vector.StrokePath(screen, &r.path, &r.SoAABB, &r.Dpo)
}

// getBodyColor returns appropriate color based on static flag and default color.
func (r *Renderer) getBodyColor(body jel.Body, defaultColor color.RGBA) color.RGBA {
	if r.ShowStatic && body.IsStatic() {
		return colorStatic
	}
	return defaultColor
}

func (r *Renderer) WorldToScreen(v jel.Vec2) jel.Vec2 {
	return v.Scale(r.PixelsPerMeter)
}

func (r *Renderer) ScreenToWorld(v jel.Vec2) jel.Vec2 {
	return v.DivS(r.PixelsPerMeter)
}

func (r *Renderer) getAABBCorners(a *jel.AABB) [4]jel.Vec2 {
	return [4]jel.Vec2{
		{X: a.Min.X, Y: a.Min.Y},
		{X: a.Max.X, Y: a.Min.Y},
		{X: a.Max.X, Y: a.Max.Y},
		{X: a.Min.X, Y: a.Max.Y},
	}
}

func tensionColor(ratio float64) color.RGBA {
	ratio = max(-1.0, min(1.0, ratio))
	hue := 120 + 120*ratio
	return hsvToRGB(hue, 1.0, 1.0)
}

func hsvToRGB(h, s, v float64) color.RGBA {
	h = math.Mod(h, 360)
	if h < 0 {
		h += 360
	}
	c := v * s
	x := c * (1 - math.Abs(math.Mod(h/60, 2)-1))
	m := v - c
	var r, g, b float64
	switch {
	case h < 60:
		r, g, b = c, x, 0
	case h < 120:
		r, g, b = x, c, 0
	case h < 180:
		r, g, b = 0, c, x
	case h < 240:
		r, g, b = 0, x, c
	case h < 300:
		r, g, b = x, 0, c
	default:
		r, g, b = c, 0, x
	}
	return color.RGBA{
		R: uint8((r + m) * 255),
		G: uint8((g + m) * 255),
		B: uint8((b + m) * 255),
		A: 255,
	}
}
