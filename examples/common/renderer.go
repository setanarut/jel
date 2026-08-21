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

// 1x1 piksellik beyaz doku (Düz renkler için GPU'ya gönderilecek sabit texture)
var solidImage = ebiten.NewImage(1, 1)

func init() {
	ebiten.SetScreenClearedEveryFrame(false)
	solidImage.Fill(color.White)
}

// PathBatch, aynı renge sahip olan çizgi (stroke) path'lerini gruplamak için kullanılır.
type PathBatch struct {
	Path *vector.Path
	Used bool
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

	So     vector.StrokeOptions
	SoAABB vector.StrokeOptions
	Dpo    vector.DrawPathOptions
	Fo     vector.FillOptions

	// Triangle Fan dolguları için Vertex/Index tamponları (Zero-Allocation)
	vertices  []ebiten.Vertex
	indices   []uint16
	tmpPoints []jel.Vec2

	// Stroke Batching için path'ler
	aabbPath      vector.Path
	strokeBatches map[color.RGBA]*PathBatch
}

func NewRenderer(pixelsPerMeter float64) *Renderer {
	return &Renderer{
		Antialias: true,
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

		strokeBatches: make(map[color.RGBA]*PathBatch),
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

// getStrokePath, verilen renk için batch path'i döndürür.
func (r *Renderer) getStrokePath(clr color.RGBA) *vector.Path {
	b, ok := r.strokeBatches[clr]
	if !ok {
		b = &PathBatch{Path: &vector.Path{}}
		r.strokeBatches[clr] = b
	}
	b.Used = true
	return b.Path
}

// fillPolygon, kapalı poligonları Triangle Fan yöntemiyle tek bir Vertex/Index dizisine ekler.
func (r *Renderer) fillPolygon(points []jel.Vec2, clr color.RGBA) {
	n := len(points)
	if n < 3 {
		return
	}

	// 1. Merkez Nokta Hesaplama (Triangle Fan Yapısı)
	var cx, cy float64
	for i := 0; i < n; i++ {
		cx += points[i].X
		cy += points[i].Y
	}
	cx /= float64(n)
	cy /= float64(n)

	baseIndex := uint16(len(r.vertices))

	rCol := float32(clr.R) / 255.0
	gCol := float32(clr.G) / 255.0
	bCol := float32(clr.B) / 255.0
	aCol := float32(clr.A) / 255.0

	// Merkez Vertex
	r.vertices = append(r.vertices, ebiten.Vertex{
		DstX:   float32(cx),
		DstY:   float32(cy),
		ColorR: rCol, ColorG: gCol, ColorB: bCol, ColorA: aCol,
	})

	// Çevre Vertex'ler
	for i := 0; i < n; i++ {
		r.vertices = append(r.vertices, ebiten.Vertex{
			DstX:   float32(points[i].X),
			DstY:   float32(points[i].Y),
			ColorR: rCol, ColorG: gCol, ColorB: bCol, ColorA: aCol,
		})
	}

	// Üçgen Bağlantı İndeksleri
	for i := 0; i < n; i++ {
		v1 := baseIndex + uint16(i+1)
		v2 := baseIndex + uint16(i+2)
		if i == n-1 {
			v2 = baseIndex + 1
		}
		r.indices = append(r.indices, baseIndex, v1, v2)
	}
}

func (r *Renderer) Draw(screen *ebiten.Image, world *jel.World) {
	screen.Fill(colorBackground)

	// Her frame öncesi bellek ayırmadan (Zero-Allocation) tamponları sıfırla
	r.vertices = r.vertices[:0]
	r.indices = r.indices[:0]

	r.aabbPath.Reset()
	for _, b := range r.strokeBatches {
		b.Used = false
		b.Path.Reset()
	}

	// Objeleri dönüp geometri verilerini topla
	for _, body := range world.Bodies {
		if r.ShowAABB {
			r.strokeAABB(body)
		}

		if r.ShowFillPointMasses || r.ShowStrokePointMasses {
			r.drawPointMasses(body)
		}

		if r.ShowFillGlobalShape || r.ShowStrokeGlobalShape {
			r.drawGlobalShape(body)
		}

		if r.ShowPointMassDots {
			r.fillPointMassDots(body)
		}

		if r.ShowSpring {
			r.drawSprings(body)
		}

		if r.ShowPointMassIndices {
			r.drawPointMassIndices(screen, body)
		}
	}

	// ----------------------------------------------------
	// TOPLU ÇİZİM (BATCH RENDERING) AŞAMASI
	// ----------------------------------------------------

	// Tüm kapalı dolgular (FillPath yerine) tek bir DrawTriangles ile GPU'ya aktarılır
	if len(r.indices) > 0 {
		op := &ebiten.DrawTrianglesOptions{}
		screen.DrawTriangles(r.vertices, r.indices, solidImage, op)
	}

	// AABB'leri tek seferde çiz
	if r.ShowAABB {
		r.Dpo.ColorScale.Reset()
		r.Dpo.ColorScale.ScaleWithColor(colorAABB)
		vector.StrokePath(screen, &r.aabbPath, &r.SoAABB, &r.Dpo)
	}

	// Çizgileri tek seferde (renk başına 1 draw call) çiz
	for clr, b := range r.strokeBatches {
		if b.Used {
			r.Dpo.ColorScale.Reset()
			r.Dpo.ColorScale.ScaleWithColor(clr)
			vector.StrokePath(screen, b.Path, &r.So, &r.Dpo)
		}
	}

	ebitenutil.DebugPrintAt(
		screen,
		fmt.Sprintf("FPS: %v\nTPS: %v", ebiten.ActualFPS(), ebiten.ActualTPS()), 20, 20,
	)
}

func (r *Renderer) drawPointMasses(body jel.Body) {
	pts := body.GetPointMasses()
	if len(pts) < 3 {
		return
	}

	if r.ShowFillPointMasses {
		clr := r.getBodyColor(body, colorFillPointMasses)
		r.tmpPoints = r.tmpPoints[:0]
		for _, pm := range pts {
			pos := r.WorldToScreen(pm.Pos).Add(r.DrawOffset)
			r.tmpPoints = append(r.tmpPoints, pos)
		}
		r.fillPolygon(r.tmpPoints, clr)
	}

	if r.ShowStrokePointMasses {
		clr := r.getBodyColor(body, colorStrokePointMasses)
		p := r.getStrokePath(clr)
		for i, pm := range pts {
			pos := r.WorldToScreen(pm.Pos).Add(r.DrawOffset)
			if i == 0 {
				p.MoveTo(float32(pos.X), float32(pos.Y))
			} else {
				p.LineTo(float32(pos.X), float32(pos.Y))
			}
		}
		p.Close()
	}
}

func (r *Renderer) drawGlobalShape(body jel.Body) {
	base := body.GetBaseBody()
	if base == nil || base.GlobalShape == nil || len(base.GlobalShape) < 3 {
		return
	}

	if r.ShowFillGlobalShape {
		clr := r.getBodyColor(body, colorFillGlobalShape)
		r.tmpPoints = r.tmpPoints[:0]
		for _, v := range base.GlobalShape {
			pos := r.WorldToScreen(v).Add(r.DrawOffset)
			r.tmpPoints = append(r.tmpPoints, pos)
		}
		r.fillPolygon(r.tmpPoints, clr)
	}

	if r.ShowStrokeGlobalShape {
		clr := r.getBodyColor(body, colorStrokeGlobalShape)
		p := r.getStrokePath(clr)
		for i, v := range base.GlobalShape {
			pos := r.WorldToScreen(v).Add(r.DrawOffset)
			if i == 0 {
				p.MoveTo(float32(pos.X), float32(pos.Y))
			} else {
				p.LineTo(float32(pos.X), float32(pos.Y))
			}
		}
		p.Close()
	}
}

func (r *Renderer) fillPointMassDots(body jel.Body) {
	radius := r.So.Width * 1.5
	clr := colorPointMassDots
	const sides = 8 // Noktalar için 8 gen daire simülasyonu

	for _, pm := range body.GetPointMasses() {
		center := r.WorldToScreen(pm.Pos).Add(r.DrawOffset)
		r.tmpPoints = r.tmpPoints[:0]
		for i := 0; i < sides; i++ {
			angle := float64(i) / float64(sides) * 2 * math.Pi
			r.tmpPoints = append(r.tmpPoints, jel.Vec2{
				X: center.X + math.Cos(angle)*float64(radius),
				Y: center.Y + math.Sin(angle)*float64(radius),
			})
		}
		r.fillPolygon(r.tmpPoints, clr)
	}
}

func (r *Renderer) drawPointMassIndices(screen *ebiten.Image, body jel.Body) {
	for i, pm := range body.GetPointMasses() {
		pos := r.WorldToScreen(pm.Pos).Add(r.DrawOffset)
		ebitenutil.DebugPrintAt(screen, fmt.Sprintf("%d", i), int(pos.X), int(pos.Y+float64(r.So.Width)))
	}
}

func (r *Renderer) drawSprings(body jel.Body) {
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

		p := r.getStrokePath(clr)
		p.MoveTo(float32(aScreen.X), float32(aScreen.Y))
		p.LineTo(float32(bScreen.X), float32(bScreen.Y))
	}
}

func (r *Renderer) strokeAABB(body jel.Body) {
	corners := r.getAABBCorners(body.GetAABB())
	for i, corner := range corners {
		pos := r.WorldToScreen(corner).Add(r.DrawOffset)
		if i == 0 {
			r.aabbPath.MoveTo(float32(pos.X), float32(pos.Y))
		} else {
			r.aabbPath.LineTo(float32(pos.X), float32(pos.Y))
		}
	}
	r.aabbPath.Close()
}

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
