package renderer

import (
	"fmt"
	"image/color"
	"math"

	"github.com/hajimehoshi/ebiten/v2"
	"github.com/hajimehoshi/ebiten/v2/ebitenutil"
	"github.com/hajimehoshi/ebiten/v2/inpututil"
	"github.com/hajimehoshi/ebiten/v2/vector"
	"github.com/setanarut/jel"
)

var solidImage = ebiten.NewImage(1, 1)
var rgb = Rgb

func init() {
	ebiten.SetScreenClearedEveryFrame(false)
	solidImage.Fill(color.White)
}

type Colors struct {
	Background        color.RGBA
	AABB              color.RGBA
	FillPointMasses   color.RGBA
	PointMassDots     color.RGBA
	StrokeGlobalShape color.RGBA
	BodyEdge          color.RGBA
	EdgeSpring        color.RGBA
	ExtraEdgeSpring   color.RGBA
	Static            color.RGBA
	Joint             color.RGBA
}

func DefaultColors() Colors {
	return Colors{
		Background:        rgb(47, 91, 46),
		AABB:              rgb(190, 254, 254),
		FillPointMasses:   rgb(187, 140, 102),
		PointMassDots:     rgb(255, 255, 255),
		StrokeGlobalShape: rgb(0, 0, 0),
		BodyEdge:          rgb(134, 186, 255),
		EdgeSpring:        rgb(234, 234, 234),
		ExtraEdgeSpring:   rgb(255, 215, 70),
		Static:            rgb(118, 118, 118),
		Joint:             rgb(255, 0, 242),
	}
}

type JelEbitenRenderer struct {
	ShowAABB                bool
	ShowFillPointMasses     bool
	ShowPointMassDots       bool
	ShowFillGlobalShape     bool
	ShowStrokeGlobalShape   bool
	ShowEdgeSprings         bool
	ShowExtraSprings        bool
	ShowEdgeSpringTensions  bool
	ShowExtraSpringTensions bool
	ShowPointMassIndices    bool
	ShowJoints              bool
	ShowBodyEdges           bool
	ShowTPS_FPS             bool
	Antialias               bool
	DrawOffset              jel.Vec2
	PixelsPerMeter          float64
	LineThickness           float64
	Colors                  Colors
	UseVectorPackage        bool
	UseTriangleFan          bool
	vertices                []ebiten.Vertex
	indices                 []uint16
	vCount                  uint16
	iCount                  uint32
	fillPointMassPath       vector.Path
	tmpPoints               []jel.Vec2
	tmpIndices              []int
	isDragging              bool
	dragBody                *jel.Body
	dragPoint               int
	targetPos               jel.Vec2
	prevTarget              jel.Vec2
	grabOffset              jel.Vec2
}

func NewJelEbitenRenderer(pixelsPerMeter float64) *JelEbitenRenderer {
	return &JelEbitenRenderer{
		Antialias:        true,
		PixelsPerMeter:   pixelsPerMeter,
		LineThickness:    2.0,
		Colors:           DefaultColors(),
		UseVectorPackage: false,
		UseTriangleFan:   false,
		vertices:         make([]ebiten.Vertex, 10000),
		indices:          make([]uint16, 30000),
		tmpPoints:        make([]jel.Vec2, 0, 64),
		tmpIndices:       make([]int, 0, 128),
	}
}
func Rgb(r, g, b uint8) color.RGBA {
	return color.RGBA{r, g, b, 255}
}
func colorToFloat(c color.RGBA) (r, g, b, a float32) {
	return float32(c.R) / 255.0, float32(c.G) / 255.0, float32(c.B) / 255.0, float32(c.A) / 255.0
}
func (r *JelEbitenRenderer) ensureCapacity(addV uint16, addI uint32) {
	if int(r.vCount+addV) > len(r.vertices) {
		newVerts := make([]ebiten.Vertex, len(r.vertices)*2)
		copy(newVerts, r.vertices)
		r.vertices = newVerts
	}
	if int(r.iCount+addI) > len(r.indices) {
		newInds := make([]uint16, len(r.indices)*2)
		copy(newInds, r.indices)
		r.indices = newInds
	}
}
func (r *JelEbitenRenderer) SetAll(enabled bool) {
	r.ShowAABB = enabled
	r.ShowFillPointMasses = enabled
	r.ShowPointMassDots = enabled
	r.ShowFillGlobalShape = enabled
	r.ShowStrokeGlobalShape = enabled
	r.ShowEdgeSprings = enabled
	r.ShowExtraSprings = enabled
	r.ShowEdgeSpringTensions = enabled
	r.ShowExtraSpringTensions = enabled
	r.ShowPointMassIndices = enabled
	r.ShowJoints = enabled
	r.ShowBodyEdges = enabled
}
func signedArea(points []jel.Vec2) float64 {
	area := 0.0
	n := len(points)
	for i := range n {
		j := (i + 1) % n
		area += points[i].X*points[j].Y - points[j].X*points[i].Y
	}
	return area / 2.0
}
func isConvex(a, b, c jel.Vec2, isCW bool) bool {
	cross := (b.X-a.X)*(c.Y-b.Y) - (b.Y-a.Y)*(c.X-b.X)
	if isCW {
		return cross < 0
	}
	return cross > 0
}
func pointInTriangle(p, a, b, c jel.Vec2) bool {
	areaABC := math.Abs((a.X*(b.Y-c.Y) + b.X*(c.Y-a.Y) + c.X*(a.Y-b.Y)) / 2.0)
	areaPBC := math.Abs((p.X*(b.Y-c.Y) + b.X*(c.Y-p.Y) + c.X*(p.Y-b.Y)) / 2.0)
	areaPCA := math.Abs((p.X*(c.Y-a.Y) + c.X*(a.Y-p.Y) + a.X*(p.Y-c.Y)) / 2.0)
	areaPAB := math.Abs((p.X*(a.Y-b.Y) + a.X*(b.Y-p.Y) + b.X*(p.Y-a.Y)) / 2.0)
	epsilon := 1e-9
	return math.Abs(areaABC-(areaPBC+areaPCA+areaPAB)) < epsilon
}
func earClip(points []jel.Vec2) [][3]int {
	n := len(points)
	if n < 3 {
		return nil
	}
	area := signedArea(points)
	if area == 0 {
		return nil
	}
	isCW := area < 0
	indices := make([]int, n)
	for i := range n {
		indices[i] = i
	}
	var triangles [][3]int
	for len(indices) > 3 {
		earFound := false
		for i := 0; i < len(indices); i++ {
			prevIdx := indices[(i-1+len(indices))%len(indices)]
			currIdx := indices[i]
			nextIdx := indices[(i+1)%len(indices)]
			if isConvex(points[prevIdx], points[currIdx], points[nextIdx], isCW) {
				isEar := true
				for _, idx := range indices {
					if idx == prevIdx || idx == currIdx || idx == nextIdx {
						continue
					}
					if pointInTriangle(points[idx], points[prevIdx], points[currIdx], points[nextIdx]) {
						isEar = false
						break
					}
				}
				if isEar {
					triangles = append(triangles, [3]int{prevIdx, currIdx, nextIdx})
					indices = append(indices[:i], indices[i+1:]...)
					earFound = true
					break
				}
			}
		}
		if !earFound {
			for j := 1; j < len(indices)-1; j++ {
				triangles = append(triangles, [3]int{indices[0], indices[j], indices[j+1]})
			}
			break
		}
	}
	if len(indices) == 3 {
		triangles = append(triangles, [3]int{indices[0], indices[1], indices[2]})
	}
	return triangles
}
func (r *JelEbitenRenderer) fillPolygon(center jel.Vec2, points []jel.Vec2, clr color.RGBA) {
	n := len(points)
	if n < 3 {
		return
	}
	rCol, gCol, bCol, aCol := colorToFloat(clr)
	if r.UseTriangleFan {
		r.ensureCapacity(uint16(n+1), uint32(n*3))
		startV := r.vCount
		r.vertices[r.vCount] = ebiten.Vertex{
			DstX:   float32(center.X),
			DstY:   float32(center.Y),
			ColorR: rCol,
			ColorG: gCol,
			ColorB: bCol,
			ColorA: aCol,
		}
		r.vCount++
		for i := range n {
			r.vertices[r.vCount] = ebiten.Vertex{
				DstX:   float32(points[i].X),
				DstY:   float32(points[i].Y),
				ColorR: rCol,
				ColorG: gCol,
				ColorB: bCol,
				ColorA: aCol,
			}
			r.vCount++
		}
		for i := range n {
			r.indices[r.iCount+0] = startV
			r.indices[r.iCount+1] = startV + 1 + uint16(i)
			nextIdx := i + 1
			if nextIdx == n {
				nextIdx = 0
			}
			r.indices[r.iCount+2] = startV + 1 + uint16(nextIdx)
			r.iCount += 3
		}
		return
	}
	triangles := earClip(points)
	if len(triangles) == 0 {
		return
	}
	r.ensureCapacity(uint16(n), uint32(len(triangles)*3))
	startV := r.vCount
	for _, p := range points {
		r.vertices[r.vCount] = ebiten.Vertex{
			DstX:   float32(p.X),
			DstY:   float32(p.Y),
			ColorR: rCol,
			ColorG: gCol,
			ColorB: bCol,
			ColorA: aCol,
		}
		r.vCount++
	}
	for _, tri := range triangles {
		r.indices[r.iCount+0] = startV + uint16(tri[0])
		r.indices[r.iCount+1] = startV + uint16(tri[1])
		r.indices[r.iCount+2] = startV + uint16(tri[2])
		r.iCount += 3
	}
}
func (r *JelEbitenRenderer) appendPolygon(path *vector.Path, points []jel.Vec2) {
	if len(points) < 3 {
		return
	}
	path.MoveTo(float32(points[0].X), float32(points[0].Y))
	for i := 1; i < len(points); i++ {
		path.LineTo(float32(points[i].X), float32(points[i].Y))
	}
	path.Close()
}
func (r *JelEbitenRenderer) addLineAA(a, b jel.Vec2, thickness float64, clr color.RGBA) {
	dir := b.Sub(a)
	mag := dir.Mag()
	if mag < 0.0001 {
		return
	}
	nx := -dir.Y / mag
	ny := dir.X / mag
	feather := 1.0
	halfThick := thickness / 2.0
	innerDist := halfThick
	outerDist := halfThick + feather
	rCol, gCol, bCol, aCol := colorToFloat(clr)
	r.ensureCapacity(8, 18)
	startV := r.vCount
	ax, ay := float32(a.X), float32(a.Y)
	bx, by := float32(b.X), float32(b.Y)
	nxf, nyf := float32(nx), float32(ny)
	innerF, outerF := float32(innerDist), float32(outerDist)
	r.vertices[r.vCount+0] = ebiten.Vertex{DstX: ax - nxf*outerF, DstY: ay - nyf*outerF, ColorR: 0, ColorG: 0, ColorB: 0, ColorA: 0}
	r.vertices[r.vCount+1] = ebiten.Vertex{DstX: bx - nxf*outerF, DstY: by - nyf*outerF, ColorR: 0, ColorG: 0, ColorB: 0, ColorA: 0}
	r.vertices[r.vCount+2] = ebiten.Vertex{DstX: ax - nxf*innerF, DstY: ay - nyf*innerF, ColorR: rCol, ColorG: gCol, ColorB: bCol, ColorA: aCol}
	r.vertices[r.vCount+3] = ebiten.Vertex{DstX: bx - nxf*innerF, DstY: by - nyf*innerF, ColorR: rCol, ColorG: gCol, ColorB: bCol, ColorA: aCol}
	r.vertices[r.vCount+4] = ebiten.Vertex{DstX: ax + nxf*innerF, DstY: ay + nyf*innerF, ColorR: rCol, ColorG: gCol, ColorB: bCol, ColorA: aCol}
	r.vertices[r.vCount+5] = ebiten.Vertex{DstX: bx + nxf*innerF, DstY: by + nyf*innerF, ColorR: rCol, ColorG: gCol, ColorB: bCol, ColorA: aCol}
	r.vertices[r.vCount+6] = ebiten.Vertex{DstX: ax + nxf*outerF, DstY: ay + nyf*outerF, ColorR: 0, ColorG: 0, ColorB: 0, ColorA: 0}
	r.vertices[r.vCount+7] = ebiten.Vertex{DstX: bx + nxf*outerF, DstY: by + nyf*outerF, ColorR: 0, ColorG: 0, ColorB: 0, ColorA: 0}
	r.vCount += 8
	r.indices[r.iCount+0] = startV + 0
	r.indices[r.iCount+1] = startV + 1
	r.indices[r.iCount+2] = startV + 3
	r.indices[r.iCount+3] = startV + 0
	r.indices[r.iCount+4] = startV + 3
	r.indices[r.iCount+5] = startV + 2
	r.indices[r.iCount+6] = startV + 2
	r.indices[r.iCount+7] = startV + 3
	r.indices[r.iCount+8] = startV + 5
	r.indices[r.iCount+9] = startV + 2
	r.indices[r.iCount+10] = startV + 5
	r.indices[r.iCount+11] = startV + 4
	r.indices[r.iCount+12] = startV + 4
	r.indices[r.iCount+13] = startV + 5
	r.indices[r.iCount+14] = startV + 7
	r.indices[r.iCount+15] = startV + 4
	r.indices[r.iCount+16] = startV + 7
	r.indices[r.iCount+17] = startV + 6
	r.iCount += 18
}
func (r *JelEbitenRenderer) Draw(screen *ebiten.Image, world *jel.World) {
	screen.Fill(r.Colors.Background)
	r.vCount = 0
	r.iCount = 0
	if r.UseVectorPackage {
		r.fillPointMassPath.Reset()
	}
	for _, body := range world.Bodies {
		if r.ShowAABB {
			r.drawAABB(body)
		}
		if r.ShowFillPointMasses {
			if r.UseVectorPackage {
				r.appendFillPointMassesVector(body)
			} else {
				r.drawFillPointMassesTriangles(body)
			}
		}
		if !body.IsStatic {
			if r.ShowStrokeGlobalShape {
				r.drawStrokeGlobalShape(body)
			}
		}
		if r.ShowBodyEdges {
			r.drawBodyEdges(body)
		}
		if r.ShowEdgeSprings || r.ShowExtraSprings {
			r.drawSprings(body)
		}
		if r.ShowPointMassDots {
			r.drawPointMassDots(body)
		}
	}
	if r.UseVectorPackage && r.ShowFillPointMasses {
		r.drawFillPointMassesVector(screen)
	}
	if r.ShowJoints {
		for _, joint := range world.Joints {
			pos1 := joint.LinkA().Position()
			pos2 := joint.LinkB().Position()
			jointColor := r.Colors.Joint
			if sj, ok := joint.(*jel.SpringJoint); ok && r.ShowEdgeSpringTensions {
				jointColor = tensionColor(jel.CalcSpringTension(pos1, pos2, sj.RestDistance))
			}
			screenPos1 := r.WorldToScreen(pos1).Add(r.DrawOffset)
			screenPos2 := r.WorldToScreen(pos2).Add(r.DrawOffset)
			_, ok := joint.(*jel.PinJoint)
			if ok {
				p := r.WorldToScreen(joint.LinkA().Position())
				r.addStrokeCircle(p, 9, r.LineThickness, jointColor)
			} else {
				r.addLineAA(screenPos1, screenPos2, r.LineThickness, jointColor)
			}
		}
	}
	if r.iCount > 0 {
		op := &ebiten.DrawTrianglesOptions{}
		screen.DrawTriangles(r.vertices[:r.vCount], r.indices[:r.iCount], solidImage, op)
	}
	for _, body := range world.Bodies {
		if r.ShowPointMassIndices {
			r.drawPointMassIndices(screen, body)
		}
	}
	if r.ShowTPS_FPS {
		ebitenutil.DebugPrintAt(
			screen,
			fmt.Sprintf("FPS: %v\nTPS: %v", ebiten.ActualFPS(), ebiten.ActualTPS()), 20, 20,
		)
	}
}
func (r *JelEbitenRenderer) drawFillPointMassesTriangles(body *jel.Body) {
	pts := body.PointMasses
	if len(pts) < 3 {
		return
	}
	clr := r.colorStatic(body.IsStatic, r.Colors.FillPointMasses)
	if userColor, ok := body.UserData.(color.Color); ok {
		r_c, g_c, b_c, a_c := userColor.RGBA()
		clr = color.RGBA{
			R: uint8(r_c >> 8),
			G: uint8(g_c >> 8),
			B: uint8(b_c >> 8),
			A: uint8(a_c >> 8),
		}
	}
	r.tmpPoints = r.tmpPoints[:0]
	for _, pm := range pts {
		pos := r.WorldToScreen(pm.Position).Add(r.DrawOffset)
		r.tmpPoints = append(r.tmpPoints, pos)
	}
	screenCenter := r.WorldToScreen(body.DerivedPos).Add(r.DrawOffset)
	r.fillPolygon(screenCenter, r.tmpPoints, clr)
}
func (r *JelEbitenRenderer) appendFillPointMassesVector(body *jel.Body) {
	pts := body.PointMasses
	if len(pts) < 3 {
		return
	}
	r.tmpPoints = r.tmpPoints[:0]
	for _, pm := range pts {
		pos := r.WorldToScreen(pm.Position).Add(r.DrawOffset)
		r.tmpPoints = append(r.tmpPoints, pos)
	}
	r.appendPolygon(&r.fillPointMassPath, r.tmpPoints)
}
func (r *JelEbitenRenderer) drawFillPointMassesVector(screen *ebiten.Image) {
	if r.fillPointMassPath.Bounds().Empty() {
		return
	}
	var colorScale ebiten.ColorScale
	colorScale.ScaleWithColor(r.Colors.FillPointMasses)
	vector.FillPath(
		screen,
		&r.fillPointMassPath,
		&vector.FillOptions{
			FillRule: vector.FillRuleNonZero,
		},
		&vector.DrawPathOptions{
			AntiAlias:  r.Antialias,
			ColorScale: colorScale,
		},
	)
}
func (r *JelEbitenRenderer) drawAABB(body *jel.Body) {
	corners := r.getAABBCorners(&body.AABB)
	for i := range 4 {
		a := r.WorldToScreen(corners[i]).Add(r.DrawOffset)
		b := r.WorldToScreen(corners[(i+1)%4]).Add(r.DrawOffset)
		r.addLineAA(a, b, r.LineThickness, r.Colors.AABB)
	}
}

func (r *JelEbitenRenderer) drawStrokeGlobalShape(body *jel.Body) {
	if body == nil || body.GlobalShape == nil || len(body.GlobalShape) < 3 {
		return
	}
	clr := r.colorStatic(body.IsStatic, r.Colors.StrokeGlobalShape)
	shape := body.GlobalShape
	for i := range shape {
		a := r.WorldToScreen(shape[i]).Add(r.DrawOffset)
		b := r.WorldToScreen(shape[(i+1)%len(shape)]).Add(r.DrawOffset)
		r.addLineAA(a, b, r.LineThickness, clr)
	}
}
func (r *JelEbitenRenderer) drawPointMassDots(body *jel.Body) {
	radius := r.LineThickness * 1.5
	clr := r.Colors.PointMassDots
	for _, pm := range body.PointMasses {
		center := r.WorldToScreen(pm.Position).Add(r.DrawOffset)
		r.addCircle(center, radius, clr)
	}
}
func (r *JelEbitenRenderer) drawBodyEdges(body *jel.Body) {
	if body == nil || body.Edges == nil {
		return
	}
	for _, edge := range body.Edges {
		a := r.WorldToScreen(edge.Start).Add(r.DrawOffset)
		b := r.WorldToScreen(edge.End).Add(r.DrawOffset)
		r.addLineAA(a, b, r.LineThickness, r.colorStatic(body.IsStatic, r.Colors.BodyEdge))
	}
}
func (r *JelEbitenRenderer) drawPointMassIndices(screen *ebiten.Image, body *jel.Body) {
	for i, pm := range body.PointMasses {
		pos := r.WorldToScreen(pm.Position).Add(r.DrawOffset)
		ebitenutil.DebugPrintAt(screen, fmt.Sprintf("%d", i), int(pos.X), int(pos.Y+r.LineThickness))
	}
}
func (r *JelEbitenRenderer) drawSprings(body *jel.Body) {
	springComp := body.GetComponent[*jel.SpringComponent]()
	if springComp == nil {
		return
	}
	pts := body.PointMasses
	for _, spring := range springComp.Springs {
		if spring.PointMassA >= len(pts) || spring.PointMassB >= len(pts) {
			continue
		}
		if spring.Type == jel.EdgeSpring && !r.ShowEdgeSprings {
			continue
		}
		if spring.Type == jel.ExtraSpring && !r.ShowExtraSprings {
			continue
		}
		a := pts[spring.PointMassA]
		b := pts[spring.PointMassB]
		baseColor := r.Colors.EdgeSpring
		if spring.Type == jel.ExtraSpring {
			baseColor = r.Colors.ExtraEdgeSpring
		}
		clr := r.colorStatic(body.IsStatic, baseColor)
		showTension := false
		switch spring.Type {
		case jel.EdgeSpring:
			showTension = r.ShowEdgeSpringTensions
		case jel.ExtraSpring:
			showTension = r.ShowExtraSpringTensions
		}
		if !body.IsStatic && showTension {
			normalized := jel.CalcSpringTension(a.Position, b.Position, spring.RestDistance)
			clr = tensionColor(normalized)
		}
		aScreen := r.WorldToScreen(a.Position).Add(r.DrawOffset)
		bScreen := r.WorldToScreen(b.Position).Add(r.DrawOffset)
		r.addLineAA(aScreen, bScreen, r.LineThickness, clr)
	}
}
func (r *JelEbitenRenderer) colorStatic(s bool, defaultColor color.RGBA) color.RGBA {
	if s {
		return r.Colors.Static
	}
	return defaultColor
}
func (r *JelEbitenRenderer) WorldToScreen(v jel.Vec2) jel.Vec2 {
	return v.Scale(r.PixelsPerMeter)
}
func (r *JelEbitenRenderer) ScreenToWorld(v jel.Vec2) jel.Vec2 {
	return v.DivS(r.PixelsPerMeter)
}
func (r *JelEbitenRenderer) CursorPosition() jel.Vec2 {
	x, y := ebiten.CursorPosition()
	return jel.Vec2{X: float64(x), Y: float64(y)}
}
func (r *JelEbitenRenderer) CursorWorldPosition() jel.Vec2 {
	return r.ScreenToWorld(r.CursorPosition())
}
func (r *JelEbitenRenderer) getAABBCorners(a *jel.AABB) [4]jel.Vec2 {
	return [4]jel.Vec2{
		{X: a.Min.X, Y: a.Min.Y},
		{X: a.Max.X, Y: a.Min.Y},
		{X: a.Max.X, Y: a.Max.Y},
		{X: a.Min.X, Y: a.Max.Y},
	}
}
func tensionColor(ratio float64) color.RGBA {
	ratio = max(-1.0, min(1.0, ratio))
	absRatio := math.Abs(ratio)
	hue := 60.0 * (1.0 - absRatio)
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
func (r *JelEbitenRenderer) addCircle(center jel.Vec2, radius float64, clr color.RGBA) {
	if radius < 1 {
		radius = 2
	}
	segments := 12
	r.ensureCapacity(uint16(segments+1), uint32(segments*3))
	startV := r.vCount
	rCol, gCol, bCol, aCol := colorToFloat(clr)
	cx, cy := float32(center.X), float32(center.Y)
	r.vertices[r.vCount] = ebiten.Vertex{DstX: cx, DstY: cy, ColorR: rCol, ColorG: gCol, ColorB: bCol, ColorA: aCol}
	r.vCount++
	for i := range segments {
		angle := float64(i) / float64(segments) * 2 * math.Pi
		x := cx + float32(math.Cos(angle)*radius)
		y := cy + float32(math.Sin(angle)*radius)
		r.vertices[r.vCount] = ebiten.Vertex{DstX: x, DstY: y, ColorR: rCol, ColorG: gCol, ColorB: bCol, ColorA: aCol}
		r.vCount++
	}
	for i := uint16(0); i < uint16(segments); i++ {
		r.indices[r.iCount] = startV
		r.indices[r.iCount+1] = startV + 1 + i
		if i == uint16(segments-1) {
			r.indices[r.iCount+2] = startV + 1
		} else {
			r.indices[r.iCount+2] = startV + 2 + i
		}
		r.iCount += 3
	}
}
func (r *JelEbitenRenderer) addStrokeCircle(center jel.Vec2, radius float64, thickness float64, clr color.RGBA) {
	if radius < 1 {
		radius = 2
	}
	segments := 16
	prev := jel.Vec2{X: center.X + radius, Y: center.Y}
	for i := 1; i <= segments; i++ {
		angle := float64(i) / float64(segments) * 2 * math.Pi
		curr := jel.Vec2{
			X: center.X + math.Cos(angle)*radius,
			Y: center.Y + math.Sin(angle)*radius,
		}
		r.addLineAA(prev, curr, thickness, clr)
		prev = curr
	}
}
func (r *JelEbitenRenderer) HandleMouseDragPoint(world *jel.World) {
	r.targetPos = r.ScreenToWorld(r.CursorPosition())
	if inpututil.IsMouseButtonJustPressed(ebiten.MouseButtonLeft) {
		maxDist := 30.0 / r.PixelsPerMeter
		var closestBody *jel.Body
		closestIndex, minDist := -1, maxDist
		for _, b := range world.Bodies {
			expanded := b.AABB.Expanded(maxDist)
			if !expanded.Contains(r.targetPos) {
				continue
			}
			if idx, dist := b.ClosestPointMass(r.targetPos); dist >= 0 && dist < minDist {
				closestBody, closestIndex, minDist = b, idx, dist
			}
		}
		if closestBody != nil && closestIndex >= 0 {
			r.isDragging = true
			r.dragBody = closestBody
			r.dragPoint = closestIndex
			r.grabOffset = r.targetPos.Sub(closestBody.PointMasses[closestIndex].Position)
		}
	}
	if inpututil.IsMouseButtonJustReleased(ebiten.MouseButtonLeft) {
		r.isDragging = false
		r.dragBody = nil
	}
	if !r.isDragging || r.dragBody == nil || r.dragPoint < 0 {
		return
	}
	pm := r.dragBody.PointMasses[r.dragPoint]
	targetPos := r.targetPos.Sub(r.grabOffset)
	const dragSoftness = 0.5
	stiffness, damping := jel.StiffnessDampingFrom(pm.Mass, dragSoftness)
	error := targetPos.Sub(pm.Position)
	force := error.Scale(stiffness).Sub(pm.Velocity.Scale(damping))
	maxForce := 500.0
	if force.Mag() > maxForce {
		force = force.Unit().Scale(maxForce)
	}
	pm.ApplyForce(force)
}
