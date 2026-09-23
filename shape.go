package jel

import (
	"math"
	"slices"
	"strconv"
	"strings"

	"github.com/setanarut/v"
)

// Shape contains a set of points that is equivalent as the local shape of a [Body].
//
// Points must be added in a counter-clockwise (CCW) fashion to align with screen space
// coordinates where the y-axis grows downwards, ensuring outward-facing edge normals.
type Shape []v.Vec

func (sh *Shape) Clone() Shape {
	return slices.Clone(*sh)
}

// Adds a vertex to this shape
func (sh *Shape) AddVertex(pos v.Vec) {
	*sh = append(*sh, pos)
}

// Adds a vertex to this shape
func (sh *Shape) AddVertexXY(x, y float64) {
	sh.AddVertex(v.Vec{X: x, Y: y})
}

// Recenter re-centers the points of this shape in-place so its centroid
// lies at (0, 0).
func (sh *Shape) Recenter() {
	center := AverageVec2(*sh)
	for i, v := range *sh {
		(*sh)[i] = v.Sub(center)
	}
}

// Reverse reverses the vertices, so they rotate clockwise if they were counterclockwise, and counterclockwise if they were clockwise.
func (sh *Shape) Reverse() {
	slices.Reverse(*sh)
}

// --------- Shape Transformations -----------

// TransformByMatrixToTarget transforms the points on this this shape using a given transformation
// matrix, applying the result into a given target slice of points.
//
//   - note: The target slice of points must have the **same** count of
//     vertices as this shape.
func (sh *Shape) TransformByMatrixToTarget(target []v.Vec, matrix Matrix3x3) {
	// if len(target) != len(*c) {
	// 	panic("target length must equal len(c)")
	// }
	for i := range target {
		target[i] = matrix.Apply((*sh)[i])
	}
}

// TransformByMatrix transforms vertices on this this shape inplace using a given Matrix3x3.
func (sh *Shape) TransformByMatrix(m Matrix3x3) {
	for i := range *sh {
		(*sh)[i] = m.Apply((*sh)[i])
	}
}

// Scale scales vertices on this this shape inplace using a given x, y.
func (sh *Shape) Scale(x, y float64) {
	for i, vertex := range *sh {
		vertex.X *= x
		vertex.Y *= y
		(*sh)[i] = vertex
	}
}

// IsCCW reports whether the shape's vertices are wound counter-clockwise (CCW)
// in screen space, where the Y axis grows downward. Returns false for shapes
// with fewer than 3 vertices, since winding is undefined.
func (sh Shape) IsCCW() bool {
	if len(sh) < 3 {
		return false
	}
	var sum float64
	n := len(sh)
	for i := range n {
		p1 := sh[i]
		p2 := sh[(i+1)%n]
		sum += p1.X*p2.Y - p2.X*p1.Y
	}
	return sum < 0
}

func (sh *Shape) fit(target float64, width bool) (size float64) {
	if len(*sh) == 0 {
		return
	}

	minV, maxV := (*sh)[0], (*sh)[0]
	for _, p := range (*sh)[1:] {
		minV = minV.Min(p)
		maxV = maxV.Max(p)
	}

	if width {
		size = maxV.X - minV.X
	} else {
		size = maxV.Y - minV.Y
	}

	sh.Scale(target/size, target/size)
	return
}

// SetWidth uniformly scales the shape so its width becomes exactly w.
func (sh *Shape) SetWidth(w float64) { sh.fit(w, true) }

// SetHeight uniformly scales the shape so its height becomes exactly h.
func (sh *Shape) SetHeight(h float64) { sh.fit(h, false) }

func (sh *Shape) TranslateVerticesToTarget(target Shape, pos v.Vec) {
	if len(target) != len(*sh) {
		panic("target length must equal len(c.LocalVertices)")
	}
	for i := range target {
		target[i] = (*sh)[i].Add(pos)

	}
}

// ---------- Shape creation methods ----------

// Square returns a shape that represents a square, with side of the
// specified length
func Square(length float64) Shape {
	return Rectangle(length, length)
}

// Rectangle creates a rectangle shape with optional corner chamfering.
// Parameters:
//   - w, h: width and height of the rectangle.
//   - cornerRatio: optional, ratio of corner cut from 0.0 to 0.5.
//     0.5 means the cut reaches exactly the center of the shortest edge.
func Rectangle(w, h float64, cornerRatio ...float64) (s Shape) {
	w /= 2.0
	h /= 2.0
	cr := 0.0
	if len(cornerRatio) > 0 {
		cr = cornerRatio[0]
	}
	if cr <= 0 {
		s.AddVertexXY(-w, h)
		s.AddVertexXY(w, h)
		s.AddVertexXY(w, -h)
		s.AddVertexXY(-w, -h)
		return
	}
	cr = min(cr, 0.5)
	cut := min(w, h) * 2.0 * cr
	s.AddVertexXY(-w, h-cut)
	s.AddVertexXY(-w+cut, h)
	s.AddVertexXY(w-cut, h)
	s.AddVertexXY(w, h-cut)
	s.AddVertexXY(w, -h+cut)
	s.AddVertexXY(w-cut, -h)
	s.AddVertexXY(-w+cut, -h)
	s.AddVertexXY(-w, -h+cut)
	return
}

// RegularPolygon returns a new regular polygon shape with radius and n number of vertices.
// It can also be used to create circles.
// The polygon is rotated so that an edge is always flat at the bottom.
// Vertices are in CCW (counter-clockwise) order.
func RegularPolygon(radius float64, n int) (s Shape) {
	offset := math.Pi/2 + Tau/(2*float64(n))
	for i := range n {
		a := offset - Tau*(float64(i)/float64(n))
		s.AddVertexXY(math.Cos(a)*radius, math.Sin(a)*radius)
	}
	return
}

// "M 0.29 0.22 L -0.09 -0.37 L -0.3 0.24"
// 0.29, 0.22, -0.09, -0.37, -0.3, 0.24

// ShapeFromCoords creates a shape from a flat list of x, y, x, y, ...
// coordinates. Points must be given in counter-clockwise (CCW) order.
//
// If the attribute contains an odd number of coordinates, the last one will be ignored.
//
//		Example:
//	 ShapeFromCoords(0.29, 0.22, -0.09, -0.37, -0.3, 0.24)
//	 ShapeFromCoords([]float64{1, -1, 2, 4, 3, 1})
func ShapeFromCoords(coords ...float64) (s Shape) {
	// Tek sayıda koordinat varsa sonuncuyu ignore et
	if len(coords)%2 != 0 {
		coords = coords[:len(coords)-1] // son elemanı kes
	}
	for i := 0; i < len(coords); i += 2 {
		s.AddVertexXY(coords[i], coords[i+1])
	}
	return
}

// ShapeFromSVGPath makes Shape from an SVG path data string ('d' attribute)
// Note: Only 'M' (MoveTo) and 'L' (LineTo) commands are supported.
//
// https://developer.mozilla.org/en-US/docs/Web/SVG/Reference/Attribute/d
//
// If the attribute contains an odd number of coordinates, the last one will be ignored.
// Points must be given in counter-clockwise (CCW) order.
//
//		Example:
//	 ShapeFromSVGPath("M 0.29 0.22 L -0.09 -0.37 L -0.3 0.24")
func ShapeFromSVGPath(path string) Shape {
	numbers := strings.NewReplacer("M", "", "L", "").Replace(path)
	return ShapeFromSVGPolygonPoints(numbers)
}

// ShapeFromSVGPolygonPoints makes shape from SVG polygon points string.
//
// https://developer.mozilla.org/en-US/docs/Web/SVG/Reference/Attribute/points
//
// If the attribute contains an odd number of coordinates, the last one will be ignored.
// Points must be given in counter-clockwise (CCW) order.
//
//		Example:
//	 ShapeFromSVGPolygonPoints("0.29 0.22 -0.09 -0.37 -0.3 0.24")
func ShapeFromSVGPolygonPoints(points string) (s Shape) {
	tokens := strings.Fields(points)

	if len(tokens)%2 != 0 {
		tokens = tokens[:len(tokens)-1]
	}

	for i := 0; i+1 < len(tokens); i += 2 {
		x, errX := strconv.ParseFloat(tokens[i], 64)
		y, errY := strconv.ParseFloat(tokens[i+1], 64)
		if errX == nil && errY == nil {
			s.AddVertexXY(x, y)
		}
	}
	return
}

// PolygonPointsString returns the vertices in the format used by the SVG <polygon> element's points,
// which is also the format accepted by [ShapeFromSVGPolygonPoints].
// Precision specifies the number of digits after the decimal point.
func (sh Shape) PolygonPointsString(precision int) string {
	var b strings.Builder

	for i, v := range sh {
		if i > 0 {
			b.WriteByte(' ')
		}

		b.WriteString(strconv.FormatFloat(v.X, 'f', precision, 64))
		b.WriteByte(' ')
		b.WriteString(strconv.FormatFloat(v.Y, 'f', precision, 64))
	}

	return b.String()
}
