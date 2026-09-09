package jel

import (
	"math"
	"slices"
	"strconv"
	"strings"
)

// Shape contains a set of points that is equivalent as the local shape of a [Body].
//
// Points must be added in a counter-clockwise (CCW) fashion to align with screen space
// coordinates where the y-axis grows downwards, ensuring outward-facing edge normals.
type Shape []Vec2

func (c *Shape) Clone() Shape {
	return slices.Clone(*c)
}

// Adds a vertex to this shape
func (c *Shape) AddVertex(pos Vec2) {
	*c = append(*c, pos)
}

// Adds a vertex to this shape
func (c *Shape) AddVertexXY(x, y float64) {
	c.AddVertex(Vec2{X: x, Y: y})
}

// Recenter re-centers the points of this shape in-place so its centroid
// lies at (0, 0).
func (c *Shape) Recenter() {
	center := AverageVec2(*c)
	for i, v := range *c {
		(*c)[i] = v.Sub(center)
	}
}

// Reverse reverses the vertices, so they rotate clockwise if they were counterclockwise, and counterclockwise if they were clockwise.
func (c *Shape) Reverse() {
	slices.Reverse(*c)
}

// --------- Shape Transformations -----------

// TransformByMatrixToTarget transforms the points on this this shape using a given transformation
// matrix, applying the result into a given target slice of points.
//
//   - note: The target slice of points must have the **same** count of
//     vertices as this shape.
func (c *Shape) TransformByMatrixToTarget(target []Vec2, matrix Matrix3x3) {
	// if len(target) != len(*c) {
	// 	panic("target length must equal len(c)")
	// }
	for i := range target {
		target[i] = matrix.Apply((*c)[i])
	}
}

// TransformByMatrix transforms vertices on this this shape inplace using a given Matrix3x3.
func (c *Shape) TransformByMatrix(m Matrix3x3) {
	for i := range *c {
		(*c)[i] = m.Apply((*c)[i])
	}
}

// Scale scales vertices on this this shape inplace using a given x, y.
func (c *Shape) Scale(x, y float64) {
	for i, vertex := range *c {
		vertex.X *= x
		vertex.Y *= y
		(*c)[i] = vertex
	}
}

// IsCCW reports whether the shape's vertices are wound counter-clockwise (CCW)
// in screen space, where the Y axis grows downward. Returns false for shapes
// with fewer than 3 vertices, since winding is undefined.
func (c Shape) IsCCW() bool {
	if len(c) < 3 {
		return false
	}
	var sum float64
	n := len(c)
	for i := range n {
		p1 := c[i]
		p2 := c[(i+1)%n]
		sum += p1.X*p2.Y - p2.X*p1.Y
	}
	return sum < 0
}

func (c *Shape) fit(target float64, width bool) (size float64) {
	if len(*c) == 0 {
		return
	}

	minV, maxV := (*c)[0], (*c)[0]
	for _, p := range (*c)[1:] {
		minV = minV.Min(p)
		maxV = maxV.Max(p)
	}

	if width {
		size = maxV.X - minV.X
	} else {
		size = maxV.Y - minV.Y
	}

	c.Scale(target/size, target/size)
	return
}

// FitWidth uniformly scales the shape so its width becomes exactly w.
func (c *Shape) FitWidth(w float64) { c.fit(w, true) }

// FitHeight uniformly scales the shape so its height becomes exactly h.
func (c *Shape) FitHeight(h float64) { c.fit(h, false) }

func (c *Shape) TranslateVerticesToTarget(target Shape, pos Vec2) {
	if len(target) != len(*c) {
		panic("target length must equal len(c.LocalVertices)")
	}
	for i := range target {
		target[i] = (*c)[i].Add(pos)

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
func (s Shape) PolygonPointsString(precision int) string {
	var b strings.Builder

	for i, v := range s {
		if i > 0 {
			b.WriteByte(' ')
		}

		b.WriteString(strconv.FormatFloat(v.X, 'f', precision, 64))
		b.WriteByte(' ')
		b.WriteString(strconv.FormatFloat(v.Y, 'f', precision, 64))
	}

	return b.String()
}
