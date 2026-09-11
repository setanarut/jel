package jel

import (
	"fmt"
	"math"
	"math/bits"
	"strings"
	"testing"
)

const bitmaskBitSize = 64

func TestBitmask(t *testing.T) {
	var b Bitmask = 0
	b.SetOn(1)
	b.SetOn(2)
	b.SetOn(3)

	if b != 7 {
		t.Errorf("Bitmask should be 7, got %d", b)
	}
}

func TestBitmaskSetRange(t *testing.T) {
	newBitmask := func(offset, count int) Bitmask {
		return Bitmask((^uint(0) >> (bits.UintSize - count)) << max(0, offset-1))
	}
	assertBitmasksMatch(t, newBitmask(0, int(bitmaskBitSize)), Bitmask(^uint64(0)))
	assertBitmasksMatch(t, newBitmask(1, 1), Bitmask(0b01))
	assertBitmasksMatch(t, newBitmask(1, 2), Bitmask(0b11))
	assertBitmasksMatch(t, newBitmask(10, 2), Bitmask(0b00000110_00000000))
}

func TestGenerateBitmaskMinimum(t *testing.T) {
	world := NewWorld()
	world.SetWorldLimits(Vec2{X: -20, Y: -20}, Vec2{X: 20, Y: 20})
	aabb := AABB{Min: world.worldLimits.Min, Max: world.worldLimits.Min}
	bitmasks := world.bitmask(aabb)
	assertBitmasksMatch(t, bitmasks.X, Bitmask(1))
	assertBitmasksMatch(t, bitmasks.Y, Bitmask(1))
}
func TestGenerateBitmaskMaximum(t *testing.T) {
	world := NewWorld()
	world.SetWorldLimits(Vec2{X: -20, Y: -20}, Vec2{X: 20, Y: 20})
	aabb := AABB{Min: world.worldLimits.Max, Max: world.worldLimits.Max}
	bitmasks := world.bitmask(aabb)
	expected := Bitmask(1) << (bitmaskBitSize - 1)
	assertBitmasksMatch(t, bitmasks.X, expected)
	assertBitmasksMatch(t, bitmasks.Y, expected)
}
func TestGenerateBitmaskCenter(t *testing.T) {
	world := NewWorld()
	world.SetWorldLimits(Vec2{X: -20, Y: -20}, Vec2{X: 20, Y: 20})
	aabb := AABB{Min: Vec2{X: 0, Y: 0}, Max: Vec2{X: 0, Y: 0}}
	bitmasks := world.bitmask(aabb)
	expected := Bitmask(1) << (bitmaskBitSize/2 - 1)
	assertBitmasksMatch(t, bitmasks.X, expected)
	assertBitmasksMatch(t, bitmasks.Y, expected)
}
func TestGenerateBitmaskFilling(t *testing.T) {
	world := NewWorld()
	world.SetWorldLimits(Vec2{X: -20, Y: -20}, Vec2{X: 20, Y: 20})
	aabb := AABB{Min: world.worldLimits.Min, Max: world.worldLimits.Max}
	bitmasks := world.bitmask(aabb)
	assertBitmasksMatch(t, bitmasks.X, Bitmask(^uint64(0)))
	assertBitmasksMatch(t, bitmasks.Y, Bitmask(^uint64(0)))
}
func TestGenerateBitmaskQuarter(t *testing.T) {
	world := NewWorld()
	world.SetWorldLimits(Vec2{X: -20, Y: -20}, Vec2{X: 20, Y: 20})
	aabb := AABB{Min: Vec2{X: 0, Y: 0}, Max: world.worldLimits.Max}
	bitmasks := world.bitmask(aabb)
	expected := Bitmask(0b11111111_11111111_11111111_11111111_10000000_00000000_00000000_00000000)
	assertBitmasksMatch(t, bitmasks.X, expected)
	assertBitmasksMatch(t, bitmasks.Y, expected)
}
func TestGenerateBitmaskSmallRect(t *testing.T) {
	world := NewWorld()
	world.SetWorldLimits(Vec2{X: -20, Y: -20}, Vec2{X: 20, Y: 20})
	aabb := AABB{Min: Vec2{X: -4, Y: -4}, Max: Vec2{X: 0, Y: 0}}
	bitmasks := world.bitmask(aabb)
	expected := Bitmask(0x0000_0000_FF00_0000)
	assertBitmasksMatch(t, bitmasks.X, expected)
	assertBitmasksMatch(t, bitmasks.Y, expected)
}
func TestGenerateBitmaskEmptyRect(t *testing.T) {
	world := NewWorld()
	world.SetWorldLimits(Vec2{X: -20, Y: -20}, Vec2{X: 20, Y: 20})
	aabb := AABB{Min: Vec2{X: 0, Y: 0}, Max: Vec2{X: 0, Y: 0}}
	bitmasks := world.bitmask(aabb)
	expected := Bitmask(0x0000_0000_8000_0000)
	assertBitmasksMatch(t, bitmasks.X, expected)
	assertBitmasksMatch(t, bitmasks.Y, expected)
}
func TestGenerateBitmaskLimitBounds(t *testing.T) {
	world := NewWorld()
	world.SetWorldLimits(Vec2{X: -20, Y: -20}, Vec2{X: 20, Y: 20})
	aabb := AABB{Min: Vec2{X: 30, Y: 30}, Max: Vec2{X: 32, Y: 32}}
	bitmasks := world.bitmask(aabb)
	expected := Bitmask(0x8000_0000_0000_0000)
	assertBitmasksMatch(t, bitmasks.X, expected)
	assertBitmasksMatch(t, bitmasks.Y, expected)
	aabb = AABB{Min: Vec2{X: -33, Y: -33}, Max: Vec2{X: -32, Y: -32}}
	bitmasks = world.bitmask(aabb)
	expected = Bitmask(0x0000_0000_0000_0001)
	assertBitmasksMatch(t, bitmasks.X, expected)
	assertBitmasksMatch(t, bitmasks.Y, expected)
}
func TestGenerateBitmaskNaN(t *testing.T) {
	world := NewWorld()
	world.SetWorldLimits(Vec2{X: -20, Y: -20}, Vec2{X: 20, Y: 20})
	aabb := AABB{Min: Vec2{X: math.NaN(), Y: 0}, Max: Vec2{X: 0, Y: 0}}
	bitmasks := world.bitmask(aabb)
	if bitmasks.X != 0 {
		t.Errorf("Expected X to be 0, got %v", bitmasks.X)
	}
	if bitmasks.Y != 0 {
		t.Errorf("Expected Y to be 0, got %v", bitmasks.Y)
	}
}
func TestBitmasksIntersect(t *testing.T) {
	world := NewWorld()
	bit1 := bitmaskPair{X: 0b1111_0000, Y: 0b0000_1111}
	bit2 := bitmaskPair{X: 0b0000_1111, Y: 0b1111_0000}
	bit3 := bitmaskPair{X: 0b0001_1110, Y: 0b0111_1000}
	if !world.bitmasksIntersect(bit1, bit1) {
		t.Error("bit1 should intersect with itself")
	}
	if world.bitmasksIntersect(bit1, bit2) {
		t.Error("bit1 should NOT intersect with bit2")
	}
	if !world.bitmasksIntersect(bit1, bit3) {
		t.Error("bit1 should intersect with bit3")
	}
	if !world.bitmasksIntersect(bit2, bit3) {
		t.Error("bit2 should intersect with bit3")
	}
}
func TestRayCast(t *testing.T) {
	world := NewWorld()
	world.SetWorldLimits(Vec2{X: -20, Y: -20}, Vec2{X: 20, Y: 20})
	body1 := NewBody(Square(6), Vec2{3, 3}, 0, 1)
	world.AddBodies(body1, NewBody(Rectangle(8, 6), Vec2{10, 6}, 0, 1))
	pt, body := world.RayCast(Vec2{-10, -10}, Vec2{10, 10}, 0, nil)
	if body == nil {
		t.Fatal("Should have found body")
	}
	if body != body1 {
		t.Errorf("Expected body1, got %v", body)
	}
	if !pt.IsZero() {
		t.Errorf("Expected point (0,0), got %v", pt)
	}
}
func TestRayCast2(t *testing.T) {
	world := NewWorld()
	world.SetWorldLimits(Vec2{-20, -20}, Vec2{20, 20})
	body := NewBody(RegularPolygon(6, 10), Vec2{2, 10}, 0, 1, world)
	_, bd := world.RayCast(Vec2{0, -10}, Vec2{4, 20}, 0, nil)
	if bd == nil {
		t.Fatal("Should have found body")
	}
	if bd != body {
		t.Errorf("Expected body, got %v", bd)
	}
}
func assertBitmasksMatch(t *testing.T, actual, expected Bitmask) {
	if actual != expected {
		message := fmt.Sprintf("Bitmasks do not match, expected:\n%s\nfound:\n%s",
			formatBinary(expected), formatBinary(actual))
		t.Error(message)
	}
}
func formatBinary(value Bitmask) string {
	base := fmt.Sprintf("%b", uint64(value))
	pad := strings.Repeat("0", bitmaskBitSize-len(base))
	resultPreSpace := pad + base
	var result strings.Builder
	for bit := 0; bit < bitmaskBitSize; bit += 8 {
		if bit > 0 {
			result.WriteString(" ")
		}
		result.WriteString(resultPreSpace[bit : bit+8])
	}
	return "0b" + result.String()
}

func TestBody_UpdateEdgesAndNormals(t *testing.T) {
	// Tests the Body doesn't evaluate nan for point normals for edges
	// that are exactly overlapping one another

	// Make a shape with two parallel edges overlapping
	// Looks roughly like this:
	//  .___.__.
	//  |  /
	//  | /
	//  |/
	//  .
	//

	shape := Shape{}
	shape.AddVertexXY(0, 0)
	shape.AddVertexXY(1, 0)
	shape.AddVertexXY(0.5, 0)
	shape.AddVertexXY(0, 1)

	body := NewBody(shape, Vec2{}, 0, 1)

	body.updateEdgesAndNormals()

	for _, point := range body.PointMasses {
		if math.IsNaN(point.Normal.X) {
			t.Errorf("point.Normal.X is NaN")
		}
		if math.IsNaN(point.Normal.Y) {
			t.Errorf("point.Normal.Y is NaN")
		}
	}
}

func TestAABB(t *testing.T) {
	aabb1 := NewAABB(Vec2{}, Vec2{10, 10})
	aabb2 := NewAABB(Vec2{-1, -1}, Vec2{})
	if !aabb1.Contains(Vec2{}) {
		t.Error("aabb1.Contains(vec) should be true")
	}
	if !aabb2.Intersects(aabb1) {
		t.Error("aabb2.Intersects(aabb1) should be true")
	}
}

func TestAABBWithPointsSimple(t *testing.T) {
	// Tests AABB minimum/maximum coordinates calculation

	point1 := Vec2{X: 1, Y: 2}
	point2 := Vec2{X: 10, Y: 20}

	aabb := NewAABBFromPoints([]Vec2{point1, point2})

	expectedmin := Vec2{X: 1, Y: 2}
	expectedMax := Vec2{X: 10, Y: 20}

	if aabb.Min != expectedmin {
		t.Errorf("Expected Min %v, got %v", Vec2{X: 1, Y: 2}, aabb.Min)
	}
	if aabb.Max != expectedMax {
		t.Errorf("Expected Max %v, got %v", Vec2{X: 10, Y: 20}, aabb.Max)
	}
}

func TestAABBWithPointsMixed(t *testing.T) {
	// Tests AABB minimum/maximum coordinates calculation
	// This test mixes minimum and maximum x and y axis between the
	// vectors

	point1 := Vec2{X: 10, Y: 2}
	point2 := Vec2{X: 1, Y: 20}

	aabb := NewAABBFromPoints([]Vec2{point1, point2})

	if aabb.Min != (Vec2{X: 1, Y: 2}) {
		t.Errorf("Expected Min %v, got %v", Vec2{X: 1, Y: 2}, aabb.Min)
	}
	if aabb.Max != (Vec2{X: 10, Y: 20}) {
		t.Errorf("Expected Max %v, got %v", Vec2{X: 10, Y: 20}, aabb.Max)
	}
}

func TestAABBIntersection(t *testing.T) {
	//
	// Tests AABB intersection with a configuration:
	//  ___
	// |  _|_
	// |_|_| |
	//   |___|
	//

	aabb1 := NewAABB(Vec2{X: 0, Y: 0}, Vec2{X: 10, Y: 10})
	aabb2 := NewAABB(Vec2{X: 5, Y: 5}, Vec2{X: 15, Y: 15})

	if !aabb1.Intersects(aabb2) {
		t.Errorf("Expected aabb1 to intersect aabb2")
	}
}

func TestAABBIntersectionSharingEdges(t *testing.T) {
	//
	// Tests AABB intersection with a configuration:
	//  _______
	// |   |   |
	// |___|___|
	//
	// Sharing edge should be detected as intersection

	aabb1 := NewAABB(Vec2{X: 0, Y: 0}, Vec2{X: 5, Y: 5})
	aabb2 := NewAABB(Vec2{X: 5, Y: 0}, Vec2{X: 10, Y: 5})

	if !aabb1.Intersects(aabb2) {
		t.Errorf("Expected aabb1 to intersect aabb2")
	}
}

func TestAABBComplexIntersection(t *testing.T) {
	//
	// Tests AABB intersection with a configuration:
	//     ___
	//  __|___|__
	// |  |   |  |
	// |__|___|__|
	//    |___|
	//

	aabb1 := NewAABB(Vec2{X: 5, Y: 0}, Vec2{X: 10, Y: 15})
	aabb2 := NewAABB(Vec2{X: 0, Y: 5}, Vec2{X: 15, Y: 10})

	if !aabb1.Intersects(aabb2) {
		t.Errorf("Expected aabb1 to intersect aabb2")
	}
}

func TestAABBNoIntersection(t *testing.T) {
	//
	// Tests AABB intersection with a configuration:
	//  ___
	// |   |
	// |___| ___
	//      |   |
	//      |___|
	//
	// Should not report intersection!
	//

	aabb1 := NewAABB(Vec2{X: 0, Y: 0}, Vec2{X: 10, Y: 10})
	aabb2 := NewAABB(Vec2{X: 11, Y: 11}, Vec2{X: 15, Y: 15})

	if aabb1.Intersects(aabb2) {
		t.Errorf("Expected aabb1 to NOT intersect aabb2")
	}
}

func TestAABBNoIntersectionComplex(t *testing.T) {
	//
	// Tests AABB intersection with a configuration:
	//   _____
	//  |     |
	//  |     |
	//  |_____| _____
	//         |     |
	//         |     |
	//         |_____|
	//
	// This is a mixture of complex AABB creation and non-intersection
	// detection.

	aabb1 := NewAABB(Vec2{X: 5, Y: 0}, Vec2{X: 10, Y: 10})
	aabb2 := NewAABB(Vec2{X: 6, Y: 11}, Vec2{X: 14, Y: 20})

	if aabb1.Intersects(aabb2) {
		t.Errorf("Expected aabb1 to NOT intersect aabb2")
	}
}

type testObserver struct {
	*UnimplementedCollisionObserver // Embed
	Collisions                      []CollisionInfo
}

func (t *testObserver) BodiesDidCollide(infos []CollisionInfo) {
	t.Collisions = infos
}

func TestCollisionSolveSquares(t *testing.T) {
	// Test simple collision detection between two overlapping square-shaped
	// bodies
	//
	// Simulation looks roughly like this:
	//   ______
	//  |      | ⟋  ⟍
	//  |      <       > (that's supposed to be two squares, with the right
	//  |______| ⟍  ⟋    one twisted 45° counter-clockwise.)
	//

	world := NewWorld()
	observer := &testObserver{}
	world.CollisionObserver = observer
	shape := Square(10)
	world.AddBody(NewBody(shape, Vec2{}, 0, 1))
	world.AddBody(NewBody(shape, Vec2{11, 0}, Pi/4, 1))

	world.Update(1.0 / 200.0)

	if len(observer.Collisions) != 1 {
		t.Errorf("Expected 1 collision, got %d", len(observer.Collisions))
	}
}

const epsilon = 1e-15

func almostEqual(a, b float64) bool {
	return math.Abs(a-b) < epsilon
}

func TestLineIntersect(t *testing.T) {
	tests := []struct {
		name      string
		aStart    Vec2
		aEnd      Vec2
		bStart    Vec2
		bEnd      Vec2
		wantHit   bool
		wantHitPt Vec2
		wantUa    float64
		wantUb    float64
	}{
		{
			name:      "Normal Intersection (X-shape)",
			aStart:    Vec2{X: 0, Y: 0},
			aEnd:      Vec2{X: 10, Y: 10},
			bStart:    Vec2{X: 0, Y: 10},
			bEnd:      Vec2{X: 10, Y: 0},
			wantHit:   true,
			wantHitPt: Vec2{X: 5, Y: 5},
			wantUa:    0.5,
			wantUb:    0.5,
		},
		{
			name:    "Parallel Lines (No Intersection)",
			aStart:  Vec2{X: 0, Y: 0},
			aEnd:    Vec2{X: 10, Y: 0},
			bStart:  Vec2{X: 0, Y: 5},
			bEnd:    Vec2{X: 10, Y: 5},
			wantHit: false,
		},
		{
			name:      "Endpoint Intersection",
			aStart:    Vec2{X: 0, Y: 0},
			aEnd:      Vec2{X: 5, Y: 5},
			bStart:    Vec2{X: 5, Y: 5},
			bEnd:      Vec2{X: 10, Y: 0},
			wantHit:   true,
			wantHitPt: Vec2{X: 5, Y: 5},
			wantUa:    1.0,
			wantUb:    0.0,
		},
		{
			name:    "Missed Intersection - Outside Segment A (Ua > 1)",
			aStart:  Vec2{X: 0, Y: 0},
			aEnd:    Vec2{X: 2, Y: 2},
			bStart:  Vec2{X: 0, Y: 10},
			bEnd:    Vec2{X: 10, Y: 0},
			wantHit: false,
		},
		{
			name:    "Missed Intersection - Outside Segment B (Ub < 0)",
			aStart:  Vec2{X: 0, Y: 0},
			aEnd:    Vec2{X: 10, Y: 10},
			bStart:  Vec2{X: 6, Y: 4},
			bEnd:    Vec2{X: 10, Y: 0},
			wantHit: false,
		},
		{
			name:      "T-shaped Intersection",
			aStart:    Vec2{X: 5, Y: 0},
			aEnd:      Vec2{X: 5, Y: 10},
			bStart:    Vec2{X: 0, Y: 5},
			bEnd:      Vec2{X: 10, Y: 5},
			wantHit:   true,
			wantHitPt: Vec2{X: 5, Y: 5},
			wantUa:    0.5,
			wantUb:    0.5,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			gotRes, gotHit := LineIntersect(tt.aStart, tt.aEnd, tt.bStart, tt.bEnd)

			if gotHit != tt.wantHit {
				t.Fatalf("LineIntersect() gotHit = %v, want %v", gotHit, tt.wantHit)
			}

			if gotHit {
				if !almostEqual(gotRes.HitPt.X, tt.wantHitPt.X) || !almostEqual(gotRes.HitPt.Y, tt.wantHitPt.Y) {
					t.Errorf("LineIntersect() HitPt = %+v, want %+v", gotRes.HitPt, tt.wantHitPt)
				}
				if !almostEqual(gotRes.Ua, tt.wantUa) {
					t.Errorf("LineIntersect() Ua = %v, want %v", gotRes.Ua, tt.wantUa)
				}
				if !almostEqual(gotRes.Ub, tt.wantUb) {
					t.Errorf("LineIntersect() Ub = %v, want %v", gotRes.Ub, tt.wantUb)
				}
			}
		})
	}
}

var testShape Shape = ShapeFromCoords(0, 0, 1, 0, 1, 1, 0, 1)

func init() {
	testShape.Recenter()
}

func TestMatrixScale(t *testing.T) {
	vector := Vec2{X: 10, Y: -20}
	expected := Vec2{X: 5, Y: -40}
	matrix := Matrix3x3{}
	matrix.Scale(0.5, 2)
	transformed := matrix.Apply(vector)
	if transformed != expected {
		t.Errorf("Expected %v, got %v", expected, transformed)
	}
}
func TestMatrixRotate(t *testing.T) {
	vector := Vec2{X: 10, Y: 10}
	expected := Vec2{X: -10, Y: 10}
	matrix := Matrix3x3{}
	matrix.Rotate(Pi / 2)
	transformed := matrix.Apply(vector)
	if math.Abs(transformed.X-expected.X) > delta {
		t.Errorf("Expected X: %v, got %v", expected.X, transformed.X)
	}
	if math.Abs(transformed.Y-expected.Y) > delta {
		t.Errorf("Expected Y: %v, got %v", expected.Y, transformed.Y)
	}
}

func TestCompoundMatrix(t *testing.T) {
	vector := Vec2{X: 10, Y: 10}
	expected := Vec2{X: 5, Y: 15}
	matrix := NewMatrix3x3(
		Vec2{X: 0.5, Y: 0.5},
		Pi/2,
		Vec2{X: 10, Y: 10},
	)
	transformed := matrix.Apply(vector)
	if math.Abs(transformed.X-expected.X) > delta {
		t.Errorf("Expected X: %v, got %v", expected.X, transformed.X)
	}
	if math.Abs(transformed.Y-expected.Y) > delta {
		t.Errorf("Expected Y: %v, got %v", expected.Y, transformed.Y)
	}
}

func TestTranslateVertices(t *testing.T) {
	// Create the ts shape with no modifications
	ts := make(Shape, len(testShape))

	testShape.TranslateVerticesToTarget(ts, Vec2One)
	// Assert that both shapes are equal
	unit := Vec2One
	if !ts[0].Equals(testShape[0].Add(unit)) {
		t.Errorf("Vertex 0 mismatch. Expected: %v, Got: %v",
			testShape[0].Add(unit), ts[0])
	}
	if !ts[1].Equals(testShape[1].Add(unit)) {
		t.Errorf("Vertex 1 mismatch. Expected: %v, Got: %v",
			testShape[1].Add(unit), ts[1])
	}
	if !ts[2].Equals(testShape[2].Add(unit)) {
		t.Errorf("Vertex 2 mismatch. Expected: %v, Got: %v",
			testShape[2].Add(unit), ts[2])
	}
	if !ts[3].Equals(testShape[3].Add(unit)) {
		t.Errorf("Vertex 3 mismatch. Expected: %v, Got: %v",
			testShape[3].Add(unit), ts[3])
	}
}

func TestShapeTransformByMatrixToTarget(t *testing.T) {
	expected := ShapeFromCoords(1, 1, 0, 1, 0, 0, 1, 0)
	expected.Recenter()
	transformed := make(Shape, len(testShape))
	matrix := NewMatrix3x3(Vec2One, Pi, Vec2{})
	testShape.TransformByMatrixToTarget(transformed, matrix)

	// Since we rotated a box 180º, the edges are the same, but offset by 1.
	for i := range expected {
		diff := expected[i].Sub(transformed[i])
		if math.Abs(diff.X) > delta {
			t.Errorf("Vertex %d X mismatch. Expected: %v, Got: %v, Diff: %v",
				i, expected[i], transformed[i], diff)
		}
		if math.Abs(diff.Y) > delta {
			t.Errorf("Vertex %d Y mismatch. Expected: %v, Got: %v, Diff: %v",
				i, expected[i], transformed[i], diff)
		}
	}
}

func TestVelocityAccumulation(t *testing.T) {
	p := NewPointMass(0.2, Vec2{X: 0, Y: 0})
	p.Force = p.Force.Add(Vec2{X: 1, Y: 2})
	p.Force = p.Force.Add(Vec2{X: 1, Y: 2})
	p.Force = p.Force.Add(Vec2{X: 1, Y: 2})

	p.Integrate(1.0)

	expectedVelocity := Vec2{X: 3, Y: 6}.DivS(0.2)
	if !p.Velocity.Equals(expectedVelocity) {
		t.Errorf("The velocity did not accumulate as expected! Got: %v, Expected: %v", p.Velocity, expectedVelocity)
	}

	if !p.Force.Equals(Vec2{X: 0, Y: 0}) {
		t.Errorf("After integrating a point mass, the force should reset to 0! Got: %v", p.Force)
	}

	expectedPosition := Vec2{X: 3, Y: 6}.DivS(0.2)
	if !p.Position.Equals(expectedPosition) {
		t.Errorf("The position of the point mass should be modified on the same integration the velocity is modified! Got: %v, Expected: %v", p.Position, expectedPosition)
	}
}

func TestNewShapeSquare(t *testing.T) {
	s := Square(5)

	if s[0].Dist(s[1]) != 5 {
		t.Errorf("Side 0-1 expected 5, got %v", s[0].Dist(s[1]))
	}
	if s[1].Dist(s[2]) != 5 {
		t.Errorf("Side 1-2 expected 5, got %v", s[1].Dist(s[2]))
	}
	if s[2].Dist(s[3]) != 5 {
		t.Errorf("Side 2-3 expected 5, got %v", s[2].Dist(s[3]))
	}
	if s[3].Dist(s[0]) != 5 {
		t.Errorf("Side 3-0 expected 5, got %v", s[3].Dist(s[0]))
	}

	if len(s) != 4 {
		t.Errorf("Expected 4 vertices, got %d", len(s))
	}
}

func TestNewShapeRectangle(t *testing.T) {
	shape := Rectangle(2, 4)

	if shape[0].Dist(shape[1]) != 2 {
		t.Errorf("Side 0-1 expected 2, got %v", shape[0].Dist(shape[1]))
	}
	if shape[1].Dist(shape[2]) != 4 {
		t.Errorf("Side 1-2 expected 4, got %v", shape[1].Dist(shape[2]))
	}
	if shape[2].Dist(shape[3]) != 2 {
		t.Errorf("Side 2-3 expected 2, got %v", shape[2].Dist(shape[3]))
	}
	if shape[3].Dist(shape[0]) != 4 {
		t.Errorf("Side 3-0 expected 4, got %v", shape[3].Dist(shape[0]))
	}

	if len(shape) != 4 {
		t.Errorf("Expected 4 vertices, got %d", len(shape))
	}
}

func TestNewShapeCircle(t *testing.T) {
	shape := RegularPolygon(1, 4)

	testDelta := delta * 2
	sqrt2 := math.Sqrt(2)

	if math.Abs(shape[0].Dist(shape[1])-sqrt2) > testDelta {
		t.Errorf("Distance 0-1 expected %v, got %v", sqrt2, shape[0].Dist(shape[1]))
	}
	if math.Abs(shape[1].Dist(shape[2])-sqrt2) > testDelta {
		t.Errorf("Distance 1-2 expected %v, got %v", sqrt2, shape[1].Dist(shape[2]))
	}
	if math.Abs(shape[2].Dist(shape[3])-sqrt2) > testDelta {
		t.Errorf("Distance 2-3 expected %v, got %v", sqrt2, shape[2].Dist(shape[3]))
	}
	if math.Abs(shape[3].Dist(shape[0])-sqrt2) > testDelta {
		t.Errorf("Distance 3-0 expected %v, got %v", sqrt2, shape[3].Dist(shape[0]))
	}

	if len(shape) != 4 {
		t.Errorf("Expected 4 vertices, got %d", len(shape))
	}
}

func TestNewShapeCircleCenter(t *testing.T) {
	shape := RegularPolygon(1, 8)

	// Verify all points lay the same distance to center, and are
	// approximately the same distance appart - that proves this is a circle
	radi := shape[0].Mag()
	dist := shape[0].Dist(shape[1])

	testDelta := delta * 10

	lastPoint := shape[7]
	for i, p := range shape {
		if math.Abs(p.Mag()-radi) > delta {
			t.Errorf("Failed radius check on point %d. Expected: %v, Got: %v",
				i, radi, p.Mag())
		}
		if math.Abs(p.Dist(lastPoint)-dist) > testDelta {
			t.Errorf("Failed distance check on point %d. Expected: %v, Got: %v",
				i, dist, p.Dist(lastPoint))
		}
		lastPoint = p
	}

	if len(shape) != 8 {
		t.Errorf("Expected 8 vertices, got %d", len(shape))
	}
}

func TestNewShapeFromPolygonPoints(t *testing.T) {
	tests := []struct {
		name  string
		input string
		want  Shape
	}{
		{
			name:  "basic points",
			input: "126.2 85.14 251.6 93.57 257.6 31.67",
			want: Shape{
				{X: 126.2, Y: 85.14},
				{X: 251.6, Y: 93.57},
				{X: 257.6, Y: 31.67},
			},
		},
		{
			name:  "negative numbers",
			input: "0 0 0.519 -0.777 0.182 -0.666 -0.183 -0.873",
			want: Shape{
				{X: 0, Y: 0},
				{X: 0.519, Y: -0.777},
				{X: 0.182, Y: -0.666},
				{X: -0.183, Y: -0.873},
			},
		},
		{
			name:  "empty string",
			input: "",
			want:  nil,
		},
		{
			name:  "odd number of tokens ignores last",
			input: "1 2 3",
			want: Shape{
				{X: 1, Y: 2},
			},
		},
		{
			name:  "invalid token skips pair",
			input: "1 2 foo bar 3 4",
			want: Shape{
				{X: 1, Y: 2},
				{X: 3, Y: 4},
			},
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			shape := ShapeFromSVGPolygonPoints(tt.input)
			assertVertices(t, shape, tt.want)
		})
	}
}

func TestNewShapeFromSVGPath(t *testing.T) {
	tests := []struct {
		name  string
		input string
		want  Shape
	}{
		{
			name:  "M and L commands",
			input: "M 0 0 L 0.519 -0.777 L 0.182 -0.666 L -0.183 -0.873",
			want: Shape{
				{X: 0, Y: 0},
				{X: 0.519, Y: -0.777},
				{X: 0.182, Y: -0.666},
				{X: -0.183, Y: -0.873},
			},
		},
		{
			name:  "only M command",
			input: "M 1 2",
			want: Shape{
				{X: 1, Y: 2},
			},
		},
		{
			name:  "empty path",
			input: "",
			want:  nil,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			shape := ShapeFromSVGPath(tt.input)
			assertVertices(t, shape, tt.want)
		})
	}
}

// assertVertices compares the shape's vertices against the expected list.
func assertVertices(t *testing.T, shape Shape, want Shape) {
	t.Helper()

	if len(shape) != len(want) {
		t.Fatalf("got %d vertices, want %d", len(shape), len(want))
	}

	for i, v := range shape {
		if v.X != want[i].X || v.Y != want[i].Y {
			t.Errorf("vertex %d = %+v, want %+v", i, v, want[i])
		}
	}
}

var delta float64 = 1e-14

func TestVec2Perp(t *testing.T) {
	vec1 := Vec2{X: 0, Y: 1}
	vecPerp := vec1.Perp()
	if math.Abs(vecPerp.X-(-vec1.Y)) > delta || math.Abs(vecPerp.Y-vec1.X) > delta {
		t.Error("Perpendicular test failed!")
	}
}
func TestVec2Dist(t *testing.T) {
	vec1 := Vec2{X: 4, Y: 8}
	vec2 := Vec2{X: 14, Y: 13}
	dx := float64(4 - 14)
	dy := float64(8 - 13)
	dis := vec1.Dist(vec2)
	dissq := vec1.DistSq(vec2)
	sss := math.Sqrt(dx*dx + dy*dy)
	if math.Abs(dis-sss) > delta {
		t.Error("Distance test failed!")
	}
	if math.Abs(dissq-(dx*dx+dy*dy)) > delta {
		t.Error("Distance squared test failed!")
	}
}
func TestVec2Math(t *testing.T) {
	vec1 := Vec2{X: 4, Y: 6}
	vec2 := Vec2{X: 9, Y: 7}
	dot := vec1.Dot(vec2)
	expectedDot := float64(4*9 + 6*7)
	if math.Abs(dot-expectedDot) > delta {
		t.Error("DOT product test failed!")
	}
	cross := vec1.Cross(vec2)
	expectedCross := float64(4*7 - 6*9)
	if math.Abs(cross-expectedCross) > delta {
		t.Error("CROSS product test failed!")
	}
}

func TestLineLerp(t *testing.T) {
	pt1 := Vec2{X: 0, Y: 0}
	pt2 := Vec2{X: 10, Y: 10}

	// Edge cases
	if !pt1.Lerp(pt2, 0).Equals(pt1) {
		t.Error("Failed to calculate point in line correctly at ratio 0")
	}
	if !pt1.Lerp(pt2, 1).Equals(pt2) {
		t.Error("Failed to calculate point in line correctly at ratio 1")
	}

	// Mid-way
	expected := pt1.Add(pt2).DivS(2)
	result := pt1.Lerp(pt2, 0.5)
	if !result.Equals(expected) {
		t.Errorf("Failed to calculate point in line correctly at ratio 0.5. Expected: %v, Got: %v", expected, result)
	}
}

func TestLineLerpCentralized(t *testing.T) {
	pt1 := Vec2{X: -10, Y: -10}
	pt2 := Vec2{X: 10, Y: 10}

	// Edge cases
	if !pt1.Lerp(pt2, 0).Equals(pt1) {
		t.Error("Failed to calculate point in line correctly at ratio 0")
	}
	if !pt1.Lerp(pt2, 1).Equals(pt2) {
		t.Error("Failed to calculate point in line correctly at ratio 1")
	}

	// Mid-way
	result := pt1.Lerp(pt2, 0.5)
	if result.DistSq(Vec2{X: 0, Y: 0}) > delta {
		t.Errorf("Failed to calculate point in line correctly at ratio 0.5. Expected: (0,0), Got: %v", result)
	}
}

func TestBroadPhaseCandidatesDeduplicatesAndOrdersPairs(t *testing.T) {
	w := NewWorld()
	bodies := []*Body{
		{AABB: NewAABB(Vec2{X: 0, Y: 0}, Vec2{X: 2, Y: 2})},
		{AABB: NewAABB(Vec2{X: 1, Y: 1}, Vec2{X: 3, Y: 3})},
		{AABB: NewAABB(Vec2{X: 10, Y: 10}, Vec2{X: 11, Y: 11})},
	}

	pairs := w.broadPhaseCandidates(bodies)
	if len(pairs) != 1 {
		t.Fatalf("got %d broad-phase pairs, want 1", len(pairs))
	}
	if got, want := pairs[0], uint64(1); got != want {
		t.Errorf("pair key = %d, want %d (body indices 0 and 1)", got, want)
	}

	// Reusing the scratch maps must not retain pairs from the previous frame.
	pairs = w.broadPhaseCandidates(bodies[1:])
	if len(pairs) != 0 {
		t.Errorf("got %d stale broad-phase pairs after reuse, want 0", len(pairs))
	}
}

func TestClosestCollisionEdgesMatchesBruteForceSearch(t *testing.T) {
	body := NewBody(RegularPolygon(10, 32), Vec2{}, 0, 1)
	point := Vec2{X: 2.75, Y: -1.5}
	pointNormal := Vec2{X: 1, Y: 0}

	away, same, foundAway := body.closestCollisionEdges(point, pointNormal)
	bruteAway, bruteSame, bruteFoundAway := CollisionInfo{}, CollisionInfo{}, false
	closestAway, closestSame := Infinity, Infinity
	for edgeIndex := range body.Edges {
		hit, normal, edgeD, distance := body.ClosestPointOnEdgeSq(point, edgeIndex)
		info := CollisionInfo{BodyBpmA: edgeIndex, BodyBpmB: (edgeIndex + 1) % len(body.PointMasses), EdgeD: edgeD, HitPt: hit, Normal: normal, Penetration: distance}
		if pointNormal.Dot(normal) <= 0 {
			if distance < closestAway {
				closestAway, bruteAway, bruteFoundAway = distance, info, true
			}
		} else if distance < closestSame {
			closestSame, bruteSame = distance, info
		}
	}
	if foundAway != bruteFoundAway || away != bruteAway || same != bruteSame {
		t.Fatalf("tree query mismatch: away=%+v same=%+v found=%t; want away=%+v same=%+v found=%t", away, same, foundAway, bruteAway, bruteSame, bruteFoundAway)
	}
}
