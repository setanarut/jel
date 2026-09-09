package jel

// AABB is an axis-aligned bounding box. Valid is false for an empty/unset box.
type AABB struct {
	Valid bool
	Min   Vec2
	Max   Vec2
}

// NewAABBOf creates an AABB enclosing the given points.
func NewAABBOf(points ...Vec2) AABB {
	return NewAABBFromPoints(points)
}

// NewAABB creates a valid AABB with the given min and max corners.
func NewAABB(min, max Vec2) AABB {
	return AABB{
		Valid: true,
		Min:   min,
		Max:   max,
	}
}

// NewAABBFromPoints creates an AABB enclosing all the given points.
func NewAABBFromPoints(points []Vec2) (aabb AABB) {
	aabb.ExpandToIncludePoints(points)
	return
}

// midX returns the horizontal center of the box.
func (a AABB) midX() float64 {
	return (a.Min.X + a.Max.X) / 2
}

// Clear marks the box as invalid/empty.
func (a *AABB) Clear() {
	a.Valid = false
}

// Expanded returns a copy of the AABB grown outward by margin on every side.
func (a AABB) Expanded(margin float64) AABB {
	m := Vec2{X: margin, Y: margin}
	return AABB{
		Valid: a.Valid,
		Min:   a.Min.Sub(m),
		Max:   a.Max.Add(m),
	}
}

// ExpandToInclude grows the box, if needed, so it contains point.
func (a *AABB) ExpandToInclude(point Vec2) {
	if !a.Valid {
		a.Min = point
		a.Max = point
		a.Valid = true
	} else {
		a.Min = a.Min.Min(point)
		a.Max = a.Max.Max(point)
	}
}

// ExpandToIncludePoints grows the box, if needed, so it contains all points.
func (a *AABB) ExpandToIncludePoints(points []Vec2) {
	for _, p := range points {
		a.ExpandToInclude(p)
	}
}

// Contains reports whether point lies inside the box (inclusive).
func (a AABB) Contains(point Vec2) bool {
	if !a.Valid {
		return false
	}
	return point.X >= a.Min.X && point.Y >= a.Min.Y && point.X <= a.Max.X && point.Y <= a.Max.Y
}

// Intersects reports whether the two boxes overlap.
func (a AABB) Intersects(box AABB) bool {
	if !a.Valid || !box.Valid {
		return false
	}
	return a.Min.X <= box.Max.X && a.Min.Y <= box.Max.Y && a.Max.X >= box.Min.X && a.Max.Y >= box.Min.Y
}
