package jel

type AABB struct {
	Min      Vec2
	Max      Vec2
	Validity bool
}

func NewAABB(minPt, maxPt Vec2) *AABB {
	return &AABB{Min: minPt, Max: maxPt, Validity: true}
}

func (a *AABB) Clear() {
	*a = AABB{}
}

func (a *AABB) ExpandToIncludePos(pt Vec2) {
	if !a.Validity {
		a.Min = pt
		a.Max = pt
		a.Validity = true
		return
	}
	a.Min.X = min(pt.X, a.Min.X)
	a.Max.X = max(pt.X, a.Max.X)
	a.Min.Y = min(pt.Y, a.Min.Y)
	a.Max.Y = max(pt.Y, a.Max.Y)
}

func (a *AABB) Contains(pt Vec2) bool {
	if !a.Validity {
		return false
	}
	return pt.X >= a.Min.X && pt.X <= a.Max.X && pt.Y >= a.Min.Y && pt.Y <= a.Max.Y
}

func (a *AABB) Intersects(box *AABB) bool {
	if !a.Validity || !box.Validity {
		return false
	}
	return a.Min.X <= box.Max.X && a.Max.X >= box.Min.X &&
		a.Min.Y <= box.Max.Y && a.Max.Y >= box.Min.Y
}
