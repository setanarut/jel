package jel

// Specifies a rest distance for a body joint or spring.
// Distances can either be fixed by a distance, or ranged so forces only apply
// when distance is outside a tolerance range.
type RestDistance struct {
	// Distance is ranged between a minimum and maximum value
	IsRanged bool
	// Fixed distance (used when IsRanged is false)
	Fixed float64
	// Minimum distance for this rest distance (used when IsRanged is true)
	min float64
	// Maximum distance for this rest distance (used when IsRanged is true)
	max float64
}

func NewFixedRestDistance(value float64) RestDistance {
	return RestDistance{Fixed: value}
}
func NewRangedRestDistance(min, max float64) RestDistance {
	return RestDistance{IsRanged: true, min: min, max: max}
}

// MinDist returns the minimum distance for this rest distance.
// If [RestDistance.IsRanged] is false (fixed mode), returns the fixed distance.
// If [RestDistance.IsRanged] is true (ranged mode), returns the minimum distance.
func (r *RestDistance) MinDist() float64 {
	if r.IsRanged {
		return r.min
	}
	return r.Fixed
}

// SetMinDist sets the minimum distance for this rest distance.
// If [RestDistance.IsRanged] is false (fixed mode), updates the [RestDistance.Fixed] value.
// If [RestDistance.IsRanged] is true (ranged mode), updates the min value.
func (r *RestDistance) SetMinDist(value float64) {
	if r.IsRanged {
		r.min = value
	} else {
		r.Fixed = value
	}
}

// MaxDist returns the maximum distance for this rest distance.
// If [RestDistance.IsRanged] is false (fixed mode), returns the fixed distance.
// If [RestDistance.IsRanged] is true (ranged mode), returns the maximum distance.
func (r *RestDistance) MaxDist() float64 {
	if r.IsRanged {
		return r.max
	}
	return r.Fixed
}

// SetMaximumDistance sets the maximum distance for this rest distance.
// If [RestDistance.IsRanged] is false (fixed mode), updates the [RestDistance.Fixed] value.
// If [RestDistance.IsRanged] is true (ranged mode), updates the max value.
func (r *RestDistance) SetMaxDist(value float64) {
	if r.IsRanged {
		r.max = value
	} else {
		r.Fixed = value
	}
}

// InRange returns whether a given value is within the range of this rest
// distance. If [RestDistance.IsRanged] is false (fixed mode), checks for exact equality.
// If [RestDistance.IsRanged] is true (ranged mode), performs value >= min && value <= max.
func (r RestDistance) InRange(value float64) bool {
	if r.IsRanged {
		return value >= r.min && value <= r.max
	}
	return value == r.Fixed
}

// Clamp clamps a given value to be within the range of this rest distance.
// If [RestDistance.IsRanged] is false (fixed mode), the value parameter is ignored and
// [RestDistance.Fixed] is returned.
// If [RestDistance.IsRanged] is true (ranged mode), the value is clamped between min and max.
func (r RestDistance) Clamp(value float64) float64 {
	if r.IsRanged {
		return max(r.min, min(r.max, value))
	}
	return r.Fixed
}

// Squared returns a new rest distance structure which represents the square
// of this rest distance's parameters.
// If [RestDistance.IsRanged] is false (fixed mode), returns a new fixed rest distance with squared [RestDistance.Fixed] value.
// If [RestDistance.IsRanged] is true (ranged mode), returns a new ranged rest distance with squared min and max.
func (r RestDistance) Squared() RestDistance {
	if r.IsRanged {
		return NewRangedRestDistance(r.min*r.min, r.max*r.max)
	}
	return NewFixedRestDistance(r.Fixed * r.Fixed)
}
