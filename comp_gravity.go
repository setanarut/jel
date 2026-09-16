package jel

import "github.com/setanarut/v"

// Represents a Gravity component that can be added to a body to make it
// constantly affected by gravity
type GravityComponent struct {
	Gravity   v.Vec // The gravity vector to apply to the body
	Relaxable bool
	baseComponent
}

func NewGravityComponent(gravityX, gravityY float64, relaxable bool) *GravityComponent {
	return &GravityComponent{Gravity: v.Vec{gravityX, gravityY}, Relaxable: relaxable}
}

func (g *GravityComponent) AccumulateExternalForces(body *Body, world *World) {
	if world.IsRelaxing() && !g.Relaxable {
		return
	}
	for i, pm := range body.PointMasses {
		body.ApplyForceToPointAt(g.Gravity.Scale(pm.Mass), i)
	}
}
