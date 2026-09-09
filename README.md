# jel

<video src="https://github.com/user-attachments/assets/00f1a1d6-09ac-4e36-b70f-8dd4a48fac07" width="100%" controls></video>

2D soft-body physics library written in Go.

## Overview

**jel** simulates deformable bodies made of connected point masses. Physics behavior comes from attachable **Components** (an extensible interface), bodies connect via **Joints**, and collision resolution is per-material-pair. Use it for ragdolls, fabric, fluid bodies, or any soft-body effect.

## Components

Components are interfaces you attach to bodies to add physics behavior. The library includes:

- **SpringComponent** - Edge springs connecting adjacent point masses; supports custom extra springs
- **ShapeMatchComponent** - Constrains body to original shape, creating elasticity and rigidity
- **GravityComponent** - Applies directional force (gravity, wind, etc.)
- **PressureComponent** - Internal pressure pushes outward on body edges
- **StickyRayComponent** - Casts rays outward and sticks point masses to contacted surfaces via spring forces

Implement the `Component` interface to write your own.

## Joints & JointLinks

**Joints** rigidly or elastically connect bodies. They work with **JointLinks** - abstractions that represent connection points:

- **BodyJointLink** - Connects to the entire body's center (DerivedPos)
- **PointJointLink** - Connects to a specific point mass within a body
- **EdgeJointLink** - Connects to a point along an edge between two point masses, with interpolation ratio
- **ShapeJointLink** - Connects to a group of point masses (weighted average position)

Joint types:

- **SpringJoint** - Maintains rest distance with spring forces; supports plasticity
- **PinJoint** - Rigidly welds two points together like a weld

Example - connect tire to car body:

```go
tireLink := jel.NewBodyJointLink(tire)
carLink := jel.NewShapeJointLink(carBody, []int{1, 3, 4, 5})
joint := jel.NewPinJoint(carLink, tireLink, 0)
world.AddJoint(joint)
```

## Raycast & Collision

**Raycast** - Cast rays through the world to test line-of-sight or detect surfaces:

```go
func (b *Body) Raycast(start, end Vec2) (closestHit Vec2, ok bool)
```

**Collision** - Automatic collision detection between bodies with configurable:

- **Material pairs** - Set friction, elasticity, and enable/disable collisions per material combination
- **Collision observers** - Listen to collision events and react with custom logic (sounds, damage, etc.)
- **Bitmask filtering** - Fine-grained collision groups using bitmask operations

## Example

```go
world := jel.NewWorld()

// Create a soft body
shape := jel.RegularPolygon(radius, 12)
body := jel.NewBody(shape, position, angle, mass, world)

// Add physics components
body.AddComponent(jel.NewSpringComponent(stiffness, damping))
body.AddComponent(jel.NewShapeMatchComponent(stiffness, damping, nil))
body.AddComponent(jel.NewGravityComponent(0, 9.8, true))

// Define materials
mat1 := world.AddMaterial()
mat2 := world.AddMaterial()
world.SetMaterialPairData(mat1, mat2, friction, elasticity)
body.Material = mat1

// Listen to collisions
world.CollisionObserver = myObserver

// Update world
world.Update(deltaTime)
```
