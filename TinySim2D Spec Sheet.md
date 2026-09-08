# TinySim2D Spec Sheet

1. ### Mission Statement:

A simple, tiny 2D physics simulator as a learning resource for more complex robotics software stack.

2. ### Scope

V0.1 \- advance simple bodies through time under forces and render their motion (linear and angular)

3. ### Observable Behavior

* Create a rigid body  
* Give it mass, position, velocity  
* Apply gravity  
* Apply external force  
* Advance the world by dt  
* Inspect resulting position/velocity  
* Draw the result

4. ### Data models

Vec2

* x  
* y

RigidBody

* Position  
* Angle  
* Velocity  
* angularVelocity  
* Mass  
* Interia  
* accumulatedForce  
* accumulatedTorque

World

* Bodies  
* Gravity  
* timestep

5. ### Operations

* update(dt)  
* applyForce(F, dt)  
* render


6. ### Program Flow

Start \-\> create world \-\> create bodies \-\> add bodies to World \-\>   
\-\> repeat:

* collect/apply forces  
* Compute acceleration  
* Integrate velocities  
* Integrate positions  
* render

Word::step(dt)

For each body:  
	Apply gravity

	Acceleration \= force / mass  
	angularAcceleration \= torque / inertia  
	  
	Velocity \= acceleration \* dt  
	angularVelocity \= angularAccelaration \* dt

	Position \= \+= velocity \* dt  
	Angle \+= angularvelocity \* dt

	Clear force  
	Clear torque

7. ### Responsibilities 

Vec2

* Basic vector math

RigidBody

* Stores physical state  
* Applies forces/torques to body

Word:

* Owns bodies  
* Advances simulation

Renderer:

* Visualizes bodies


8. ### Invariants

   Mass \>= 0  
     
   Dynamic body:  
   	Mass \> 0  
   	inverseMass \= 1 / mass  
   Static body:  
   	Mass \= 0  
     
   Forces cleared after each timestep  
   Physics use meters, kilograms, seconds

	Angels use radians  
	Simulation state independent of renderer state

### 

9. ### Interfaces (not in scope)

10. ### Acceptance tests

Test A: no forces

Body:   
Position \= (0,0)  
Velocity \= (1,0)

After 2 seconds:  
Position \= (2,0)  
Velocity \= (1,0)

Test B: gravity  
Body start at rest

Gravity \= (0, \-10)

After simulation  
Vertical velocity becomes negative

Test C: force through COM  
Linear acceleration   
No angular acceleration 

Test D: force offset from COM  
Linear acceleration   
Angular acceleration 

Implementation Flow

Vec2-\>RigidBody-\>Word-\>gravity-\>applyForce-\>basic render-\>tests

Notes:

