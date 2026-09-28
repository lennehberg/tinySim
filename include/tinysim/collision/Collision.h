#ifndef COLLISION_H
#define COLLISION_H

#include "tinysim/math/Vec2.h"
#include "tinysim/dynamics/RigidBody.h"

typedef struct Collision{

    bool colliding = false;
    Vec2 normal{};
    float depth = 0.0f;

}Collision;

Collision detectCollision(const RigidBody *bodyA, const RigidBody *bodyB);

#endif // COLLISION_H
