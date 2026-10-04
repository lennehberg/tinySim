#ifndef COLLISION_H
#define COLLISION_H

#include "tinysim/math/Vec2.h"
#include "tinysim/dynamics/RigidBody.h"
#include <vector>

typedef struct Collision{

    bool colliding = false;
    Vec2 normal{};
    float depth = 0.0f;
    std::vector<Vec2> contactPoints{}; // Up to two world-space manifold points.

}Collision;

Collision detectCollision(const RigidBody *bodyA, const RigidBody *bodyB);

#endif // COLLISION_H
