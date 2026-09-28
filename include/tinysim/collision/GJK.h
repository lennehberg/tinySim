#ifndef GJK_H
#define GJK_H

#include "tinysim/collision/Collision.h"
#include "tinysim/dynamics/RigidBody.h"
#include "tinysim/collision/Simplex.h"

Vec2 support(const RigidBody *bodyA, const RigidBody *bodyB, const Vec2 &direction);

bool gjkIntersect(const RigidBody *bodyA, const RigidBody *bodyB, Simplex &simplex);

#endif // GJK_H
