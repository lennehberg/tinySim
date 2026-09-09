#ifndef GJK_H
#define GJK_H

#include "Collision.h"
#include "RigidBody.h"

Collision gjkIntersect(const RigidBody *bodyA, const RigidBody *bodyB);

#endif // GJK_H
