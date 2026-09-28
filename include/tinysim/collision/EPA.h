#ifndef EPA_H
#define EPA_H

#include "tinysim/collision/Simplex.h"
#include "tinysim/math/Vec2.h"
#include "tinysim/collision/Collision.h"
#include "tinysim/dynamics/RigidBody.h"
#include <vector>

Collision epaIntersect(const RigidBody *bodyA, const RigidBody *bodyB, std::vector<Vec2> polytope);


#endif // EPA_H
