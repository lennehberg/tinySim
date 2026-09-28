#include "tinysim/collision/Collision.h"
#include "tinysim/dynamics/RigidBody.h"
#include "tinysim/collision/GJK.h"
#include "tinysim/math/Vec2.h"
#include "tinysim/collision/Simplex.h"
#include <cmath>


Vec2 support(const RigidBody *bodyA, const RigidBody *bodyB, const Vec2 &direction) {
    // Get the support point in the given direction for body A
    Vec2 supportA = bodyA->getShape()->support(direction, bodyA->getPosition(), bodyA->getOrientation());

    // Get the support point in the opposite direction for body B
    Vec2 supportB = bodyB->getShape()->support(-direction, bodyB->getPosition(), bodyB->getOrientation());

    // Return the Minkowski difference point
    return supportA - supportB;
}

bool handleLine(Simplex &simplex, Vec2 &direction){
    // constexpr float EPS = 1e-6f;
    Vec2 origin = Vec2();

    Vec2 A = simplex.getA();
    Vec2 B = simplex.getB();

    Vec2 AB = B - A;
    Vec2 AO = origin - A;

    float ab2 = d2(AB);

    // If the line segment is degenerate (A and B are the same point), we treat it as a single point
    if (ab2 < EPS_SQUARED) {
        simplex.set(A);
        direction = AO;
        return d2(AO) < EPS_SQUARED;
    }

    // check if the origin is on the line segment AB
    if (std::fabs(cross(AB, AO)) < EPS) {
        float t = dot(AO, AB) / ab2;

        if (t >= 0.0f && t <= 1.0f) {
            // Origin lies on AB.
            return true;
        }

        if (t < 0.0f) {
            // Origin lies beyond A.
            simplex.set(A);
            direction = AO;
        } else {
            // Origin lies beyond B.
            simplex.set(B);
            direction = origin - B;
        }

        return false;
    }

    // choose which direction to go next
    if (dot(AB, AO) > 0.0f) {
        direction = cross(cross(AB, AO), AB);
    } else {
        simplex.set(A);
        direction = AO;
    }

    return false;
}

bool handleTriangle(Simplex &simplex, Vec2 &direction){
    // constexpr float EPS = 1e-6f;
    Vec2 origin = Vec2();
    Vec2 A = simplex.getA();
    Vec2 B = simplex.getB();
    Vec2 C = simplex.getC();

    // compute edges
    Vec2 AB = B - A;
    Vec2 AC = C - A;
    Vec2 AO = origin - A;

    // check if the triangle is degenerate (points are collinear)
    if (std::fabs(cross(AB, AC)) < EPS) {
        float ab2 = d2(B - A);
        float ac2 = d2(C - A);

        if (ab2 >= ac2) {
            simplex.set(A, B);
        } else {
            simplex.set(A, C);
        }

        return handleLine(simplex, direction);
    }

    // compute normals
    Vec2 ABperp = cross(cross(AC, AB), AB);
    Vec2 ACperp = cross(cross(AB, AC), AC);

    // check if the origin is in the region of AB
    if (dot(ABperp, AO) > 0.0f) {
        simplex.set(A, B);
        direction = ABperp;
        return false;
    } else if (dot(ACperp, AO) > 0.0f) {
        simplex.set(A, C);
        direction = ACperp;
        return false;
    } else {
        return true; // MUTANT
    }
}

bool containsOrigin(Simplex &simplex, Vec2& direction) {
    // constexpr float EPS = 1e-6f;

    if (simplex.size() == 0){
        return false;
    }
    if (simplex.size() == 1){
        return d2(simplex.getA()) < EPS_SQUARED; // Check if the single point is at the origin
    }
    if (simplex.size() == 2){
        return handleLine(simplex, direction);
    }
    if (simplex.size() == 3){
        return handleTriangle(simplex, direction);
    }
    return false;
}

bool gjkIntersect(const RigidBody *bodyA,
                        const RigidBody *bodyB,
                        Simplex &simplex){
    // constexpr float EPS = 1e-6f;
    constexpr int MAX_ITERATIONS = 100; // Prevent infinite loops

    Vec2 direction = bodyB->getPosition() - bodyA->getPosition();

    if (d2(direction) < EPS_SQUARED) {
        direction = Vec2(1, 0); // Arbitrary direction if bodies are at the same position
    }

    // add the support to the simplex
    Vec2 A = support(bodyA, bodyB, direction);
    simplex.pushFront(A);
    direction = -A; // set the direction opposite to the first support point

    for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
        if (d2(direction) < EPS_SQUARED) {
            return true; // The origin is at the support point, indicating a collision
        }
        A = support(bodyA, bodyB, direction);
        // make sure we passed the origin
        if (dot(A, direction) < 0.0f){
            return false; // TODO add more relevant information when collision is flashed out
        }

        simplex.pushFront(A);

        if (containsOrigin(simplex, direction)){
            // check if the simplex contains the origin
            return true; // TODO add more relevant information when collision is flashed out
        }
    }
    return false; // If we reach here, we didn't find a collision within the iteration limit
}
