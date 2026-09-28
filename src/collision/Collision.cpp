#include "tinysim/collision/GJK.h"
#include "tinysim/collision/EPA.h"
#include "tinysim/dynamics/RigidBody.h"
#include "tinysim/collision/Simplex.h"
#include "tinysim/math/Vec2.h"
#include <cmath>
#include <optional>
#include <vector>

Vec2 fallbackContactNormal(const RigidBody *bodyA,
                           const RigidBody *bodyB,
                           const Simplex &simplex) {
    Vec2 centerDirection = bodyB->getPosition() - bodyA->getPosition();
    Vec2 longestEdge{};

    for (std::size_t i = 0; i < simplex.size(); ++i) {
        for (std::size_t j = i + 1; j < simplex.size(); ++j) {
            Vec2 edge = simplex[j] - simplex[i];
            if (d2(edge) > d2(longestEdge)) {
                longestEdge = edge;
            }
        }
    }

    if (d2(longestEdge) >= EPS_SQUARED) {
        Vec2 normal(longestEdge.getY(), -longestEdge.getX());
        normal = normal * (1.0f / std::sqrt(d2(normal)));

        if (dot(normal, centerDirection) < 0.0f) {
            normal = -normal;
        }

        return normal;
    }

    if (d2(centerDirection) >= EPS_SQUARED) {
        return centerDirection * (1.0f / std::sqrt(d2(centerDirection)));
    }

    return Vec2(1.0f, 0.0f);
}


std::optional<std::vector<Vec2>> windTriangleCounterClockWise(Vec2 a, Vec2 b, Vec2 c) {
    // if the cross product is negative, swap b and c.
    float signedArea = cross(b - a, c - b);
    if (std::fabs(signedArea) < EPS){
        return std::nullopt;
    }

    if (signedArea < 0.0f)
    {
        std::swap(b, c);
    }

    return std::vector<Vec2>{a, b, c};
}

std::optional<std::vector<Vec2>> buildInitialPolytope(const RigidBody *bodyA, const RigidBody *bodyB, Simplex &simplex) {
    if (simplex.size() == 3){ // return a counterclockwise triangle
        return windTriangleCounterClockWise(simplex[0], simplex[1], simplex[2]);
    }

    if (simplex.size() == 2){ // find a new vertex to complete the triangle
        Vec2 a = simplex[0];
        Vec2 b = simplex[1];
        Vec2 e = b - a;

        float eLengthSquare = d2(e);

        if (eLengthSquare < EPS_SQUARED){
            return std::nullopt;
        }

        // compute perpendicular to edge
        Vec2 perp(e.getY(), -e.getX());
        perp = perp * (1.0f / std::sqrt(eLengthSquare));

        // try both sides
        Vec2 positiveSupport = support(bodyA, bodyB, perp);
        Vec2 negativeSupport = support(bodyA, bodyB, -perp);

        float positiveArea = std::fabs(cross(e, positiveSupport - a));
        float negativeArea = std::fabs(cross(e, negativeSupport - a));

        Vec2 c = positiveArea >= negativeArea ? positiveSupport : negativeSupport;

        return windTriangleCounterClockWise(a, b, c);
    }
    return std::nullopt;
}

Collision detectCollision(const RigidBody *bodyA, const RigidBody *bodyB){
    Simplex simplex{};

    if (!gjkIntersect(bodyA, bodyB, simplex)){
        return {false, {}, 0};
    }

    if (simplex.size() == 1){
        // return a normal perpendicular to the shapes
        Vec2 normal = bodyB->getPosition() - bodyA->getPosition();

        if (d2(normal) < EPS_SQUARED){
            normal = Vec2(1.0f, 0.0f);
        } else {
            normal = normal * (1.0f / std::sqrt(d2(normal)));
        }

        return {true, normal, 0};
    }

    auto polytope = buildInitialPolytope(bodyA, bodyB, simplex);

    // degenerate touching case - requires seperate policy
    if (!polytope) {
        return {true, fallbackContactNormal(bodyA, bodyB, simplex), 0.0f};
    }

    return epaIntersect(bodyA, bodyB, std::move(*polytope));
}
