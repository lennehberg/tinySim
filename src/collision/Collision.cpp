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
        normal = normal.normalize();

        if (dot(normal, centerDirection) < 0.0f) {
            normal = -normal;
        }

        return normal;
    }

    if (d2(centerDirection) >= EPS_SQUARED) {
        return centerDirection.normalize();
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
        perp = perp.normalize();

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

std::vector<Vec2> clip(const Vec2 &v1, const Vec2 &v2, Vec2 normal, double o) {
    std::vector<Vec2> clippedPoints;

    double d1 = dot(normal, v1) - o;
    double d2 = dot(normal, v2) - o;

    // if either points is past o along n,
    // we can keep the point
    if (d1 >= 0.0f) {
        clippedPoints.push_back(v1);
    }
    if (d2 >= 0.0f) {
        clippedPoints.push_back(v2);
    }

    // check if on opposite sides to compute the correct point
    if (d1 * d2 < 0.0f) {
        Vec2 e = v2 - v1;
        // compute the location along e
        double u = d1 / (d1 - d2);
        Vec2 intersection = v1 + e * u;
        clippedPoints.push_back(intersection);
    }
    return clippedPoints;
}

std::vector<Vec2> clipEdges(const Feature &ref, const Feature &inc, bool flip) {
    std::vector<Vec2> clippedPoints;

    // get the reference edge from the feature
    Vec2 refv = ref.vertices[1].position - ref.vertices[0].position;
    refv.normalize();

    double o1 = dot(refv, ref.vertices[0].position);
    // clip the incident edge against the reference edge
    clippedPoints = clip(inc.vertices[0].position, inc.vertices[1].position, refv, o1);

    // if we have less than 2 points, fail the clipping
    if (clippedPoints.size() < 2) {
        return {};
    }

    // clip against the other side of the reference edge
    Vec2 refNorm = Vec2(-refv.getY(), refv.getX());
    // if we flipped the edges, negate the reference normal
    if (flip) {
        refNorm = -refNorm;
    }

    // get the largest depth
    double max = dot(refNorm, ref.vertices[0].position);
    // make sure the final points are not past the max
    if (dot(refNorm, clippedPoints[0]) - max < 0.0f) {
        clippedPoints.erase(clippedPoints.begin());
    }
    if (dot(refNorm, clippedPoints[1]) - max < 0.0f) {
        clippedPoints.erase(clippedPoints.begin() + 1);
    }
    return clippedPoints;
}

std::vector<Vec2> getCollisionPoints(const RigidBody *A, const RigidBody *B, const Vec2 &normal) {
    Feature featureA = A->getShape()->supportFeature(normal, A->getPosition(), A->getOrientation());
    Feature featureB = B->getShape()->supportFeature(-normal, B->getPosition(), B->getOrientation());
    bool flip = false;
    Feature ref, inc;

    // find the reference edge and incident edge (if both features are edges)
    if (featureA.isEdge() && featureB.isEdge()) {
        // the reference edge is the most perpendicular to the collision normal
        if (std::fabs(dot(normal, (featureA.vertices[1].position - featureA.vertices[0].position).normalize())) <
            std::fabs(dot(normal, (featureB.vertices[1].position - featureB.vertices[0].position).normalize()))) {
            ref = featureA;
            inc = featureB;
        } else {
            flip = true;
            ref = featureB;
            inc = featureA;
        }
    }
    return clipEdges(ref, inc, flip);
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
            normal = normal.normalize();
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
