#include "tinysim/collision/EPA.h"
#include "tinysim/collision/Collision.h"
#include "tinysim/math/Vec2.h"
#include "tinysim/collision/GJK.h"
#include <cstddef>
#include <cmath>
#include <limits>
#include <assert.h>

typedef struct Edge {
    Vec2 normal;
    float distance;
    std::size_t index;
} Edge;

// void windTriangleCounterClockWise(std::vector<Vec2> &polytope);
Edge findClosestEdge(const std::vector<Vec2> &polytope);

Collision epaIntersect(const RigidBody *bodyA, const RigidBody *bodyB, std::vector<Vec2> polytope){
    int MAX_ITER = 1000;
    float tolerance = 1e-4;
    Edge e;

    // make sure polytope is wound counter clock wise before expanding
    // windTriangleCounterClockWise(polytope);

    for (int i = 0; i < MAX_ITER; ++i){
        // find the feature (edge) closest to origin
        e = findClosestEdge(polytope);
        // find new support in direction of edge normal
        Vec2 p = support(bodyA, bodyB, e.normal);
        // check distance from origin to edge along e
        float d = dot(p, e.normal);

        if (d - e.distance < tolerance){
            // if distance is smaller than EPS we have the solution
            return {true, e.normal, e.distance};
        } else {
            // expand the polytope
            polytope.insert(polytope.begin() + e.index, p);
        }
    }

    return {true, e.normal, e.distance};
}

Edge findClosestEdge(const std::vector<Vec2> &polytope) {
    Edge closest{};
    float minDistance = std::numeric_limits<float>::max();

    for (std::size_t i = 0; i < polytope.size(); ++i){
        std::size_t j = (i + 1) % polytope.size();
        // get current point and next point to form an edge
        Vec2 a = polytope[i];
        Vec2 b = polytope[j];

        // compute the edge and the vector from a to the origin
        Vec2 e = b - a;

        // outward normal for a counter-clockwise polytope
        if (d2(e) < EPS_SQUARED) {
            continue; // Skip degenerate edges
        }

        Vec2 n(e.getY(), -e.getX()); // Perpendicular to edge
        n = n * (1.0f / std::sqrt(d2(n))); // Normalize the normal

        // compute the distance from the origin to the edge along the normal
        float distance = dot(n, a);
        // check if this is the closest edge so far
        if (distance < minDistance){
            minDistance = distance;
            closest = {n, distance, j};
        }
    }
    return closest;
}
