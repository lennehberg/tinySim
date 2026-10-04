#include "tinysim/geometry/BoxShape.h"
#include "tinysim/collision/Feature.h"
#include "tinysim/math/Vec2.h"
#include "tinysim/math/Transform.h"

Vec2 BoxShape::findFurthestPointInDirection(const Vec2& direction) const {
    float halfWidth = width / 2.0f;
    float halfHeight = height / 2.0f;

    float supportX = (direction.getX() >= 0) ? halfWidth : -halfWidth;
    float supportY = (direction.getY() >= 0) ? halfHeight : -halfHeight;

    return Vec2(supportX, supportY);

}

Vec2 BoxShape::support(const Vec2 &direction, const Vec2 &position, float orientation) const {
    // Box is axis-aligned in its local frame.
    // Transform the search direction into local space,
    // choose the extreme corner, then transform it back to world space.

    // rotate the direction vector to local space (assuming box is axis-aligned in local space)
    Vec2 localDirection = rotate(direction, -orientation);

    // Determine the support point in local space
    Vec2 localSupportPoint(findFurthestPointInDirection(localDirection));

    // Convert the local support point to world space
    Vec2 worldSupportPoint = localToWorld(localSupportPoint, position, orientation);
    return worldSupportPoint;
}

Feature BoxShape::supportFeature(const Vec2& direction, const Vec2 &position, float orientaion) const{
    Feature feature{};

    // Box is axis-aligned in its local frame.
    // Transform the search direction into local space
    Vec2 localDirection = rotate(direction, -orientaion);

    // Determine the furthest point in local space in direction
    Vec2 furthestPoint = findFurthestPointInDirection(localDirection);
    feature.vertices[0].position = localToWorld(furthestPoint, position, orientaion);
    feature.vertices[0].id = 0; // ID can be arbitrary
    feature.count = 1; // Start with one vertex

    // check vertices adjacent to find the closest edge to the direction vector
    Vec2 leftVertex(-furthestPoint.getX(), furthestPoint.getY());
    Vec2 rightVertex(furthestPoint.getX(), -furthestPoint.getY());

    Vec2 ledge = (leftVertex - furthestPoint);
    Vec2 redge = (rightVertex - furthestPoint);

    bool hasLeft = d2(ledge) >= EPS_SQUARED;
    bool hasRight = d2(redge) >= EPS_SQUARED;

    if ((!hasLeft && !hasRight) || d2(localDirection) < EPS_SQUARED) {
        // A point-shaped box or a zero direction has no preferred edge.
        return feature;
    }

    Vec2 supportVertex;
    if (!hasLeft) {
        supportVertex = rightVertex;
    } else if (!hasRight) {
        supportVertex = leftVertex;
    } else {
        // Compare directions rather than edge lengths when both edges exist.
        float leftDot = dot(ledge.normalize(), localDirection);
        float rightDot = dot(redge.normalize(), localDirection);
        supportVertex = (leftDot > rightDot) ? leftVertex : rightVertex;
    }

    feature.vertices[1].position = localToWorld(supportVertex, position, orientaion);
    feature.vertices[1].id = 1; // ID can be arbitrary
    feature.count = 2; // Now we have two vertices forming an edge
    return feature;
}
