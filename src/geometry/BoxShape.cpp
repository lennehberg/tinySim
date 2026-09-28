#include "tinysim/geometry/BoxShape.h"
#include "tinysim/math/Vec2.h"
#include "tinysim/math/Transform.h"

Vec2 BoxShape::support(const Vec2 &direction, const Vec2 &position, float orientation) const {
    // Box is axis-aligned in its local frame.
    // Transform the search direction into local space,
    // choose the extreme corner, then transform it back to world space.
    float halfWidth = width / 2.0f;
    float halfHeight = height / 2.0f;

    // rotate the direction vector to local space (assuming box is axis-aligned in local space)
    Vec2 localDirection = rotate(direction, -orientation);

    // Determine the support point in local space
    float supportX = (localDirection.getX() >= 0) ? halfWidth : -halfWidth;
    float supportY = (localDirection.getY() >= 0) ? halfHeight : -halfHeight;

    Vec2 localSupportPoint(supportX, supportY);

    // Convert the local support point to world space
    Vec2 worldSupportPoint = localToWorld(localSupportPoint, position, orientation);
    return worldSupportPoint;
}
