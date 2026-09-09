#include "CircleShape.h"
#include "Vec2.h"
#include <cmath>

Vec2 CircleShape::support(const Vec2 &direction, const Vec2 &position, float orientation) const {
    // For a circle, the support point in any direction is simply the center of the circle
    // plus the radius in that direction.
    if (d2(direction) == 0) {
        // If the direction is zero, return the center of the circle
        return position;
    }

    Vec2 normalizedDirection = direction * (1.0f / std::sqrt(d2(direction))); // Normalize the direction
    Vec2 supportPoint = position + normalizedDirection * radius; // Move from center to edge in the given direction
    return supportPoint;
}
