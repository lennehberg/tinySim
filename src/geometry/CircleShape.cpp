#include "tinysim/geometry/CircleShape.h"
#include "tinysim/math/Vec2.h"

Vec2 CircleShape::support(const Vec2 &direction, const Vec2 &position, float orientation) const {
    (void)orientation; // Orientation is irrelevant for circles, so we ignore it

    // For a circle, the support point in any direction is simply the center of the circle
    // plus the radius in that direction.
    if (d2(direction) < EPS_SQUARED) {
        // A direction too short to normalize uses the center of the circle.
        return position;
    }

    Vec2 normalizedDirection = direction.normalize(); // Normalize the direction
    Vec2 supportPoint = position + normalizedDirection * radius; // Move from center to edge in the given direction
    return supportPoint;
}

Feature CircleShape::supportFeature(const Vec2 &direction, const Vec2 &position, float orientation) const {
    Feature feature;
    feature.count = 1; // A circle has only one support feature in any direction
    feature.vertices[0].position = support(direction, position, orientation); // The support point
    feature.vertices[0].id = 0; // ID can be arbitrary since a circle
    return feature; // For a circle, the support feature is the same as the support point
}
