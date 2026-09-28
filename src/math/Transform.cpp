#include "tinysim/math/Transform.h"

Vec2 rotate(const Vec2& v, float angle) {
    float cosTheta = std::cos(angle);
    float sinTheta = std::sin(angle);
    return Vec2(
        v.getX() * cosTheta - v.getY() * sinTheta,
        v.getX() * sinTheta + v.getY() * cosTheta
    );
}

Vec2 worldToLocal(const Vec2& worldPoint, const Vec2& bodyPosition, float bodyOrientation) {
    Vec2 localPoint = worldPoint - bodyPosition;
    return rotate(localPoint, -bodyOrientation);
}

Vec2 localToWorld(const Vec2& localPoint, const Vec2& bodyPosition, float bodyOrientation) {
    Vec2 worldPoint = rotate(localPoint, bodyOrientation);
    return worldPoint + bodyPosition;
}
