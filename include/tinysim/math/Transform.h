#ifndef TRANSFORM_H
#define TRANSFORM_H

#include "tinysim/math/Vec2.h"
#include <cmath>

Vec2 rotate(const Vec2& v, float angle);

Vec2 worldToLocal(const Vec2& worldPoint, const Vec2& bodyPosition, float bodyOrientation);

Vec2 localToWorld(const Vec2& localPoint, const Vec2& bodyPosition, float bodyOrientation);

#endif // TRANSFORM_H
