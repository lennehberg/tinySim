#ifndef CIRCLE_SHAPE_H
#define CIRCLE_SHAPE_H

#include "tinysim/geometry/Shape.h"

class CircleShape : public Shape {
  private:
    float radius; // Radius of the circle
  public:
    CircleShape(float radius) : radius(radius) {
        setType(ShapeType::CIRCLE); // Set the shape type to CIRCLE
    }

    float calculateInertia(float mass) const override {
        // Inertia for a solid circle: I = (1/2) * m * r^2
        return 0.5f * mass * radius * radius;
    }

    float getRadius() const { return radius; }

    Vec2 support(const Vec2 &direction, const Vec2 &position, float orientation) const override;
    Feature supportFeature(const Vec2 &direction, const Vec2 &position, float orientation) const override;
};

#endif // CIRCLE_SHAPE_H
