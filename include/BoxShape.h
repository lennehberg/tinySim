#ifndef BOX_SHAPE_H
#define BOX_SHAPE_H

#include "Shape.h"

class BoxShape : public Shape {
  private:
    float width;  // Width of the box
    float height; // Height of the box
  public:
    BoxShape(float width, float height) : width(width), height(height) {
        setType(ShapeType::RECTANGLE); // Set the shape type to RECTANGLE
    }
    float calculateInertia(float mass) const override {
        // Inertia for a solid rectangle: I = (1/12) * m * (w^2 + h^2)
        return (1.0f / 12.0f) * mass * (width * width + height * height);
    }

    float getWidth() const { return width; }
    float getHeight() const { return height; }

    Vec2 support(const Vec2 &direction, const Vec2 &position, float orientation) const override;
};

#endif // BOX_SHAPE_H
