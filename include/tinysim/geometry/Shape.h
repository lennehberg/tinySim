#ifndef SHAPE_H
#define SHAPE_H

#include "tinysim/math/Vec2.h"
#include "tinysim/collision/Feature.h"

enum class ShapeType {
    CIRCLE,
    RECTANGLE,
    POLYGON
};

class Shape {
    public:
        virtual ~Shape() = default; // Virtual destructor for proper cleanup of derived classes

        virtual float calculateInertia(float mass) const = 0; // Pure virtual function to calculate inertia based on mass

        ShapeType getType() const { return type; } // Getter for the shape type
        void setType(ShapeType t) { type = t; } // Setter for the shape type

        virtual Vec2 support(const Vec2 &direction, const Vec2 &position, float orientation) const = 0; // Function to get the point on the shape farthest in a given direction
        virtual Feature supportFeature(const Vec2 &direction, const Vec2 &position, float orientation) const = 0;

    private:
        ShapeType type; // Type of the shape (CIRCLE, RECTANGLE, POLYGON)
};

#endif // SHAPE_H
