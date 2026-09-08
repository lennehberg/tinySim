#ifndef SHAPE_H
#define SHAPE_H

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

    private:
        ShapeType type; // Type of the shape (CIRCLE, RECTANGLE, POLYGON)
};

#endif // SHAPE_H
