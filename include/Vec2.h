#ifndef VEC_H
#define VEC_H
#include <cstddef>  // for std::size_t

/**
 * @brief A simple 2D vector class.
 */
class Vec2 {
 private:
    float x;
    float y;
public:
    Vec2();
    Vec2(float x, float y);
    float getX() const;
    float getY() const;

    Vec2 operator-() const; // Unary minus operator to negate the vector

    Vec2 operator+(const Vec2& other) const;
    Vec2 operator-(const Vec2& other) const;
    Vec2 operator*(float scalar) const;

    Vec2 &operator+=(const Vec2& other);

    bool operator==(const Vec2& other) const {
        return x == other.x && y == other.y;
    }

    float& operator[](std::size_t index);
    const float& operator[](std::size_t index) const;
};

// dot product of two vectors
float dot(const Vec2& a, const Vec2& b);
// cross product of two vectors (in 2D, this returns a scalar)
float cross(const Vec2& a, const Vec2& b);
Vec2 cross(const Vec2& v, float s);
Vec2 cross(float s, const Vec2& v);
// squared magnitude of a vector
float d2(const Vec2& v);

#endif // VEC_H
