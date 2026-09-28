#include "tinysim/math/Vec2.h"
#include <stdexcept>


Vec2::Vec2() : x(0), y(0) {}

Vec2::Vec2(float x, float y) : x(x), y(y) {}

float Vec2::getX() const {
    return x;
}

float Vec2::getY() const {
    return y;
}

Vec2 Vec2::operator-() const {
    return Vec2(-x, -y);
}

Vec2 Vec2::operator+(const Vec2& other) const {
    return Vec2(x + other.x, y + other.y);
}

Vec2 Vec2::operator-(const Vec2& other) const {
    return Vec2(x - other.x, y - other.y);
}

Vec2 Vec2::operator*(float scalar) const {
    return Vec2(x * scalar, y * scalar);
}

Vec2& Vec2::operator+=(const Vec2& other) {
    x += other.x;
    y += other.y;
    return *this;
}

float& Vec2::operator[](std::size_t index) {
    if (index > 1) throw std::out_of_range("Index out of range for Vec2");
    return index == 0 ? x : y;
}

const float& Vec2::operator[](std::size_t index) const {
    if (index > 1) throw std::out_of_range("Index out of range for Vec2");
    return index == 0 ? x : y;
}

float dot(const Vec2& a, const Vec2& b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
}

float cross(const Vec2& a, const Vec2& b) {
    return a.getX() * b.getY() - a.getY() * b.getX();
}

Vec2 cross(const Vec2& v, float s)
{
    return Vec2(
         s * v.getY(),
        -s * v.getX()
    );
}

Vec2 cross(float s, const Vec2& v)
{
    return Vec2(
        -s * v.getY(),
         s * v.getX()
    );
}

float d2(const Vec2& v) {
    return dot(v, v);
}
