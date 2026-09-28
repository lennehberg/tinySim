#include "tinysim/collision/Simplex.h"

void Simplex::pushFront(const Vec2& point) {
    if (currentSize < 3) {
        // Shift existing points to the right
        for (std::size_t i = currentSize; i > 0; --i) {
            points[i] = points[i - 1];
        }
        points[0] = point;
        ++currentSize;
    } else {
        // If the simplex is full, replace the last point with the new one
        points[2] = points[1];
        points[1] = points[0];
        points[0] = point;
    }
}

const Vec2& Simplex::operator[](std::size_t index) const {
    if (index >= currentSize) {
        throw std::out_of_range("Index out of range for Simplex");
    }
    return points[index];
}

std::size_t Simplex::size() const {
    return currentSize;
}

const Vec2& Simplex::getA() const {
    if (currentSize < 1) {
        throw std::out_of_range("Simplex does not have point A");
    }
    return points[0];
}

const Vec2& Simplex::getB() const {
    if (currentSize < 2) {
        throw std::out_of_range("Simplex does not have point B");
    }
    return points[1];
}

const Vec2& Simplex::getC() const {
    if (currentSize < 3) {
        throw std::out_of_range("Simplex does not have point C");
    }
    return points[2];
}

void Simplex::set(const Vec2 &a) {
    points[0] = a;
     // Reset other points
    points[1] = Vec2();
    points[2] = Vec2();
    currentSize = 1;
}

void Simplex::set(const Vec2& a, const Vec2 &b) {
    points[0] = a;
    points[1] = b;
    points[2] = Vec2(); // Reset the third point
    currentSize = 2;
}

void Simplex::set(const Vec2& a, const Vec2 &b, const Vec2 &c) {
    points[0] = a;
    points[1] = b;
    points[2] = c;
    currentSize = 3;
}
