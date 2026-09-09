#ifndef SIMPLEX_H
#define SIMPLEX_H

#include "Vec2.h"
#include <array>

class Simplex {

    public:
        void pushFront(const Vec2& point);

        const Vec2& operator[](std::size_t index) const;

        std::size_t size() const;

        const Vec2& getLast() const { return points[currentSize - 1]; }
        const Vec2& getA() const;
        const Vec2& getB() const;
        const Vec2& getC() const;

        void set(const Vec2 &a);
        void set(const Vec2& a, const Vec2 &b);
        void set(const Vec2& a, const Vec2 &b, const Vec2 &c);

    private:
        std::array<Vec2, 3> points{};
        std::size_t currentSize = 0;

};

#endif // SIMPLEX_H
