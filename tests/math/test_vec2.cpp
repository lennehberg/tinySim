#include <stdexcept>

#include "tinysim/math/Vec2.h"
#include "support/test_utils.h"

// --- construction -----------------------------------------------------------

TEST(Vec2, DefaultConstructorIsZero) {
    Vec2 v;
    CHECK_VEC_NEAR(v, 0.0f, 0.0f);
}

TEST(Vec2, ComponentConstructorStoresComponents) {
    Vec2 v(3.0f, -4.0f);
    CHECK_FLOAT_EQ(v.getX(), 3.0f);
    CHECK_FLOAT_EQ(v.getY(), -4.0f);
}

// --- arithmetic -------------------------------------------------------------

TEST(Vec2, Addition) {
    CHECK_VEC_NEAR(Vec2(1.0f, 2.0f) + Vec2(3.0f, 4.0f), 4.0f, 6.0f);
}

TEST(Vec2, AdditionDoesNotMutateOperands) {
    Vec2 a(1.0f, 2.0f);
    Vec2 b(3.0f, 4.0f);
    (void)(a + b);
    CHECK_VEC_NEAR(a, 1.0f, 2.0f);
    CHECK_VEC_NEAR(b, 3.0f, 4.0f);
}

TEST(Vec2, Subtraction) {
    CHECK_VEC_NEAR(Vec2(1.0f, 2.0f) - Vec2(3.0f, 4.0f), -2.0f, -2.0f);
}

TEST(Vec2, SubtractionOfSelfIsZero) {
    Vec2 a(7.5f, -1.25f);
    CHECK_VEC_NEAR(a - a, 0.0f, 0.0f);
}

TEST(Vec2, ScalarMultiplication) {
    CHECK_VEC_NEAR(Vec2(1.5f, -2.0f) * 2.0f, 3.0f, -4.0f);
}

TEST(Vec2, ScalarMultiplicationByZero) {
    CHECK_VEC_NEAR(Vec2(3.0f, 4.0f) * 0.0f, 0.0f, 0.0f);
}

TEST(Vec2, ScalarMultiplicationByNegativeFlipsDirection) {
    CHECK_VEC_NEAR(Vec2(3.0f, -4.0f) * -1.0f, -3.0f, 4.0f);
}

TEST(Vec2, PlusEqualsMutatesInPlace) {
    Vec2 a(1.0f, 1.0f);
    a += Vec2(2.0f, 3.0f);
    CHECK_VEC_NEAR(a, 3.0f, 4.0f);
}

TEST(Vec2, PlusEqualsReturnsSelfReference) {
    Vec2 a(1.0f, 1.0f);
    Vec2 &result = (a += Vec2(2.0f, 3.0f));
    CHECK(&result == &a);
}

TEST(Vec2, PlusEqualsAccumulates) {
    Vec2 a;
    for (int i = 0; i < 4; ++i) {
        a += Vec2(1.0f, -0.5f);
    }
    CHECK_VEC_NEAR(a, 4.0f, -2.0f);
}

// --- indexing ---------------------------------------------------------------

TEST(Vec2, IndexReadsComponents) {
    Vec2 v(3.0f, -4.0f);
    CHECK_FLOAT_EQ(v[0], 3.0f);
    CHECK_FLOAT_EQ(v[1], -4.0f);
}

TEST(Vec2, IndexReturnsWritableReference) {
    Vec2 v(3.0f, -4.0f);
    v[0] = 10.0f;
    v[1] = 20.0f;
    CHECK_VEC_NEAR(v, 10.0f, 20.0f);
}

TEST(Vec2, ConstIndexReadsComponents) {
    const Vec2 v(3.0f, -4.0f);
    CHECK_FLOAT_EQ(v[0], 3.0f);
    CHECK_FLOAT_EQ(v[1], -4.0f);
}

TEST(Vec2, IndexOutOfRangeThrows) {
    Vec2 v(1.0f, 2.0f);
    CHECK_THROWS(v[2], std::out_of_range);
}

TEST(Vec2, ConstIndexOutOfRangeThrows) {
    const Vec2 v(1.0f, 2.0f);
    CHECK_THROWS(v[2], std::out_of_range);
}

TEST(Vec2, NegativeIndexWrapsAndThrows) {
    // The parameter is std::size_t, so a negative literal wraps to a huge
    // value and must still be rejected rather than reading out of bounds.
    Vec2 v(1.0f, 2.0f);
    CHECK_THROWS(v[static_cast<std::size_t>(-1)], std::out_of_range);
}

// --- free functions ---------------------------------------------------------

TEST(Vec2, DotProduct) {
    CHECK_FLOAT_EQ(dot(Vec2(1.0f, 2.0f), Vec2(3.0f, 4.0f)), 11.0f);
}

TEST(Vec2, DotProductOfPerpendicularVectorsIsZero) {
    CHECK_FLOAT_EQ(dot(Vec2(1.0f, 0.0f), Vec2(0.0f, 1.0f)), 0.0f);
}

TEST(Vec2, DotProductIsCommutative) {
    Vec2 a(1.5f, -2.0f);
    Vec2 b(-3.0f, 0.25f);
    CHECK_FLOAT_EQ(dot(a, b), dot(b, a));
}

TEST(Vec2, CrossProductOfBasisVectors) {
    CHECK_FLOAT_EQ(cross(Vec2(1.0f, 0.0f), Vec2(0.0f, 1.0f)), 1.0f);
}

TEST(Vec2, CrossProductIsAntiCommutative) {
    Vec2 a(1.5f, -2.0f);
    Vec2 b(-3.0f, 0.25f);
    CHECK_FLOAT_EQ(cross(a, b), -cross(b, a));
}

TEST(Vec2, CrossProductOfParallelVectorsIsZero) {
    Vec2 a(2.0f, 3.0f);
    CHECK_FLOAT_EQ(cross(a, a * 4.0f), 0.0f);
}

TEST(Vec2, SquaredMagnitude) {
    CHECK_FLOAT_EQ(d2(Vec2(3.0f, 4.0f)), 25.0f);
}

TEST(Vec2, SquaredMagnitudeOfZeroIsZero) {
    CHECK_FLOAT_EQ(d2(Vec2()), 0.0f);
}

TEST(Vec2, SquaredMagnitudeIsNeverNegative) {
    CHECK(d2(Vec2(-3.0f, -4.0f)) >= 0.0f);
    CHECK_FLOAT_EQ(d2(Vec2(-3.0f, -4.0f)), 25.0f);
}
