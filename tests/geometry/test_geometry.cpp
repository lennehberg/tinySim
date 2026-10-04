// Tests for the pieces GJK is built on: the rigid transforms, the shape
// support functions, and the simplex container. A bug in any of these shows up
// as a wrong collision verdict, so they are worth pinning separately.

#include <cmath>
#include <stdexcept>

#include "tinysim/geometry/BoxShape.h"
#include "tinysim/geometry/CircleShape.h"
#include "tinysim/collision/Simplex.h"
#include "tinysim/math/Transform.h"
#include "tinysim/math/Vec2.h"
#include "support/test_utils.h"

namespace {
constexpr float kPi = 3.14159265f;

bool near(const Vec2 &a, const Vec2 &b, float eps = 1e-3f) {
    return std::fabs(a.getX() - b.getX()) <= eps &&
           std::fabs(a.getY() - b.getY()) <= eps;
}

// An edge feature is the segment between its two vertices; which end comes
// first is an implementation detail, so compare in either order.
bool isEdgeBetween(const Feature &feature, const Vec2 &p, const Vec2 &q) {
    if (feature.count != 2) {
        return false;
    }
    const Vec2 &a = feature.vertices[0].position;
    const Vec2 &b = feature.vertices[1].position;
    return (near(a, p) && near(b, q)) || (near(a, q) && near(b, p));
}
}  // namespace

// --- rotate -----------------------------------------------------------------

TEST(Transform, RotateByZeroIsIdentity) {
    CHECK_VEC_NEAR(rotate(Vec2(3.0f, -4.0f), 0.0f), 3.0f, -4.0f);
}

// Positive angles rotate counter-clockwise: +x maps onto +y.
TEST(Transform, RotateQuarterTurnIsCounterClockwise) {
    CHECK_VEC_NEAR(rotate(Vec2(1.0f, 0.0f), kPi / 2.0f), 0.0f, 1.0f);
}

TEST(Transform, RotateHalfTurnNegatesVector) {
    CHECK_VEC_NEAR(rotate(Vec2(2.0f, 3.0f), kPi), -2.0f, -3.0f);
}

TEST(Transform, RotateFullTurnIsIdentity) {
    CHECK_VEC_NEAR(rotate(Vec2(2.0f, 3.0f), 2.0f * kPi), 2.0f, 3.0f);
}

TEST(Transform, RotatePreservesLength) {
    const Vec2 v(3.0f, -4.0f);
    CHECK_FLOAT_EQ(d2(rotate(v, 0.9f)), d2(v));
}

TEST(Transform, RotateByNegativeAngleUndoesRotation) {
    const Vec2 v(3.0f, -4.0f);
    CHECK_VEC_NEAR(rotate(rotate(v, 0.7f), -0.7f), 3.0f, -4.0f);
}

TEST(Transform, RotateZeroVectorStaysZero) {
    CHECK_VEC_NEAR(rotate(Vec2(0.0f, 0.0f), 1.234f), 0.0f, 0.0f);
}

// --- worldToLocal / localToWorld -------------------------------------------

TEST(Transform, LocalToWorldWithIdentityPoseIsUnchanged) {
    CHECK_VEC_NEAR(localToWorld(Vec2(2.0f, 3.0f), Vec2(0.0f, 0.0f), 0.0f), 2.0f,
                   3.0f);
}

TEST(Transform, LocalToWorldTranslatesByBodyPosition) {
    CHECK_VEC_NEAR(localToWorld(Vec2(2.0f, 3.0f), Vec2(10.0f, -5.0f), 0.0f),
                   12.0f, -2.0f);
}

TEST(Transform, LocalToWorldRotatesThenTranslates) {
    // Rotating (1,0) a quarter turn gives (0,1); then offset by (10,10).
    CHECK_VEC_NEAR(
        localToWorld(Vec2(1.0f, 0.0f), Vec2(10.0f, 10.0f), kPi / 2.0f), 10.0f,
        11.0f);
}

TEST(Transform, WorldToLocalOfBodyPositionIsOrigin) {
    const Vec2 position(7.0f, -3.0f);
    CHECK_VEC_NEAR(worldToLocal(position, position, 1.1f), 0.0f, 0.0f);
}

TEST(Transform, WorldToLocalUndoesLocalToWorld) {
    const Vec2 local(2.0f, -1.5f);
    const Vec2 position(4.0f, 9.0f);
    const float orientation = 0.6f;
    const Vec2 world = localToWorld(local, position, orientation);
    CHECK_VEC_NEAR(worldToLocal(world, position, orientation), 2.0f, -1.5f);
}

TEST(Transform, LocalToWorldUndoesWorldToLocal) {
    const Vec2 world(12.0f, -7.0f);
    const Vec2 position(4.0f, 9.0f);
    const float orientation = -0.9f;
    const Vec2 local = worldToLocal(world, position, orientation);
    CHECK_VEC_NEAR(localToWorld(local, position, orientation), 12.0f, -7.0f);
}

// --- CircleShape::support ---------------------------------------------------

TEST(Support, CircleSupportIsCentrePlusRadiusAlongDirection) {
    const CircleShape circle(2.0f);
    CHECK_VEC_NEAR(circle.support(Vec2(1.0f, 0.0f), Vec2(5.0f, 0.0f), 0.0f),
                   7.0f, 0.0f);
}

TEST(Support, CircleSupportNormalisesTheDirection) {
    const CircleShape circle(2.0f);
    // A longer direction vector must give the same point.
    CHECK_VEC_NEAR(circle.support(Vec2(10.0f, 0.0f), Vec2(5.0f, 0.0f), 0.0f),
                   7.0f, 0.0f);
}

TEST(Support, CircleSupportWorksDiagonally) {
    const CircleShape circle(2.0f);
    const float expected = 2.0f / std::sqrt(2.0f);
    CHECK_VEC_NEAR(circle.support(Vec2(1.0f, 1.0f), Vec2(0.0f, 0.0f), 0.0f),
                   expected, expected);
}

TEST(Support, CircleSupportIgnoresOrientation) {
    const CircleShape circle(2.0f);
    // A circle is rotationally symmetric, so orientation must not matter.
    const Vec2 unrotated =
        circle.support(Vec2(1.0f, 0.0f), Vec2(0.0f, 0.0f), 0.0f);
    const Vec2 rotated =
        circle.support(Vec2(1.0f, 0.0f), Vec2(0.0f, 0.0f), 1.3f);
    CHECK_VEC_NEAR(rotated, unrotated.getX(), unrotated.getY());
}

TEST(Support, CircleSupportWithZeroDirectionReturnsCentre) {
    const CircleShape circle(2.0f);
    CHECK_VEC_NEAR(circle.support(Vec2(0.0f, 0.0f), Vec2(5.0f, -1.0f), 0.0f),
                   5.0f, -1.0f);
}

// --- BoxShape::support ------------------------------------------------------

TEST(Support, BoxSupportPicksTheExtremeCorner) {
    const BoxShape box(4.0f, 2.0f);  // half extents (2, 1)
    CHECK_VEC_NEAR(box.support(Vec2(1.0f, 1.0f), Vec2(0.0f, 0.0f), 0.0f), 2.0f,
                   1.0f);
    CHECK_VEC_NEAR(box.support(Vec2(-1.0f, -1.0f), Vec2(0.0f, 0.0f), 0.0f),
                   -2.0f, -1.0f);
    CHECK_VEC_NEAR(box.support(Vec2(1.0f, -1.0f), Vec2(0.0f, 0.0f), 0.0f), 2.0f,
                   -1.0f);
    CHECK_VEC_NEAR(box.support(Vec2(-1.0f, 1.0f), Vec2(0.0f, 0.0f), 0.0f),
                   -2.0f, 1.0f);
}

TEST(Support, BoxSupportIsOffsetByPosition) {
    const BoxShape box(4.0f, 2.0f);
    CHECK_VEC_NEAR(box.support(Vec2(1.0f, 1.0f), Vec2(10.0f, 20.0f), 0.0f),
                   12.0f, 21.0f);
}

TEST(Support, BoxSupportIsUnaffectedByDirectionMagnitude) {
    const BoxShape box(4.0f, 2.0f);
    const Vec2 near = box.support(Vec2(1.0f, 1.0f), Vec2(0.0f, 0.0f), 0.0f);
    const Vec2 far = box.support(Vec2(500.0f, 500.0f), Vec2(0.0f, 0.0f), 0.0f);
    CHECK_VEC_NEAR(far, near.getX(), near.getY());
}

// A quarter turn swaps the box's reach along each axis: a 4x2 box extends 2
// along x when unrotated, but only 1 once turned.
TEST(Support, BoxSupportHonoursOrientation) {
    const BoxShape box(4.0f, 2.0f);
    CHECK_FLOAT_EQ(
        box.support(Vec2(1.0f, 0.0f), Vec2(0.0f, 0.0f), 0.0f).getX(), 2.0f);
    CHECK_NEAR(
        box.support(Vec2(1.0f, 0.0f), Vec2(0.0f, 0.0f), kPi / 2.0f).getX(),
        1.0f, 1e-3f);
}

// A half turn maps a centred box onto itself, so the extreme point in a given
// world direction is unchanged -- it is simply the opposite local corner.
TEST(Support, BoxSupportUnderHalfTurnIsUnchanged) {
    const BoxShape box(4.0f, 2.0f);
    const Vec2 turned = box.support(Vec2(1.0f, 1.0f), Vec2(0.0f, 0.0f), kPi);
    CHECK_NEAR(turned.getX(), 2.0f, 1e-3f);
    CHECK_NEAR(turned.getY(), 1.0f, 1e-3f);
}

TEST(Support, SquareSupportUnderQuarterTurnKeepsSameReach) {
    const BoxShape square(2.0f, 2.0f);
    CHECK_NEAR(square.support(Vec2(1.0f, 0.0f), Vec2(0.0f, 0.0f), 0.0f).getX(),
               1.0f, 1e-3f);
    CHECK_NEAR(
        square.support(Vec2(1.0f, 0.0f), Vec2(0.0f, 0.0f), kPi / 2.0f).getX(),
        1.0f, 1e-3f);
}

// --- CircleShape::supportFeature --------------------------------------------

// A circle has no flat faces, so its support feature is always a single point.
TEST(SupportFeature, CircleFeatureIsASinglePoint) {
    const CircleShape circle(2.0f);
    Feature feature =
        circle.supportFeature(Vec2(1.0f, 0.0f), Vec2(5.0f, 0.0f), 0.0f);
    CHECK(feature.isPoint());
    CHECK(!feature.isEdge());
    CHECK_VEC_NEAR(feature.vertices[0].position, 7.0f, 0.0f);
}

TEST(SupportFeature, CircleFeatureMatchesSupport) {
    const CircleShape circle(1.5f);
    const Vec2 position(-3.0f, 4.0f);
    const Vec2 directions[] = {Vec2(1.0f, 0.0f), Vec2(0.0f, -1.0f),
                               Vec2(3.0f, 4.0f), Vec2(-2.0f, 0.5f)};
    for (const Vec2 &direction : directions) {
        const Feature feature =
            circle.supportFeature(direction, position, 0.7f);
        const Vec2 expected = circle.support(direction, position, 0.7f);
        CHECK_VEC_NEAR(feature.vertices[0].position, expected.getX(),
                       expected.getY());
    }
}

TEST(SupportFeature, CircleFeatureWithZeroDirectionIsTheCentre) {
    const CircleShape circle(2.0f);
    Feature feature =
        circle.supportFeature(Vec2(0.0f, 0.0f), Vec2(5.0f, -1.0f), 0.0f);
    CHECK(feature.isPoint());
    CHECK_VEC_NEAR(feature.vertices[0].position, 5.0f, -1.0f);
}

// --- BoxShape::supportFeature -----------------------------------------------

// Along each face normal the support feature is that whole face.
TEST(SupportFeature, BoxFeatureAlongFaceNormalsIsThatFace) {
    const BoxShape box(4.0f, 2.0f);  // half extents (2, 1)
    const Vec2 origin(0.0f, 0.0f);

    CHECK(isEdgeBetween(box.supportFeature(Vec2(1.0f, 0.0f), origin, 0.0f),
                        Vec2(2.0f, 1.0f), Vec2(2.0f, -1.0f)));
    CHECK(isEdgeBetween(box.supportFeature(Vec2(-1.0f, 0.0f), origin, 0.0f),
                        Vec2(-2.0f, 1.0f), Vec2(-2.0f, -1.0f)));
    CHECK(isEdgeBetween(box.supportFeature(Vec2(0.0f, 1.0f), origin, 0.0f),
                        Vec2(2.0f, 1.0f), Vec2(-2.0f, 1.0f)));
    CHECK(isEdgeBetween(box.supportFeature(Vec2(0.0f, -1.0f), origin, 0.0f),
                        Vec2(2.0f, -1.0f), Vec2(-2.0f, -1.0f)));
}

// Off-axis, the chosen face is the one whose normal is closest to the
// direction -- i.e. the edge most perpendicular to it.
TEST(SupportFeature, BoxFeaturePicksFaceMostFacingTheDirection) {
    const BoxShape box(2.0f, 2.0f);
    const Vec2 origin(0.0f, 0.0f);

    CHECK(isEdgeBetween(box.supportFeature(Vec2(1.0f, 0.3f), origin, 0.0f),
                        Vec2(1.0f, 1.0f), Vec2(1.0f, -1.0f)));
    CHECK(isEdgeBetween(box.supportFeature(Vec2(0.3f, 1.0f), origin, 0.0f),
                        Vec2(1.0f, 1.0f), Vec2(-1.0f, 1.0f)));
    CHECK(isEdgeBetween(box.supportFeature(Vec2(-1.0f, -0.3f), origin, 0.0f),
                        Vec2(-1.0f, 1.0f), Vec2(-1.0f, -1.0f)));
    CHECK(isEdgeBetween(box.supportFeature(Vec2(-0.3f, -1.0f), origin, 0.0f),
                        Vec2(1.0f, -1.0f), Vec2(-1.0f, -1.0f)));
}

// The face choice depends on the direction only, not on how long each edge
// is. A wide box seen from mostly-above still presents its top face...
TEST(SupportFeature, WideBoxFeatureDoesNotDependOnEdgeLength) {
    const BoxShape box(4.0f, 2.0f);  // half extents (2, 1)
    CHECK(isEdgeBetween(
        box.supportFeature(Vec2(0.6f, 0.8f), Vec2(0.0f, 0.0f), 0.0f),
        Vec2(2.0f, 1.0f), Vec2(-2.0f, 1.0f)));
}

// ...and a tall box seen from mostly-right still presents its right face.
TEST(SupportFeature, TallBoxFeatureDoesNotDependOnEdgeLength) {
    const BoxShape box(2.0f, 4.0f);  // half extents (1, 2)
    CHECK(isEdgeBetween(
        box.supportFeature(Vec2(0.8f, 0.6f), Vec2(0.0f, 0.0f), 0.0f),
        Vec2(1.0f, 2.0f), Vec2(1.0f, -2.0f)));
}

// The feature always contains the support point.
TEST(SupportFeature, BoxFeatureContainsTheSupportPoint) {
    const BoxShape box(3.0f, 1.0f);
    const Vec2 position(2.0f, -1.0f);
    const float orientation = 0.4f;
    const Vec2 directions[] = {Vec2(1.0f, 0.0f), Vec2(0.0f, 1.0f),
                               Vec2(-1.0f, 0.2f), Vec2(0.3f, -0.9f),
                               Vec2(-0.7f, -0.7f)};
    for (const Vec2 &direction : directions) {
        const Feature feature =
            box.supportFeature(direction, position, orientation);
        const Vec2 expected = box.support(direction, position, orientation);
        CHECK(near(feature.vertices[0].position, expected) ||
              (feature.count == 2 &&
               near(feature.vertices[1].position, expected)));
    }
}

TEST(SupportFeature, BoxFeatureIsOffsetByPosition) {
    const BoxShape box(4.0f, 2.0f);
    CHECK(isEdgeBetween(
        box.supportFeature(Vec2(1.0f, 0.0f), Vec2(10.0f, 20.0f), 0.0f),
        Vec2(12.0f, 21.0f), Vec2(12.0f, 19.0f)));
}

TEST(SupportFeature, BoxFeatureIsUnaffectedByDirectionMagnitude) {
    const BoxShape box(4.0f, 2.0f);
    CHECK(isEdgeBetween(
        box.supportFeature(Vec2(300.0f, 400.0f), Vec2(0.0f, 0.0f), 0.0f),
        Vec2(2.0f, 1.0f), Vec2(-2.0f, 1.0f)));
}

// A 4x2 box turned a quarter turn stands upright, so looking straight up hits
// its short local +x face, which now spans x in [-1, 1] at y = 2.
TEST(SupportFeature, BoxFeatureHonoursOrientation) {
    const BoxShape box(4.0f, 2.0f);
    CHECK(isEdgeBetween(
        box.supportFeature(Vec2(0.0f, 1.0f), Vec2(0.0f, 0.0f), kPi / 2.0f),
        Vec2(-1.0f, 2.0f), Vec2(1.0f, 2.0f)));
}

// A 2x2 box turned 45 degrees presents an edge to a direction along one of its
// own face normals, not to the world axes.
TEST(SupportFeature, RotatedBoxFeatureFollowsItsOwnFaceNormal) {
    const BoxShape box(2.0f, 2.0f);
    const float r = std::sqrt(2.0f);
    CHECK(isEdgeBetween(
        box.supportFeature(Vec2(1.0f, 1.0f), Vec2(0.0f, 0.0f), kPi / 4.0f),
        Vec2(0.0f, r), Vec2(r, 0.0f)));
}

// A zero-size box is a single point; it has no edge to report.
TEST(SupportFeature, ZeroSizeBoxFeatureIsASinglePoint) {
    const BoxShape box(0.0f, 0.0f);
    Feature feature =
        box.supportFeature(Vec2(1.0f, 1.0f), Vec2(3.0f, 4.0f), 0.0f);
    CHECK(feature.isPoint());
    CHECK_VEC_NEAR(feature.vertices[0].position, 3.0f, 4.0f);
}

TEST(SupportFeature, ZeroWidthBoxFeatureKeepsItsSegment) {
    const BoxShape box(0.0f, 2.0f);
    CHECK(isEdgeBetween(
        box.supportFeature(Vec2(1.0f, 0.0f), Vec2(3.0f, 4.0f), 0.0f),
        Vec2(3.0f, 3.0f), Vec2(3.0f, 5.0f)));
}

// --- Simplex ----------------------------------------------------------------

TEST(Simplex, StartsEmpty) {
    const Simplex simplex;
    CHECK(simplex.size() == 0);
}

TEST(Simplex, PushFrontGrowsTheSimplex) {
    Simplex simplex;
    simplex.pushFront(Vec2(1.0f, 0.0f));
    CHECK(simplex.size() == 1);
    simplex.pushFront(Vec2(2.0f, 0.0f));
    CHECK(simplex.size() == 2);
    simplex.pushFront(Vec2(3.0f, 0.0f));
    CHECK(simplex.size() == 3);
}

// The newest point must end up at index 0 -- GJK relies on A being newest.
TEST(Simplex, PushFrontPutsNewestPointFirst) {
    Simplex simplex;
    simplex.pushFront(Vec2(1.0f, 0.0f));
    simplex.pushFront(Vec2(2.0f, 0.0f));
    simplex.pushFront(Vec2(3.0f, 0.0f));
    CHECK_VEC_NEAR(simplex.getA(), 3.0f, 0.0f);
    CHECK_VEC_NEAR(simplex.getB(), 2.0f, 0.0f);
    CHECK_VEC_NEAR(simplex.getC(), 1.0f, 0.0f);
}

TEST(Simplex, IndexingMatchesGetters) {
    Simplex simplex;
    simplex.pushFront(Vec2(1.0f, 0.0f));
    simplex.pushFront(Vec2(2.0f, 0.0f));
    CHECK_VEC_NEAR(simplex[0], 2.0f, 0.0f);
    CHECK_VEC_NEAR(simplex[1], 1.0f, 0.0f);
}

TEST(Simplex, PushFrontWhenFullKeepsSizeAndDropsOldest) {
    Simplex simplex;
    simplex.pushFront(Vec2(1.0f, 0.0f));
    simplex.pushFront(Vec2(2.0f, 0.0f));
    simplex.pushFront(Vec2(3.0f, 0.0f));
    simplex.pushFront(Vec2(4.0f, 0.0f));
    CHECK(simplex.size() == 3);
    CHECK_VEC_NEAR(simplex.getA(), 4.0f, 0.0f);
    CHECK_VEC_NEAR(simplex.getB(), 3.0f, 0.0f);
    CHECK_VEC_NEAR(simplex.getC(), 2.0f, 0.0f);
}

TEST(Simplex, SetOnePointShrinksToSizeOne) {
    Simplex simplex;
    simplex.pushFront(Vec2(1.0f, 0.0f));
    simplex.pushFront(Vec2(2.0f, 0.0f));
    simplex.pushFront(Vec2(3.0f, 0.0f));
    simplex.set(Vec2(9.0f, 9.0f));
    CHECK(simplex.size() == 1);
    CHECK_VEC_NEAR(simplex.getA(), 9.0f, 9.0f);
}

TEST(Simplex, SetTwoPointsShrinksToSizeTwo) {
    Simplex simplex;
    simplex.pushFront(Vec2(1.0f, 0.0f));
    simplex.pushFront(Vec2(2.0f, 0.0f));
    simplex.pushFront(Vec2(3.0f, 0.0f));
    simplex.set(Vec2(8.0f, 0.0f), Vec2(9.0f, 0.0f));
    CHECK(simplex.size() == 2);
    CHECK_VEC_NEAR(simplex.getA(), 8.0f, 0.0f);
    CHECK_VEC_NEAR(simplex.getB(), 9.0f, 0.0f);
}

TEST(Simplex, SetThreePointsGivesSizeThree) {
    Simplex simplex;
    simplex.set(Vec2(1.0f, 1.0f), Vec2(2.0f, 2.0f), Vec2(3.0f, 3.0f));
    CHECK(simplex.size() == 3);
    CHECK_VEC_NEAR(simplex.getC(), 3.0f, 3.0f);
}

TEST(Simplex, GetLastReturnsTheOldestPoint) {
    Simplex simplex;
    simplex.pushFront(Vec2(1.0f, 0.0f));
    simplex.pushFront(Vec2(2.0f, 0.0f));
    CHECK_VEC_NEAR(simplex.getLast(), 1.0f, 0.0f);
}

TEST(Simplex, IndexOutOfRangeThrows) {
    Simplex simplex;
    simplex.pushFront(Vec2(1.0f, 0.0f));
    CHECK_THROWS(simplex[1], std::out_of_range);
}

TEST(Simplex, AccessingMissingPointsThrows) {
    Simplex simplex;
    CHECK_THROWS(simplex.getA(), std::out_of_range);
    simplex.pushFront(Vec2(1.0f, 0.0f));
    CHECK_THROWS(simplex.getB(), std::out_of_range);
    simplex.pushFront(Vec2(2.0f, 0.0f));
    CHECK_THROWS(simplex.getC(), std::out_of_range);
}
