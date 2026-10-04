// Contact generation tests: detectCollision() = GJK + EPA.
//
// Conventions pinned down here:
//   * normal is unit length and points from body A towards body B;
//   * depth is the minimum translation distance, so moving B by
//     normal * depth (or A by -normal * depth) brings the pair to touching;
//   * separated pairs report colliding == false and depth == 0;
//   * exactly touching pairs report colliding == true and depth ~= 0.
//
// EPA approximates curved shapes (circles) with a polytope, so circle depths
// are checked with a looser tolerance than box depths.

#include <cmath>
#include <memory>
#include <vector>

#include "tinysim/geometry/BoxShape.h"
#include "tinysim/geometry/CircleShape.h"
#include "tinysim/collision/Collision.h"
#include "tinysim/dynamics/RigidBody.h"
#include "tinysim/math/Vec2.h"
#include "support/test_utils.h"

namespace {

constexpr float kPi = 3.14159265f;
constexpr float kBoxEps = 1e-3f;
constexpr float kCircleEps = 1e-2f;
// EPA stops once a new support point gains less than 1e-4, which on a unit
// circle still allows the normal to be off by about sqrt(2 * 1e-4) ~= 0.014.
constexpr float kCircleNormalEps = 2e-2f;

RigidBody makeCircle(const Vec2 &position, float radius,
                     float orientation = 0.0f) {
    RigidBody body(position, orientation, 1.0f);
    body.setShape(std::make_unique<CircleShape>(radius));
    return body;
}

RigidBody makeBox(const Vec2 &position, float width, float height,
                  float orientation = 0.0f) {
    RigidBody body(position, orientation, 1.0f);
    body.setShape(std::make_unique<BoxShape>(width, height));
    return body;
}

Collision contact(const RigidBody &a, const RigidBody &b) {
    return detectCollision(&a, &b);
}

// Rebuilds b's shape at a new position. RigidBody owns its shape through a
// unique_ptr, so there is no copy; the tests only need circles and boxes.
RigidBody movedCircle(const RigidBody &b, const Vec2 &offset) {
    const auto *circle = static_cast<const CircleShape *>(b.getShape());
    return makeCircle(b.getPosition() + offset, circle->getRadius(),
                      b.getOrientation());
}

RigidBody movedBox(const RigidBody &b, const Vec2 &offset) {
    const auto *box = static_cast<const BoxShape *>(b.getShape());
    return makeBox(b.getPosition() + offset, box->getWidth(), box->getHeight(),
                   b.getOrientation());
}

RigidBody moved(const RigidBody &b, const Vec2 &offset) {
    return b.getShape()->getType() == ShapeType::CIRCLE ? movedCircle(b, offset)
                                                        : movedBox(b, offset);
}

float length(const Vec2 &v) { return std::sqrt(d2(v)); }

}  // namespace

// --- separated pairs --------------------------------------------------------

TEST(Contact, SeparatedCirclesReportNoContact) {
    const Collision c = contact(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                                makeCircle(Vec2(5.0f, 0.0f), 1.0f));
    CHECK(!c.colliding);
    CHECK_FLOAT_EQ(c.depth, 0.0f);
}

TEST(Contact, SeparatedBoxesReportNoContact) {
    const Collision c = contact(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                                makeBox(Vec2(2.5f, 2.5f), 2.0f, 2.0f));
    CHECK(!c.colliding);
    CHECK_FLOAT_EQ(c.depth, 0.0f);
}

TEST(Contact, SeparatedCircleAndBoxReportNoContact) {
    const Collision c = contact(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                                makeBox(Vec2(2.001f, 0.0f), 2.0f, 2.0f));
    CHECK(!c.colliding);
    CHECK_FLOAT_EQ(c.depth, 0.0f);
}

// --- circle vs circle -------------------------------------------------------

TEST(Contact, OverlappingCirclesAlongX) {
    // Radii sum to 4, centres 1 apart: depth 3.
    const Collision c = contact(makeCircle(Vec2(0.0f, 0.0f), 2.0f),
                                makeCircle(Vec2(1.0f, 0.0f), 2.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 3.0f, kCircleEps);
    CHECK_NEAR(c.normal.getX(), 1.0f, kCircleNormalEps);
    CHECK_NEAR(c.normal.getY(), 0.0f, kCircleNormalEps);
}

TEST(Contact, OverlappingCirclesDiagonally) {
    // Centres sqrt(2) apart, radii sum to 2.
    const Collision c = contact(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                                makeCircle(Vec2(1.0f, 1.0f), 1.0f));
    const float invSqrt2 = 1.0f / std::sqrt(2.0f);
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 2.0f - std::sqrt(2.0f), kCircleEps);
    CHECK_NEAR(c.normal.getX(), invSqrt2, kCircleNormalEps);
    CHECK_NEAR(c.normal.getY(), invSqrt2, kCircleNormalEps);
}

TEST(Contact, OverlappingCirclesOfDifferentRadii) {
    // Radii 3 + 1 = 4, centres 2.5 apart (along -y): depth 1.5.
    const Collision c = contact(makeCircle(Vec2(0.0f, 0.0f), 3.0f),
                                makeCircle(Vec2(0.0f, -2.5f), 1.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 1.5f, kCircleEps);
    CHECK_NEAR(c.normal.getX(), 0.0f, kCircleNormalEps);
    CHECK_NEAR(c.normal.getY(), -1.0f, kCircleNormalEps);
}

// The very first Minkowski support point is the origin here, so GJK returns a
// one-point simplex and detectCollision takes its single-point branch.
TEST(Contact, TouchingCirclesReportZeroDepthAlongCentreLine) {
    const Collision c = contact(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                                makeCircle(Vec2(2.0f, 0.0f), 1.0f));
    CHECK(c.colliding);
    CHECK_FLOAT_EQ(c.depth, 0.0f);
    CHECK_VEC_NEAR(c.normal, 1.0f, 0.0f);
}

// Concentric circles have no preferred axis, so only depth and unit length
// are pinned down. Depth is the sum of the radii.
TEST(Contact, ConcentricCircles) {
    const Collision c = contact(makeCircle(Vec2(4.0f, -2.0f), 1.0f),
                                makeCircle(Vec2(4.0f, -2.0f), 2.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 3.0f, kCircleEps);
    CHECK_NEAR(length(c.normal), 1.0f, kEps);
}

// --- box vs box -------------------------------------------------------------

TEST(Contact, OverlappingBoxesAlongX) {
    const Collision c = contact(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                                makeBox(Vec2(1.5f, 0.0f), 2.0f, 2.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 0.5f, kBoxEps);
    CHECK_NEAR(c.normal.getX(), 1.0f, kBoxEps);
    CHECK_NEAR(c.normal.getY(), 0.0f, kBoxEps);
}

TEST(Contact, OverlappingBoxesAlongNegativeX) {
    const Collision c = contact(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                                makeBox(Vec2(-1.5f, 0.0f), 2.0f, 2.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 0.5f, kBoxEps);
    CHECK_NEAR(c.normal.getX(), -1.0f, kBoxEps);
    CHECK_NEAR(c.normal.getY(), 0.0f, kBoxEps);
}

TEST(Contact, OverlappingBoxesVertically) {
    // 2x4 boxes 3 apart in y overlap by 1.
    const Collision c = contact(makeBox(Vec2(0.0f, 0.0f), 2.0f, 4.0f),
                                makeBox(Vec2(0.0f, 3.0f), 2.0f, 4.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 1.0f, kBoxEps);
    CHECK_NEAR(c.normal.getX(), 0.0f, kBoxEps);
    CHECK_NEAR(c.normal.getY(), 1.0f, kBoxEps);
}

// Overlap is 1 in x and 3 in y: EPA must pick the *shallower* axis.
TEST(Contact, BoxesPickAxisOfLeastPenetration) {
    const Collision c = contact(makeBox(Vec2(0.0f, 0.0f), 4.0f, 4.0f),
                                makeBox(Vec2(3.0f, 1.0f), 4.0f, 4.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 1.0f, kBoxEps);
    CHECK_NEAR(c.normal.getX(), 1.0f, kBoxEps);
    CHECK_NEAR(c.normal.getY(), 0.0f, kBoxEps);
}

// A 2x2 box turned 45 degrees reaches sqrt(2) along x; the neighbour's face
// sits at x = 1.2, so the corner pokes in by sqrt(2) - 1.2.
TEST(Contact, RotatedCornerIntoFace) {
    const Collision c =
        contact(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f, kPi / 4.0f),
                makeBox(Vec2(2.2f, 0.0f), 2.0f, 2.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, std::sqrt(2.0f) - 1.2f, kBoxEps);
    CHECK_NEAR(c.normal.getX(), 1.0f, kBoxEps);
    CHECK_NEAR(c.normal.getY(), 0.0f, kBoxEps);
}

// Two boxes rotated together are just a rotated copy of an axis-aligned pair,
// so the normal must rotate with them and the depth must not change.
TEST(Contact, NormalRotatesWithTheBodies) {
    const float angle = 0.6f;
    const Vec2 axis(std::cos(angle), std::sin(angle));
    const Collision c = contact(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f, angle),
                                makeBox(axis * 1.5f, 2.0f, 2.0f, angle));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 0.5f, kBoxEps);
    CHECK_NEAR(c.normal.getX(), axis.getX(), kBoxEps);
    CHECK_NEAR(c.normal.getY(), axis.getY(), kBoxEps);
}

TEST(Contact, TouchingBoxesReportZeroDepth) {
    const Collision c = contact(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                                makeBox(Vec2(2.0f, 0.0f), 2.0f, 2.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 0.0f, kBoxEps);
    CHECK_NEAR(length(c.normal), 1.0f, kEps);
}

TEST(Contact, IdenticalBoxesAtIdenticalPose) {
    // Any face direction is a valid answer; depth is the full width.
    const Collision c = contact(makeBox(Vec2(3.0f, -4.0f), 2.0f, 2.0f),
                                makeBox(Vec2(3.0f, -4.0f), 2.0f, 2.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 2.0f, kBoxEps);
    CHECK_NEAR(length(c.normal), 1.0f, kEps);
}

// --- circle vs box ----------------------------------------------------------

TEST(Contact, CirclePenetratingBoxFace) {
    // Circle reaches x = 1, box face at x = 0.5.
    const Collision c = contact(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                                makeBox(Vec2(1.5f, 0.0f), 2.0f, 2.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 0.5f, kCircleEps);
    CHECK_NEAR(c.normal.getX(), 1.0f, kCircleNormalEps);
    CHECK_NEAR(c.normal.getY(), 0.0f, kCircleNormalEps);
}

TEST(Contact, BoxPenetratingCircleFromAbove) {
    // Box bottom face at y = 0.5, circle top at y = 1.
    const Collision c = contact(makeBox(Vec2(0.0f, 1.5f), 2.0f, 2.0f),
                                makeCircle(Vec2(0.0f, 0.0f), 1.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 0.5f, kCircleEps);
    CHECK_NEAR(c.normal.getX(), 0.0f, kCircleNormalEps);
    CHECK_NEAR(c.normal.getY(), -1.0f, kCircleNormalEps);
}

// Circle centred in a 4x4 box: escaping through any face takes 2 + 0.5.
TEST(Contact, CircleDeepInsideBox) {
    const Collision c = contact(makeCircle(Vec2(0.0f, 0.0f), 0.5f),
                                makeBox(Vec2(0.0f, 0.0f), 4.0f, 4.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 2.5f, kCircleEps);
    CHECK_NEAR(std::fabs(c.normal.getX()) + std::fabs(c.normal.getY()), 1.0f,
               kCircleNormalEps);
}

// --- degenerate shapes ------------------------------------------------------

// A zero-size box is a single point; the Minkowski difference is then just a
// translated circle.
TEST(Contact, PointInsideCircle) {
    const Collision c = contact(makeBox(Vec2(0.5f, 0.0f), 0.0f, 0.0f),
                                makeCircle(Vec2(0.0f, 0.0f), 1.0f));
    CHECK(c.colliding);
    CHECK_NEAR(c.depth, 0.5f, kCircleEps);
    CHECK_NEAR(c.normal.getX(), -1.0f, kCircleNormalEps);
    CHECK_NEAR(c.normal.getY(), 0.0f, kCircleNormalEps);
}

// --- properties -------------------------------------------------------------

namespace {

struct Pair {
    RigidBody a;
    RigidBody b;
};

std::vector<Pair> penetratingPairs() {
    std::vector<Pair> pairs;
    pairs.push_back({makeCircle(Vec2(0.0f, 0.0f), 2.0f),
                     makeCircle(Vec2(1.0f, 0.0f), 2.0f)});
    pairs.push_back({makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                     makeCircle(Vec2(1.0f, 1.0f), 1.0f)});
    pairs.push_back({makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                     makeBox(Vec2(1.5f, 0.3f), 2.0f, 2.0f)});
    pairs.push_back({makeBox(Vec2(0.0f, 0.0f), 4.0f, 4.0f),
                     makeBox(Vec2(3.0f, 1.0f), 4.0f, 4.0f)});
    pairs.push_back({makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f, kPi / 4.0f),
                     makeBox(Vec2(2.2f, 0.0f), 2.0f, 2.0f)});
    pairs.push_back({makeBox(Vec2(0.0f, 0.0f), 3.0f, 1.0f, 0.3f),
                     makeBox(Vec2(1.0f, 0.8f), 1.0f, 2.0f, -0.5f)});
    pairs.push_back({makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                     makeBox(Vec2(1.5f, 0.0f), 2.0f, 2.0f)});
    pairs.push_back({makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                     makeBox(Vec2(1.2f, 1.2f), 2.0f, 2.0f, 0.4f)});
    return pairs;
}

}  // namespace

TEST(Contact, NormalIsUnitLength) {
    for (const Pair &pair : penetratingPairs()) {
        const Collision c = contact(pair.a, pair.b);
        CHECK(c.colliding);
        CHECK_NEAR(length(c.normal), 1.0f, kEps);
    }
}

TEST(Contact, DepthIsPositiveWhenPenetrating) {
    for (const Pair &pair : penetratingPairs()) {
        CHECK(contact(pair.a, pair.b).depth > 0.0f);
    }
}

// The normal points from A to B, so it must have a non-negative component
// along the line from A's centre to B's centre.
TEST(Contact, NormalPointsFromAToB) {
    for (const Pair &pair : penetratingPairs()) {
        const Collision c = contact(pair.a, pair.b);
        CHECK(dot(c.normal, pair.b.getPosition() - pair.a.getPosition()) >=
              0.0f);
    }
}

// Swapping the bodies negates the Minkowski difference: same depth, opposite
// normal.
TEST(Contact, SwappingBodiesNegatesNormalAndKeepsDepth) {
    for (const Pair &pair : penetratingPairs()) {
        const Collision ab = contact(pair.a, pair.b);
        const Collision ba = contact(pair.b, pair.a);
        CHECK(ba.colliding);
        CHECK_NEAR(ba.depth, ab.depth, kCircleEps);
        CHECK_NEAR(ba.normal.getX(), -ab.normal.getX(), kCircleNormalEps);
        CHECK_NEAR(ba.normal.getY(), -ab.normal.getY(), kCircleNormalEps);
    }
}

TEST(Contact, TranslatingBothBodiesDoesNotChangeContact) {
    const Vec2 offset(123.0f, -45.0f);
    for (const Pair &pair : penetratingPairs()) {
        const Collision here = contact(pair.a, pair.b);
        const Collision there =
            contact(moved(pair.a, offset), moved(pair.b, offset));
        CHECK(there.colliding);
        CHECK_NEAR(there.depth, here.depth, kCircleEps);
        CHECK_NEAR(there.normal.getX(), here.normal.getX(), kCircleNormalEps);
        CHECK_NEAR(there.normal.getY(), here.normal.getY(), kCircleNormalEps);
    }
}

// The defining property of the minimum translation vector: pushing B out by
// slightly more than normal * depth separates the pair, while pushing it by
// noticeably less leaves them overlapping.
TEST(Contact, PushingOutByDepthResolvesPenetration) {
    for (const Pair &pair : penetratingPairs()) {
        const Collision c = contact(pair.a, pair.b);

        const RigidBody resolved = moved(pair.b, c.normal * (c.depth + 0.02f));
        CHECK(!contact(pair.a, resolved).colliding);

        const RigidBody partial = moved(pair.b, c.normal * (c.depth * 0.5f));
        CHECK(contact(pair.a, partial).colliding);
    }
}

// --- contact points ---------------------------------------------------------

TEST(ContactPoints, SeparatedBodiesHaveNoPoints) {
    const Collision c = contact(makeBox(Vec2(), 2.0f, 2.0f),
                                makeBox(Vec2(5.0f, 0.0f), 2.0f, 2.0f));
    CHECK(!c.colliding);
    CHECK(c.contactPoints.empty());
}

TEST(ContactPoints, FaceOverlapHasTwoPoints) {
    const Collision c = contact(makeBox(Vec2(), 2.0f, 2.0f),
                                makeBox(Vec2(1.5f, 0.0f), 2.0f, 2.0f));
    CHECK(c.colliding);
    CHECK(c.contactPoints.size() == 2);
    CHECK_NEAR(c.contactPoints[0].getX(), c.contactPoints[1].getX(), kBoxEps);
    CHECK_NEAR(std::fabs(c.contactPoints[0].getY()), 1.0f, kBoxEps);
    CHECK_NEAR(std::fabs(c.contactPoints[1].getY()), 1.0f, kBoxEps);
    CHECK(c.contactPoints[0].getY() * c.contactPoints[1].getY() < 0.0f);
}

TEST(ContactPoints, ClippingKeepsPointsWithinBothFaces) {
    const Collision c = contact(makeBox(Vec2(), 2.0f, 2.0f),
                                makeBox(Vec2(0.75f, 1.5f), 2.0f, 2.0f));
    CHECK(c.colliding);
    CHECK(c.contactPoints.size() == 2);
    for (const Vec2 &point : c.contactPoints) {
        CHECK(point.getX() >= -0.25f - kBoxEps);
        CHECK(point.getX() <= 1.0f + kBoxEps);
    }
}

TEST(ContactPoints, CirclePairHasOneMidpoint) {
    const Collision c = contact(makeCircle(Vec2(), 2.0f),
                                makeCircle(Vec2(1.0f, 0.0f), 2.0f));
    CHECK(c.colliding);
    CHECK(c.contactPoints.size() == 1);
    CHECK_VEC_NEAR(c.contactPoints[0], 0.5f, 0.0f);
}

TEST(ContactPoints, CircleBoxHasOnePointInEitherBodyOrder) {
    const RigidBody circle = makeCircle(Vec2(), 1.0f);
    const RigidBody box = makeBox(Vec2(1.5f, 0.0f), 2.0f, 2.0f);
    const Collision cb = contact(circle, box);
    const Collision bc = contact(box, circle);
    CHECK(cb.contactPoints.size() == 1);
    CHECK(bc.contactPoints.size() == 1);
    CHECK_NEAR(cb.contactPoints[0].getX(), 1.0f, kCircleEps);
    CHECK_NEAR(bc.contactPoints[0].getX(), 1.0f, kCircleEps);
    CHECK_NEAR(cb.contactPoints[0].getY(), 0.0f, kCircleEps);
    CHECK_NEAR(bc.contactPoints[0].getY(), 0.0f, kCircleEps);
}

TEST(ContactPoints, RotatedCornerHasOnePoint) {
    const Collision c = contact(makeBox(Vec2(), 2.0f, 2.0f, kPi / 4.0f),
                                makeBox(Vec2(2.2f, 0.0f), 2.0f, 2.0f));
    CHECK(c.colliding);
    CHECK(c.contactPoints.size() == 1);
    CHECK_NEAR(c.contactPoints[0].getX(), std::sqrt(2.0f), kBoxEps);
    CHECK_NEAR(c.contactPoints[0].getY(), 0.0f, kBoxEps);
}

TEST(ContactPoints, FlatBoxCanProvideTwoContacts) {
    const Collision c = contact(makeBox(Vec2(), 0.0f, 2.0f, kPi / 2.0f),
                                makeBox(Vec2(0.0f, -0.5f), 2.0f, 2.0f));
    CHECK(c.colliding);
    CHECK(c.contactPoints.size() == 2);
    CHECK_NEAR(std::fabs(c.contactPoints[0].getX()), 1.0f, kBoxEps);
    CHECK_NEAR(std::fabs(c.contactPoints[1].getX()), 1.0f, kBoxEps);
    CHECK(c.contactPoints[0].getX() * c.contactPoints[1].getX() < 0.0f);
}

TEST(ContactPoints, PenetratingPairsProducePoints) {
    for (const Pair &pair : penetratingPairs()) {
        const Collision c = contact(pair.a, pair.b);
        CHECK(c.colliding);
        CHECK(!c.contactPoints.empty());
        CHECK(c.contactPoints.size() <= 2);
    }
}
