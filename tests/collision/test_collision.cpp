// GJK intersection tests.
//
// Convention note: GJK reports intersection when the Minkowski difference
// contains the origin, and two shapes that exactly touch have a separation of
// zero, so touching counts as colliding. The "exactly touching" tests below
// pin that down deliberately.

#include <memory>
#include <vector>

#include "tinysim/geometry/BoxShape.h"
#include "tinysim/geometry/CircleShape.h"
#include "tinysim/collision/GJK.h"
#include "tinysim/dynamics/RigidBody.h"
#include "tinysim/collision/Simplex.h"
#include "tinysim/math/Vec2.h"
#include "support/test_utils.h"

namespace {

constexpr float kPi = 3.14159265f;

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

bool collides(const RigidBody &a, const RigidBody &b) {
    Simplex simplex;
    return gjkIntersect(&a, &b, simplex);
}

}  // namespace

// --- circle vs circle -------------------------------------------------------

TEST(Collision, CirclesClearlySeparated) {
    // Centres 5 apart, radii sum to 2.
    CHECK(!collides(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                    makeCircle(Vec2(5.0f, 0.0f), 1.0f)));
}

TEST(Collision, CirclesClearlyOverlapping) {
    // Centres 1 apart, radii sum to 4.
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 2.0f),
                   makeCircle(Vec2(1.0f, 0.0f), 2.0f)));
}

// Separation is exactly zero here. Treated as colliding.
TEST(Collision, CirclesExactlyTouching) {
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                   makeCircle(Vec2(2.0f, 0.0f), 1.0f)));
}

TEST(Collision, CirclesJustSeparated) {
    CHECK(!collides(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                    makeCircle(Vec2(2.001f, 0.0f), 1.0f)));
}

TEST(Collision, CirclesJustOverlapping) {
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                   makeCircle(Vec2(1.999f, 0.0f), 1.0f)));
}

TEST(Collision, ConcentricCirclesOfDifferentRadii) {
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                   makeCircle(Vec2(0.0f, 0.0f), 3.0f)));
}

TEST(Collision, ConcentricIdenticalCircles) {
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 2.0f),
                   makeCircle(Vec2(0.0f, 0.0f), 2.0f)));
}

// Concentric bodies make the initial search direction degenerate (B - A is the
// zero vector), which gjkIntersect has to substitute an arbitrary axis for.
TEST(Collision, ConcentricCirclesAwayFromTheWorldOrigin) {
    CHECK(collides(makeCircle(Vec2(37.0f, -12.0f), 1.0f),
                   makeCircle(Vec2(37.0f, -12.0f), 2.5f)));
}

TEST(Collision, CirclesSeparatedDiagonally) {
    CHECK(!collides(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                    makeCircle(Vec2(4.0f, 4.0f), 1.0f)));
}

TEST(Collision, CirclesOverlappingDiagonally) {
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 2.0f),
                   makeCircle(Vec2(1.0f, 1.0f), 2.0f)));
}

TEST(Collision, SmallCircleFullyInsideLargeCircle) {
    CHECK(collides(makeCircle(Vec2(0.5f, 0.0f), 0.25f),
                   makeCircle(Vec2(0.0f, 0.0f), 5.0f)));
}

// --- box vs box -------------------------------------------------------------

TEST(Collision, BoxesClearlySeparated) {
    CHECK(!collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                    makeBox(Vec2(5.0f, 0.0f), 2.0f, 2.0f)));
}

TEST(Collision, BoxesOverlappingAxisAligned) {
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 4.0f, 4.0f),
                   makeBox(Vec2(2.0f, 0.0f), 4.0f, 4.0f)));
}

// Two 2x2 boxes 2 apart share exactly one edge.
TEST(Collision, BoxesExactlyTouchingEdgeToEdge) {
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                   makeBox(Vec2(2.0f, 0.0f), 2.0f, 2.0f)));
}

TEST(Collision, BoxesJustSeparated) {
    CHECK(!collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                    makeBox(Vec2(2.001f, 0.0f), 2.0f, 2.0f)));
}

TEST(Collision, BoxesJustOverlapping) {
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                   makeBox(Vec2(1.999f, 0.0f), 2.0f, 2.0f)));
}

TEST(Collision, BoxesSeparatedVertically) {
    CHECK(!collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                    makeBox(Vec2(0.0f, 5.0f), 2.0f, 2.0f)));
}

TEST(Collision, BoxesOverlappingVertically) {
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 4.0f),
                   makeBox(Vec2(0.0f, 3.0f), 2.0f, 4.0f)));
}

// Corner-diagonal separation: the gap along each axis is small but the boxes
// still miss each other.
TEST(Collision, BoxesSeparatedDiagonally) {
    CHECK(!collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                    makeBox(Vec2(2.5f, 2.5f), 2.0f, 2.0f)));
}

TEST(Collision, SmallBoxFullyInsideLargeBox) {
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 1.0f, 1.0f),
                   makeBox(Vec2(0.0f, 0.0f), 8.0f, 8.0f)));
}

// --- identical shapes at identical poses ------------------------------------

TEST(Collision, IdenticalBoxesAtIdenticalPose) {
    CHECK(collides(makeBox(Vec2(3.0f, -4.0f), 2.0f, 2.0f),
                   makeBox(Vec2(3.0f, -4.0f), 2.0f, 2.0f)));
}

TEST(Collision, IdenticalRotatedBoxesAtIdenticalPose) {
    CHECK(collides(makeBox(Vec2(3.0f, -4.0f), 2.0f, 5.0f, 0.7f),
                   makeBox(Vec2(3.0f, -4.0f), 2.0f, 5.0f, 0.7f)));
}

TEST(Collision, BodyComparedWithItself) {
    // Not something World should ever ask, but it must not hang or crash.
    const RigidBody body = makeBox(Vec2(3.0f, 3.0f), 2.0f, 2.0f, 0.4f);
    CHECK(collides(body, body));
}

// --- rotated boxes ----------------------------------------------------------

TEST(Collision, RotatedBoxOverlappingBox) {
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f, kPi / 4.0f),
                   makeBox(Vec2(1.0f, 0.0f), 2.0f, 2.0f)));
}

TEST(Collision, RotatedBoxClearlySeparatedFromBox) {
    CHECK(!collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f, kPi / 4.0f),
                    makeBox(Vec2(5.0f, 0.0f), 2.0f, 2.0f)));
}

// A 2x2 box turned 45 degrees reaches sqrt(2) ~= 1.414 along x instead of 1,
// so at a centre distance of 2.2 the rotated corner still overlaps the
// neighbour. This only passes if support() honours orientation.
TEST(Collision, RotationBringsCornerIntoContact) {
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f, kPi / 4.0f),
                   makeBox(Vec2(2.2f, 0.0f), 2.0f, 2.0f)));
    // Unrotated, the same pair is comfortably apart.
    CHECK(!collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                    makeBox(Vec2(2.2f, 0.0f), 2.0f, 2.0f)));
}

TEST(Collision, RotatedCornerStillMissesBeyondItsReach) {
    CHECK(!collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f, kPi / 4.0f),
                    makeBox(Vec2(2.6f, 0.0f), 2.0f, 2.0f)));
}

// A square is unchanged by a quarter turn, so the verdict must not change.
TEST(Collision, SquareRotatedQuarterTurnGivesSameVerdict) {
    const RigidBody probeNear = makeBox(Vec2(1.5f, 0.0f), 2.0f, 2.0f);
    const RigidBody probeFar = makeBox(Vec2(2.5f, 0.0f), 2.0f, 2.0f);

    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f, 0.0f), probeNear));
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f, kPi / 2.0f), probeNear));

    CHECK(!collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f, 0.0f), probeFar));
    CHECK(!collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f, kPi / 2.0f), probeFar));
}

// A 4x2 box becomes 2x4 after a quarter turn, so its x reach shrinks from 2 to
// 1 and a gap opens up. The verdict *must* change here.
TEST(Collision, RectangleRotatedQuarterTurnChangesVerdict) {
    const RigidBody probe = makeBox(Vec2(2.5f, 0.0f), 2.0f, 2.0f);
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 4.0f, 2.0f, 0.0f), probe));
    CHECK(!collides(makeBox(Vec2(0.0f, 0.0f), 4.0f, 2.0f, kPi / 2.0f), probe));
}

// A half turn maps a centred box onto itself.
TEST(Collision, BoxRotatedHalfTurnGivesSameVerdict) {
    const RigidBody probe = makeBox(Vec2(2.5f, 0.0f), 2.0f, 2.0f);
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 4.0f, 2.0f, 0.0f), probe));
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 4.0f, 2.0f, kPi), probe));
}

// --- circle vs box ----------------------------------------------------------

TEST(Collision, CircleAndBoxSeparated) {
    CHECK(!collides(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                    makeBox(Vec2(5.0f, 0.0f), 2.0f, 2.0f)));
}

TEST(Collision, CircleAndBoxOverlapping) {
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 2.0f),
                   makeBox(Vec2(2.0f, 0.0f), 2.0f, 2.0f)));
}

TEST(Collision, CircleInsideBox) {
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 0.5f),
                   makeBox(Vec2(0.0f, 0.0f), 4.0f, 4.0f)));
}

// Zero separation must count as a collision, exactly as it already does for
// circle/circle and box/box.
//
// THIS TEST CURRENTLY FAILS, deliberately, to mark a known bug:
// BoxShape::support breaks the y tie with (localDirection.getY() >= 0), so a
// search direction of exactly (-1, 0) returns the corner (1, 1) instead of the
// face midpoint (1, 0). The first Minkowski point then misses the origin and
// GJK's strict inequalities never recover it.
TEST(Collision, CircleExactlyTangentToBoxFaceCollides) {
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                   makeBox(Vec2(2.0f, 0.0f), 2.0f, 2.0f)));
}

// The neighbours on either side are classified correctly. These pass, which
// localises the failure above to exact contact rather than to the circle/box
// path in general.
TEST(Collision, CircleJustInsideBoxFaceCollides) {
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                   makeBox(Vec2(1.9999f, 0.0f), 2.0f, 2.0f)));
}

TEST(Collision, CircleJustOutsideBoxFaceDoesNotCollide) {
    CHECK(!collides(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                    makeBox(Vec2(2.001f, 0.0f), 2.0f, 2.0f)));
}

// The same zero separation with the geometry mirrored -- box at the origin,
// circle beside it. This one already passes, so once the tie-break is fixed
// the two orientations will agree instead of contradicting each other.
TEST(Collision, MirroredExactTangentialContactCollides) {
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                   makeCircle(Vec2(2.0f, 0.0f), 1.0f)));
}

// Nudged off the shared axis there is no exact tie, and contact at the same
// zero separation is detected.
TEST(Collision, TangentialContactOffTheSharedAxisCollides) {
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                   makeBox(Vec2(2.0f, 0.3f), 2.0f, 2.0f)));
}

// --- properties -------------------------------------------------------------

// Swapping the arguments negates the Minkowski difference, which must not
// change whether it contains the origin.
TEST(Collision, VerdictIsSymmetricInArgumentOrder) {
    struct Pair {
        RigidBody a;
        RigidBody b;
    };
    std::vector<Pair> pairs;
    pairs.push_back({makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                     makeCircle(Vec2(5.0f, 0.0f), 1.0f)});
    pairs.push_back({makeCircle(Vec2(0.0f, 0.0f), 2.0f),
                     makeCircle(Vec2(1.0f, 0.0f), 2.0f)});
    pairs.push_back({makeCircle(Vec2(0.0f, 0.0f), 1.0f),
                     makeCircle(Vec2(2.0f, 0.0f), 1.0f)});
    pairs.push_back({makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f, kPi / 4.0f),
                     makeBox(Vec2(2.2f, 0.0f), 2.0f, 2.0f)});
    pairs.push_back({makeCircle(Vec2(0.0f, 0.0f), 2.0f),
                     makeBox(Vec2(2.0f, 0.0f), 2.0f, 2.0f)});
    pairs.push_back({makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                     makeCircle(Vec2(9.0f, 3.0f), 1.0f)});

    for (const Pair &pair : pairs) {
        CHECK(collides(pair.a, pair.b) == collides(pair.b, pair.a));
    }
}

TEST(Collision, TranslatingBothBodiesEquallyDoesNotChangeVerdict) {
    const Vec2 offset(123.0f, -456.0f);
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 2.0f),
                   makeCircle(Vec2(1.0f, 0.0f), 2.0f)) ==
          collides(makeCircle(Vec2(0.0f, 0.0f) + offset, 2.0f),
                   makeCircle(Vec2(1.0f, 0.0f) + offset, 2.0f)));
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 2.0f, 2.0f),
                   makeBox(Vec2(5.0f, 0.0f), 2.0f, 2.0f)) ==
          collides(makeBox(Vec2(0.0f, 0.0f) + offset, 2.0f, 2.0f),
                   makeBox(Vec2(5.0f, 0.0f) + offset, 2.0f, 2.0f)));
}

TEST(Collision, WorksFarFromTheWorldOrigin) {
    const Vec2 far(1.0e5f, 1.0e5f);
    CHECK(collides(makeCircle(far, 1.0f),
                   makeCircle(far + Vec2(1.0f, 0.0f), 1.0f)));
    CHECK(!collides(makeCircle(far, 1.0f),
                    makeCircle(far + Vec2(50.0f, 0.0f), 1.0f)));
}

// --- degenerate shapes ------------------------------------------------------

TEST(Collision, ZeroRadiusCircleSeparated) {
    CHECK(!collides(makeCircle(Vec2(0.0f, 0.0f), 0.0f),
                    makeCircle(Vec2(5.0f, 0.0f), 1.0f)));
}

TEST(Collision, ZeroRadiusCircleInsideAnother) {
    CHECK(collides(makeCircle(Vec2(0.0f, 0.0f), 0.0f),
                   makeCircle(Vec2(0.0f, 0.0f), 1.0f)));
}

TEST(Collision, ZeroSizeBoxSeparated) {
    CHECK(!collides(makeBox(Vec2(0.0f, 0.0f), 0.0f, 0.0f),
                    makeCircle(Vec2(5.0f, 0.0f), 1.0f)));
}

TEST(Collision, ZeroSizeBoxInsideCircle) {
    CHECK(collides(makeBox(Vec2(0.0f, 0.0f), 0.0f, 0.0f),
                   makeCircle(Vec2(0.0f, 0.0f), 1.0f)));
}
