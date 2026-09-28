#include <memory>

#include "tinysim/geometry/BoxShape.h"
#include "tinysim/geometry/CircleShape.h"
#include "tinysim/dynamics/RigidBody.h"
#include "tinysim/math/Vec2.h"
#include "support/test_utils.h"

// --- construction -----------------------------------------------------------

TEST(RigidBody, DefaultConstructorIsAtRestAndStatic) {
    RigidBody body;
    CHECK_VEC_NEAR(body.getPosition(), 0.0f, 0.0f);
    CHECK_VEC_NEAR(body.getVelocity(), 0.0f, 0.0f);
    CHECK_FLOAT_EQ(body.getOrientation(), 0.0f);
    CHECK_FLOAT_EQ(body.getAngularVelocity(), 0.0f);
    CHECK_FLOAT_EQ(body.getMass(), 0.0f);
    CHECK_VEC_NEAR(body.getAccumulatedForce(), 0.0f, 0.0f);
    CHECK_FLOAT_EQ(body.getAccumulatedTorque(), 0.0f);
}

TEST(RigidBody, ConstructorStoresPoseAndMass) {
    RigidBody body(Vec2(1.0f, 2.0f), 0.5f, 2.0f);
    CHECK_VEC_NEAR(body.getPosition(), 1.0f, 2.0f);
    CHECK_FLOAT_EQ(body.getOrientation(), 0.5f);
    CHECK_FLOAT_EQ(body.getMass(), 2.0f);
}

TEST(RigidBody, ConstructorStartsAtRest) {
    RigidBody body(Vec2(1.0f, 2.0f), 0.5f, 2.0f);
    CHECK_VEC_NEAR(body.getVelocity(), 0.0f, 0.0f);
    CHECK_FLOAT_EQ(body.getAngularVelocity(), 0.0f);
}

// Spec invariant: dynamic bodies have mass > 0. Inertia is currently a
// placeholder (mass * 1) until shapes exist; this pins the current behaviour
// so a real inertia calculation shows up as a deliberate test change.
TEST(RigidBody, DynamicBodyHasPositiveInertia) {
    RigidBody body(Vec2(), 0.0f, 2.0f);
    CHECK(body.getInertia() > 0.0f);
    CHECK_FLOAT_EQ(body.getInertia(), 2.0f);
}

// Spec invariant: a static body has mass 0 (and therefore zero inertia).
TEST(RigidBody, StaticBodyHasZeroInertia) {
    RigidBody body(Vec2(), 0.0f, 0.0f);
    CHECK_FLOAT_EQ(body.getMass(), 0.0f);
    CHECK_FLOAT_EQ(body.getInertia(), 0.0f);
}

// --- setters ----------------------------------------------------------------

TEST(RigidBody, SetVelocity) {
    RigidBody body(Vec2(), 0.0f, 1.0f);
    body.setVelocity(Vec2(1.0f, -2.0f));
    CHECK_VEC_NEAR(body.getVelocity(), 1.0f, -2.0f);
}

TEST(RigidBody, SetAngularVelocity) {
    RigidBody body(Vec2(), 0.0f, 1.0f);
    body.setAngularVelocity(1.5f);
    CHECK_FLOAT_EQ(body.getAngularVelocity(), 1.5f);
}

// --- setShape ---------------------------------------------------------------

namespace {
// Shape that reports its own destruction, so ownership can be observed.
struct ProbeShape : Shape {
    bool *destroyed;
    explicit ProbeShape(bool *flag) : destroyed(flag) {
        setType(ShapeType::CIRCLE);
    }
    ~ProbeShape() override { *destroyed = true; }
    float calculateInertia(float mass) const override { return mass; }
    Vec2 support(const Vec2 &, const Vec2 &position, float) const override {
        return position;
    }
};
}  // namespace

TEST(RigidBody, SetShapeStoresShape) {
    RigidBody body(Vec2(), 0.0f, 2.0f);
    auto circle = std::make_unique<CircleShape>(3.0f);
    const Shape *expected = circle.get();
    body.setShape(std::move(circle));
    CHECK(body.getShape() == expected);
}

// Attaching a shape must replace the constructor's placeholder inertia with
// the shape's real formula: circle I = 1/2 m r^2 = 0.5 * 2 * 9 = 9.
TEST(RigidBody, SetShapeRecomputesInertiaForCircle) {
    RigidBody body(Vec2(), 0.0f, 2.0f);
    CHECK_FLOAT_EQ(body.getInertia(), 2.0f);  // placeholder before
    body.setShape(std::make_unique<CircleShape>(3.0f));
    CHECK_FLOAT_EQ(body.getInertia(), 9.0f);
}

// Box I = 1/12 m (w^2 + h^2) = (1/12) * 2 * (4 + 16) = 10/3.
TEST(RigidBody, SetShapeRecomputesInertiaForBox) {
    RigidBody body(Vec2(), 0.0f, 2.0f);
    body.setShape(std::make_unique<BoxShape>(2.0f, 4.0f));
    CHECK_FLOAT_EQ(body.getInertia(), 10.0f / 3.0f);
}

TEST(RigidBody, SetShapeOverwritesPreviousShape) {
    RigidBody body(Vec2(), 0.0f, 2.0f);
    body.setShape(std::make_unique<CircleShape>(3.0f));
    auto box = std::make_unique<BoxShape>(2.0f, 4.0f);
    const Shape *expected = box.get();
    body.setShape(std::move(box));
    CHECK(body.getShape() == expected);
    CHECK_FLOAT_EQ(body.getInertia(), 10.0f / 3.0f);
}

TEST(RigidBody, SetShapeOnStaticBodyLeavesZeroInertia) {
    RigidBody body(Vec2(), 0.0f, 0.0f);
    body.setShape(std::make_unique<CircleShape>(3.0f));
    CHECK_FLOAT_EQ(body.getInertia(), 0.0f);
}

// The real check that inverseInertia was updated alongside Inertia: a torque
// must produce alpha = tau / I using the *shape's* inertia.
// I = 9, r = (0,1), F = (4,0) => tau = -4 => alpha = -4/9.
// dt = 0.5 => omega = -2/9, theta = -1/9.
TEST(RigidBody, SetShapeChangesAngularResponseToTorque) {
    RigidBody body(Vec2(), 0.0f, 2.0f);
    body.setShape(std::make_unique<CircleShape>(3.0f));

    body.applyForce(Vec2(4.0f, 0.0f), Vec2(0.0f, 1.0f));
    body.integrate(0.5f);

    CHECK_FLOAT_EQ(body.getAngularVelocity(), -2.0f / 9.0f);
    CHECK_FLOAT_EQ(body.getOrientation(), -1.0f / 9.0f);
}

// A bigger shape resists rotation more, for the same mass and torque.
TEST(RigidBody, LargerShapeRotatesMoreSlowlyUnderSameTorque) {
    RigidBody small(Vec2(), 0.0f, 2.0f);
    RigidBody large(Vec2(), 0.0f, 2.0f);
    small.setShape(std::make_unique<CircleShape>(1.0f));
    large.setShape(std::make_unique<CircleShape>(4.0f));

    small.applyForce(Vec2(4.0f, 0.0f), Vec2(0.0f, 1.0f));
    large.applyForce(Vec2(4.0f, 0.0f), Vec2(0.0f, 1.0f));
    small.integrate(0.5f);
    large.integrate(0.5f);

    CHECK(std::fabs(large.getAngularVelocity()) <
          std::fabs(small.getAngularVelocity()));
}

// --- shape ownership --------------------------------------------------------

// The point of the unique_ptr: the body owns the shape and frees it.
TEST(RigidBody, BodyDestroysItsShape) {
    bool destroyed = false;
    {
        RigidBody body(Vec2(), 0.0f, 1.0f);
        body.setShape(std::make_unique<ProbeShape>(&destroyed));
        CHECK(!destroyed);
    }
    CHECK(destroyed);
}

TEST(RigidBody, ReplacingShapeDestroysThePreviousOne) {
    bool firstDestroyed = false;
    RigidBody body(Vec2(), 0.0f, 1.0f);
    body.setShape(std::make_unique<ProbeShape>(&firstDestroyed));
    CHECK(!firstDestroyed);

    body.setShape(std::make_unique<CircleShape>(1.0f));
    CHECK(firstDestroyed);
}

// Detaching used to segfault when shape was a raw pointer; owning it makes
// this well defined, falling back to the placeholder inertia.
TEST(RigidBody, SetShapeNullptrDetachesShapeSafely) {
    bool destroyed = false;
    RigidBody body(Vec2(), 0.0f, 2.0f);
    body.setShape(std::make_unique<ProbeShape>(&destroyed));

    body.setShape(nullptr);

    CHECK(destroyed);
    CHECK(body.getShape() == nullptr);
    CHECK_FLOAT_EQ(body.getInertia(), 2.0f);
}

// --- applyForce -------------------------------------------------------------

TEST(RigidBody, ApplyForceAtCenterOfMassProducesNoTorque) {
    RigidBody body(Vec2(1.0f, 2.0f), 0.0f, 1.0f);
    body.applyForce(Vec2(3.0f, 4.0f), body.getPosition());
    CHECK_VEC_NEAR(body.getAccumulatedForce(), 3.0f, 4.0f);
    CHECK_FLOAT_EQ(body.getAccumulatedTorque(), 0.0f);
}

TEST(RigidBody, ApplyForceOffsetFromCenterProducesTorque) {
    RigidBody body(Vec2(), 0.0f, 1.0f);
    // r = (0,1), F = (1,0)  =>  tau = cross(r, F) = 0*0 - 1*1 = -1
    body.applyForce(Vec2(1.0f, 0.0f), Vec2(0.0f, 1.0f));
    CHECK_VEC_NEAR(body.getAccumulatedForce(), 1.0f, 0.0f);
    CHECK_FLOAT_EQ(body.getAccumulatedTorque(), -1.0f);
}

TEST(RigidBody, TorqueIsMeasuredRelativeToBodyPosition) {
    // Same world-space application point, but the body sits on it, so the
    // lever arm is zero.
    RigidBody body(Vec2(0.0f, 1.0f), 0.0f, 1.0f);
    body.applyForce(Vec2(1.0f, 0.0f), Vec2(0.0f, 1.0f));
    CHECK_FLOAT_EQ(body.getAccumulatedTorque(), 0.0f);
}

TEST(RigidBody, ForcesAccumulateAcrossCalls) {
    RigidBody body(Vec2(), 0.0f, 1.0f);
    body.applyForce(Vec2(1.0f, 0.0f), Vec2());
    body.applyForce(Vec2(0.0f, 2.0f), Vec2());
    body.applyForce(Vec2(-0.5f, 0.5f), Vec2());
    CHECK_VEC_NEAR(body.getAccumulatedForce(), 0.5f, 2.5f);
}

TEST(RigidBody, OpposingForcesCancel) {
    RigidBody body(Vec2(), 0.0f, 1.0f);
    body.applyForce(Vec2(5.0f, -3.0f), Vec2());
    body.applyForce(Vec2(-5.0f, 3.0f), Vec2());
    CHECK_VEC_NEAR(body.getAccumulatedForce(), 0.0f, 0.0f);
}

TEST(RigidBody, ClearForcesResetsForceAndTorque) {
    RigidBody body(Vec2(), 0.0f, 1.0f);
    body.applyForce(Vec2(1.0f, 0.0f), Vec2(0.0f, 1.0f));
    body.clearForces();
    CHECK_VEC_NEAR(body.getAccumulatedForce(), 0.0f, 0.0f);
    CHECK_FLOAT_EQ(body.getAccumulatedTorque(), 0.0f);
}

// --- integrate --------------------------------------------------------------

TEST(RigidBody, IntegrateWithNoForceAndNoVelocityDoesNothing) {
    RigidBody body(Vec2(1.0f, 2.0f), 0.25f, 1.0f);
    body.integrate(0.5f);
    CHECK_VEC_NEAR(body.getPosition(), 1.0f, 2.0f);
    CHECK_VEC_NEAR(body.getVelocity(), 0.0f, 0.0f);
    CHECK_FLOAT_EQ(body.getOrientation(), 0.25f);
}

// Semi-implicit Euler: velocity is updated first, then position uses the
// *new* velocity. m = 2, F = (4,0) => a = 2; dt = 0.5 => v = 1, x = 0.5.
TEST(RigidBody, IntegrateUsesSemiImplicitEuler) {
    RigidBody body(Vec2(), 0.0f, 2.0f);
    body.applyForce(Vec2(4.0f, 0.0f), Vec2());
    body.integrate(0.5f);
    CHECK_VEC_NEAR(body.getVelocity(), 1.0f, 0.0f);
    CHECK_VEC_NEAR(body.getPosition(), 0.5f, 0.0f);
}

TEST(RigidBody, IntegrateScalesAccelerationByInverseMass) {
    RigidBody light(Vec2(), 0.0f, 1.0f);
    RigidBody heavy(Vec2(), 0.0f, 4.0f);
    light.applyForce(Vec2(4.0f, 0.0f), Vec2());
    heavy.applyForce(Vec2(4.0f, 0.0f), Vec2());
    light.integrate(1.0f);
    heavy.integrate(1.0f);
    // Same force, four times the mass => a quarter of the velocity.
    CHECK_FLOAT_EQ(light.getVelocity().getX(), 4.0f);
    CHECK_FLOAT_EQ(heavy.getVelocity().getX(), 1.0f);
}

TEST(RigidBody, IntegrateAdvancesOrientationFromTorque) {
    // m = 2 => I = 2. r = (0,1), F = (4,0) => tau = -4 => alpha = -2.
    // dt = 0.5 => omega = -1, theta = -0.5.
    RigidBody body(Vec2(), 0.0f, 2.0f);
    body.applyForce(Vec2(4.0f, 0.0f), Vec2(0.0f, 1.0f));
    body.integrate(0.5f);
    CHECK_FLOAT_EQ(body.getAngularVelocity(), -1.0f);
    CHECK_FLOAT_EQ(body.getOrientation(), -0.5f);
}

TEST(RigidBody, IntegrateCarriesExistingVelocityWithZeroForce) {
    RigidBody body(Vec2(), 0.0f, 1.0f);
    body.setVelocity(Vec2(2.0f, -1.0f));
    body.integrate(0.5f);
    CHECK_VEC_NEAR(body.getVelocity(), 2.0f, -1.0f);
    CHECK_VEC_NEAR(body.getPosition(), 1.0f, -0.5f);
}

TEST(RigidBody, IntegrateCarriesExistingAngularVelocity) {
    RigidBody body(Vec2(), 0.0f, 1.0f);
    body.setAngularVelocity(2.0f);
    body.integrate(0.25f);
    CHECK_FLOAT_EQ(body.getAngularVelocity(), 2.0f);
    CHECK_FLOAT_EQ(body.getOrientation(), 0.5f);
}

TEST(RigidBody, StaticBodyDoesNotMoveUnderForce) {
    RigidBody body(Vec2(1.0f, 2.0f), 0.0f, 0.0f);
    body.applyForce(Vec2(1000.0f, 1000.0f), Vec2(5.0f, 5.0f));
    body.integrate(1.0f);
    CHECK_VEC_NEAR(body.getPosition(), 1.0f, 2.0f);
    CHECK_VEC_NEAR(body.getVelocity(), 0.0f, 0.0f);
    CHECK_FLOAT_EQ(body.getOrientation(), 0.0f);
    CHECK_FLOAT_EQ(body.getAngularVelocity(), 0.0f);
}

TEST(RigidBody, IntegrateWithZeroDtIsANoOp) {
    RigidBody body(Vec2(1.0f, 2.0f), 0.0f, 1.0f);
    body.setVelocity(Vec2(3.0f, 4.0f));
    body.applyForce(Vec2(10.0f, 10.0f), Vec2());
    body.integrate(0.0f);
    CHECK_VEC_NEAR(body.getPosition(), 1.0f, 2.0f);
    CHECK_VEC_NEAR(body.getVelocity(), 3.0f, 4.0f);
}

// integrate() deliberately leaves the accumulator alone; World::step() is what
// clears it once per timestep. Pinning this stops the responsibility from
// silently moving.
TEST(RigidBody, IntegrateDoesNotClearAccumulatedForce) {
    RigidBody body(Vec2(), 0.0f, 1.0f);
    body.applyForce(Vec2(1.0f, 2.0f), Vec2());
    body.integrate(0.5f);
    CHECK_VEC_NEAR(body.getAccumulatedForce(), 1.0f, 2.0f);
}

// --- spec acceptance tests --------------------------------------------------

// Spec Test A: no forces. Position (0,0), velocity (1,0); after 2 s the body
// should be at (2,0) still moving at (1,0).
TEST(SpecAcceptance, TestA_NoForcesConstantVelocity) {
    RigidBody body(Vec2(0.0f, 0.0f), 0.0f, 1.0f);
    body.setVelocity(Vec2(1.0f, 0.0f));

    const float dt = 0.5f;
    for (int i = 0; i < 4; ++i) {  // 4 * 0.5 s = 2 s
        body.integrate(dt);
    }

    CHECK_VEC_NEAR(body.getPosition(), 2.0f, 0.0f);
    CHECK_VEC_NEAR(body.getVelocity(), 1.0f, 0.0f);
}

// Spec Test C: force through the centre of mass produces linear acceleration
// and no angular acceleration.
TEST(SpecAcceptance, TestC_ForceThroughCenterOfMass) {
    RigidBody body(Vec2(), 0.0f, 2.0f);
    body.applyForce(Vec2(4.0f, 0.0f), body.getPosition());
    body.integrate(0.5f);

    CHECK(body.getVelocity().getX() > 0.0f);
    CHECK_FLOAT_EQ(body.getVelocity().getX(), 1.0f);
    CHECK_FLOAT_EQ(body.getAngularVelocity(), 0.0f);
    CHECK_FLOAT_EQ(body.getOrientation(), 0.0f);
}

// Spec Test D: force offset from the centre of mass produces both linear and
// angular acceleration.
TEST(SpecAcceptance, TestD_ForceOffsetFromCenterOfMass) {
    RigidBody body(Vec2(), 0.0f, 2.0f);
    body.applyForce(Vec2(4.0f, 0.0f), Vec2(0.0f, 1.0f));
    body.integrate(0.5f);

    CHECK(body.getVelocity().getX() > 0.0f);
    CHECK(body.getAngularVelocity() != 0.0f);
    CHECK_FLOAT_EQ(body.getVelocity().getX(), 1.0f);
    CHECK_FLOAT_EQ(body.getAngularVelocity(), -1.0f);
}

// The linear part of an offset force matches the centred case exactly: only
// the rotation differs.
TEST(SpecAcceptance, OffsetForceMatchesCenteredForceLinearly) {
    RigidBody centered(Vec2(), 0.0f, 2.0f);
    RigidBody offset(Vec2(), 0.0f, 2.0f);
    centered.applyForce(Vec2(4.0f, 0.0f), Vec2(0.0f, 0.0f));
    offset.applyForce(Vec2(4.0f, 0.0f), Vec2(0.0f, 1.0f));
    centered.integrate(0.5f);
    offset.integrate(0.5f);

    CHECK_FLOAT_EQ(centered.getVelocity().getX(), offset.getVelocity().getX());
    CHECK_FLOAT_EQ(centered.getAngularVelocity(), 0.0f);
    CHECK(offset.getAngularVelocity() != 0.0f);
}
