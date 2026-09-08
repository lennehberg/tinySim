#include "RigidBody.h"
#include "Vec2.h"
#include "World.h"
#include "test_utils.h"

namespace {
constexpr float kEarthGravity = 9.81f;
}  // namespace

// --- body creation ----------------------------------------------------------

TEST(World, CreateBodyReturnsNonNull) {
    World world;
    RigidBody *body = world.createBody(Vec2(1.0f, 2.0f), 0.5f, 3.0f);
    CHECK(body != nullptr);
}

TEST(World, CreateBodyForwardsConstructorArguments) {
    World world;
    RigidBody *body = world.createBody(Vec2(1.0f, 2.0f), 0.5f, 3.0f);
    CHECK_VEC_NEAR(body->getPosition(), 1.0f, 2.0f);
    CHECK_FLOAT_EQ(body->getOrientation(), 0.5f);
    CHECK_FLOAT_EQ(body->getMass(), 3.0f);
}

TEST(World, CreateBodyReturnsDistinctBodies) {
    World world;
    RigidBody *a = world.createBody(Vec2(0.0f, 0.0f), 0.0f, 1.0f);
    RigidBody *b = world.createBody(Vec2(1.0f, 1.0f), 0.0f, 1.0f);
    CHECK(a != b);
    CHECK_VEC_NEAR(a->getPosition(), 0.0f, 0.0f);
    CHECK_VEC_NEAR(b->getPosition(), 1.0f, 1.0f);
}

// The world owns its bodies in a vector of unique_ptr, so pointers handed out
// earlier must stay valid as the vector grows and reallocates.
TEST(World, BodyPointersRemainValidAfterMoreBodiesAreAdded) {
    World world(Vec2(0.0f, 0.0f));
    RigidBody *first = world.createBody(Vec2(7.0f, 8.0f), 0.0f, 1.0f);
    for (int i = 0; i < 64; ++i) {
        world.createBody(Vec2(0.0f, 0.0f), 0.0f, 1.0f);
    }
    CHECK_VEC_NEAR(first->getPosition(), 7.0f, 8.0f);
}

TEST(World, StepOnEmptyWorldIsSafe) {
    World world;
    world.step(0.1f);
}

// --- gravity ----------------------------------------------------------------

TEST(World, DefaultGravityAcceleratesBodyDownward) {
    World world;
    RigidBody *body = world.createBody(Vec2(0.0f, 0.0f), 0.0f, 1.0f);
    world.step(1.0f);
    CHECK_FLOAT_EQ(body->getVelocity().getY(), -kEarthGravity);
}

TEST(World, CustomGravityIsUsed) {
    World world(Vec2(0.0f, -10.0f));
    RigidBody *body = world.createBody(Vec2(0.0f, 0.0f), 0.0f, 1.0f);
    world.step(0.1f);
    CHECK_FLOAT_EQ(body->getVelocity().getY(), -1.0f);
}

TEST(World, HorizontalGravityAcceleratesSideways) {
    World world(Vec2(3.0f, 0.0f));
    RigidBody *body = world.createBody(Vec2(0.0f, 0.0f), 0.0f, 2.0f);
    world.step(1.0f);
    CHECK_FLOAT_EQ(body->getVelocity().getX(), 3.0f);
    CHECK_FLOAT_EQ(body->getVelocity().getY(), 0.0f);
}

// Gravity scales with mass, acceleration does not: a heavy and a light body
// must fall identically.
TEST(World, GravitationalAccelerationIsIndependentOfMass) {
    World world(Vec2(0.0f, -10.0f));
    RigidBody *light = world.createBody(Vec2(), 0.0f, 1.0f);
    RigidBody *heavy = world.createBody(Vec2(), 0.0f, 100.0f);
    world.step(0.1f);
    CHECK_FLOAT_EQ(light->getVelocity().getY(), heavy->getVelocity().getY());
    CHECK_FLOAT_EQ(light->getPosition().getY(), heavy->getPosition().getY());
}

// Gravity acts at the centre of mass, so free fall must never induce spin.
TEST(World, GravityProducesNoRotation) {
    World world;
    RigidBody *body = world.createBody(Vec2(3.0f, 4.0f), 0.0f, 2.0f);
    for (int i = 0; i < 10; ++i) {
        world.step(0.1f);
    }
    CHECK_FLOAT_EQ(body->getAngularVelocity(), 0.0f);
    CHECK_FLOAT_EQ(body->getOrientation(), 0.0f);
}

// Semi-implicit Euler free fall has a closed form:
//   v_n = -g * n * dt
//   y_n = -g * dt^2 * n(n+1)/2
TEST(World, FreeFallMatchesSemiImplicitEulerClosedForm) {
    const float g = 10.0f;
    const float dt = 0.5f;
    const int steps = 2;

    World world(Vec2(0.0f, -g));
    RigidBody *body = world.createBody(Vec2(), 0.0f, 1.0f);
    for (int i = 0; i < steps; ++i) {
        world.step(dt);
    }

    CHECK_FLOAT_EQ(body->getVelocity().getY(), -g * steps * dt);
    CHECK_FLOAT_EQ(body->getPosition().getY(),
                   -g * dt * dt * (steps * (steps + 1)) / 2.0f);
}

TEST(World, StaticBodyIsUnaffectedByGravity) {
    World world;
    RigidBody *body = world.createBody(Vec2(1.0f, 2.0f), 0.0f, 0.0f);
    for (int i = 0; i < 10; ++i) {
        world.step(0.1f);
    }
    CHECK_VEC_NEAR(body->getPosition(), 1.0f, 2.0f);
    CHECK_VEC_NEAR(body->getVelocity(), 0.0f, 0.0f);
}

TEST(World, StaticAndDynamicBodiesCoexist) {
    World world(Vec2(0.0f, -10.0f));
    RigidBody *statik = world.createBody(Vec2(0.0f, 0.0f), 0.0f, 0.0f);
    RigidBody *dynamic = world.createBody(Vec2(0.0f, 0.0f), 0.0f, 1.0f);
    world.step(0.1f);
    CHECK_FLOAT_EQ(statik->getVelocity().getY(), 0.0f);
    CHECK(dynamic->getVelocity().getY() < 0.0f);
}

// --- external forces --------------------------------------------------------

TEST(World, ExternalForceAppliedBeforeStepIsIntegrated) {
    World world(Vec2(0.0f, 0.0f));
    RigidBody *body = world.createBody(Vec2(), 0.0f, 2.0f);
    body->applyForce(Vec2(4.0f, 0.0f), body->getPosition());
    world.step(0.5f);
    CHECK_FLOAT_EQ(body->getVelocity().getX(), 1.0f);
}

// A force exactly opposing weight leaves the body hovering.
TEST(World, ForceCancellingWeightLeavesBodyStationary) {
    World world;
    RigidBody *body = world.createBody(Vec2(), 0.0f, 1.0f);
    body->applyForce(Vec2(0.0f, kEarthGravity), body->getPosition());
    world.step(1.0f);
    CHECK_VEC_NEAR(body->getVelocity(), 0.0f, 0.0f);
    CHECK_VEC_NEAR(body->getPosition(), 0.0f, 0.0f);
}

// Spec invariant: "Forces cleared after each timestep."
TEST(World, StepClearsAccumulatedForces) {
    World world;
    RigidBody *body = world.createBody(Vec2(), 0.0f, 1.0f);
    body->applyForce(Vec2(5.0f, 5.0f), Vec2(1.0f, 1.0f));
    world.step(0.1f);
    CHECK_VEC_NEAR(body->getAccumulatedForce(), 0.0f, 0.0f);
    CHECK_FLOAT_EQ(body->getAccumulatedTorque(), 0.0f);
}

// A one-shot force must not keep accelerating the body on later steps.
TEST(World, ClearedForceDoesNotPersistAcrossSteps) {
    World world(Vec2(0.0f, 0.0f));
    RigidBody *body = world.createBody(Vec2(), 0.0f, 1.0f);
    body->applyForce(Vec2(1.0f, 0.0f), body->getPosition());
    world.step(1.0f);
    const float velocityAfterFirstStep = body->getVelocity().getX();
    world.step(1.0f);
    CHECK_FLOAT_EQ(body->getVelocity().getX(), velocityAfterFirstStep);
}

// --- spec acceptance tests --------------------------------------------------

// Spec Test A, driven through the world with gravity disabled.
TEST(SpecAcceptance, TestA_NoForcesInWorld) {
    World world(Vec2(0.0f, 0.0f));
    RigidBody *body = world.createBody(Vec2(0.0f, 0.0f), 0.0f, 1.0f);
    body->setVelocity(Vec2(1.0f, 0.0f));

    const float dt = 0.5f;
    for (int i = 0; i < 4; ++i) {  // 4 * 0.5 s = 2 s
        world.step(dt);
    }

    CHECK_VEC_NEAR(body->getPosition(), 2.0f, 0.0f);
    CHECK_VEC_NEAR(body->getVelocity(), 1.0f, 0.0f);
}

// Spec Test B: a body starting at rest under gravity (0,-10) must develop a
// negative vertical velocity.
TEST(SpecAcceptance, TestB_GravityMakesVerticalVelocityNegative) {
    World world(Vec2(0.0f, -10.0f));
    RigidBody *body = world.createBody(Vec2(0.0f, 0.0f), 0.0f, 1.0f);
    CHECK_FLOAT_EQ(body->getVelocity().getY(), 0.0f);

    world.step(0.1f);

    CHECK(body->getVelocity().getY() < 0.0f);
    CHECK(body->getPosition().getY() < 0.0f);
}
