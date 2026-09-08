#include <cmath>
#include <memory>
#include <string>

#include "BoxShape.h"
#include "CircleShape.h"
#include "Renderer.h"
#include "RigidBody.h"
#include "Vec2.h"
#include "World.h"
#include "test_utils.h"

namespace {
// Window size used by the fixtures below. Screen y is measured from the top,
// so world y=0 lands on kHeight.
constexpr unsigned kWidth = 800;
constexpr unsigned kHeight = 600;

// Mirrors Renderer::pixelsPerMeter, which is private with no accessor. If that
// default changes, the scale tests below fail and point here.
constexpr float kPixelsPerMeter = 10.0f;

Renderer makeRenderer() {
    // Deliberately not calling init(): no window is created, so these tests
    // stay headless and safe to run anywhere.
    return Renderer(kWidth, kHeight, "test");
}
}  // namespace

// --- lifecycle --------------------------------------------------------------

TEST(Renderer, IsNotOpenBeforeInit) {
    const Renderer renderer = makeRenderer();
    CHECK(!renderer.isOpen());
}

TEST(Renderer, ConstructionDoesNotOpenAWindow) {
    // Constructing must not touch the display; only init() may.
    Renderer renderer = makeRenderer();
    CHECK(!renderer.isOpen());
    renderer.processEvents();
    CHECK(!renderer.isOpen());
}

// --- null-safety guards -----------------------------------------------------

TEST(Renderer, ProcessEventsBeforeInitIsSafe) {
    Renderer renderer = makeRenderer();
    renderer.processEvents();  // must not dereference the null window
}

TEST(Renderer, DrawNullBodyIsSafe) {
    Renderer renderer = makeRenderer();
    renderer.draw(nullptr);
}

TEST(Renderer, DrawBeforeInitIsSafe) {
    Renderer renderer = makeRenderer();
    RigidBody body(Vec2(1.0f, 2.0f), 0.0f, 1.0f);
    renderer.draw(&body);  // no window yet, must bail out rather than crash
}

TEST(Renderer, DrawBodyWithoutShapeIsSafe) {
    Renderer renderer = makeRenderer();
    RigidBody body(Vec2(1.0f, 2.0f), 0.5f, 2.0f);
    CHECK(body.getShape() == nullptr);
    renderer.draw(&body);
}

// --- worldToScreen ----------------------------------------------------------

TEST(Renderer, WorldOriginMapsToBottomLeft) {
    const Renderer renderer = makeRenderer();
    const sf::Vector2f screen = renderer.worldToScreen(Vec2(0.0f, 0.0f));
    CHECK_FLOAT_EQ(screen.x, 0.0f);
    CHECK_FLOAT_EQ(screen.y, static_cast<float>(kHeight));
}

TEST(Renderer, WorldToScreenScalesXByPixelsPerMeter) {
    const Renderer renderer = makeRenderer();
    const sf::Vector2f screen = renderer.worldToScreen(Vec2(1.0f, 0.0f));
    CHECK_FLOAT_EQ(screen.x, kPixelsPerMeter);
    CHECK_FLOAT_EQ(screen.y, static_cast<float>(kHeight));
}

// The core of the conversion: physics y points up, screen y points down.
TEST(Renderer, WorldToScreenFlipsYAxis) {
    const Renderer renderer = makeRenderer();
    const sf::Vector2f screen = renderer.worldToScreen(Vec2(0.0f, 1.0f));
    CHECK_FLOAT_EQ(screen.x, 0.0f);
    CHECK_FLOAT_EQ(screen.y, static_cast<float>(kHeight) - kPixelsPerMeter);
}

TEST(Renderer, WorldToScreenCombinesScaleAndFlip) {
    const Renderer renderer = makeRenderer();
    const sf::Vector2f screen = renderer.worldToScreen(Vec2(2.0f, 3.0f));
    CHECK_FLOAT_EQ(screen.x, 20.0f);
    CHECK_FLOAT_EQ(screen.y, 570.0f);
}

// Rising in the world must mean moving up the screen, i.e. a smaller y.
TEST(Renderer, HigherWorldYGivesSmallerScreenY) {
    const Renderer renderer = makeRenderer();
    const float low = renderer.worldToScreen(Vec2(0.0f, 1.0f)).y;
    const float high = renderer.worldToScreen(Vec2(0.0f, 5.0f)).y;
    CHECK(high < low);
}

TEST(Renderer, FurtherRightWorldXGivesLargerScreenX) {
    const Renderer renderer = makeRenderer();
    const float near = renderer.worldToScreen(Vec2(1.0f, 0.0f)).x;
    const float far = renderer.worldToScreen(Vec2(5.0f, 0.0f)).x;
    CHECK(far > near);
}

TEST(Renderer, WorldToScreenIsLinearInDistance) {
    const Renderer renderer = makeRenderer();
    const sf::Vector2f a = renderer.worldToScreen(Vec2(1.0f, 1.0f));
    const sf::Vector2f b = renderer.worldToScreen(Vec2(2.0f, 2.0f));
    const sf::Vector2f c = renderer.worldToScreen(Vec2(3.0f, 3.0f));
    CHECK_FLOAT_EQ(b.x - a.x, c.x - b.x);
    CHECK_FLOAT_EQ(b.y - a.y, c.y - b.y);
}

// Documents the current origin choice: world (0,0) is the bottom-left corner,
// so anything at negative world x or y falls outside the window.
TEST(Renderer, NegativeWorldCoordinatesFallOutsideTheWindow) {
    const Renderer renderer = makeRenderer();
    const sf::Vector2f screen = renderer.worldToScreen(Vec2(-1.0f, -1.0f));
    CHECK(screen.x < 0.0f);
    CHECK(screen.y > static_cast<float>(kHeight));
}

TEST(Renderer, WindowHeightSetsTheGroundLine) {
    // A shorter window puts world y=0 higher up in pixel terms.
    const Renderer tall(kWidth, 600, "tall");
    const Renderer shortWindow(kWidth, 300, "short");
    CHECK_FLOAT_EQ(tall.worldToScreen(Vec2(0.0f, 0.0f)).y, 600.0f);
    CHECK_FLOAT_EQ(shortWindow.worldToScreen(Vec2(0.0f, 0.0f)).y, 300.0f);
}

// --- worldAngleToScreen -----------------------------------------------------

TEST(Renderer, ZeroAngleIsUnchanged) {
    const Renderer renderer = makeRenderer();
    CHECK_FLOAT_EQ(renderer.worldAngleToScreen(0.0f).asRadians(), 0.0f);
}

// Physics is counter-clockwise positive; flipping y reverses that on screen.
TEST(Renderer, WorldAngleIsNegatedForScreen) {
    const Renderer renderer = makeRenderer();
    const float quarterTurn = 3.14159265f / 2.0f;
    CHECK_FLOAT_EQ(renderer.worldAngleToScreen(quarterTurn).asRadians(),
                   -quarterTurn);
}

TEST(Renderer, NegativeWorldAngleBecomesPositiveOnScreen) {
    const Renderer renderer = makeRenderer();
    CHECK_FLOAT_EQ(renderer.worldAngleToScreen(-1.25f).asRadians(), 1.25f);
}

TEST(Renderer, AngleConversionIsItsOwnInverse) {
    const Renderer renderer = makeRenderer();
    const float angle = 0.7f;
    const float roundTrip =
        renderer.worldAngleToScreen(
                    renderer.worldAngleToScreen(angle).asRadians())
            .asRadians();
    CHECK_FLOAT_EQ(roundTrip, angle);
}

TEST(Renderer, AngleConversionTreatsInputAsRadiansNotDegrees) {
    const Renderer renderer = makeRenderer();
    // 180 degrees expressed in radians must come back as -180 degrees.
    CHECK_NEAR(renderer.worldAngleToScreen(3.14159265f).asDegrees(), -180.0f,
               1e-2f);
}

// --- draw with a shape attached ---------------------------------------------

// Now that setShape() exists, draw() gets past its `!shape` guard and reaches
// the window. With no window these must still bail out rather than crash --
// this is what actually exercises the `!window` check.
TEST(Renderer, DrawCircleBodyBeforeInitIsSafe) {
    Renderer renderer = makeRenderer();
    RigidBody body(Vec2(1.0f, 2.0f), 0.25f, 1.0f);
    body.setShape(std::make_unique<CircleShape>(0.5f));
    CHECK(body.getShape() != nullptr);
    renderer.draw(&body);
}

TEST(Renderer, DrawBoxBodyBeforeInitIsSafe) {
    Renderer renderer = makeRenderer();
    RigidBody body(Vec2(1.0f, 2.0f), 0.25f, 1.0f);
    body.setShape(std::make_unique<BoxShape>(1.0f, 2.0f));
    CHECK(body.getShape() != nullptr);
    renderer.draw(&body);
}

TEST(Renderer, DrawBodyFromWorldWithShapeIsSafe) {
    World world;
    RigidBody *body = world.createBody(Vec2(1.0f, 1.0f), 0.0f, 1.0f);
    body->setShape(std::make_unique<CircleShape>(0.5f));

    Renderer renderer = makeRenderer();
    renderer.draw(body);
}
