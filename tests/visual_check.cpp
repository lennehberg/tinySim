// Visual check harness: drives the real World and Renderer through a series of
// scenes so motion can be eyeballed. Not part of `make test` -- it needs a
// display and a human. Run with `make visual`.
//
// Each scene builds its own World (gravity is fixed at construction), runs for
// a fixed duration, then the next one starts. The list loops.

#include <SFML/System.hpp>

#include <cstdio>
#include <memory>
#include <vector>

#include "BoxShape.h"
#include "CircleShape.h"
#include "Renderer.h"
#include "RigidBody.h"
#include "Vec2.h"
#include "World.h"

namespace {

constexpr unsigned kWidth = 800;
constexpr unsigned kHeight = 600;
constexpr float kDt = 1.0f / 60.0f;

// Renderer draws at 10 px per metre with world (0,0) at the bottom-left
// corner, so the visible region is about x in [0, 80], y in [0, 60] metres.
// Scenes below keep bodies inside that box.
constexpr float kWorldRight = 80.0f;
constexpr float kWorldTop = 60.0f;

using Bodies = std::vector<RigidBody *>;

RigidBody *addBox(World &world, Bodies &bodies, const Vec2 &position,
                  float width, float height, float mass) {
    RigidBody *body = world.createBody(position, 0.0f, mass);
    body->setShape(std::make_unique<BoxShape>(width, height));
    bodies.push_back(body);
    return body;
}

RigidBody *addCircle(World &world, Bodies &bodies, const Vec2 &position,
                     float radius, float mass) {
    RigidBody *body = world.createBody(position, 0.0f, mass);
    body->setShape(std::make_unique<CircleShape>(radius));
    bodies.push_back(body);
    return body;
}

// A static (mass 0) slab near the bottom, purely as a visual reference.
// There is no collision detection yet, so bodies fall straight through it.
void addGround(World &world, Bodies &bodies) {
    addBox(world, bodies, Vec2(kWorldRight / 2.0f, 1.5f), kWorldRight - 4.0f,
           3.0f, 0.0f);
}

// --- scenes -----------------------------------------------------------------

void setupFreeFall(World &world, Bodies &bodies) {
    addGround(world, bodies);
    addBox(world, bodies, Vec2(40.0f, kWorldTop - 5.0f), 6.0f, 4.0f, 1.0f);
}

void setupProjectile(World &world, Bodies &bodies) {
    addGround(world, bodies);
    RigidBody *box = addBox(world, bodies, Vec2(4.0f, 15.0f), 6.0f, 4.0f, 1.0f);
    box->setVelocity(Vec2(15.0f, 22.0f));
}

void setupSpinInPlace(World &world, Bodies &bodies) {
    RigidBody *box =
        addBox(world, bodies, Vec2(40.0f, 30.0f), 14.0f, 6.0f, 1.0f);
    box->setAngularVelocity(1.5f);
}

void setupTumble(World &world, Bodies &bodies) {
    addGround(world, bodies);
    RigidBody *box = addBox(world, bodies, Vec2(4.0f, 15.0f), 6.0f, 4.0f, 1.0f);
    box->setVelocity(Vec2(15.0f, 22.0f));
    box->setAngularVelocity(3.0f);
}

void setupTorqueKick(World &world, Bodies &bodies) {
    addBox(world, bodies, Vec2(20.0f, 30.0f), 8.0f, 5.0f, 2.0f);
}

// Push on the top edge for the first half second: off-centre, so the body
// picks up both linear and angular velocity, then coasts.
void torqueKickPerFrame(const Bodies &bodies, float elapsed) {
    if (elapsed < 0.5f) {
        RigidBody *box = bodies.front();
        box->applyForce(Vec2(20.0f, 0.0f),
                        box->getPosition() + Vec2(0.0f, 2.5f));
    }
}

void setupMassIndependence(World &world, Bodies &bodies) {
    addGround(world, bodies);
    addBox(world, bodies, Vec2(25.0f, kWorldTop - 5.0f), 6.0f, 4.0f, 1.0f);
    addBox(world, bodies, Vec2(55.0f, kWorldTop - 5.0f), 6.0f, 4.0f, 50.0f);
}

void setupCircleVersusBox(World &world, Bodies &bodies) {
    RigidBody *circle = addCircle(world, bodies, Vec2(22.0f, 30.0f), 6.0f, 1.0f);
    circle->setAngularVelocity(2.0f);
    RigidBody *box = addBox(world, bodies, Vec2(56.0f, 30.0f), 10.0f, 10.0f, 1.0f);
    box->setAngularVelocity(2.0f);
}

void setupSpinBothWays(World &world, Bodies &bodies) {
    // Positive angular velocity is counter-clockwise in physics, and the
    // renderer's y-flip keeps it counter-clockwise on screen, so:
    //   +omega -> spins left,  -omega -> spins right.
    RigidBody *left =
        addBox(world, bodies, Vec2(22.0f, 30.0f), 12.0f, 5.0f, 1.0f);
    left->setAngularVelocity(1.5f);

    RigidBody *right =
        addBox(world, bodies, Vec2(56.0f, 30.0f), 12.0f, 5.0f, 1.0f);
    right->setAngularVelocity(-1.5f);
}

void setupSpinRight(World &world, Bodies &bodies) {
    RigidBody *box =
        addBox(world, bodies, Vec2(40.0f, 30.0f), 14.0f, 6.0f, 1.0f);
    box->setAngularVelocity(-1.5f);
}

void setupTumbleRight(World &world, Bodies &bodies) {
    addGround(world, bodies);
    RigidBody *box = addBox(world, bodies, Vec2(4.0f, 15.0f), 6.0f, 4.0f, 1.0f);
    box->setVelocity(Vec2(15.0f, 22.0f));
    box->setAngularVelocity(-3.0f);
}

void setupOpposingTorques(World &world, Bodies &bodies) {
    addBox(world, bodies, Vec2(15.0f, 42.0f), 8.0f, 5.0f, 2.0f);  // top push
    addBox(world, bodies, Vec2(15.0f, 18.0f), 8.0f, 5.0f, 2.0f);  // bottom push
}

// Identical rightward force on both, but applied above the centre of mass on
// one and below it on the other. Opposite lever arms give opposite torque
// signs, so they spin in opposite directions while drifting together.
void opposingTorquesPerFrame(const Bodies &bodies, float elapsed) {
    if (elapsed >= 0.5f) {
        return;
    }
    RigidBody *upper = bodies[0];
    RigidBody *lower = bodies[1];
    upper->applyForce(Vec2(20.0f, 0.0f),
                      upper->getPosition() + Vec2(0.0f, 2.5f));
    lower->applyForce(Vec2(20.0f, 0.0f),
                      lower->getPosition() + Vec2(0.0f, -2.5f));
}

struct Scene {
    const char *name;
    const char *lookFor;
    Vec2 gravity;
    float duration;
    void (*setup)(World &, Bodies &);
    void (*perFrame)(const Bodies &, float);
};

const Scene kScenes[] = {
    {"free fall",
     "green box drops straight down, speeding up as it goes",
     Vec2(0.0f, -9.81f), 3.3f, setupFreeFall, nullptr},

    {"projectile",
     "box flies across the screen on a parabolic arc",
     Vec2(0.0f, -9.81f), 4.8f, setupProjectile, nullptr},

    {"spin in place",
     "no gravity: box spins LEFT (counter-clockwise), no drifting",
     Vec2(0.0f, 0.0f), 5.0f, setupSpinInPlace, nullptr},

    {"tumbling projectile",
     "box arcs across the screen, tumbling LEFT",
     Vec2(0.0f, -9.81f), 4.8f, setupTumble, nullptr},

    {"off-centre force",
     "brief push above the centre: drifts right, spins RIGHT",
     Vec2(0.0f, 0.0f), 8.0f, setupTorqueKick, torqueKickPerFrame},

    {"mass independence",
     "1 kg and 50 kg boxes fall side by side and stay level",
     Vec2(0.0f, -9.81f), 3.3f, setupMassIndependence, nullptr},

    {"circle vs box",
     "both spin LEFT at the same rate, but the circle looks motionless",
     Vec2(0.0f, 0.0f), 5.0f, setupCircleVersusBox, nullptr},

    {"spin right",
     "no gravity: box spins RIGHT (clockwise) -- negative angular velocity",
     Vec2(0.0f, 0.0f), 5.0f, setupSpinRight, nullptr},

    {"spin both ways",
     "left box spins LEFT, right box spins RIGHT, same rate",
     Vec2(0.0f, 0.0f), 5.0f, setupSpinBothWays, nullptr},

    {"tumbling right",
     "box arcs across the screen, tumbling RIGHT",
     Vec2(0.0f, -9.81f), 4.8f, setupTumbleRight, nullptr},

    {"opposing torques",
     "same push above vs below centre: top box spins RIGHT, bottom spins LEFT",
     Vec2(0.0f, 0.0f), 8.0f, setupOpposingTorques, opposingTorquesPerFrame},
};

constexpr int kSceneCount = static_cast<int>(sizeof(kScenes) / sizeof(kScenes[0]));

}  // namespace

int main() {
    Renderer renderer(kWidth, kHeight, "TinySim2D visual check");
    renderer.init();

    if (!renderer.isOpen()) {
        std::fprintf(stderr, "could not open a window\n");
        return 1;
    }

    std::printf("TinySim2D visual check -- %d scenes, looping.\n", kSceneCount);
    std::printf("Close the window to quit.\n");
    std::printf("Note: there is no collision detection yet, so bodies fall "
                "straight through the static slab.\n\n");

    int sceneIndex = 0;
    while (renderer.isOpen()) {
        const Scene &scene = kScenes[sceneIndex];
        std::printf("[%d/%d] %s\n        look for: %s\n", sceneIndex + 1,
                    kSceneCount, scene.name, scene.lookFor);
        std::fflush(stdout);

        World world(scene.gravity);
        Bodies bodies;
        scene.setup(world, bodies);

        for (float elapsed = 0.0f;
             renderer.isOpen() && elapsed < scene.duration; elapsed += kDt) {
            renderer.processEvents();

            if (scene.perFrame != nullptr) {
                scene.perFrame(bodies, elapsed);
            }
            world.step(kDt);

            renderer.clear();
            for (RigidBody *body : bodies) {
                renderer.draw(body);
            }
            renderer.display();

            // Renderer::init() sets no framerate limit, so pace the loop here
            // to keep the simulation running at roughly real time.
            sf::sleep(sf::seconds(kDt));
        }

        sceneIndex = (sceneIndex + 1) % kSceneCount;
    }

    return 0;
}
