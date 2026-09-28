#include "tinysim/dynamics/World.h"
#include "tinysim/dynamics/RigidBody.h"
#include <memory>
#include <vector>

RigidBody *World::createBody(const Vec2 &position, float orientation, float mass) {
    auto body = std::make_unique<RigidBody>(position, orientation, mass); // Create a unique_ptr to a new RigidBody
    RigidBody *bodyPtr = body.get(); // Get raw pointer before moving
    bodies.push_back(std::move(body)); // Move unique_ptr into the vector
    return bodyPtr; // Return the raw pointer to the caller
}

void World::step(float dt) {
    // apply gravity to all dynamic bodies
    for (auto& body : bodies) {
        if (body->getMass() > 0) { // Only apply gravity to dynamic bodies
            Vec2 gravityForce = gravity * body->getMass(); // F = m * g
            body->applyForce(gravityForce, body->getPosition()); // Apply gravity at the center of mass
        }
    }

    for (auto& body : bodies) {
        body->integrate(dt); // Integrate each body's motion
        body->clearForces(); // Clear forces after integration
    }
}
