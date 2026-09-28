#include "tinysim/dynamics/RigidBody.h"
#include "tinysim/math/Vec2.h"
#include "tinysim/geometry/Shape.h"
#include <memory>

RigidBody::RigidBody() : position(), orientation(0), velocity(), angularVelocity(), mass(0), Inertia(0), accumulatedForce(), accumulatedTorque(0), inverseMass(0), inverseInertia(0), shape(nullptr) {}

RigidBody::RigidBody(const Vec2& position, const float orientation, float mass)
    : position(position), orientation(orientation), velocity(), angularVelocity(), mass(mass), Inertia(0), accumulatedForce(), accumulatedTorque(0), shape(nullptr) {
    if (mass > 0) { // for dynamic bodies
        inverseMass = 1.0f / mass;
        // No shape attached yet, so fall back to a unit-inertia placeholder.
        // TODO: require a shape once createBody() can take one.
        Inertia = (shape != nullptr) ? shape->calculateInertia(mass) : mass;
        inverseInertia = 1.0f / Inertia;
    } else { // for static bodies
        inverseMass = 0;
        inverseInertia = 0;
    }
}

void RigidBody::setShape(std::unique_ptr<Shape> shape) {
    this->shape = std::move(shape);
    // Query the member, not the parameter: after the move the parameter is
    // null, so testing it would always take the fallback branch.
    Inertia = (this->shape != nullptr) ? this->shape->calculateInertia(mass)
                                       : mass;
    inverseInertia = (Inertia > 0) ? 1.0f / Inertia : 0;
}

void RigidBody::applyForce(const Vec2& force, const Vec2 &point) {
    accumulatedForce += force;
    // Calculate torque: torque = r x F, where r is the vector from the center of mass to the point of application
    Vec2 r = point - position; // Vector from center of mass to point of force application
    accumulatedTorque += cross(r, force); // cross product gives the torque
}

void RigidBody::integrate(float dt) {
    if (inverseMass > 0) { // Only integrate if the body is dynamic
        // Update linear motion
        Vec2 acceleration = accumulatedForce * inverseMass; // a = F/m
        velocity += acceleration * dt;
        position += velocity * dt;

        // Update angular motion
        float angularAcceleration = accumulatedTorque * inverseInertia; // α = τ/I
        angularVelocity += angularAcceleration * dt;
        orientation += angularVelocity * dt;
    }
}
