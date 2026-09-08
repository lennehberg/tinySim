#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include "Vec2.h"
#include "Shape.h"
#include <memory>

class RigidBody {
    // Class definition for a rigid body in the simulation
    private:
        Vec2 position; // Position of the rigid body
        float orientation; // Orientation of the rigid body
        Vec2 velocity; // Velocity of the rigid body
        float angularVelocity; // Angular velocity of the rigid body
        float mass; // Mass of the rigid body
        float Inertia; // Moment of inertia of the rigid body

        Vec2 accumulatedForce; // Accumulated force acting on the rigid body
        float accumulatedTorque; // Accumulated torque acting on the rigid body

        float inverseMass; // Inverse of the mass (for efficiency)
        float inverseInertia; // Inverse of the moment of inertia (for efficiency)

        // TODO: Implement shape handling for the rigid body
        std::unique_ptr<Shape> shape; // Pointer to the shape of the rigid body

    public:
        RigidBody();
        RigidBody(const Vec2& position, const float orientation, float mass);

        void setShape(std::unique_ptr<Shape> shape);

        Shape *getShape() const { return shape.get(); }

        const Vec2& getPosition() const { return position; }
        const Vec2& getAccumulatedForce() const { return accumulatedForce; }
        const Vec2& getVelocity() const { return velocity; }
        float getAngularVelocity() const { return angularVelocity; }
        float getOrientation() const { return orientation; }
        float getMass() const { return mass; }
        float getInertia() const { return Inertia; }
        float getAccumulatedTorque() const { return accumulatedTorque; }

        void setVelocity(const Vec2& velocity) { this->velocity = velocity; }
        void setAngularVelocity(float angularVelocity) { this->angularVelocity = angularVelocity; }

        void applyForce(const Vec2& force, const Vec2& point);
        void integrate(float dt);
        // void translateShape(Se(2) translation); // Placeholder for shape translation (not implemented yet)
        void clearForces() { accumulatedForce = Vec2(0, 0); accumulatedTorque = 0; }
};

#endif // RIGID_BODY_H
