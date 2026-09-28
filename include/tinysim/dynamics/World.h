#ifndef WORLD_H
#define WORLD_H

#include "tinysim/math/Vec2.h"
#include "tinysim/dynamics/RigidBody.h"
#include <memory>
#include <vector>

class World {
    private:
        std::vector< std::unique_ptr<RigidBody> > bodies; // Using unique_ptr for automatic memory management
        Vec2 gravity; // Gravity vector for the world, can be set to (0, -9.81) for Earth-like gravity
    public:
    World() : bodies(), gravity(0, -9.81f) {} // Default gravity pointing downwards;
    World(const Vec2& gravity) : gravity(gravity) {}

    RigidBody *createBody(const Vec2 &position, float orientation, float mass);
    void step(float dt);
};

#endif // WORLD_H
