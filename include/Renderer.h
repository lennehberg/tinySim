#ifndef RENDERER_H
#define RENDERER_H

#include <SFML/Graphics.hpp>

#include "RigidBody.h"
#include "BoxShape.h"
#include "CircleShape.h"

class Renderer {
public:
    Renderer(unsigned int width, unsigned int height, const std::string& title);

    void init();

    bool isOpen() const;
    void processEvents();

    void draw(const RigidBody* body);
    void display();
    void clear();

    // Pure coordinate conversions. Public so they can be unit tested, and
    // useful later for mapping mouse input back into world space.
    sf::Vector2f worldToScreen(const Vec2& position) const;
    sf::Angle worldAngleToScreen(float radians) const;

private:
    float pixelsPerMeter = 10.0f; // Scale factor for converting simulation units to pixels
    unsigned int width;
    unsigned int height;
    std::string title;

    std::unique_ptr<sf::RenderWindow> window;


    void drawBox(const BoxShape &boxShape, const Vec2 &bodyPosition, float orientation);
    void drawCircle(const CircleShape &circleShape, const Vec2 &bodyPosition, float orientation);
};

#endif // RENDERER_H
