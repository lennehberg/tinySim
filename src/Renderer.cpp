#include "../include/Renderer.h"
#include "RigidBody.h"
#include "Shape.h"
#include "Vec2.h"
#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

Renderer::Renderer(unsigned int width, unsigned int height, const std::string &title) :
    width(width), height(height), title(title) {
    // Initialize rendering context (e.g., OpenGL, DirectX, etc.)
}

void Renderer::init() {
    // Initialize the rendering window using SFML
    window = std::make_unique<sf::RenderWindow>(sf::VideoMode({width, height}), title);
    window->clear();
}

bool Renderer::isOpen() const {
    return window && window->isOpen();
}

void Renderer::processEvents() {
    if (!window) return;

    while (const std::optional event = window->pollEvent()) {
        if (event->is<sf::Event::Closed>()) {
            window->close();
        }
    }
}

void Renderer::display() {
    if (window) {
        window->display();
    }
}

void Renderer::clear() {
    if (window) {
        window->clear(sf::Color::Black);
    }
}

// Convert physics-world coordinates into screen coordinates.
//
// Physics:
//      +y points upward
//
// SFML:
//      +y points downward
sf::Vector2f Renderer::worldToScreen(const Vec2& position) const
{
    return {
        position.getX() * pixelsPerMeter,
        static_cast<float>(height) -
            position.getY() * pixelsPerMeter
    };
}


// Physics uses positive counter-clockwise rotation.
// Flipping the y-axis reverses that visually on the screen.
sf::Angle Renderer::worldAngleToScreen(float radians) const
{
    return sf::radians(-radians);
}


void Renderer::drawBox(const BoxShape& boxShape,
                       const Vec2& bodyPosition,
                       float orientation)
{
    const float rectWidth =
        boxShape.getWidth() * pixelsPerMeter;

    const float rectHeight =
        boxShape.getHeight() * pixelsPerMeter;

    sf::RectangleShape rectangle(
        sf::Vector2f(rectWidth, rectHeight)
    );

    rectangle.setFillColor(sf::Color::Green);

    // Make the center of the rectangle its local origin.
    // This makes SFML rotate the box around its center.
    rectangle.setOrigin({
        rectWidth / 2.0f,
        rectHeight / 2.0f
    });

    rectangle.setPosition(
        worldToScreen(bodyPosition)
    );

    rectangle.setRotation(
        worldAngleToScreen(orientation)
    );

    window->draw(rectangle);
}


void Renderer::drawCircle(const CircleShape& circleShape,
                          const Vec2& bodyPosition,
                          float orientation)
{
    const float radius =
        circleShape.getRadius() * pixelsPerMeter;

    sf::CircleShape circle(radius);

    circle.setFillColor(sf::Color::Blue);

    // SFML's circle local coordinates go from roughly
    // (0,0) to (2r,2r), so its center is (r,r).
    circle.setOrigin({
        radius,
        radius
    });

    circle.setPosition(
        worldToScreen(bodyPosition)
    );

    circle.setRotation(
        worldAngleToScreen(orientation)
    );

    window->draw(circle);
}

void Renderer::draw(const RigidBody *body){
   if (!window || !body) return;

   Shape *shape = body->getShape();
   if (!shape) return;

   switch (shape->getType()) {
       case ShapeType::RECTANGLE: {
           BoxShape *boxShape = dynamic_cast<BoxShape*>(shape);
           if (boxShape) {
               drawBox(*boxShape, body->getPosition(), body->getOrientation());
           }
           break;
       }
       case ShapeType::CIRCLE: {
           CircleShape *circleShape = dynamic_cast<CircleShape*>(shape);
           if (circleShape) {
               drawCircle(*circleShape, body->getPosition(), body->getOrientation());
           }
           break;
       }
       default:
           // Handle other shape types or do nothing
           break;
   }

}
