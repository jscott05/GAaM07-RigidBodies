#include "RigidBody.h"
#include "Vector2Utils.h"
#include <algorithm>

using namespace PhysicsUtils;

RigidBody::RigidBody(sf::Vector2f pos, float r, float m, sf::Color col, bool stat = false)
    :position(pos), 
    radius(r), 
    mass(m), 
    colour(col), 
    isStatic(stat),
    velocity(0.0f, 0.0f), 
    acceleration(0.0f, 0.0f),
    rotation(0.0f),
    angularVelocity(0.0f),
    angularAcceleration(0.0f),
    restitution(0.8f),
    friction(0.0f),
    trailTimer(0.0f),
    impactIntensity(0.0f),
    squashStretch(1.0f),
    appliedForce(0.0f,0.0f),
    isResting(false)
{
    inertia = 0.05f * mass * radius * radius;
}

void RigidBody::applyForce(const sf::Vector2f& force)
{
    if (!isStatic && !isResting)
    {
        acceleration += force / mass;
        appliedForce = force;
    }
}

void RigidBody::applyForceAtPoint(const sf::Vector2f& force, const sf::Vector2f& point)
{
    // apply force at set point (not at centre of mass) creating linear and angular acceleration
    // torque generation results from force being applied off centre
    // torgue = r * f (cross product of radius vector and force;
    if (!isStatic)
    {
        acceleration += force / mass;
        sf::Vector2f r = point - position;
        float torque = cross(r, force);
    }
}

void RigidBody::applyTorque(float torque)
{
    // torque is a rotation force
    // angular analog of linear force
    // causes angular acceleration (rotational equivalent of f=ma)
    // t=ia (torque = moment of inertia * angular acceleration
    if (!isStatic)
    {
        float velocityMagnitude = length(velocity);
        float accelrationMagnitude = length(acceleration);

        // missed this

        angularAcceleration = +torque / inertia;
    }
}

void RigidBody::update(float deltaTime)
{
    if (!isStatic)
    {
        velocity += acceleration * deltaTime;
        position += velocity * deltaTime;

        velocity *= 0.99f;

        // rotation
        rotation = 0.0f;
        angularVelocity = 0.0f;
        angularAcceleration = 0.0f;

        // calculate trail
        trailTimer += deltaTime;
        if (trailTimer >= TRAIL_UPDATE_INTERVAL)
        {
            trailTimer = 0.0f;
            motionTrail.push_front({ position, 1.0f });
            if (motionTrail.size() > MAX_TRAIL_LENGTH)
            {
                motionTrail.pop_back();
            }
        }

        for (auto& trail : motionTrail)
        {
            trail.alpha *= 0.55;
        }

        impactIntensity *= 0.9f;
        squashStretch += (1.0f - squashStretch) * 0.2f;
        
        acceleration = sf::Vector2f(0.0f, 0.0f);
        angularAcceleration = 0.0f;
        appliedForce = sf::Vector2f(0.0f, 0.0f);
    }
}

void RigidBody::checkBoundaryCollision(float width, float height)
{
    if (!isStatic)
    {

        bool collided = false;
        sf::Vector2f normal(0.0f, 0.0f);


        //check collision with boundaries

        //horizontal
        if (position.x - radius < 0)
        {
            position.x = radius;
            velocity.x = -velocity.x * restitution;

            //friction
            velocity.y *= (1.0f - friction);

            angularVelocity *= (1.0f - friction);
            normal = sf::Vector2f(1.0f, 0.0f);

            collided = true;
        }
        if (position.x + radius > width) // if position.x is outside of screen width
        {
            position.x = width - radius;
            velocity.x = -velocity.x * restitution;

            velocity.y *= (1.0f - friction);


            angularVelocity *= (1.0f - friction);
            normal = sf::Vector2f(-1.0f, 0.0f);

            collided = true;
        }

        //vertical
        if (position.y - radius < 0)
        {
            position.y = radius;
            velocity.y = -velocity.y * restitution;

            //friction
            velocity.x *= (1.0f - friction);

            angularVelocity *= (1.0f - friction);
            normal = sf::Vector2f(0.0f, 1.0f);

            collided = true;
        }
        if (position.y + radius > height) // if position.y is outside of screen height
        {
            position.y = height - radius;
            velocity.y = -velocity.y * restitution;

            velocity.x *= (1.0f - friction);

            angularVelocity *= (1.0f - friction);
            normal = sf::Vector2f(0.0f, -1.0f);

            collided = true;
        }

        if (collided)
        {
            float intensity = length(velocity) / 100.0f;
            impactIntensity = std::min(1.0f, intensity);
        }
    }
}

void RigidBody::draw(sf::RenderWindow& window, bool showVelocity)
{
    sf::Color displayColour = colour;

    float flashIntensity = impactIntensity * 100.0f;

    displayColour.r = std::min(255, static_cast<int>(displayColour.r + flashIntensity));
    displayColour.g = std::min(255, static_cast<int>(displayColour.g+ flashIntensity));
    displayColour.b = std::min(255, static_cast<int>(displayColour.b + flashIntensity));

    if (!isStatic)
    {
        for (int i = 3; i > 0; --i)
        {
            float glowRadius = radius + (i * 4.0f) + (impactIntensity * 5);
            float alpha = 20.0f / (i + 1) + (impactIntensity * 30.f);

            sf::CircleShape glow(glowRadius);
            glow.setPosition(position - sf::Vector2f(glowRadius, glowRadius));
            glow.setFillColor(sf::Color(displayColour.r, displayColour.g, displayColour.b, static_cast<uint8_t>(alpha)));

            window.draw(glow); // flawed - to be efficient need to reduce draw calls - buffer would be smarter, then draw the buffer | better to implement working concepts prior to optimisation
            // dont get caught up with optimisation before it works, fix it later, once it works
        }
    }

    sf::Vector2f scale(1.0f, 1.0f);
    if (impactIntensity > 0.01f)
    {
        float sqaushAmount = 1.0f - (impactIntensity * 0.3f);
        float stretchAmount = 1.0f + (impactIntensity * 3.0f);

        sf::Vector2f velocityDir = length(velocity) > 0.1f ? normalise(velocity) : sf::Vector2f(0.0f, 1.0f);

        float angle = std::atan2(velocityDir.y, velocityDir.x);

        scale.x = sqaushAmount * std::cos(angle) * std::cos(angle) + stretchAmount + std::sin(angle) * std::sin(angle);
        scale.y = sqaushAmount * std::sin(angle) * std::sin(angle) + stretchAmount + std::cos(angle) * std::cos(angle);

    }

    sf::CircleShape shape(radius);
    shape.setPosition(position - sf::Vector2f(radius, radius));
    shape.scale(scale);
    shape.setFillColor(displayColour);
    
    // shorthand if
    // float eitherOneORZero = true ? 1 : 0;

    // Old DisplayColour Code
    // sf::Color displayColour = isResting ? sf::Color(colour.r / 2, colour.g / 2, colour.b / 2) : colour; // ? is short hand if statement, so if resting colour.r etc else colour

    if (isStatic)
    {
        shape.setOutlineThickness(2.0f);
        shape.setOutlineColor(sf::Color(60, 60, 70, 150));
    }
    else
    {
        shape.setOutlineThickness(2.0f);
        shape.setOutlineColor(
            sf::Color(
                static_cast<float>(displayColour.r * 1.3),
                static_cast<float>(displayColour.g * 1.3),
                static_cast<float>(displayColour.b * 1.3), 
                200));
    }

    window.draw(shape);

    // velocity vector arrow

    if (!isStatic && std::abs(angularVelocity) > 0.1f)
    {
        sf::Vector2f lineEnd = position + rotate(sf::Vector2f(radius, 0.0f), rotation);

        std::array line =
        {
            sf::Vertex(position, sf::Color(255,255,255,150)),
            sf::Vertex(lineEnd, sf::Color(255,255,0,128))
        };

        window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
    }

    if (showVelocity && !isStatic && length(velocity) > 1.0f)
    {
        std::array line =
        {
            sf::Vertex(sf::Vector2f(position), sf::Color(255,255,0,200)),
            sf::Vertex(sf::Vector2f(position + normalise(velocity) * (radius * 2.0f)), sf::Color(255,255,0,200))
        };

        window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
    }
}

void RigidBody::drawMotionTrail(sf::RenderWindow& window) // & - pointer without referencing and dereferencing or smthn
{
    if (motionTrail.size() < 2) return;

    for (size_t i = 1; i < motionTrail.size(); ++i) // size_t is practically an integer, but is optimised for iterating in loops, should use where we can
    {
        uint8_t alpha = static_cast<uint8_t>(motionTrail[i].alpha * 150);

        std::array<sf::Vertex, 2> glowLine =
        {
            sf::Vertex(motionTrail[i - 1].position, sf::Color(colour.r, colour.g, colour.b, alpha / 3)),
            sf::Vertex(motionTrail[i].position, sf::Color(colour.r, colour.g, colour.b, alpha / 3))
        };

        std::array<sf::Vertex, 2> line =
        {
            sf::Vertex(motionTrail[i - 1].position, sf::Color(colour.r, colour.g, colour.b, alpha)),
            sf::Vertex(motionTrail[i].position, sf::Color(colour.r, colour.g, colour.b, alpha))
        };

        window.draw(glowLine.data(), glowLine.size(), sf::PrimitiveType::Lines);
        window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);

        if (i % 2 == 0)
        {
            sf::CircleShape dot(1.5f);
            dot.setPosition(motionTrail[i].position - sf::Vector2f(1.5f, 1.5f));
            dot.setFillColor(sf::Color(colour.r, colour.g, colour.b, static_cast<uint8_t>(alpha)));
            window.draw(dot);
        }
    }
}
void RigidBody::drawDebug(sf::RenderWindow& window)
{

}

void RigidBody::wake()
{
    isResting = false;
}
