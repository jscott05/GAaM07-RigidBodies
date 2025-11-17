#include "RigidBody.h"
#include "Vector2Utils.h"
#include <algorithm>

using namespace PhysicsUtils;

RigidBody::RigidBody(sf::Vector2f pos, float r, float m, sf::Color col, bool stat)
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
    restitution(0.6f),
    friction(0.3f), 
    trailTimer(0.0f),
    impactIntensity(0.0f),
    squashStretch(1.0f),
    appliedForce(0.0f, 0.0f),
    isResting(false)
{
    inertia = 0.5f * mass * radius * radius; 
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

    //apply force at a specific point ( not the center of mass) create both linear and angular acceleration 
    //torque generation thats of force being applied off center 
    //torque = r x f (cross product of the radius vector and force)

    if (!isStatic)
    {
        acceleration += force / mass;
        sf::Vector2f r = point - position; 
        float torque = cross(r, force);
        applyTorque(torque);
    }
}
void RigidBody::applyTorque(float torque)
{
    //torque is a rotation force 
    //angular analog of linear force 
    //causes angular acceleration (rotational equivalent of F=ma)
    //t = i x a (torque = moment of inertia * angular acceleration)
    if (!isStatic && !isResting)
    {
        angularAcceleration += torque / inertia; 
    }
}



void RigidBody::update(float deltaTime)
{
    if (!isStatic)
    {
		float velocityMagnitude = length(velocity);
		float accelerationMagnitude = length(acceleration);
		float angularVelocityMagnitude = std::abs(angularVelocity);
        
        if (velocityMagnitude < REST_VELOCITY_THRESHOLD &&
            accelerationMagnitude < REST_ACCELERATION_THRESHOLD &&
            angularVelocityMagnitude < REST_ANGULAR_VELOCITY_THRESHOLD)
        {
            velocity = sf::Vector2f(0.0f, 0.0f);
            acceleration = sf::Vector2f(0.0f, 0.0f);
            angularVelocity = 0, angularAcceleration = 0;
            isResting = true;
        }
        else
        {
			isResting = false;
        }
        if (!isResting)
        {
            velocity += acceleration * deltaTime;
            position += velocity * deltaTime;

            velocity *= 0.99f;

            //rotation 
            angularVelocity += angularAcceleration * deltaTime;
            rotation += angularVelocity * deltaTime;

            angularVelocity *= 0.98f;



            //lets calculate our trail 
            trailTimer += deltaTime;
            if (trailTimer >= TRAIL_UPDATE_INTERVAL)
            {
                trailTimer = 0.f;
                motionTrail.push_front({ position, 1.f });
                if (motionTrail.size() > MAX_TRAIL_LENGTH)
                {
                    motionTrail.pop_back();
                }
            }
            for (auto& trail : motionTrail)
            {
                trail.alpha *= 0.55;
            }

        }

        impactIntensity *= 0.9f;
        squashStretch += (1.0f - squashStretch) * 0.2f;

        acceleration = sf::Vector2f(0.0f, 0.0f);
        angularAcceleration = 0.f;
        appliedForce = sf::Vector2f(0.0f, 0.f);
    }
}

void RigidBody::checkBoundaryCollision(float width, float height)
{
    if (!isStatic)
    {
        bool collided = false;
        sf::Vector2f normal(0.f, 0.f);
        //check collision with boundaries
        {
            if (position.x - radius < 0)
            {
                position.x = radius;
                velocity.x = -velocity.x * restitution;

                //apply friction
                velocity.y *= (1.0f - friction);
                angularVelocity *= (1.f - friction);
                normal = sf::Vector2f(1.f, 0.f);

                collided = true;
            }

            if (position.x + radius > width) //so if position.x is outside the width of the screen
            {
                position.x = width - radius;
                velocity.x = -velocity.x * restitution;

                velocity.y *= (1.0f - friction);
                angularVelocity *= (1.f - friction);
                normal = sf::Vector2f(-1.f, 0.f);

                collided = true;
            }

            //check the vertical 

            if (position.y - radius < 0)
            {
                position.y = radius;
                velocity.y = -velocity.y * restitution;

                //apply friction
                velocity.x *= (1.0f - friction);
                angularVelocity *= (1.f - friction);
                normal = sf::Vector2f(0.f, 1.f);
                collided = true;
            }

            if (position.y + radius > height) //so if position.x is outside the width of the screen
            {
                position.y = height - radius;
                velocity.y = -velocity.y * restitution;

                velocity.x *= (1.0f - friction);
                angularVelocity *= (1.f - friction);
                normal = sf::Vector2f(0.f, -1.f);
                collided = true;
            }


            if (collided)
            {
                float intensity = length(velocity) / 100.f;
                impactIntensity = std::min(1.f, intensity);
            }
        }
    }
}

void RigidBody::draw(sf::RenderWindow& window, bool showVelocity)
{

    sf::Color displayColour = isResting ? sf::Color(colour.r/2, colour.g/2, colour.b/2) : colour;

    float flashIntensity = impactIntensity * 100.f;

    displayColour.r = std::min(255, static_cast<int>(displayColour.r + flashIntensity));
    displayColour.g = std::min(255, static_cast<int>(displayColour.g + flashIntensity));
    displayColour.b = std::min(255, static_cast<int>(displayColour.b + flashIntensity));

    if (!isStatic)
    {
        for (int i = 3; i > 0; --i)
        {
            float glowRadius = radius + (i * 4.f) + (impactIntensity * 5.f);
            float alpha = 20.f / (i + 1) + (impactIntensity * 30.f);

            sf::CircleShape glow(glowRadius);
            glow.setPosition(position - sf::Vector2f(glowRadius, glowRadius));

            glow.setFillColor(sf::Color(displayColour.r, displayColour.g, displayColour.b, static_cast<uint8_t>(alpha)));

            window.draw(glow);
        }
    }

    sf::Vector2f scale(1.f, 1.f);
    if (impactIntensity > 0.01f)
    {
        float squashAmount = 1.f - (impactIntensity * 0.3f);
        float stretchAmount = 1.f + (impactIntensity * 3.f);

        sf::Vector2f velocityDir = length(velocity) > 0.1f ? normalise(velocity) : sf::Vector2f(0.f, 1.f);

        float angle = std::atan2(velocityDir.y, velocityDir.x);

        scale.x = squashAmount * std::cos(angle) * std::cos(angle) + stretchAmount + std::sin(angle) * std::sin(angle);
        scale.y = squashAmount * std::sin(angle) * std::sin(angle) + stretchAmount + std::cos(angle) * std::cos(angle);
    }


    sf::CircleShape shape(radius);
    shape.setPosition(position - sf::Vector2f(radius, radius));
    shape.scale(scale);
    shape.setFillColor(displayColour);

    if (isStatic)
    {
        shape.setOutlineThickness(2.f);
        shape.setOutlineColor(sf::Color(60,60,70, 150));
    }
    else
    {
        shape.setOutlineThickness(1.5f);
        shape.setOutlineColor(
            sf::Color(
            static_cast<float>(displayColour.r * 1.3f),
            static_cast<float>(displayColour.g * 1.3f),
            static_cast<float>(displayColour.b * 1.3f), 
                200));
    }

    window.draw(shape);


    if (!isStatic && !isResting)
    {
        sf::CircleShape core(radius * 0.4f);
		core.setPosition(position - sf::Vector2f(radius * 0.4f, radius * 0.4f));
        core.setFillColor(
            sf::Color(
                static_cast<float>(displayColour.r * 1.5f),
                static_cast<float>(displayColour.g * 1.5f),
                static_cast<float>(displayColour.b * 1.5f),
                180));
		window.draw(core);
    }

    //draw velocity vector
    if (!isStatic && std::abs(angularVelocity) > 0.1f)
    {
        sf::Vector2f lineEnd = position + rotate(sf::Vector2f(radius, 0.f), rotation);

        std::array<sf::Vertex, 2> line =
        {
            sf::Vertex(position, sf::Color(255,255,255, 150)),
            sf::Vertex(lineEnd, sf::Color(255,255,0,128))
        };

        window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);

    }

    if (showVelocity && !isStatic && length(velocity) > 1.f)
    {
        std::array line =
        {
            sf::Vertex(sf::Vector2f(position), sf::Color(255,255,0, 200)),
            sf::Vertex(sf::Vector2f(position + normalise(velocity) * (radius * 2.f)), sf::Color(255,255,0,200))
        };

        window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
    }
}

void RigidBody::drawMotionTrail(sf::RenderWindow& window)
{
    if (motionTrail.size() < 2) return; 


    for (size_t i = 1; i < motionTrail.size(); ++i)
    {
        uint8_t alpha = static_cast<uint8_t>(motionTrail[i].alpha * 150);


        std::array<sf::Vertex, 2> glowLine = {
            sf::Vertex(motionTrail[i - 1].position, sf::Color(colour.r, colour.g, colour.b, alpha / 3)),
            sf::Vertex(motionTrail[i].position, sf::Color(colour.r, colour.g, colour.b, alpha / 3))
        };


        std::array<sf::Vertex, 2> line = {
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
    for (const auto& info : collisionInfos)
    {
        uint8_t alpha = static_cast<uint8_t>(info.lifetime * 255.0f);

        sf::CircleShape contactPoint(3.0f);
        contactPoint.setPosition(info.contactPoint - sf::Vector2f(3.f, 3.f));
        contactPoint.setFillColor(sf::Color(255, 0, 0, alpha));
        window.draw(contactPoint);

        //is the line for the normal 
        sf::Vector2f normalEnd = info.contactPoint + info.normal * 30.f;
        std::array<sf::Vertex, 2> normalLine =
        {
            sf::Vertex(info.contactPoint, sf::Color(0,255,255,alpha)),
            sf::Vertex(normalEnd, sf::Color(0,255,255,alpha))
        };
        window.draw(normalLine.data(), normalLine.size(), sf::PrimitiveType::Lines);


        sf::Vector2f arrowLeft = 
            normalEnd + rotate(sf::Vector2f(-5.f, 0.f), std::atan2(info.normal.y, info.normal.x) + 2.7f);
        sf::Vector2f arrowRight =
            normalEnd + rotate(sf::Vector2f(-5.f, 0.f), std::atan2(info.normal.y, info.normal.x) - 2.7f);


        std::array<sf::Vertex, 2> arrow1 =
        {
            sf::Vertex(normalEnd, sf::Color(0,255,255,alpha)),
            sf::Vertex(arrowLeft, sf::Color(0,255,255,alpha))
        };
        std::array<sf::Vertex, 2> arrow2 =
        {
            sf::Vertex(normalEnd, sf::Color(0,255,255,alpha)),
            sf::Vertex(arrowRight, sf::Color(0,255,255,alpha))
        };
        window.draw(arrow1.data(), arrow1.size(), sf::PrimitiveType::Lines);
        window.draw(arrow2.data(), arrow2.size(), sf::PrimitiveType::Lines);
    }

    if (length(appliedForce) > 0.1f)
    if (length(appliedForce) > 0.1f)
    {
        sf::Vector2f forceEnd = position + normalise(appliedForce) * (length(appliedForce) / 50.f);

        std::array<sf::Vertex, 2> forceLine =
        {
            sf::Vertex(position, sf::Color(0,255,255,200)),
            sf::Vertex(forceEnd, sf::Color(0,255,255,200))
        };
        window.draw(forceLine.data(), forceLine.size(), sf::PrimitiveType::Lines);

    }
}

void RigidBody::addCollisionInfo(const sf::Vector2f& point, const sf::Vector2f& normal, float penetration)
{
    collisionInfos.push_back({ point, normal, penetration, 1.f });
}

void RigidBody::updateCollisionInfo(float deltaTime)
{
    for (auto& info : collisionInfos)
    {
        info.lifetime -= deltaTime * 2.0f;
    }

    collisionInfos.erase(
        std::remove_if(collisionInfos.begin(), collisionInfos.end(),
            [](const CollisionInfo& info)
            { return info.lifetime <= 0.0f; }), collisionInfos.end());
}
void RigidBody::wake()
{
    isResting = false;
}