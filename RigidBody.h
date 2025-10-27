#pragma once

#include <SFML/Graphics.hpp>
#include <deque>

class RigidBody
{
public:

    struct MotionTrail
    {
        sf::Vector2f position;
        float alpha;
    };

    // structs are used as data objects commonly
    struct CollisionInfo
    {
        sf::Vector2f contactPoint; // Object Contact Point
        sf::Vector2f normal; // direction of collision force
        float penetration; // how deep the objects overlap
        float lifetime; // how long after the incident we want to display (for visualisation)

    };

    sf::Vector2f position, velocity, acceleration;

    float mass, radius, restitution, friction;
    float inertia, impactIntensity, squashStretch;
    
    std::deque<MotionTrail> motionTrail;
    //Trail Config
    static constexpr size_t MAX_TRAIL_LENGTH = 30; // Max Trail Points
    static constexpr size_t TRAIL_UPDATE_INTERVAL = 0.05f; // Add A Point Every 0.05s
    float trailTimer;

    std::vector<CollisionInfo> collisionInfo;
    sf::Color colour;

    bool isStatic, isResting;

    // angular motion state (rational analog of linear motion)

    float rotation, angularVelocity, angularAcceleration; 
    // rotation - angle in radians | angular velocity - rotation speed (radians/second) | angular acceleration - rotation acceleration (radians/second^2)

    // debug
    sf::Vector2f lastImpulse, appliedForce;

    // Thresholds
    static constexpr float REST_VELOCITY_THRESHOLD = 5.0f, REST_ACCELERATION_THRESHOLD = 10.0f;

    RigidBody(sf::Vector2f pos, float r, float m, sf::Color col, bool stat = false);

    void applyForce(const sf::Vector2f& force);

    void applyForceAtPoint(const sf::Vector2f& force, const sf::Vector2f& point);

    void applyTorque(float torque);

    void update(float deltaTime);

    void checkBoundaryCollision(float width, float height);
    
    void draw(sf::RenderWindow& window, bool showVelocity);

    void drawMotionTrail(sf::RenderWindow& window);

    void drawDebug(sf::RenderWindow& window);

    //debug/visualisation helpers

    void addCollisionInfo(const sf::Vector2f& point, const sf::Vector2f& normal, float penetration);
    void updateCollisionInfo(float deltaTime);

    void wake();

    // Getters and Setters - Manipulate Physics States
    sf::Vector2f getPosition() const { return position; }
    sf::Vector2f getVelocity() const { return velocity; }

    float getRadius() const { return radius; }
    float getMass() const { return mass; }

    float getRotation() const { return rotation; }
    float getAngularVelocity() const { return angularVelocity; }
    float getInertia() const { return inertia; }

    bool getIsStatic() const { return isStatic; }
    sf::Color getColour() const { return colour; }
    const std::deque<MotionTrail>& getMotionTrail() const { return motionTrail; }

    //setters
    void setPosition(const sf::Vector2f pos) { position = pos; }
    void setVelocity(const sf::Vector2f vel) { velocity = vel; }
    void setAngularVelocity(float av) { angularVelocity = av; }
    void setRestitution(float r) { restitution = r; }
    void setFriction(float f) { friction = f; }


    
};

