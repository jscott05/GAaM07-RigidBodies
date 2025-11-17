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

    struct CollisionInfo
    {
        sf::Vector2f contactPoint; //where the objects touch
        sf::Vector2f normal; //direction of collision force
        float penetration; //how deep the objects overlapped
        float lifetime; //how long after the incident we want to display (for visualisation)
    };

    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::Vector2f acceleration;

    float mass, radius, restitution, friction;

    float inertia, impactIntensity, squashStretch;

    std::deque<MotionTrail> motionTrail;
    //trail configuration 
    static constexpr size_t MAX_TRAIL_LENGTH = 30; //this max trail points 
    static constexpr float TRAIL_UPDATE_INTERVAL = 0.05f; // add a point every 0.05s 
    float trailTimer; //time until the next trail point 

    std::vector<CollisionInfo> collisionInfos;

    //debug 
    sf::Vector2f lastImpulse;
    sf::Vector2f appliedForce;

    //angular motion state (rational analog of linear motion)
    float rotation; //curent angle in radians
    float angularVelocity; //rotation speed (radians/second)
    float angularAcceleration; //rotation acceleration (radians/second^2)

    sf::Color colour;

    bool isStatic;
    bool isResting;

    //rest thresholds 
    static constexpr float REST_VELOCITY_THRESHOLD = 5.0f;
    static constexpr float REST_ACCELERATION_THRESHOLD = 10.0f;

    static constexpr float REST_ANGULAR_VELOCITY_THRESHOLD = 0.1f;   // copied from github idk the val

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

    void addCollisionInfo(const sf::Vector2f& point, const sf::Vector2f& normal, float peneratraion);
    void updateCollisionInfo(float deltaTime);


    void wake();




    //getters and setters - manipulating the physics states
    sf::Vector2f getPosition() const { return position; }
    sf::Vector2f getVelocity() const { return velocity; }
    float getRadius() const { return radius; }
    float getMass() const { return mass; }


    float getRotation() const { return rotation;  }
    float getAngularVelocity() const { return angularVelocity; }
    float getInertia() const { return inertia; }

    bool getIsStatic() const { return isStatic; }
    bool getIsResting() const { return isResting; }
    sf::Color getColour() const { return colour; }
    const std::deque<MotionTrail>& getMotionTrail() const { return motionTrail; }

    //setters - for setting the physics state
    void setPosition(const sf::Vector2f& pos) { position = pos; }
    void setVelocity(const sf::Vector2f& vel) { velocity = vel; }
    void setAngularVelocity(float av) { angularVelocity = av; }
    void setRestitution(float r) { restitution = r; }
    void setFriction(float f) { friction = f; }



};
