// include
#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <random>
#include <memory>
#include <string>
#include <algorithm>
#include <iostream>


class RigidBody
{
public:
    sf::Vector2f position, velocity, acceleration;

    float mass, radius, restitution, friction;
    sf::Color colour;

    bool isStatic, isResting;

    //Thresholds
    static constexpr float REST_VELOCITY_THRESHOLD = 5.0f, REST_ACCELERATION_THRESHOLD = 10.0f;

    RigidBody(sf::Vector2f pos, float r, float m, sf::Color col, bool stat = false)
        :position(pos), radius(r), mass(m), colour(col), isStatic(stat),
        velocity(0.0f, 0.0f), acceleration(0.0f, 0.0f), restitution(0.8f),
        friction(0.3f), isResting(false)
    {

    }

    void applyForce(const sf::Vector2f& force) 
    {
        if (!isStatic && !isResting)
        {
            acceleration += force / mass;
        }
    }
    void update(float deltaTime)
    {
        if (!isStatic)
        {
            float velocityMagnitude = length(velocity);
            float accelerationMagnitude = length(acceleration);

            if (velocityMagnitude < REST_VELOCITY_THRESHOLD && accelerationMagnitude < REST_ACCELERATION_THRESHOLD)
            {
                velocity = sf::Vector2f(0.0f, 0.0f);
                acceleration = sf::Vector2f(0.0f, 0.0f);
                isResting = true;
            }
            else
            {
                isResting = false;
            }

            if (!isResting)
            {
                //Euler
                velocity += acceleration * deltaTime;
                position += velocity * deltaTime;

                //Dampening - Simulate Air Resistance
                velocity *= 0.995f;
            }
            acceleration = sf::Vector2f(0.0f, 0.0f);
        }
    }

    void checkBoundaryCollision(float width, float height)
    {
        if (!isStatic)
        {
            //check collision with boundaries

            //horizontal
            if (position.x - radius < 0)
            {
                position.x = radius;
                velocity.x = -velocity.x * restitution;

                //friction
                velocity.y *= (1.0f - friction);
                isResting = false;
            }
            if (position.x + radius > width) // if position.x is outside of screen width
            {
                position.x = width - radius;
                velocity.x = -velocity.x * restitution;

                velocity.y *= (1.0f - friction);
                isResting = false;
            }

            //vertical
            if (position.y - radius < 0)
            {
                position.y = radius;
                velocity.y = -velocity.y * restitution;

                //friction
                velocity.x *= (1.0f - friction);
                isResting = false;
            }
            if (position.y + radius > height) // if position.y is outside of screen height
            {
                position.y = height - radius;
                velocity.y = -velocity.y * restitution;

                velocity.x *= (1.0f - friction);
                isResting = false;
            }
        }
    }

    void draw(sf::RenderWindow& window)
    {
        sf::CircleShape shape(radius);
        shape.setPosition(position - sf::Vector2f(radius, radius));

        //shorthand if
        //float eitherOneORZero = true ? 1 : 0;

        sf::Color displayColour = isResting ? sf::Color(colour.r/2, colour.g/2, colour.b/2) : colour; // ? is short hand if statement, so if resting colour.r etc else colour
        shape.setFillColor(displayColour);

        shape.setOutlineThickness(2.0f);
        shape.setOutlineColor(sf::Color(static_cast<float>(displayColour.r * 0.7), 
            static_cast<float>(displayColour.g * 0.7), 
            static_cast<float>(displayColour.b * 0.7)));

        window.draw(shape);

        //velocity vector arrow

        if (!isStatic && !isResting)
        {
            std::array line =
            {
                sf::Vertex(sf::Vector2f(position), sf::Color(255,255,0,128)),
                sf::Vertex(sf::Vector2f(position + normalise(velocity) * (radius * 1.5f)), sf::Color(255,255,0,128))
            };

            window.draw(line.data(), line.size(), sf::PrimitiveType::Lines);
        }
    }

    void wake()
    {
        isResting = false;
    }
};

//physics engine class
class PhysicsEngine
{
private:
    std::vector<std::unique_ptr<RigidBody>> bodies;
    sf::Vector2f gravity;

    float worldWidth;
    float worldHeight;

public:
    PhysicsEngine(float width, float height) : worldWidth(width), worldHeight(height), gravity(0.0f, 500.0f)
    {

    }

    void addBody(std::unique_ptr<RigidBody> body)
    {
        bodies.push_back(std::move(body));
    }

    void clearDynamicBodies()
    {
        bodies.erase(
            std::remove_if(
                bodies.begin(),
                bodies.end(),
                [](const std::unique_ptr<RigidBody>& body) { return !body->isStatic; }),
            bodies.end());
    }

    void update(float deltaTime)
    {
        //auto - dont know type, figure it out
        //apply gravity to all bodies - stage 1
        for (auto& body : bodies)
        {
            if (!body->isStatic && !body->isResting)
            {
                body->applyForce(gravity * body->mass);
            }
        }

        //update positions - stage 2
        for (auto& body : bodies)
        {
            body->update(deltaTime);
            body->checkBoundaryCollision(worldWidth, worldHeight);
        }

        //check for collisions between bodies - stage 3
        // O(n^2) -- not entirely efficient
        for (size_t i = 0; i < bodies.size(); ++i) //pre increment i
        {
            for (size_t j = i + 1; j < bodies.size(); ++j) // pre increment j
            {
                //do collision checks - comparing every particle to every other particle
                //optimisation - look at data structures and algorithms - bounding / volume
                checkCollision(*bodies[i], *bodies[j]);
            }
        }
    }

    void checkCollision(RigidBody& body1, RigidBody& body2)
    {
        sf::Vector2f difference = body2.position - body1.position;

        float distance = length(difference);
        float minDistance = body1.radius + body2.radius;

        if (distance < minDistance)
        {
            //floating point bs -- prevent division by 0 for overlaps in bodies
            if (distance < 0.001)
            {
                distance = 0.001f;
                difference = sf::Vector2f(1.0f, 0.0f) * minDistance;
            }
            //wake up resting bodies
            body1.wake();
            body2.wake();

            //collision detected
            sf::Vector2f normal = normalise(difference);

            //separate the bodies
            float overlap = minDistance - distance;
            float seperationFactor = 1.01f;

            if (!body1.isStatic && !body2.isStatic)
            {
                body1.position -= normal * (overlap * 0.5f * seperationFactor);
                body2.position += normal * (overlap * 0.5f * seperationFactor);
            }
            else if (!body1.isStatic)
            {
                body1.position -= normal * overlap;
            }
            else if (!body2.isStatic)
            {
                body2.position += normal * overlap;
            }

            //calculate relative velocity
            sf::Vector2f relativeVelocity = body2.velocity - body1.velocity;
            float velocityAlongNormal = dot(relativeVelocity, normal);
            if (velocityAlongNormal < 0)
            {
                return;
            }


            //restitution calc
            //min - returns lowest of 2 values
            float e = std::min(body1.restitution, body2.restitution);

            //calculate impulse scalar
            float j = -(1 + e) * velocityAlongNormal;

            if (!body1.isStatic && !body2.isStatic)
            {
                j /= (1 / body1.mass +1 / body2.mass);
            }
            else if (!body1.isStatic)
            {
                j /= (1 / body2.mass);
            }
            else if (!body2.isStatic)
            {
                j /= (1 / body1.mass);
            }

            sf::Vector2f impulse = j * normal;


            //at this point he realised and raised the question should it have been structured aaround the 3 options instead due to how many checks we make
            if (!body1.isStatic)
            {
                body1.velocity -= impulse / body1.mass;
            }
            if (!body2.isStatic)
            {
                body2.velocity += impulse / body2.mass;
            }


            //Apply Friction (tangential component)
            sf::Vector2f tangent = relativeVelocity - normal * velocityAlongNormal;
            if (length(tangent) > 0.001f)
            {
                tangent = normalise(tangent);

                // Calculate Friction Coefficient
                float frictionCoEff = (body1.friction + body2.friction) * 0.5f;
                float jt = -dot(relativeVelocity, tangent);

                if (!body1.isStatic && !body2.isStatic)
                {
                    jt /= (1 / body1.mass + 1 / body2.mass);
                }
                else if (!body1.isStatic)
                {
                    jt /= (1 / body2.mass);
                }
                else if (!body2.isStatic)
                {
                    jt /= (1 / body1.mass);
                }

                //coulomb's law - limit friction
                float frictionLimit = std::abs(j * frictionCoEff);
                jt = std::clamp(jt, -frictionLimit, frictionLimit);

                sf::Vector2f frictionImpulse = jt * tangent;
                if (!body1.isStatic)
                {
                    body1.velocity -= frictionImpulse / body1.mass;
                }
                if (!body2.isStatic)
                {
                    body2.velocity += frictionImpulse / body2.mass;
                }
            }
        }
    }


    // at large counts - inefficient 
    // frame buffer is better
    void draw(sf::RenderWindow& window)
    {
        for (auto& body : bodies)
        {
            body->draw(window);
        }
    }

    void setGravity(const sf::Vector2f& g)
    {
        gravity = g;
        for (auto& body : bodies)
        {
            body->wake();
        }
    }

    //Next Session - Figure out call finding bodies at specific locations (so boundaries / volume check??)

    RigidBody* getBodyAt(const sf::Vector2f& point)
    {
        for (auto& body : bodies)
        {
            sf::Vector2f diff = body->position - point;
            if (length(diff) <= body->radius)
            {
                return body.get();
            }
        }
        return nullptr; // it expects a pointer return to a rigidbody, so must return to a null pointer as opposed to null
    }
};

int main()
{
    sf::RenderWindow window(sf::VideoMode({ 1200,800 }), "Rigid Bodies");
    window.setFramerateLimit(60);

    //Init Phys Engine
    PhysicsEngine physics(1200.0f, 800.0f);

    //Rand Num Gen
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<float> posX(100.0f, 1100.0f);
    std::uniform_real_distribution<float> posY(100.0f, 300.0f);
    std::uniform_real_distribution<float> radiusDist(15.0f, 40.0f);
    std::uniform_real_distribution<float> massDist(1.0f, 5.0f);
    std::uniform_int_distribution<int>   colourDist(100, 255);

    sf::Font font;
    font.openFromFile("C:/Windows/Fonts/arial.ttf");
    sf::Text it(font, "Controls: "
        "\nLeft Click - Spawn Particle"
        "\nRight Click - Drag Particle"
        "\nSpace - Spawn 5 Particles"
        "\nC - Clear Particles"
        "\nG - Toggle Gravity");

    //Add Static Objs For Collisions
    physics.addBody(std::make_unique<RigidBody>(sf::Vector2f(300.0f, 500.0f), 50, 10, sf::Color(100, 100, 100), true));
    physics.addBody(std::make_unique<RigidBody>(sf::Vector2f(600.0f, 400.0f), 50, 10, sf::Color(100, 100, 100), true));
    physics.addBody(std::make_unique<RigidBody>(sf::Vector2f(900.0f, 500.0f), 50, 10, sf::Color(100, 100, 100), true));

    //Add Initial Dyn Bodies
    for (int i = 0; i < 10; ++i)
    {
        float radius = radiusDist(gen);
        float mass = massDist(gen) * (radius/20.0f);
        sf::Color colour(colourDist(gen), colourDist(gen), colourDist(gen));
        sf::Vector2f position(posX(gen), posY(gen));

        physics.addBody(std::make_unique<RigidBody>(position, radius, mass, colour));

    }

    //Drag Stuff | Mouse Interaction Variables
    RigidBody* draggedBody = nullptr;
    sf::Vector2f dragOffset;

    //Delta Time Clock
    sf::Clock clock;
    while (window.isOpen())
    {
        float deltaTime = clock.restart().asSeconds();

        //Event Loop
        while (const std::optional event = window.pollEvent())
        {
            //close window on exit event
            if (event->is<sf::Event::Closed>())
            {
                std::cout << "Window Closing";
                window.close();
            }

            //mouse button pressed
            if (const auto* mousePressed = event->getIf<sf::Event::MouseButtonPressed>())
            {
                //cast into vector
                sf::Vector2f mousePos(mousePressed->position.x, mousePressed->position.y);

                //Grab particle and drag
                if (mousePressed->button == sf::Mouse::Button::Right)
                {
                    draggedBody = physics.getBodyAt(mousePos);
                    if (draggedBody)
                    {
                        dragOffset = draggedBody->position - mousePos;
                        draggedBody->wake(); // objects sleep so that we dont check if they hit objects, as they are not moving - optimisation
                    }
                }
                //Spawn Particles
                if (mousePressed->button == sf::Mouse::Button::Left)
                {
                    float radius = radiusDist(gen);
                    float mass = massDist(gen) * (radius / 20.0f);
                    sf::Color colour(colourDist(gen), colourDist(gen), colourDist(gen));

                    physics.addBody(std::make_unique<RigidBody>(mousePos, radius, mass, colour));
                }
            }

            //on mouse release
            if (event->is<sf::Event::MouseButtonReleased>())
            {
                //if dragged body exists, reset it to a nullptr
                if (draggedBody) draggedBody = nullptr;
            }

            if (const auto* mouseMoved = event->getIf<sf::Event::MouseMoved>())
            {
                if (draggedBody && !draggedBody->isStatic)
                {
                    //cast into vector
                    sf::Vector2f mousePos(mouseMoved->position.x, mouseMoved->position.y);

                    sf::Vector2f targetPos = mousePos + dragOffset;

                    draggedBody->velocity = (targetPos - draggedBody->position) * 10.0f;
                }
            }

            //Key Press Events
            if (const auto* keyPressed = event->getIf<sf::Event::KeyPressed>())
            {
                //Space Key Scancode
                if (keyPressed->scancode == sf::Keyboard::Scancode::Space)
                {
                    //Spawn a lot of particles
                    for (int i = 0; i < 5; ++i)
                    {
                        float radius = radiusDist(gen);
                        float mass = massDist(gen) * (radius / 20.0f);
                        sf::Color colour(colourDist(gen), colourDist(gen), colourDist(gen));
                        sf::Vector2f position(posX(gen), posY(gen));

                        physics.addBody(std::make_unique<RigidBody>(position, radius, mass, colour));

                    }
                }
                //C Key Scancode
                if (keyPressed->scancode == sf::Keyboard::Scancode::C)
                {
                    physics.clearDynamicBodies();
                }

                //G Key Scancode
                if (keyPressed->scancode == sf::Keyboard::Scancode::G)
                {
                    static bool gravityOn = true;
                    gravityOn = !gravityOn;

                    physics.setGravity(gravityOn ? sf::Vector2f(0.0f, 500.0f) : sf::Vector2f(0.0f, 0.0f));
                }
            }
        }

        //Update Physics
        physics.update(deltaTime);

        //Render stuff
        window.clear(sf::Color(20,20,30));
        physics.draw(window);
        window.draw(it);
        window.display();
    }
}
