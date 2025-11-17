// include
#include <SFML/Graphics.hpp>
#include <vector>
#include <cmath>
#include <random>
#include <memory>
#include <string>
#include <algorithm>
#include <iostream>

//#include "Vector2Utils.h" // PhysicsUtils::
//#include "RigidBody.h"
#include "PhysicsEngine.h"
//#include "ParticleSystem.h"
#include "UIControls.h"

//using namespace PhysicsUtils; // idk if we add this but i added it to try make it compile

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

    // Add Static Objs For Collisions
    physics.addBody(std::make_unique<RigidBody>(sf::Vector2f(300.0f, 500.0f), 50, 10, sf::Color(100, 100, 100), true));
    physics.addBody(std::make_unique<RigidBody>(sf::Vector2f(600.0f, 400.0f), 50, 10, sf::Color(100, 100, 100), true));
    physics.addBody(std::make_unique<RigidBody>(sf::Vector2f(900.0f, 500.0f), 50, 10, sf::Color(100, 100, 100), true));

    // Add Initial Dyn Bodies
    for (int i = 0; i < 10; ++i)
    {
        float radius = radiusDist(gen);
        float mass = massDist(gen) * (radius/20.0f);
        sf::Color colour(colourDist(gen), colourDist(gen), colourDist(gen));
        sf::Vector2f position(posX(gen), posY(gen));

        physics.addBody(std::make_unique<RigidBody>(position, radius, mass, colour));

    }

    UIControls ui(font);

    // Drag Stuff | Mouse Interaction Variables
    RigidBody* draggedBody = nullptr;
    sf::Vector2f dragOffset;

    // Delta Time Clock
    sf::Clock clock;
    sf::Clock fpsTimer;
    int frameCount = 0;
    float fps = 60.0f;
    while (window.isOpen())
    {
        float deltaTime = clock.restart().asSeconds();
		frameCount++;
        if (fpsTimer.getElapsedTime().asSeconds() >=1.0f)
        {
			fps = frameCount / fpsTimer.getElapsedTime().asSeconds();
			frameCount = 0;
			fpsTimer.restart();
        }
        // Event Loop
        while (const std::optional event = window.pollEvent())
        {
            // close window on exit event
            if (event->is<sf::Event::Closed>())
            {
                std::cout << "Window Closing";
                window.close();
            }

            // mouse button pressed
            if (const auto* mousePressed = event->getIf<sf::Event::MouseButtonPressed>())
            {
                // cast into vector
                sf::Vector2f mousePos(mousePressed->position.x, mousePressed->position.y);

                // Grab particle and drag
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
                if (draggedBody && !draggedBody->getIsStatic())
                {
                    //cast into vector
                    sf::Vector2f mousePos(mouseMoved->position.x, mouseMoved->position.y);

                    sf::Vector2f targetPos = mousePos + dragOffset;

                    draggedBody->setVelocity((targetPos - draggedBody->getPosition()) * 10.0f);
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
        physics.draw(window, false, false, false);
        //window.draw(it);
        ui.draw(window);
        window.display();
    }
}
