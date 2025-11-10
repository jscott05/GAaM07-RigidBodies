#pragma once
#include <memory>
#include "RigidBody.h"

class PhysicsEngine
{
public:
	PhysicsEngine(float width, float height);

	void addBody(std::unique_ptr<RigidBody> body);
	void clearDynamicBodies();
	void update(float deltaTime);
	void draw(sf::RenderWindow& window, bool showVelocity, bool showTrails, bool showDebug);

	//setters for gravity
	void setGravity(sf::Vector2f& g);
	sf::Vector2f getGravity();

	//helper function for getting the body at a specific point
	RigidBody* getBodyAt(const sf::Vector2f& point);
	size_t getBodyCount() const { return bodies.size(); };
	size_t getDynamicBodyCount() const;

private:
	void checkCollision(RigidBody& body1, RigidBody& body2);

	std::vector<std::unique_ptr<RigidBody>> bodies;

	sf::Vector2f gravity;
	float worldWidth;
	float worldHeight; 
};

