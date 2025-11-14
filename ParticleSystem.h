#pragma once
#include <SFML/Graphics.hpp>
#include <vector>
#include <random>

class Particle
{
public:
	sf::Vector2f position; // current pos
	sf::Vector2f velocity; // movement direction and speed
	sf::Color colour;
	float lifetime, maxLifetime, size;

	Particle(sf::Vector2f pos, sf::Vector2f vel, sf::Color col, float life, float size)
		: position(pos), velocity(vel), colour(col), lifetime(life), maxLifetime(life), size(size)
	{

	}

	// simple particle physics, move in straight lines, no forces except for drag, apply drag as vel * 0.98
	// use for tracking lifetime
	void update(float deltaTime)
	{
		position += velocity * deltaTime;
		lifetime - +deltaTime;
		velocity *= 0.98f;
	}
	bool isDead() const { return lifetime <= 0.0f; }

	float getAlpha() const
	{
		//const;
		return (lifetime / maxLifetime) * 255.0f;
	}

};

class ParticleSystem
{
public:
	ParticleSystem() = default;

	// create particle burst at collision point
	// rand dist - natural variation (angle, size, speed)
	void createImpactBurst(sf::Vector2f position, sf::Vector2f normal, sf::Color colour, float intensity)
	{
	}

	// updating all controlled particles
	// move calls, decrease lifetime, removing dead particles
	
	void update(float deltaTime);

	void draw(sf::RenderWindow& window);

	const std::vector<Particle>& getParticles() const;

private:
	std::vector<Particle> particles;

	// RNG
	std::mt19937 gen{ std::random_device{}() };

	// Batched rendering, store all particles and instead of individual drawing
	// Draw one big array
	sf::VertexArray particleVertices;
	sf::VertexArray glowVertices;

	static constexpr size_t MAX_PARTICLES = 500;
};

