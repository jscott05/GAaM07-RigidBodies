#include "ParticleSystem.h"
#include "UIControls.h"
void ParticleSystem::createImpactBurst(sf::Vector2f position, sf::Vector2f normal, sf::Color colour, float intensity)
{
	// limit amount of spawned particles
	if (particles.size() >= MAX_PARTICLES)
	{
		return; // skip creating new particles if limit is reached
	}

	std::uniform_real_distribution<float> angleDist(-0.5f, 0.5f);
	std::uniform_real_distribution<float> speedDist(50.0f, 150.0f);
	std::uniform_real_distribution<float> sizeDist(1.0f, 3.0f);

	int particleCount = static_cast<int>(intensity * 20.0f);
	particleCount = std::clamp(particleCount, 5, 30);

	// further reduce if close to limit
	size_t remainingCapacity = MAX_PARTICLES - particles.size();
	if (remainingCapacity < static_cast<size_t>(particleCount))
	{
		particleCount = static_cast<int>(remainingCapacity);
	}

	for (int i = 0; i < particleCount; ++i)
	{
		float angle = std::atan2(normal.y, normal.x) + angleDist(gen);
		float speed = speedDist(gen) * intensity;
		sf::Vector2f velocity(std::cos(angle) * speed, std::sin(angle) * speed);

		particles.emplace_back(position, velocity, colour, 0.3f + intensity * 0.2f, sizeDist(gen));
	}
}

void ParticleSystem::update(float deltaTime)
{
	for (auto& particle : particles)
	{
		particle.update(deltaTime);
	}

	particles.erase(std::remove_if(particles.begin(), particles.end(),
		[](const Particle& p) { return p.isDead(); }), particles.end());
}


void ParticleSystem::draw(sf::RenderWindow& window)
{
	if (particles.empty()) return;

	// clear vertex arrays
	particleVertices.clear();
	glowVertices.clear();

	particleVertices.setPrimitiveType(sf::PrimitiveType::Triangles);
	glowVertices.setPrimitiveType(sf::PrimitiveType::Triangles);

	// reduce segment count for performance

	const int segments = 8;


	for (const auto& particle : particles)
	{
		sf::Color col = particle.colour;
		uint8_t alpha = static_cast<uint8_t>(particle.getAlpha());
		col.a = alpha;

		// draw glow x 2
		sf::Color glowCol = col;
		glowCol.a = static_cast<uint8_t>(alpha * 0.3f);

		float glowSize = particle.size * 2.0f;
		float pi = 3.14159f;
	
		for (int i = 0; i < segments; ++i)
		{
			float angle1 = (i * 2.0f * pi) / segments;
			float angle2 = ((i + 1) * 2.0f * pi) / segments;

			sf::Vector2f p1 = particle.position +
				sf::Vector2f(std::cos(angle1) * particle.size, std::sin(angle1)*glowSize);
			sf::Vector2f p2 = particle.position +
				sf::Vector2f(std::cos(angle2) * particle.size, std::sin(angle2) * glowSize);

			glowVertices.append(sf::Vertex(particle.position, col));
			glowVertices.append(sf::Vertex(p1, col));
			glowVertices.append(sf::Vertex(p2, col));
		}	

		window.draw(glowVertices);
		window.draw(particleVertices);
	}
}

