#pragma once
#include <SFML/Graphics.hpp>
#include <functional>
class UIControls
{
public:
	UIControls(sf::Font& font);
	void update(float deltaTime);
	void draw(sf::RenderWindow& window);

	void handleEvent(const std::optional<sf::Event>& event, const sf::Vector2f& mousePos);

	bool isMouseOverUI(const sf::Vector2f& mousePos);

	std::function<void(float)> onGravityChange;
	std::function<void(float)> onRestitutionChange;
	std::function<void(float)> onFrictionChange;

	bool showVelocityTrail = true, showMotionTrail = true, showDebugVisualization = false;

	void updateStats(int bodyCount, float fps);

private:
	sf::Font& font;
	sf::Text titleText, instructions, statsText;

	float gravityValue = 500.0f; // default gravity
	float restitution = 0.6f; // default restitution
	float friction = 0.3f; // default friction

	sf::RectangleShape backgroundPanel;

};

