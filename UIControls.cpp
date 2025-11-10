#include "UIControls.h"

UIControls::UIControls(sf::Font& font) : font(font), titleText(font), instructions(font), statsText(font)
{
    backgroundPanel.setSize(sf::Vector2f(300.0f, 650.0f));
	backgroundPanel.setPosition(sf::Vector2f(10.0f, 10.0f));
	backgroundPanel.setFillColor(sf::Color(30, 30, 40, 230));
    
    titleText.setString("Controls:");
    instructions.setPosition(sf::Vector2f(20.0f, 360.0f));
    instructions.setString("Controls: "
        "\nLeft Click - Spawn Particle"
        "\nRight Click - Drag Particle"
        "\nSpace - Spawn 5 Particles"
        "\nC - Clear Particles"
        "\nG - Toggle Gravity");

	statsText.setPosition(sf::Vector2f(20.0f, 590.0f));

}

void UIControls::update(float deltaTime)
{

}

void UIControls::draw(sf::RenderWindow& window)
{
    window.draw(backgroundPanel);
	window.draw(titleText);
	window.draw(instructions);
    window.draw(statsText);


}
void UIControls::handleEvent(const std::optional<sf::Event>& event, const sf::Vector2f& mousePos)
{
}

bool UIControls::isMouseOverUI(const sf::Vector2f& mousePos)
{
	return backgroundPanel.getGlobalBounds().contains(mousePos);
}

void UIControls::updateStats(int bodyCount, float fps)
{
    std::ostringstream statsStream;
    statsStream << "Particles: " << bodyCount << "\nFPS: " << std::fixed << std::setprecision(0) << fps;
	statsText.setString(statsStream.str());

}