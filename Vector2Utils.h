#pragma once
#include <sfml/System/Vector2.hpp>
#include <cmath>

namespace PhysicsUtils
{
    // Helper Functions for working with Vector2f

    inline float length(const sf::Vector2f& v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    inline sf::Vector2f normalise(const sf::Vector2f& v)
    {
        float length = std::sqrt(v.x * v.x + v.y * v.y);
        if (length != 0)
        {
            return sf::Vector2f(v.x / length, v.y / length);
        }
        return v;
    }

    inline float dot(const sf::Vector2f& a, const sf::Vector2f b)
    {
        return a.x * b.x + a.y * b.y;
    }

    inline float lengthSquared(const sf::Vector2f& v)
    {
        return v.x * v.x + v.y * v.y;
    }

    inline float cross(const sf::Vector2f& a, const sf::Vector2f b)
    {
        return a.x * b.y - a.y * b.x;
    }

    inline sf::Vector2f rotate(const sf::Vector2f& v, float angle)
    {
        float c = std::cos(angle);
        float s = std::sin(angle);
        return sf::Vector2f(v.x * c - v.y * s, v.x * s + v.y * c);
    }
}


