#pragma once
#include <SFML/System/Vector2.hpp>
#include <cmath>

/**
 * VECTOR MATHEMATICS FOR GAME PHYSICS
 * ====================================
 *
 * Vectors represent quantities with both MAGNITUDE and DIRECTION
 * Examples: velocity, force, displacement
 *
 * In 2D, a vector has two components: (x, y)
 * - x: horizontal component
 * - y: vertical component
 *
 * VISUAL REPRESENTATION:
 *        ↑ y
 *        |    v = (3, 2)
 *        |   /
 *        |  / ← vector
 *        | /
 *        |/____→ x
 *        O
 *
 * WHY VECTORS MATTER IN GAME PHYSICS:
 * - Position, velocity, acceleration are all vectors
 * - Forces combine through vector addition
 * - Direction of motion uses normalized vectors
 * - Collision responses depend on vector projections
 */
namespace PhysicsUtils {
    /**
     * NORMALIZE: Convert vector to unit length (magnitude = 1)
     *
     * FORMULA: v̂ = v / |v|
     * Where |v| = length of v
     *
     * PURPOSE:
     * - Get pure direction without magnitude
     * - Useful for direction vectors (e.g., "move this way")
     *
     * EXAMPLE:
     * v = (3, 4) has length 5
     * normalize(v) = (3/5, 4/5) = (0.6, 0.8) with length 1
     *
     * APPLICATIONS:
     * - Collision normals (direction to push apart)
     * - Movement directions (forward, aim direction)
     * - Lighting calculations (surface normals)
     *
     * @param v - Input vector
     * @return Unit vector in same direction, or original if zero-length
     */
    inline sf::Vector2f normalise(const sf::Vector2f& v) {
        float len = std::sqrt(v.x * v.x + v.y * v.y);
        if (len > 0.0001f) {  // Avoid division by zero
            return sf::Vector2f(v.x / len, v.y / len);
        }
        return v;  // Zero vector stays zero
    }

    /**
     * DOT PRODUCT: Measure alignment between two vectors
     *
     * FORMULA: a · b = ax×bx + ay×by
     *
     * GEOMETRIC MEANING: a · b = |a| × |b| × cos(θ)
     * Where θ is angle between vectors
     *
     * INTERPRETATION:
     * - Result > 0: Vectors point same direction (angle < 90°)
     * - Result = 0: Vectors perpendicular (angle = 90°)
     * - Result < 0: Vectors point opposite directions (angle > 90°)
     *
     * EXAMPLES:
     * (1,0) · (1,0) = 1    (parallel, same direction)
     * (1,0) · (0,1) = 0    (perpendicular)
     * (1,0) · (-1,0) = -1  (parallel, opposite direction)
     *
     * APPLICATIONS:
     * - Check if objects moving toward/away from each other
     * - Project one vector onto another
     * - Calculate angle between vectors: cos(θ) = (a·b) / (|a|×|b|)
     * - Lighting: cos(angle) between light and surface normal
     *
     * IN OUR PHYSICS:
     * - Velocity along collision normal (are they approaching?)
     * - Friction force (tangent to surface)
     *
     * @param a - First vector
     * @param b - Second vector
     * @return Scalar (single number) representing alignment
     */
    inline float dot(const sf::Vector2f& a, const sf::Vector2f& b) {
        return a.x * b.x + a.y * b.y;
    }

    /**
     * LENGTH (MAGNITUDE): Distance from origin
     *
     * FORMULA: |v| = √(x² + y²)  (Pythagorean theorem!)
     *
     * VISUAL:
     *        v = (3, 4)
     *        |   /
     *      4 |  / |v| = 5
     *        | /
     *        |/___
     *           3
     *
     * APPLICATIONS:
     * - Speed (magnitude of velocity vector)
     * - Distance between points: |point2 - point1|
     * - Normalization (dividing by length)
     *
     * @param v - Input vector
     * @return Length as scalar
     */
    inline float length(const sf::Vector2f& v) {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    /**
     * LENGTH SQUARED: Faster alternative when only comparing lengths
     *
     * FORMULA: |v|² = x² + y²
     *
     * WHY USE THIS?
     * - Avoids expensive sqrt() operation
     * - Still preserves ordering: if |a|² > |b|², then |a| > |b|
     *
     * WHEN TO USE:
     * - Distance comparisons: "Is A closer than B?"
     * - Checking if within radius: |v|² < r²
     * - Performance-critical code
     *
     * WHEN NOT TO USE:
     * - Need actual distance value
     * - Comparing to non-squared threshold
     *
     * @param v - Input vector
     * @return Squared length (avoids sqrt)
     */
    inline float lengthSquared(const sf::Vector2f& v) {
        return v.x * v.x + v.y * v.y;
    }

    /**
     * CROSS PRODUCT (2D version): Returns scalar, not vector!
     *
     * FORMULA: a × b = ax×by - ay×bx
     *
     * 3D vs 2D:
     * - In 3D: cross product gives perpendicular vector
     * - In 2D: cross product gives signed scalar (z-component of 3D cross)
     *
     * GEOMETRIC MEANING:
     * - Magnitude = |a| × |b| × sin(θ)
     * - Sign indicates rotation direction:
     *   • Positive: b is counter-clockwise from a
     *   • Negative: b is clockwise from a
     *   • Zero: vectors parallel
     *
     * APPLICATIONS:
     * - Calculate TORQUE: τ = r × F
     * - Determine rotation direction
     * - Test if point is left/right of line
     * - Area of parallelogram formed by vectors
     *
     * IN OUR PHYSICS:
     * - Convert linear force to angular acceleration
     * - Calculate rotational effect of off-center impacts
     *
     * EXAMPLES:
     * (1,0) × (0,1) = 1    (90° counter-clockwise)
     * (0,1) × (1,0) = -1   (90° clockwise)
     * (1,0) × (2,0) = 0    (parallel)
     *
     * @param a - First vector
     * @param b - Second vector
     * @return Scalar representing rotation amount and direction
     */
    inline float cross(const sf::Vector2f& a, const sf::Vector2f& b) {
        return a.x * b.y - a.y * b.x;
    }

    /**
     * ROTATE: Rotate vector by angle around origin
     *
     * FORMULA (rotation matrix):
     * x' = x×cos(θ) - y×sin(θ)
     * y' = x×sin(θ) + y×cos(θ)
     *
     * DERIVATION: Multiplying by 2D rotation matrix
     * [cos(θ)  -sin(θ)] [x]   [x']
     * [sin(θ)   cos(θ)] [y] = [y']
     *
     * APPLICATIONS:
     * - Rotate sprites
     * - Calculate positions on rotating objects
     * - Transform coordinates between reference frames
     *
     * IN OUR PHYSICS:
     * - Find point on circle edge given angle
     * - Calculate velocity of rotating object's surface
     *
     * @param v - Vector to rotate
     * @param angle - Rotation angle in RADIANS (positive = counter-clockwise)
     * @return Rotated vector
     *
     * NOTE: Radians vs Degrees
     * - Full circle = 360° = 2π radians
     * - 90° = π/2 radians ≈ 1.57
     * - To convert: radians = degrees × (π/180)
     */
    inline sf::Vector2f rotate(const sf::Vector2f& v, float angle) {
        float c = std::cos(angle);
        float s = std::sin(angle);
        return sf::Vector2f(v.x * c - v.y * s, v.x * s + v.y * c);
    }
}