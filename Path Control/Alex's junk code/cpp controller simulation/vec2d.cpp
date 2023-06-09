#include <iostream>
#include <stdio.h>
#include <math.h>

class Vector2D
{
public:
    float x;
    float y;

    // Constructor
    Vector2D(float x_val = 0.0f, float y_val = 0.0f) : x(x_val), y(y_val) {}

    // Subtraction
    Vector2D operator-(const Vector2D &other) const
    {
        return Vector2D(x - other.x, y - other.y);
    }

    // Addition
    Vector2D operator+(const Vector2D &other) const
    {
        return Vector2D(x + other.x, y + other.y);
    }
    // Scalar multiplication
    Vector2D operator*(float scalar) const
    {
        return Vector2D(x * scalar, y * scalar);
    }

        Vector2D operator/(float scalar) const
    {
        return Vector2D(x / scalar, y / scalar);
    }

    float operator&(const Vector2D &other) const
    {

        return (x * other.x + y * other.y);
    }

    // Norm calculation
    float norm() const
    {
        return sqrt(x * x + y * y);
    }

    // normilized version of the vector
    Vector2D hat() const
    {
        return Vector2D(x,y)/ this->norm();
    }

    void print()
    {
        std::cout << "x=" << this->x << "   y=" << this->y << '\n';
    }
};

int main()
{
    Vector2D v1(3, -6);
    Vector2D v2(-5, 7);
    // std::cout << "dot = " << (v1&v2) << '\n';
    Vector2D v3 = v1.hat();
    v3.print();
}