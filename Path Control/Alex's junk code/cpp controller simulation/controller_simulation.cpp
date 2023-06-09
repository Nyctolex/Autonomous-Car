#include <iostream>
#include <stdio.h>
#include <fstream>
#include <math.h>

#define PI 3.14159265358979323846 /* pi */

#define min(a, b) (((a) < (b)) ? (a) : (b))
#define max(a, b) (((a) > (b)) ? (a) : (b))
#define PATH_LENGTH 6

float path[PATH_LENGTH][2] = {
    {0,0},
    {1, 2},
    {1, 5},
    {5, 5},
    {5, 10},
    {1, 1}
    };

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
    // Scalar division
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

    bool is_clock_wise_angle(const Vector2D &other)
    {
        // returns if v1 is clockwise to v2
        Vector2D v1 = this->hat();
        Vector2D v2 = other.hat();
        float det = v1.x * v2.y - v1.y * v2.x;
        return det < 0;
    }

    // normilized version of the vector
    Vector2D hat() const
    {
        return Vector2D(x, y) / this->norm();
    }

    bool is_zero() const
    {
        return ((x == 0) && (y == 0));
    }

    void print()
    {
        std::cout << "x=" << this->x << "   y=" << this->y << '\n';
    }
    // Overloading the << operator
    friend std::ostream &operator<<(std::ostream &os, const Vector2D &vec)
    {
        os << "x=" << vec.x << " y=" << vec.y;
        return os;
    }
};

class Line
{
public:
    Vector2D p1;
    Vector2D p2;
    Vector2D pointing;
    // Constructor
    Line(float x1, float y1, float x2, float y2)
    {
        this->p1 = Vector2D(x1, y1);
        this->p2 = Vector2D(x2, y2);
        this->pointing = p2 - p1;
    }
    Line(Vector2D p1, Vector2D p2)
    {
        this->p1 = p1;
        this->p2 = p2;
        this->pointing = p2 - p1;
    }
};

float line_point_distance(Line line, Vector2D point)
{
    Vector2D p = line.pointing;
    Vector2D n(-p.y, p.x);
    return p & (n.hat());
}

float distance_to_end_section(Line line, Vector2D point)
{
    Vector2D p = line.pointing;
    Vector2D p3 = line.p2 - point;
    return p.hat() & p3;
}

bool passed_section(Line line, Vector2D point)
{
    return distance_to_end_section(line, point) < 0;
}

bool about_to_pass_section(Line section, Vector2D point, float threshole = 0.05)
{
    return abs(distance_to_end_section(section, point)) < threshole;
}

int find_next_section(float path[][2], Vector2D position, int current_index, float threshold)
{
    Vector2D p1(path[current_index][0], path[current_index][1]);
    Vector2D p2(path[current_index + 1][0], path[current_index + 1][1]);
    Line line(p1, p2);

    if (current_index == PATH_LENGTH - 2)
        return current_index;

    bool have_passed_section = passed_section(line, position);

    if (have_passed_section)
        return (current_index + 1);
    if ((!have_passed_section) && (distance_to_end_section(line, position) < threshold))
        return (current_index + 1);
    return current_index;
}

bool is_clock_wise_angle(Vector2D v1, Vector2D v2)
{
    // returns if v1 is clockwise to v2
    v1 = v1.hat();
    v2 = v2.hat();

    float det = v1.x * v2.y - v1.y * v2.x;
    return det < 0;
}

float non_negative_angle(float angle)
{
    // Transfer a negative angle to it's positive version (in radians)
    angle = fmod(angle, (2 * PI));
    return (angle > 0) ? angle : 2 * PI + angle;
}

float inner_angle(Vector2D v1, Vector2D v2 = Vector2D(0, 0))
{
    // Returns the inner angle between v1 and v2. By default, it would be the angle with the x axis

    float angle1 = non_negative_angle(atan2(v1.y, v1.x));
    if (v2.is_zero())
        return angle1;
    float angle2 = non_negative_angle(atan2(v2.y, v2.x));
    float angle = max(angle1, angle2) - min(angle1, angle2);
    return (angle < PI) ? angle : (2 * PI - angle);
}

float next_point_controller(Vector2D point, Vector2D position, Vector2D velocity_vector)
{
    // Path controller which return the next direction for the car
    Vector2D pointing_vector = point - position;
    float angle = inner_angle(velocity_vector, pointing_vector);
    return (is_clock_wise_angle(pointing_vector, velocity_vector)) ? angle : -1 * angle;
}

Line getSection(float path[][2], int index)
{
    Vector2D p1(path[index][0], path[index][1]);
    Vector2D p2(path[index + 1][0], path[index + 1][1]);
    return Line(p1, p2);
}

bool finished(Line section, Vector2D position, int index, float threshole=0.05)
{
    if (index+2 < PATH_LENGTH)
        return false;
    return about_to_pass_section(section, position, threshole);
}



class Car
{
public:
    Vector2D position;
    float direction;
    float velocity;

    // Constructor
    Car(const Vector2D &initial_position, float initial_direction, float initial_velocity)
        : position(initial_position), direction(initial_direction), velocity(initial_velocity) {}
    Car(const Vector2D initial_position, const Vector2D initial_velocity)
    {
        this->position = initial_position;
        this->direction = inner_angle(initial_velocity);
        this->velocity = initial_velocity.norm();
    }

    void set_direction(float theta)
    {
        direction = theta;
    }

    float get_direction() const
    {
        return direction;
    }

    Vector2D get_velocity_vector() const
    {
        return Vector2D(cos(direction), sin(direction)) * velocity;
    }

    Vector2D next_position(float dt) const
    {
        return position + get_velocity_vector() * dt;
    }

    void update_position(float dt)
    {
        position = next_position(dt);
    }
};

enum CarState
{
    driving,
    rotating,
    halt
};
int carState = CarState::driving;

int main()
{
    std::ofstream file("data.csv"); // Open the file for writing
    int section_index = 0;
    if (file.is_open())
    {
        // Write the header
        file << "x,y" << std::endl;
        // set up intial
        int max_itterations = 1000;
        Vector2D initial_pos(0, 0);
        Vector2D initial_velocity(-1, 0);
        Vector2D current_pos(0, 0);
        float dt = 0.1;
        Car car(initial_pos, initial_velocity);
        Line current_section = getSection(path, section_index);
        float new_direction;
        // run simulation
        for (int i = 0; i < max_itterations; i++)
        {
            if (finished(current_section, car.position,section_index))
                break;

            // find next section
            if (about_to_pass_section(current_section, car.position) && section_index < PATH_LENGTH){
                section_index++;
                current_section = getSection(path, section_index);
            }
            new_direction = next_point_controller(current_section.p2, car.position, car.get_velocity_vector());
            car.set_direction(car.get_direction() + new_direction / 2);

            std::cout << i << "pos = " << car.position << '\n';
            car.update_position(dt);
            file << car.position.x << "," << car.position.y << std::endl;
        }

        file.close(); // Close the file
        std::cout << "Data written to data.csv" << std::endl;
    }
    else
    {
        std::cerr << "Failed to open the file." << std::endl;
    }

    return 0;
}



// int main()
// {
//     std::ofstream file("data.csv"); // Open the file for writing
//     int section_index = 0;
//     if (file.is_open())
//     {
//         // Write the header
//         file << "x,y" << std::endl;
//         // set up intial
//         int max_itterations = 100;
//         Vector2D initial_pos(0, 0);
//         Vector2D target_point(5, -5);
//         Vector2D initial_velocity(-1, 0);
//         Vector2D current_pos(0, 0);
//         float dt = 0.1;
//         Car car(initial_pos, initial_velocity);
//         Line current_section(initial_pos, target_point);
//         float new_direction;
//         // run simulation
//         for (int i = 0; i < max_itterations; i++)
//         {
//             if (about_to_pass_section(current_section, car.position))
//                 break;
//             new_direction = next_point_controller(target_point, car.position, car.get_velocity_vector());
//             car.set_direction(car.get_direction() + new_direction / 2);

//             std::cout << i << "pos = " << car.position << '\n';
//             car.update_position(dt);
//             file << car.position.x << "," << car.position.y << std::endl;
//         }

//         file.close(); // Close the file
//         std::cout << "Data written to data.csv" << std::endl;
//     }
//     else
//     {
//         std::cerr << "Failed to open the file." << std::endl;
//     }

//     return 0;
// }
