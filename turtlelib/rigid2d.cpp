#include <iostream>
#include <cmath>
#include "rigid2d.hpp"

using turtlelib::Vector2D;
using turtlelib::Transform2D;


Transform2D::Transform2D()
{
    double mat[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
}

Transform2D::Transform2D(Vector2D trans)
{
    double mat[3][3] = {
        {1, 0, trans.x},
        {0, 1, trans.y},
        {0, 0, 1}
    };
}

Transform2D::Transform2D(double radians)
{
    double mat[3][3] = {
        {cos(radians), -sin(radians), 0},
        {sin(radians), cos(radians), 0},
        {0, 0, 1}
    };
}

Transform2D::Transform2D(Vector2D trans, double radians)
{
    double mat[3][3] = {
        {cos(radians), -sin(radians), trans.x},
        {sin(radians), cos(radians), trans.y},
        {0, 0, 1}
    };
}

int main(void) {
    double d1 = 1.0;
    double d2 = 1.0;

    if (turtlelib::almost_equal(d1, d2)) {
        printf("SUCCESS");
    }
    return(0);
}

Vector2D Transform2D::operator()(Vector2D v) const
{
    double mat;
    mat = {
        {1, 0, 0, v.x}
    }
}
