#include <iostream>
#include <cmath>
#include "rigid2d.hpp"

using namespace turtlelib;


Transform2D::Transform2D()
{
    x = 0;
    y = 0; 
    ang = 0;
}

Transform2D::Transform2D(Vector2D trans)
{
    x = trans.x;
    y = trans.y;
    ang = 0;
}

Transform2D::Transform2D(double radians)
{
    x = 0;
    y = 0;
    ang = radians;
}

Transform2D::Transform2D(Vector2D trans, double radians)
{
    x = trans.x;
    y = trans.y;
    ang = radians;
}

Vector2D Transform2D::operator()(Vector2D v) const 
{
    Vector2D out;
    out.x = cos(ang)*v.x + -sin(ang)*v.y + x;
    out.y = sin(ang)*v.x + cos(ang)*v.y + y;
    return(out);
}

Transform2D Transform2D::inv() const
{
    Transform2D T;
    T.x = -x*cos(ang) - y*sin(ang);
    T.y = -y*cos(ang) + x*sin(ang);
    T.ang = -ang;

    return T;
}

// Transform2D & operator*=(const Transform2D & rhs)
// {
//     Transform2D T;
//     T.x = cos(rhs.ang)*x + -sin(rhs.ang)*y + rhs.x;
//     T.y = sin(rhs.ang)*x + cos(rhs.ang)*y + rhs.y;

//     return T;
// }

Vector2D Transform2D::translation() const
{
    Vector2D v;
    v.x = x;
    v.y = y;

    return v;
}

double Transform2D::rotation() const
{
    return ang;
}


std::ostream & operator<<(std::ostream & os, const Transform2D & tf) 
{
    os << "angle (deg.): " << tf.rotation() << " x: " << tf.translation().x << " y: " << tf.translation().y;
    return os;
}

int main(void) {
    Vector2D trans;
    trans.x = 4;
    trans.y = 5;

    Transform2D T(trans);

    // std::cout << T;
    
    return(0);
}

