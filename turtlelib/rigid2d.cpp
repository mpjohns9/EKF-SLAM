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

Twist2D Transform2D::operator()(Twist2D V) const 
{
    Twist2D out;
    out.ang = ang;
    out.x = y*V.ang + cos(ang)*V.x + -sin(ang)*V.y;
    out.y = -x*V.ang + sin(ang)*V.x + cos(ang)*V.y;
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

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
    x = cos(rhs.ang)*x + -sin(rhs.ang)*y + rhs.x;
    y = sin(rhs.ang)*x + cos(rhs.ang)*y + rhs.y;

    return *this;
}

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
    os << "deg: " << tf.rotation() << " x: " << tf.translation().x << " y: " << tf.translation().y;
    return os;
}

int main(void) {

    Transform2D T;

    std::cout << T;
    
    return(0);
}

