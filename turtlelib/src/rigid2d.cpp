#include <iostream>
#include <cmath>
#include "turtlelib/rigid2d.hpp"

/// \file
/// \brief Implementation of two-dimensional rigid body transformations.

namespace turtlelib
{
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
        return out;
    }

    Twist2D Transform2D::operator()(Twist2D V) const 
    {
        Twist2D out;
        out.ang = V.ang;
        out.x = y*V.ang + cos(ang)*V.x + -sin(ang)*V.y;
        out.y = -x*V.ang + sin(ang)*V.x + cos(ang)*V.y;
        return out;
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
        x = cos(ang)*rhs.x + -sin(ang)*rhs.y + x;
        y = sin(ang)*rhs.x + cos(ang)*rhs.y + y;
        ang = ang + rhs.ang;

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

    Transform2D integrate_twist(Twist2D V)
    {
        Vector2D v;
        
        if (!almost_equal(V.ang, 0.0))
        {
            v.x = V.y/V.ang;
            v.y = -V.x/V.ang;

            Transform2D Tsb(v);
            Transform2D Tss(V.ang);
            return Tsb.inv()*Tss*Tsb;
        }
        else
        {
            Transform2D Tbb(Vector2D{V.x, V.y});

            return Tbb;
        }

        
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    std::ostream & operator<<(std::ostream & os, const Twist2D & V)
    {
        os << "[" << V.ang << " " << V.x << " " << V.y << "]";
        return os;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf) 
    {
        double ang = rad2deg(tf.rotation());
        os << "deg: " << ang << " x: " << tf.translation().x << " y: " << tf.translation().y;
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {

        char peek = is.peek();

        if (peek == '[')
        {
            is.get();
        }

        is >> v.x >> v.y;
        return is;
    }

    std::istream & operator>>(std::istream & is, Twist2D & V)
    {
        char peek = is.peek();

        if (peek == '[')
        {
            is.get();
        }

        is >> V.ang >> V.x >> V.y;

        return is;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {

        Vector2D v;
        double ang;

        if (is.peek() == 'd')
        {
            std::string s1;
            std::string s2;
            std::string s3; 

            is >> s1 >> ang >> s2 >> v.x >> s3 >> v.y;
        }

        else 
        {
            is >> ang >> v.x >> v.y;
        }

        is.get();

        ang = deg2rad(ang);
        Transform2D T(v, ang);

        tf = T;

        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        return lhs*=rhs;
    }

    Vector2D Vector2D::hat(Vector2D v)
    {
        Vector2D v_hat;
        v_hat.x = v.x/sqrt(pow(v.x,2) + pow(v.y,2));
        v_hat.y = v.y/sqrt(pow(v.x,2) + pow(v.y,2));
        return v_hat;
    }

    double Vector2D::dot(Vector2D v1, Vector2D v2)
    {
        return (v1.x*v2.x) + (v1.y*v2.y);
    }

    double Vector2D::magnitude(Vector2D v)
    {
        return sqrt(pow(v.x,2) + pow(v.y,2));
    }

    double Vector2D::angle(Vector2D v1, Vector2D v2)
    {
        return acos(dot(v1, v2)/(magnitude(v1)*magnitude(v2)));
    }

    Vector2D & Vector2D::operator+=(const Vector2D & rhs)
    {
        x = x + rhs.x;
        y = y + rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs)
    {
        x = x - rhs.x;
        y = y - rhs.y;
        return *this;
    }
    Vector2D & Vector2D::operator*=(double rhs)
    {
        x = x * rhs;
        y = y * rhs;
        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
    {
        lhs += rhs;
        return lhs;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
    {
        lhs -= rhs;
        return lhs;
    }
    Vector2D operator*(Vector2D lhs, double rhs)
    {
        lhs *= rhs;
        return lhs;
    }
}

