#include "Color.h"
#include <iostream>
#include <iomanip>

using namespace std;

Color::Color() {}

Color::Color(double r, double g, double b)
{
    this->r = r;
    this->g = g;
    this->b = b;
}

Color::Color(const Color &other)
{
    this->r = other.r;
    this->g = other.g;
    this->b = other.b;
}

Color Color::operator-(const Color &other){
    return Color(this->r-other.r, this->g-other.g, this->b-other.b);
}

Color Color::operator+(const Color &other){
    return Color(this->r+other.r, this->g+other.g, this->b+other.b);
}

Color Color::operator/(const double d){
    return Color(this->r/d, this->g/d, this->b/d);
}

Color Color::operator*(const double d){
    return Color(this->r*d, this->g*d, this->b*d);
}

void Color::operator+=(const Color &other){
    this->r+=other.r;
    this->g+=other.g;
    this->b+=other.b;
}

void Color::operator-=(const Color &other){
    this->r-=other.r;
    this->g-=other.g;
    this->b-=other.b;
}

void Color::operator=(const Color &other){
    this->r=other.r;
    this->g=other.g;
    this->b=other.b;
}

ostream& operator<<(ostream& os, const Color& c)
{
    os << fixed << setprecision(0) << "rgb(" << c.r << ", " << c.g << ", " << c.b << ")";
    return os;
}
