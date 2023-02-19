#include "regression.h"

Regression::Regression()
{
    reset();
}

void Regression::reset()
{
    this->sum_x = 0.0;
    this->sum_y = 0.0;
    this->sum_xx = 0.0;
    this->sum_xy = 0.0;
    this->n = 0;
}

void Regression::add(double x, double y)
{
    this->sum_x += x;
    this->sum_y += y;
    this->sum_xx += x * x;
    this->sum_xy += x * y;
    this->n++;
}

void Regression::calculate()
{
    double nom = (sum_xy * n) - (sum_x * sum_y);
    double denom = (sum_xx * n) - (sum_x * sum_x);
    this->b = nom / denom;
    this->a = (sum_y - b * sum_x) / n;
    this->c = (sum_x - sum_y / b) / n;
}

double Regression::getA() const
{
    return a;
}

double Regression::getB() const
{
    return b;
}

double Regression::getC() const
{
    return c;
}

