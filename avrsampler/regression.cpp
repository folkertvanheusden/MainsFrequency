#include "regression.h"

Regression::Regression()
{
    reset();
}

void Regression::reset()
{
    this->sum_x.reset();
    this->sum_y.reset();
    this->sum_xx.reset();
    this->sum_xy.reset();
    this->n = 0;
}

void Regression::add(Double x, Double y)
{
    this->sum_x += x;
    this->sum_y += y;
    this->sum_xx += x * x;
    this->sum_xy += x * y;
    this->n++;
}

void Regression::calculate()
{
    Double nom = (sum_xy * n) - (sum_x * sum_y);
    Double denom = (sum_xx * n) - (sum_x * sum_x);
    this->b = nom / denom;
    this->a = (sum_y - b * sum_x) / n;
    this->c = (sum_x - sum_y / b) / n;
}

Double Regression::getA() const
{
    return a;
}

Double Regression::getB() const
{
    return b;
}

Double Regression::getC() const
{
    return c;
}
