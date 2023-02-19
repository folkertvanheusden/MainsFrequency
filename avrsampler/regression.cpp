#include "regression.h"

Regression::Regression()
{
    reset();
}

void Regression::reset()
{
    this->sum_x = 0;
    this->sum_y = 0;
    this->sum_xx = 0;
    this->sum_xy = 0;
    this->n = 0;
}

void Regression::add(const float64_t x, const float64_t y)
{
    this->sum_x = fp64_add(sum_x, x);  // this->sum_x += x;
    this->sum_y = fp64_add(sum_y, y);  // this->sum_y += y;
    this->sum_xx = fp64_add(sum_xx, fp64_mul(x, x));  // this->sum_xx += x * x;
    this->sum_xy = fp64_add(sum_xy, fp64_mul(x, y));  // this->sum_xy += x * y;
    this->n++;
}

void Regression::calculate()
{
    float64_t nD = fp64_uint16_to_float64(n);
    float64_t nom = fp64_sub(fp64_mul(sum_xy, nD), fp64_mul(sum_x, sum_y));  // double nom = (sum_xy * n) - (sum_x * sum_y);
    float64_t denom = fp64_sub(fp64_mul(sum_xx, nD), fp64_mul(sum_x, sum_x));  // double denom = (sum_xx * n) - (sum_x * sum_x);
    this->b = fp64_div(nom, denom);  // this->b = nom / denom;
    this->a = fp64_div(fp64_sub(sum_y, fp64_mul(b, sum_x)), nD);  // this->a = (sum_y - b * sum_x) / n;
    this->c = fp64_div(fp64_sub(sum_x, fp64_div(sum_y, b)), nD);  // this->c = (sum_x - sum_y / b) / n;
}

float64_t Regression::getA() const
{
    return a;
}

float64_t Regression::getB() const
{
    return b;
}

float64_t Regression::getC() const
{
    return c;
}
