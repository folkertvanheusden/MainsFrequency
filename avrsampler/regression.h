#include <fp64lib.h>

class Regression {

  private:
    float64_t sum_x { 0ULL };
    float64_t sum_y { 0ULL };
    float64_t sum_xx { 0ULL };
    float64_t sum_xy { 0ULL };
    uint16_t n { 0 };

    float64_t a { 0ULL };             // slope
    float64_t b { 0ULL };             // intercept on y-axis
    float64_t c { 0ULL };             // intercept on x-axis

  public:
    /**
     * Constructor.
     */
    explicit Regression();

    void reset();
    void add(const float64_t x, const float64_t y);

    // calculates the regression, results are available through getA(), getB(), getC()
    void calculate();

    float64_t getA() const;
    float64_t getB() const;
    float64_t getC() const;
};
