#include "Double.h"

class Regression {

  private:
    Double sum_x;
    Double sum_y;
    Double sum_xx;
    Double sum_xy;
    int n;

    Double a;             // slope
    Double b;             // intercept on y-axis
    Double c;             // intercept on x-axis

  public:
    /**
     * Constructor.
     */
    explicit Regression();

    void reset();
    void add(Double x, Double y);

    // calculates the regression, results are available through getA(), getB(), getC()
    void calculate();

    Double getA() const;
    Double getB() const;
    Double getC() const;
};
