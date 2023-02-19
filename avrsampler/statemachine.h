#include <stdbool.h>

#include "regression.h"

typedef enum {
    STATE_IDLE,
    STATE_LOW,
    STATE_SLOPE
} EState;

class StateMachine {

  private:
    double low { 0 };
    double high { 0 };
    EState state { STATE_IDLE };
    double result { 0 };
    Regression regression;

  public:
    /**
     * Constructor.
     */
    explicit StateMachine(const double low, const double high);

    bool process(const double time, const double v);

    double get_result();
};
