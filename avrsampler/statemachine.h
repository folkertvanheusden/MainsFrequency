#include <stdbool.h>

#include "regression.h"

typedef enum {
    STATE_IDLE,
    STATE_LOW,
    STATE_SLOPE
} EState;

class StateMachine {

  private:
    float64_t low { 0 };
    float64_t high { 0 };
    EState state { STATE_IDLE };
    float64_t result { 0 };
    Regression regression;

  public:
    /**
     * Constructor.
     */
    explicit StateMachine(const float64_t low, const float64_t high);

    bool process(const float64_t time, const float64_t v);

    float64_t get_result();
};
