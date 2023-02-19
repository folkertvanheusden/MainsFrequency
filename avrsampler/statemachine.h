#include <stdbool.h>

#include "regression.h"

typedef enum {
    STATE_IDLE,
    STATE_LOW,
    STATE_SLOPE
} EState;

class StateMachine {

  private:
    Double low;
    Double high;
    EState state { STATE_IDLE };
    Double result;
    Regression regression;

  public:
    /**
     * Constructor.
     */
    explicit StateMachine(const Double low, const Double high);

    bool process(const Double time, const Double v);

    Double get_result();
};
