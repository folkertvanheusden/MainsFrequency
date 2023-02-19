#include <fp64lib.h>
#include "statemachine.h"

StateMachine::StateMachine(const float64_t low, const float64_t high)
{
    this->low   = low;
    this->high  = high;
    this->state = STATE_IDLE;
}

bool StateMachine::process(const float64_t time, const float64_t v)
{
    switch (state) {
    case STATE_IDLE:
	if (fp64_compare(v, low) == -1) {
            this->state = STATE_LOW;
        }
        break;
    case STATE_LOW:
	if (fp64_compare(v, low) == +1) {
            // start linear regression
            this->regression.reset();
            this->regression.add(time, v);
            // start collecting slope data
            this->state = STATE_SLOPE;
        }
        break;
    case STATE_SLOPE:
	if (fp64_compare(v, high) == +1) {
            // end linear regression
            this->regression.calculate();
            this->result = this->regression.getC();
            this->state = STATE_IDLE;
            return true;
        } else {
            this->regression.add(time, v);
        }
        break;
    default:
        this->state = STATE_IDLE;
        break;
    }
    return false;
}

float64_t StateMachine::get_result()
{
    return result;
}
