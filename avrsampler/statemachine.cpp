#include <fp64lib.h>
#include "statemachine.h"

StateMachine::StateMachine(const Double low, const Double high)
{
    this->low   = low;
    this->high  = high;
    this->state = STATE_IDLE;
}

bool StateMachine::process(const Double time, const Double v)
{
    switch (state) {
    case STATE_IDLE:
	if (fp64_compare(v.raw(), low.raw()) == -1) {
            this->state = STATE_LOW;
        }
        break;
    case STATE_LOW:
	if (fp64_compare(v.raw(), low.raw()) == +1) {
            // start linear regression
            this->regression.reset();
            this->regression.add(time, v);
            // start collecting slope data
            this->state = STATE_SLOPE;
        }
        break;
    case STATE_SLOPE:
	if (fp64_compare(v.raw(), high.raw()) == +1) {
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

Double StateMachine::get_result()
{
    return result;
}
