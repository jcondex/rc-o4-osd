#pragma once

#include "types.h"

namespace state_machine {

struct TransitionResult {
    bool armed_rising = false;
    bool armed_falling = false;
};

TransitionResult update(TelemetryState &state);

}
