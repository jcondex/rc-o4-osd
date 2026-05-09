#pragma once

#include "msp.h"
#include "types.h"

namespace displayport {

void send_frame(msp::Endpoint endpoint, const TelemetryState &state);
void send_message(msp::Endpoint endpoint, const char *message);

}
