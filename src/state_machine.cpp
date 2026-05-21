#include "state_machine.h"

namespace state_machine {
namespace {

bool arm_position_selected(const RcState &rc) {
    return rc.arm_switch_middle || rc.arm_switch_high;
}

}

TransitionResult update(TelemetryState &state) {
    TransitionResult result;
    const bool was_armed = state.armed;

    if (state.rc.failsafe) {
        state.app_state = AppState::Failsafe;
        state.armed = false;
    }

    switch (state.app_state) {
    case AppState::Initializing:
        state.armed = false;
        state.app_state = AppState::Disarmed;
        break;
    case AppState::Failsafe:
        state.armed = false;
        if (state.rc.signal_valid) {
            if (state.rc.arm_switch_low) {
                state.rc.rearm_latched = true;
                state.app_state = AppState::Disarmed;
            } else if (arm_position_selected(state.rc) && state.rc.has_armed_once) {
                state.armed = true;
                state.app_state = AppState::Armed;
            } else {
                state.app_state = AppState::Disarmed;
            }
        }
        break;
    case AppState::Disarmed:
        state.armed = false;
        if (state.rc.signal_valid && state.rc.arm_switch_low) {
            state.rc.rearm_latched = true;
        }
        if (state.rc.signal_valid && arm_position_selected(state.rc) && state.rc.rearm_latched) {
            state.armed = true;
            state.rc.has_armed_once = true;
            state.rc.rearm_latched = false;
            state.app_state = AppState::Armed;
        }
        break;
    case AppState::Armed:
        state.armed = true;
        if (state.rc.signal_valid && state.rc.arm_switch_low) {
            state.rc.rearm_latched = true;
            state.armed = false;
            state.app_state = AppState::Disarmed;
        }
        break;
    default:
        break;
    }

    result.armed_rising = !was_armed && state.armed;
    result.armed_falling = was_armed && !state.armed;
    return result;
}

}
