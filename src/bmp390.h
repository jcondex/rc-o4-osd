#pragma once

#include <stdint.h>
#include "types.h"

namespace bmp390 {

struct Calibration {
    uint16_t par_t1 = 0;
    uint16_t par_t2 = 0;
    int8_t par_t3 = 0;
    int16_t par_p1 = 0;
    int16_t par_p2 = 0;
    int8_t par_p3 = 0;
    int8_t par_p4 = 0;
    uint16_t par_p5 = 0;
    uint16_t par_p6 = 0;
    int8_t par_p7 = 0;
    int8_t par_p8 = 0;
    int16_t par_p9 = 0;
    int8_t par_p10 = 0;
    int8_t par_p11 = 0;
};

void parse_calibration(const uint8_t raw[21], Calibration &cal);
double compensate_temperature(uint32_t raw_temp, const Calibration &cal, double &t_lin);
double compensate_pressure(uint32_t raw_press, const Calibration &cal, double t_lin);
float pressure_to_altitude(double pressure_pa, double reference_pa);

bool init(BaroState &state);
bool update(BaroState &state, uint32_t now_ms);
void set_zero_reference(BaroState &state);
}
