#include "telemetry_debug.h"

#include <stdio.h>

#include "config.h"

namespace telemetry_debug {
namespace {
uint32_t g_last_print_ms = 0;
}

void print(uint32_t now_ms, const TelemetryState &state, const msp::Stats &msp_rx, const msp::Stats &msp_tx, const gps::Stats &gps_stats) {
#if DEBUG_USB_SERIAL
    const uint32_t interval_ms = 1000u / DEBUG_PRINT_HZ;
    if (now_ms - g_last_print_ms < interval_ms) return;
    g_last_print_ms = now_ms;

    printf("state=%u armed=%u pwm=%u raw=%u sw=%u pin=%u edges=%lu irq=%lu pio=%lu rej=%lu rc=%u fs=%u gps=%u gps_rx=%u sats=%u vbat=%.2f baro=%u baro_id=0x%02x baro_addr=0x%02x baro_err=0x%02x baro_st=0x%02x baro_raw=%lu/%lu baro_pa=%.1f baro_c=%.1f imu=%u mag=%u alt=%.1f pitch=%.1f roll=%.1f hdg=%.1f msp_rx=%lu msp_tx=%lu msp_crc=%lu msp_unk=%lu msp_last=%u msp_bad=%u/%u msp_name=%lu osd_cfg=%lu osd_can=%lu set_can=%lu can=%u/%u dp_tx=%lu gps_b=%lu gps_$=%lu gps_nl=%lu gps_sent=%lu gps_ok=%lu gps_rmc=%lu gps_gga=%lu gps_vtg=%lu gps_unsup=%lu gps_crc=%lu gps_good=%s gps_bad=%s gps_ubx=%lu gps_pvt=%lu gps_ubx_crc=%lu gps_ubx_ovf=%lu gps_ovf=%lu gps_drop=%lu i2c_fail=%lu/%lu/%lu\n",
        unsigned(state.app_state),
        state.armed ? 1u : 0u,
        unsigned(state.rc.pulse_us),
        unsigned(state.rc.last_raw_pulse_us),
        unsigned(state.rc.sw_pulse_us),
        state.rc.pwm_pin_level ? 1u : 0u,
        static_cast<unsigned long>(state.rc.sw_edges),
        static_cast<unsigned long>(state.rc.irq_pulses),
        static_cast<unsigned long>(state.rc.pio_pulses),
        static_cast<unsigned long>(state.rc.pio_rejected),
        state.rc.signal_valid ? 1u : 0u,
        state.rc.failsafe ? 1u : 0u,
        state.gps.fix_valid ? 1u : 0u,
        state.gps.receiving ? 1u : 0u,
        unsigned(state.gps.satellites),
        double(state.battery.volts),
        state.baro.valid ? 1u : 0u,
        unsigned(state.baro.chip_id),
        unsigned(state.baro.i2c_addr),
        unsigned(state.baro.err_reg),
        unsigned(state.baro.status_reg),
        static_cast<unsigned long>(state.baro.raw_pressure),
        static_cast<unsigned long>(state.baro.raw_temperature),
        double(state.baro.pressure_pa),
        double(state.baro.temperature_c),
        state.imu.valid ? 1u : 0u,
        state.compass.valid ? 1u : 0u,
        double(state.baro.relative_altitude_m),
        double(state.imu.pitch_deg),
        double(state.imu.roll_deg),
        double(state.compass.heading_deg),
        static_cast<unsigned long>(msp_rx.rx_packets),
        static_cast<unsigned long>(msp_tx.tx_packets),
        static_cast<unsigned long>(msp_rx.checksum_errors),
        static_cast<unsigned long>(msp_rx.unknown_commands),
        unsigned(msp_rx.last_cmd),
        unsigned(msp_rx.last_bad_cmd),
        unsigned(msp_rx.last_bad_len),
        static_cast<unsigned long>(msp_rx.name_requests),
        static_cast<unsigned long>(msp_rx.osd_config_requests),
        static_cast<unsigned long>(msp_rx.osd_canvas_requests),
        static_cast<unsigned long>(msp_rx.set_osd_canvas_requests),
        unsigned(msp_rx.last_canvas_cols),
        unsigned(msp_rx.last_canvas_rows),
        static_cast<unsigned long>(msp_tx.displayport_packets),
        static_cast<unsigned long>(gps_stats.bytes),
        static_cast<unsigned long>(gps_stats.dollar_signs),
        static_cast<unsigned long>(gps_stats.newlines),
        static_cast<unsigned long>(gps_stats.sentences),
        static_cast<unsigned long>(gps_stats.parsed_sentences),
        static_cast<unsigned long>(gps_stats.rmc_sentences),
        static_cast<unsigned long>(gps_stats.gga_sentences),
        static_cast<unsigned long>(gps_stats.vtg_sentences),
        static_cast<unsigned long>(gps_stats.unsupported_sentences),
        static_cast<unsigned long>(gps_stats.checksum_errors),
        gps_stats.last_good_type[0] ? gps_stats.last_good_type : "---",
        gps_stats.last_bad_type[0] ? gps_stats.last_bad_type : "---",
        static_cast<unsigned long>(gps_stats.ubx_frames),
        static_cast<unsigned long>(gps_stats.ubx_nav_pvt),
        static_cast<unsigned long>(gps_stats.ubx_checksum_errors),
        static_cast<unsigned long>(gps_stats.ubx_oversize),
        static_cast<unsigned long>(gps_stats.overflows),
        static_cast<unsigned long>(gps_stats.uart_rx_drops),
        static_cast<unsigned long>(state.sensor_failures.imu),
        static_cast<unsigned long>(state.sensor_failures.baro),
        static_cast<unsigned long>(state.sensor_failures.compass));
#else
    (void)now_ms;
    (void)state;
    (void)msp_rx;
    (void)msp_tx;
    (void)gps_stats;
#endif
}

}
