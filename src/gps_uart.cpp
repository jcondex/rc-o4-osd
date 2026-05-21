#include "gps.h"

#include "config.h"

#ifndef RC_O4_OSD_HOST_TEST
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "hardware/watchdog.h"
#include "pico/stdlib.h"
#endif

namespace gps {

#ifndef RC_O4_OSD_HOST_TEST
namespace {

constexpr size_t RX_BUF_SIZE = GPS_UART_RX_BUFFER_SIZE;
static_assert((RX_BUF_SIZE & (RX_BUF_SIZE - 1u)) == 0, "GPS UART RX buffer size must be a power of two");

volatile uint8_t g_rx_buf[RX_BUF_SIZE] = {};
volatile uint16_t g_rx_head = 0;
volatile uint16_t g_rx_tail = 0;
volatile uint32_t g_rx_drops = 0;

void gps_uart_irq_handler() {
    while (uart_is_readable(uart1)) {
        const uint8_t byte = uart_getc(uart1);
        const uint16_t next = uint16_t((g_rx_head + 1u) & (RX_BUF_SIZE - 1u));
        if (next == g_rx_tail) {
            ++g_rx_drops;
        } else {
            g_rx_buf[g_rx_head] = byte;
            g_rx_head = next;
        }
    }
}

bool pop_rx_byte(uint8_t &byte) {
    if (g_rx_tail == g_rx_head) return false;
    byte = g_rx_buf[g_rx_tail];
    g_rx_tail = uint16_t((g_rx_tail + 1u) & (RX_BUF_SIZE - 1u));
    return true;
}

uint32_t take_rx_drops() {
    irq_set_enabled(UART1_IRQ, false);
    const uint32_t drops = g_rx_drops;
    g_rx_drops = 0;
    irq_set_enabled(UART1_IRQ, true);
    return drops;
}

void reset_rx_buffer() {
    g_rx_head = 0;
    g_rx_tail = 0;
    g_rx_drops = 0;
}

void write_ubx_frame(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, size_t len) {
    uint8_t header[6] = {0xB5, 0x62, msg_class, msg_id, uint8_t(len), uint8_t(len >> 8)};
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    ubx_checksum(&header[2], 4, ck_a, ck_b);
    for (size_t i = 0; i < len; ++i) {
        ck_a = uint8_t(ck_a + payload[i]);
        ck_b = uint8_t(ck_b + ck_a);
    }
    uart_write_blocking(uart1, header, sizeof(header));
    if (len > 0) uart_write_blocking(uart1, payload, len);
    const uint8_t checksum[2] = {ck_a, ck_b};
    uart_write_blocking(uart1, checksum, sizeof(checksum));
}

bool ubx_wait_ack(uint8_t expect_class, uint8_t expect_id, uint32_t timeout_ms) {
    enum class AckState : uint8_t { Sync1, Sync2, Class, Id, Len1, Len2, PayloadClass, PayloadId, Cka, Ckb };
    AckState st = AckState::Sync1;
    const uint32_t start = to_ms_since_boot(get_absolute_time());

    while ((to_ms_since_boot(get_absolute_time()) - start) < timeout_ms) {
        uint8_t b = 0;
        if (!pop_rx_byte(b)) {
            watchdog_update();
            tight_loop_contents();
            continue;
        }
        switch (st) {
        case AckState::Sync1: st = (b == 0xB5) ? AckState::Sync2 : AckState::Sync1; break;
        case AckState::Sync2: st = (b == 0x62) ? AckState::Class : AckState::Sync1; break;
        case AckState::Class: st = (b == 0x05) ? AckState::Id : AckState::Sync1; break;
        case AckState::Id:
            if (b == 0x00) return false;
            st = (b == 0x01) ? AckState::Len1 : AckState::Sync1;
            break;
        case AckState::Len1: st = (b == 0x02) ? AckState::Len2 : AckState::Sync1; break;
        case AckState::Len2: st = (b == 0x00) ? AckState::PayloadClass : AckState::Sync1; break;
        case AckState::PayloadClass: st = (b == expect_class) ? AckState::PayloadId : AckState::Sync1; break;
        case AckState::PayloadId: st = (b == expect_id) ? AckState::Cka : AckState::Sync1; break;
        case AckState::Cka: st = AckState::Ckb; break;
        case AckState::Ckb: return true;
        }
    }
    return false;
}

bool ubx_send_and_ack(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, size_t len) {
    write_ubx_frame(msg_class, msg_id, payload, len);
    if (ubx_wait_ack(msg_class, msg_id, 500)) return true;
    write_ubx_frame(msg_class, msg_id, payload, len);
    return ubx_wait_ack(msg_class, msg_id, 500);
}

void reinit_gps_uart(uint32_t baud) {
    irq_set_enabled(UART1_IRQ, false);
    uart_set_irq_enables(uart1, false, false);
    uart_deinit(uart1);
    reset_rx_buffer();
    uart_init(uart1, baud);
    gpio_set_function(PIN_UART1_TX, GPIO_FUNC_UART);
    gpio_set_function(PIN_UART1_RX, GPIO_FUNC_UART);
    uart_set_hw_flow(uart1, false, false);
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart1, true);
    irq_set_exclusive_handler(UART1_IRQ, gps_uart_irq_handler);
    irq_set_enabled(UART1_IRQ, true);
    uart_set_irq_enables(uart1, true, false);
}

bool wait_for_nmea(uint32_t timeout_ms) {
    const uint32_t start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start) < timeout_ms) {
        uint8_t b = 0;
        while (pop_rx_byte(b)) {
            if (b == '$') return true;
        }
        watchdog_update();
        tight_loop_contents();
    }
    return false;
}

}

void init_uart() {
    reinit_gps_uart(UART_GPS_BAUD_INIT);
}

void poll_uart(NmeaParser &parser, GpsState &state, uint32_t now_ms) {
    uint8_t byte = 0;
    while (pop_rx_byte(byte)) {
        parser.parse_byte(byte, state, now_ms);
    }
    const uint32_t drops = take_rx_drops();
    if (drops) {
        parser.add_uart_rx_drops(drops);
    }
}

void send_ubx_startup_config() {
    const uint32_t wait_start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - wait_start) < 1000u) {
        watchdog_update();
        sleep_ms(10);
    }

    const uint8_t disable_gll[] = {0xF0, 0x01, 0, 0, 0, 0, 0, 0};
    const uint8_t disable_gsv[] = {0xF0, 0x03, 0, 0, 0, 0, 0, 0};
    const uint8_t disable_gsa[] = {0xF0, 0x02, 0, 0, 0, 0, 0, 0};
    const uint8_t set_5hz[] = {0xC8, 0x00, 0x01, 0x00, 0x01, 0x00};
    const uint8_t set_baud_115200[] = {
        0x01, 0x00, 0x00, 0x00,
        0xD0, 0x08, 0x00, 0x00,
        0x00, 0xC2, 0x01, 0x00,
        0x07, 0x00,
        0x03, 0x00,
        0x00, 0x00,
        0x00, 0x00,
    };

    (void)ubx_send_and_ack(0x06, 0x01, disable_gll, sizeof(disable_gll));
    (void)ubx_send_and_ack(0x06, 0x01, disable_gsv, sizeof(disable_gsv));
    (void)ubx_send_and_ack(0x06, 0x01, disable_gsa, sizeof(disable_gsa));
    (void)ubx_send_and_ack(0x06, 0x08, set_5hz, sizeof(set_5hz));

    write_ubx_frame(0x06, 0x00, set_baud_115200, sizeof(set_baud_115200));
    const uint32_t baud_wait_start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - baud_wait_start) < 100u) {
        watchdog_update();
        sleep_ms(5);
    }
    reinit_gps_uart(UART_GPS_BAUD_FAST);
    if (!wait_for_nmea(500)) {
        reinit_gps_uart(UART_GPS_BAUD_INIT);
    }
}

#else

void init_uart() {}
void poll_uart(NmeaParser &, GpsState &, uint32_t) {}
void send_ubx_startup_config() {}

#endif

}
