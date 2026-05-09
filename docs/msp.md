# MSP Notes

The DJI O4 talks to this firmware over MSP v1 at 115200 baud.

The firmware reports a Betaflight compatible identity:

| Command | Response |
| --- | --- |
| MSP_API_VERSION | 1.42 |
| MSP_FC_VARIANT | BTFL |
| MSP_FC_VERSION | 4.0.0 |
| MSP_BOARD_INFO | PICO |
| MSP_NAME | RC-O4-OSD |

API 1.42 is used for DJI O4 compatibility.

## DisplayPort

The OSD uses MSP_DISPLAYPORT command 182.

The target canvas is 30 columns by 16 rows.

Each frame sends:

* Heartbeat
* Optional clear
* One write command per used row
* Draw command

Only plain ASCII text is used.

## GPS

MSP_RAW_GPS uses the API 1.42 payload without PDOP.

The firmware can send RAW_GPS proactively. Disable this in `src/config.h` if the O4 setup works better with request only behavior:

```c
#define MSP_RAW_GPS_PROACTIVE 0
```

## Battery

The firmware sends both MSP_ANALOG and MSP_BATTERY_STATE.

The default payloads match the legacy API 1.42 layout used by this project. Some DJI and ArduPilot paths use larger payloads. Test with the goggles before changing this.

## Empty Responses

Some configuration commands are acknowledged with an empty response so the O4 does not keep retrying them.

This includes MSP_OSD_CONFIG at this time. Canvas Mode does not require the full Betaflight OSD config table for the current design.
