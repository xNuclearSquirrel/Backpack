#pragma once

#if defined(MAVLINK_ENABLED)

#include <Arduino.h>
#include <MAVLink.h>
#include "crsf_protocol.h"

#define MAVLINK_SYSTEM_ID       1
#define MAVLINK_COMPONENT_ID    1
#define MAVLINK_SYSTEM_TYPE     1
#define MAVLINK_AUTOPILOT_TYPE  3
#define MAVLINK_SYSTEM_MODE     64
#define MAVLINK_CUSTOM_MODE     0
#define MAVLINK_SYSTEM_STATE    4
#define MAVLINK_UPTIME          0

class CRSFMavlinkBridge
{
public:
    void HandleTelemetry(const uint8_t *payload, uint8_t payloadSize, MAVLink &mavlink);

private:
    void SendHeartbeat(MAVLink &mavlink);
    void SendGpsRawInt(MAVLink &mavlink, int32_t lat, int32_t lon, int32_t alt, uint16_t groundspeed,
                       uint16_t heading, uint8_t gps_sats);
    void SendGlobalPositionInt(MAVLink &mavlink, int32_t lat, int32_t lon, int32_t alt, int32_t gps_alt,
                               uint16_t heading);
    void SendMavlinkMessage(MAVLink &mavlink, mavlink_message_t &msg);
};

#endif
