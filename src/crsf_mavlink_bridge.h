#pragma once

#if defined(MAVLINK_ENABLED)

#include <cstdint>

class MAVLink;
struct mavlink_message_t;

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
