#pragma once

#if defined(MAVLINK_ENABLED)

#include <cstdint>

class MAVLink;

class CRSFMavlinkBridge
{
public:
    void HandleTelemetry(const uint8_t *payload, uint8_t payloadSize, MAVLink &mavlink);
};

#endif
