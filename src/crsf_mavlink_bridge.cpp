#include "crsf_mavlink_bridge.h"

#if defined(MAVLINK_ENABLED)

#include <MAVLink.h>

#include "crsf_protocol.h"
#include "logging.h"

#define MAVLINK_SYSTEM_ID       1
#define MAVLINK_COMPONENT_ID    1
#define MAVLINK_SYSTEM_TYPE     1
#define MAVLINK_AUTOPILOT_TYPE  3
#define MAVLINK_SYSTEM_MODE     64
#define MAVLINK_CUSTOM_MODE     0
#define MAVLINK_SYSTEM_STATE    4
#define MAVLINK_UPTIME          0

namespace
{
void SendMavlinkMessage(MAVLink &mavlink, mavlink_message_t &msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    for (uint16_t i = 0; i < len; i++)
    {
        mavlink.ProcessMAVLinkFromTX(buf[i]);
    }
}

void SendHeartbeat(MAVLink &mavlink)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                               MAVLINK_SYSTEM_TYPE, MAVLINK_AUTOPILOT_TYPE,
                               MAVLINK_SYSTEM_MODE, MAVLINK_CUSTOM_MODE,
                               MAVLINK_SYSTEM_STATE);
    SendMavlinkMessage(mavlink, msg);
}

void SendGpsRawInt(MAVLink &mavlink, int32_t lat, int32_t lon, int32_t alt,
                   uint16_t groundspeed, uint16_t heading, uint8_t gps_sats)
{
    mavlink_message_t msg;
    const uint16_t eph = UINT16_MAX;
    const uint16_t epv = UINT16_MAX;
    const uint16_t cog = UINT16_MAX;
    const uint32_t alt_ellipsoid = 0;
    const uint32_t h_acc = 0;
    const uint32_t v_acc = 0;
    const uint32_t vel_acc = 0;
    const uint32_t hdg_acc = 0;
    const uint8_t fixType = 3;

    mavlink_msg_gps_raw_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                                 MAVLINK_UPTIME, fixType, lat, lon, alt,
                                 eph, epv, groundspeed, cog, gps_sats,
                                 alt_ellipsoid, h_acc, v_acc, vel_acc, hdg_acc,
                                 heading);
    SendMavlinkMessage(mavlink, msg);
}

void SendGlobalPositionInt(MAVLink &mavlink, int32_t lat, int32_t lon,
                           int32_t alt, int32_t gps_alt, uint16_t heading)
{
    mavlink_message_t msg;
    const int16_t velx = 0;
    const int16_t vely = 0;
    const int16_t velz = 0;

    mavlink_msg_global_position_int_pack(MAVLINK_SYSTEM_ID, MAVLINK_COMPONENT_ID, &msg,
                                         MAVLINK_UPTIME, lat, lon, gps_alt, alt,
                                         velx, vely, velz, heading);
    SendMavlinkMessage(mavlink, msg);
}
} // namespace

void CRSFMavlinkBridge::HandleTelemetry(const uint8_t *payload, uint8_t payloadSize, MAVLink &mavlink)
{
    if (payloadSize < 4)
    {
        DBGLN("CRSF_TLM packet too short for MAVLink");
        return;
    }

    uint8_t frameType = payload[2];
    if (frameType != CRSF_FRAMETYPE_GPS)
    {
        return;
    }

    if (payloadSize < sizeof(crsf_packet_gps_t))
    {
        DBGLN("CRSF_TLM GPS packet too short for MAVLink");
        return;
    }

    const crsf_packet_gps_t *packet = reinterpret_cast<const crsf_packet_gps_t *>(payload);

    int32_t lat = static_cast<int32_t>(be32toh(packet->p.lat));
    int32_t lon = static_cast<int32_t>(be32toh(packet->p.lon));
    uint16_t speed = be16toh(packet->p.speed);
    uint16_t heading = be16toh(packet->p.heading);
    uint16_t altitude = be16toh(packet->p.altitude);
    uint8_t gps_sats = packet->p.satcnt;

    const uint16_t groundspeed = static_cast<uint16_t>(speed / 36.0 * 100.0);
    const int32_t alt = static_cast<int32_t>((static_cast<int32_t>(altitude) - 1000) * 1000);
    const int32_t gps_alt = alt;

    SendHeartbeat(mavlink);
    SendGpsRawInt(mavlink, lat, lon, alt, groundspeed, heading, gps_sats);
    SendGlobalPositionInt(mavlink, lat, lon, alt, gps_alt, heading);
}

#endif
