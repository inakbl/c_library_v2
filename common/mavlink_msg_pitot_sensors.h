#pragma once
// MESSAGE PITOT_SENSORS PACKING

#define MAVLINK_MSG_ID_PITOT_SENSORS 615

MAVPACKED(
typedef struct __mavlink_pitot_sensors_t {
 float pt_rudder; /*< [Pa]  Total pressure of pitot 1 */
 float ps_rudder; /*< [Pa]  Static pressure of pitot 1 */
 float vel_rudder[3]; /*< [m/s]  3D array fluid velocity of pitot 1 */
 float atm_rudder; /*< [Pa]  Atmospheric pressure of pitot 1 */
 float pt_starboard; /*< [Pa]  Total pressure of pitot 2 */
 float ps_starboard; /*< [Pa]  Static pressure of pitot 2 */
 float vel_starboard[3]; /*< [m/s]  3D array fluid velocity of pitot 2 */
 float atm_starboard; /*< [Pa]  Atmospheric pressure of pitot 2 */
 float pt_port; /*< [Pa]  Total pressure of pitot 3 */
 float ps_port; /*< [Pa]  Static pressure of pitot 3 */
 float vel_port[3]; /*< [m/s]  3D array fluid velocity of pitot 3 */
 float atm_port; /*< [Pa]  Atmospheric pressure of pitot 3 */
 float pressure_atm; /*< [Pa]  Atmospheric pressure (average between the three pitot values) */
}) mavlink_pitot_sensors_t;

#define MAVLINK_MSG_ID_PITOT_SENSORS_LEN 76
#define MAVLINK_MSG_ID_PITOT_SENSORS_MIN_LEN 76
#define MAVLINK_MSG_ID_615_LEN 76
#define MAVLINK_MSG_ID_615_MIN_LEN 76

#define MAVLINK_MSG_ID_PITOT_SENSORS_CRC 81
#define MAVLINK_MSG_ID_615_CRC 81

#define MAVLINK_MSG_PITOT_SENSORS_FIELD_VEL_RUDDER_LEN 3
#define MAVLINK_MSG_PITOT_SENSORS_FIELD_VEL_STARBOARD_LEN 3
#define MAVLINK_MSG_PITOT_SENSORS_FIELD_VEL_PORT_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PITOT_SENSORS { \
    615, \
    "PITOT_SENSORS", \
    13, \
    {  { "pt_rudder", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_pitot_sensors_t, pt_rudder) }, \
         { "ps_rudder", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pitot_sensors_t, ps_rudder) }, \
         { "vel_rudder", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_pitot_sensors_t, vel_rudder) }, \
         { "atm_rudder", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_pitot_sensors_t, atm_rudder) }, \
         { "pt_starboard", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_pitot_sensors_t, pt_starboard) }, \
         { "ps_starboard", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_pitot_sensors_t, ps_starboard) }, \
         { "vel_starboard", NULL, MAVLINK_TYPE_FLOAT, 3, 32, offsetof(mavlink_pitot_sensors_t, vel_starboard) }, \
         { "atm_starboard", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_pitot_sensors_t, atm_starboard) }, \
         { "pt_port", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_pitot_sensors_t, pt_port) }, \
         { "ps_port", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_pitot_sensors_t, ps_port) }, \
         { "vel_port", NULL, MAVLINK_TYPE_FLOAT, 3, 56, offsetof(mavlink_pitot_sensors_t, vel_port) }, \
         { "atm_port", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_pitot_sensors_t, atm_port) }, \
         { "pressure_atm", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_pitot_sensors_t, pressure_atm) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PITOT_SENSORS { \
    "PITOT_SENSORS", \
    13, \
    {  { "pt_rudder", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_pitot_sensors_t, pt_rudder) }, \
         { "ps_rudder", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pitot_sensors_t, ps_rudder) }, \
         { "vel_rudder", NULL, MAVLINK_TYPE_FLOAT, 3, 8, offsetof(mavlink_pitot_sensors_t, vel_rudder) }, \
         { "atm_rudder", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_pitot_sensors_t, atm_rudder) }, \
         { "pt_starboard", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_pitot_sensors_t, pt_starboard) }, \
         { "ps_starboard", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_pitot_sensors_t, ps_starboard) }, \
         { "vel_starboard", NULL, MAVLINK_TYPE_FLOAT, 3, 32, offsetof(mavlink_pitot_sensors_t, vel_starboard) }, \
         { "atm_starboard", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_pitot_sensors_t, atm_starboard) }, \
         { "pt_port", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_pitot_sensors_t, pt_port) }, \
         { "ps_port", NULL, MAVLINK_TYPE_FLOAT, 0, 52, offsetof(mavlink_pitot_sensors_t, ps_port) }, \
         { "vel_port", NULL, MAVLINK_TYPE_FLOAT, 3, 56, offsetof(mavlink_pitot_sensors_t, vel_port) }, \
         { "atm_port", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_pitot_sensors_t, atm_port) }, \
         { "pressure_atm", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_pitot_sensors_t, pressure_atm) }, \
         } \
}
#endif

/**
 * @brief Pack a pitot_sensors message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pt_rudder [Pa]  Total pressure of pitot 1 
 * @param ps_rudder [Pa]  Static pressure of pitot 1 
 * @param vel_rudder [m/s]  3D array fluid velocity of pitot 1 
 * @param atm_rudder [Pa]  Atmospheric pressure of pitot 1 
 * @param pt_starboard [Pa]  Total pressure of pitot 2 
 * @param ps_starboard [Pa]  Static pressure of pitot 2 
 * @param vel_starboard [m/s]  3D array fluid velocity of pitot 2 
 * @param atm_starboard [Pa]  Atmospheric pressure of pitot 2 
 * @param pt_port [Pa]  Total pressure of pitot 3 
 * @param ps_port [Pa]  Static pressure of pitot 3 
 * @param vel_port [m/s]  3D array fluid velocity of pitot 3 
 * @param atm_port [Pa]  Atmospheric pressure of pitot 3 
 * @param pressure_atm [Pa]  Atmospheric pressure (average between the three pitot values) 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pitot_sensors_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float pt_rudder, float ps_rudder, const float *vel_rudder, float atm_rudder, float pt_starboard, float ps_starboard, const float *vel_starboard, float atm_starboard, float pt_port, float ps_port, const float *vel_port, float atm_port, float pressure_atm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PITOT_SENSORS_LEN];
    _mav_put_float(buf, 0, pt_rudder);
    _mav_put_float(buf, 4, ps_rudder);
    _mav_put_float(buf, 20, atm_rudder);
    _mav_put_float(buf, 24, pt_starboard);
    _mav_put_float(buf, 28, ps_starboard);
    _mav_put_float(buf, 44, atm_starboard);
    _mav_put_float(buf, 48, pt_port);
    _mav_put_float(buf, 52, ps_port);
    _mav_put_float(buf, 68, atm_port);
    _mav_put_float(buf, 72, pressure_atm);
    _mav_put_float_array(buf, 8, vel_rudder, 3);
    _mav_put_float_array(buf, 32, vel_starboard, 3);
    _mav_put_float_array(buf, 56, vel_port, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PITOT_SENSORS_LEN);
#else
    mavlink_pitot_sensors_t packet;
    packet.pt_rudder = pt_rudder;
    packet.ps_rudder = ps_rudder;
    packet.atm_rudder = atm_rudder;
    packet.pt_starboard = pt_starboard;
    packet.ps_starboard = ps_starboard;
    packet.atm_starboard = atm_starboard;
    packet.pt_port = pt_port;
    packet.ps_port = ps_port;
    packet.atm_port = atm_port;
    packet.pressure_atm = pressure_atm;
    mav_array_memcpy(packet.vel_rudder, vel_rudder, sizeof(float)*3);
    mav_array_memcpy(packet.vel_starboard, vel_starboard, sizeof(float)*3);
    mav_array_memcpy(packet.vel_port, vel_port, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PITOT_SENSORS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PITOT_SENSORS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PITOT_SENSORS_MIN_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_CRC);
}

/**
 * @brief Pack a pitot_sensors message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pt_rudder [Pa]  Total pressure of pitot 1 
 * @param ps_rudder [Pa]  Static pressure of pitot 1 
 * @param vel_rudder [m/s]  3D array fluid velocity of pitot 1 
 * @param atm_rudder [Pa]  Atmospheric pressure of pitot 1 
 * @param pt_starboard [Pa]  Total pressure of pitot 2 
 * @param ps_starboard [Pa]  Static pressure of pitot 2 
 * @param vel_starboard [m/s]  3D array fluid velocity of pitot 2 
 * @param atm_starboard [Pa]  Atmospheric pressure of pitot 2 
 * @param pt_port [Pa]  Total pressure of pitot 3 
 * @param ps_port [Pa]  Static pressure of pitot 3 
 * @param vel_port [m/s]  3D array fluid velocity of pitot 3 
 * @param atm_port [Pa]  Atmospheric pressure of pitot 3 
 * @param pressure_atm [Pa]  Atmospheric pressure (average between the three pitot values) 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pitot_sensors_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float pt_rudder,float ps_rudder,const float *vel_rudder,float atm_rudder,float pt_starboard,float ps_starboard,const float *vel_starboard,float atm_starboard,float pt_port,float ps_port,const float *vel_port,float atm_port,float pressure_atm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PITOT_SENSORS_LEN];
    _mav_put_float(buf, 0, pt_rudder);
    _mav_put_float(buf, 4, ps_rudder);
    _mav_put_float(buf, 20, atm_rudder);
    _mav_put_float(buf, 24, pt_starboard);
    _mav_put_float(buf, 28, ps_starboard);
    _mav_put_float(buf, 44, atm_starboard);
    _mav_put_float(buf, 48, pt_port);
    _mav_put_float(buf, 52, ps_port);
    _mav_put_float(buf, 68, atm_port);
    _mav_put_float(buf, 72, pressure_atm);
    _mav_put_float_array(buf, 8, vel_rudder, 3);
    _mav_put_float_array(buf, 32, vel_starboard, 3);
    _mav_put_float_array(buf, 56, vel_port, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PITOT_SENSORS_LEN);
#else
    mavlink_pitot_sensors_t packet;
    packet.pt_rudder = pt_rudder;
    packet.ps_rudder = ps_rudder;
    packet.atm_rudder = atm_rudder;
    packet.pt_starboard = pt_starboard;
    packet.ps_starboard = ps_starboard;
    packet.atm_starboard = atm_starboard;
    packet.pt_port = pt_port;
    packet.ps_port = ps_port;
    packet.atm_port = atm_port;
    packet.pressure_atm = pressure_atm;
    mav_array_memcpy(packet.vel_rudder, vel_rudder, sizeof(float)*3);
    mav_array_memcpy(packet.vel_starboard, vel_starboard, sizeof(float)*3);
    mav_array_memcpy(packet.vel_port, vel_port, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PITOT_SENSORS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PITOT_SENSORS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PITOT_SENSORS_MIN_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_CRC);
}

/**
 * @brief Encode a pitot_sensors struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pitot_sensors C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pitot_sensors_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pitot_sensors_t* pitot_sensors)
{
    return mavlink_msg_pitot_sensors_pack(system_id, component_id, msg, pitot_sensors->pt_rudder, pitot_sensors->ps_rudder, pitot_sensors->vel_rudder, pitot_sensors->atm_rudder, pitot_sensors->pt_starboard, pitot_sensors->ps_starboard, pitot_sensors->vel_starboard, pitot_sensors->atm_starboard, pitot_sensors->pt_port, pitot_sensors->ps_port, pitot_sensors->vel_port, pitot_sensors->atm_port, pitot_sensors->pressure_atm);
}

/**
 * @brief Encode a pitot_sensors struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pitot_sensors C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pitot_sensors_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pitot_sensors_t* pitot_sensors)
{
    return mavlink_msg_pitot_sensors_pack_chan(system_id, component_id, chan, msg, pitot_sensors->pt_rudder, pitot_sensors->ps_rudder, pitot_sensors->vel_rudder, pitot_sensors->atm_rudder, pitot_sensors->pt_starboard, pitot_sensors->ps_starboard, pitot_sensors->vel_starboard, pitot_sensors->atm_starboard, pitot_sensors->pt_port, pitot_sensors->ps_port, pitot_sensors->vel_port, pitot_sensors->atm_port, pitot_sensors->pressure_atm);
}

/**
 * @brief Send a pitot_sensors message
 * @param chan MAVLink channel to send the message
 *
 * @param pt_rudder [Pa]  Total pressure of pitot 1 
 * @param ps_rudder [Pa]  Static pressure of pitot 1 
 * @param vel_rudder [m/s]  3D array fluid velocity of pitot 1 
 * @param atm_rudder [Pa]  Atmospheric pressure of pitot 1 
 * @param pt_starboard [Pa]  Total pressure of pitot 2 
 * @param ps_starboard [Pa]  Static pressure of pitot 2 
 * @param vel_starboard [m/s]  3D array fluid velocity of pitot 2 
 * @param atm_starboard [Pa]  Atmospheric pressure of pitot 2 
 * @param pt_port [Pa]  Total pressure of pitot 3 
 * @param ps_port [Pa]  Static pressure of pitot 3 
 * @param vel_port [m/s]  3D array fluid velocity of pitot 3 
 * @param atm_port [Pa]  Atmospheric pressure of pitot 3 
 * @param pressure_atm [Pa]  Atmospheric pressure (average between the three pitot values) 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pitot_sensors_send(mavlink_channel_t chan, float pt_rudder, float ps_rudder, const float *vel_rudder, float atm_rudder, float pt_starboard, float ps_starboard, const float *vel_starboard, float atm_starboard, float pt_port, float ps_port, const float *vel_port, float atm_port, float pressure_atm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PITOT_SENSORS_LEN];
    _mav_put_float(buf, 0, pt_rudder);
    _mav_put_float(buf, 4, ps_rudder);
    _mav_put_float(buf, 20, atm_rudder);
    _mav_put_float(buf, 24, pt_starboard);
    _mav_put_float(buf, 28, ps_starboard);
    _mav_put_float(buf, 44, atm_starboard);
    _mav_put_float(buf, 48, pt_port);
    _mav_put_float(buf, 52, ps_port);
    _mav_put_float(buf, 68, atm_port);
    _mav_put_float(buf, 72, pressure_atm);
    _mav_put_float_array(buf, 8, vel_rudder, 3);
    _mav_put_float_array(buf, 32, vel_starboard, 3);
    _mav_put_float_array(buf, 56, vel_port, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PITOT_SENSORS, buf, MAVLINK_MSG_ID_PITOT_SENSORS_MIN_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_CRC);
#else
    mavlink_pitot_sensors_t packet;
    packet.pt_rudder = pt_rudder;
    packet.ps_rudder = ps_rudder;
    packet.atm_rudder = atm_rudder;
    packet.pt_starboard = pt_starboard;
    packet.ps_starboard = ps_starboard;
    packet.atm_starboard = atm_starboard;
    packet.pt_port = pt_port;
    packet.ps_port = ps_port;
    packet.atm_port = atm_port;
    packet.pressure_atm = pressure_atm;
    mav_array_memcpy(packet.vel_rudder, vel_rudder, sizeof(float)*3);
    mav_array_memcpy(packet.vel_starboard, vel_starboard, sizeof(float)*3);
    mav_array_memcpy(packet.vel_port, vel_port, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PITOT_SENSORS, (const char *)&packet, MAVLINK_MSG_ID_PITOT_SENSORS_MIN_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_CRC);
#endif
}

/**
 * @brief Send a pitot_sensors message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_pitot_sensors_send_struct(mavlink_channel_t chan, const mavlink_pitot_sensors_t* pitot_sensors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pitot_sensors_send(chan, pitot_sensors->pt_rudder, pitot_sensors->ps_rudder, pitot_sensors->vel_rudder, pitot_sensors->atm_rudder, pitot_sensors->pt_starboard, pitot_sensors->ps_starboard, pitot_sensors->vel_starboard, pitot_sensors->atm_starboard, pitot_sensors->pt_port, pitot_sensors->ps_port, pitot_sensors->vel_port, pitot_sensors->atm_port, pitot_sensors->pressure_atm);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PITOT_SENSORS, (const char *)pitot_sensors, MAVLINK_MSG_ID_PITOT_SENSORS_MIN_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_CRC);
#endif
}

#if MAVLINK_MSG_ID_PITOT_SENSORS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pitot_sensors_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float pt_rudder, float ps_rudder, const float *vel_rudder, float atm_rudder, float pt_starboard, float ps_starboard, const float *vel_starboard, float atm_starboard, float pt_port, float ps_port, const float *vel_port, float atm_port, float pressure_atm)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, pt_rudder);
    _mav_put_float(buf, 4, ps_rudder);
    _mav_put_float(buf, 20, atm_rudder);
    _mav_put_float(buf, 24, pt_starboard);
    _mav_put_float(buf, 28, ps_starboard);
    _mav_put_float(buf, 44, atm_starboard);
    _mav_put_float(buf, 48, pt_port);
    _mav_put_float(buf, 52, ps_port);
    _mav_put_float(buf, 68, atm_port);
    _mav_put_float(buf, 72, pressure_atm);
    _mav_put_float_array(buf, 8, vel_rudder, 3);
    _mav_put_float_array(buf, 32, vel_starboard, 3);
    _mav_put_float_array(buf, 56, vel_port, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PITOT_SENSORS, buf, MAVLINK_MSG_ID_PITOT_SENSORS_MIN_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_CRC);
#else
    mavlink_pitot_sensors_t *packet = (mavlink_pitot_sensors_t *)msgbuf;
    packet->pt_rudder = pt_rudder;
    packet->ps_rudder = ps_rudder;
    packet->atm_rudder = atm_rudder;
    packet->pt_starboard = pt_starboard;
    packet->ps_starboard = ps_starboard;
    packet->atm_starboard = atm_starboard;
    packet->pt_port = pt_port;
    packet->ps_port = ps_port;
    packet->atm_port = atm_port;
    packet->pressure_atm = pressure_atm;
    mav_array_memcpy(packet->vel_rudder, vel_rudder, sizeof(float)*3);
    mav_array_memcpy(packet->vel_starboard, vel_starboard, sizeof(float)*3);
    mav_array_memcpy(packet->vel_port, vel_port, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PITOT_SENSORS, (const char *)packet, MAVLINK_MSG_ID_PITOT_SENSORS_MIN_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_LEN, MAVLINK_MSG_ID_PITOT_SENSORS_CRC);
#endif
}
#endif

#endif

// MESSAGE PITOT_SENSORS UNPACKING


/**
 * @brief Get field pt_rudder from pitot_sensors message
 *
 * @return [Pa]  Total pressure of pitot 1 
 */
static inline float mavlink_msg_pitot_sensors_get_pt_rudder(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ps_rudder from pitot_sensors message
 *
 * @return [Pa]  Static pressure of pitot 1 
 */
static inline float mavlink_msg_pitot_sensors_get_ps_rudder(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field vel_rudder from pitot_sensors message
 *
 * @return [m/s]  3D array fluid velocity of pitot 1 
 */
static inline uint16_t mavlink_msg_pitot_sensors_get_vel_rudder(const mavlink_message_t* msg, float *vel_rudder)
{
    return _MAV_RETURN_float_array(msg, vel_rudder, 3,  8);
}

/**
 * @brief Get field atm_rudder from pitot_sensors message
 *
 * @return [Pa]  Atmospheric pressure of pitot 1 
 */
static inline float mavlink_msg_pitot_sensors_get_atm_rudder(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pt_starboard from pitot_sensors message
 *
 * @return [Pa]  Total pressure of pitot 2 
 */
static inline float mavlink_msg_pitot_sensors_get_pt_starboard(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field ps_starboard from pitot_sensors message
 *
 * @return [Pa]  Static pressure of pitot 2 
 */
static inline float mavlink_msg_pitot_sensors_get_ps_starboard(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field vel_starboard from pitot_sensors message
 *
 * @return [m/s]  3D array fluid velocity of pitot 2 
 */
static inline uint16_t mavlink_msg_pitot_sensors_get_vel_starboard(const mavlink_message_t* msg, float *vel_starboard)
{
    return _MAV_RETURN_float_array(msg, vel_starboard, 3,  32);
}

/**
 * @brief Get field atm_starboard from pitot_sensors message
 *
 * @return [Pa]  Atmospheric pressure of pitot 2 
 */
static inline float mavlink_msg_pitot_sensors_get_atm_starboard(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field pt_port from pitot_sensors message
 *
 * @return [Pa]  Total pressure of pitot 3 
 */
static inline float mavlink_msg_pitot_sensors_get_pt_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field ps_port from pitot_sensors message
 *
 * @return [Pa]  Static pressure of pitot 3 
 */
static inline float mavlink_msg_pitot_sensors_get_ps_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  52);
}

/**
 * @brief Get field vel_port from pitot_sensors message
 *
 * @return [m/s]  3D array fluid velocity of pitot 3 
 */
static inline uint16_t mavlink_msg_pitot_sensors_get_vel_port(const mavlink_message_t* msg, float *vel_port)
{
    return _MAV_RETURN_float_array(msg, vel_port, 3,  56);
}

/**
 * @brief Get field atm_port from pitot_sensors message
 *
 * @return [Pa]  Atmospheric pressure of pitot 3 
 */
static inline float mavlink_msg_pitot_sensors_get_atm_port(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field pressure_atm from pitot_sensors message
 *
 * @return [Pa]  Atmospheric pressure (average between the three pitot values) 
 */
static inline float mavlink_msg_pitot_sensors_get_pressure_atm(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Decode a pitot_sensors message into a struct
 *
 * @param msg The message to decode
 * @param pitot_sensors C-struct to decode the message contents into
 */
static inline void mavlink_msg_pitot_sensors_decode(const mavlink_message_t* msg, mavlink_pitot_sensors_t* pitot_sensors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    pitot_sensors->pt_rudder = mavlink_msg_pitot_sensors_get_pt_rudder(msg);
    pitot_sensors->ps_rudder = mavlink_msg_pitot_sensors_get_ps_rudder(msg);
    mavlink_msg_pitot_sensors_get_vel_rudder(msg, pitot_sensors->vel_rudder);
    pitot_sensors->atm_rudder = mavlink_msg_pitot_sensors_get_atm_rudder(msg);
    pitot_sensors->pt_starboard = mavlink_msg_pitot_sensors_get_pt_starboard(msg);
    pitot_sensors->ps_starboard = mavlink_msg_pitot_sensors_get_ps_starboard(msg);
    mavlink_msg_pitot_sensors_get_vel_starboard(msg, pitot_sensors->vel_starboard);
    pitot_sensors->atm_starboard = mavlink_msg_pitot_sensors_get_atm_starboard(msg);
    pitot_sensors->pt_port = mavlink_msg_pitot_sensors_get_pt_port(msg);
    pitot_sensors->ps_port = mavlink_msg_pitot_sensors_get_ps_port(msg);
    mavlink_msg_pitot_sensors_get_vel_port(msg, pitot_sensors->vel_port);
    pitot_sensors->atm_port = mavlink_msg_pitot_sensors_get_atm_port(msg);
    pitot_sensors->pressure_atm = mavlink_msg_pitot_sensors_get_pressure_atm(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PITOT_SENSORS_LEN? msg->len : MAVLINK_MSG_ID_PITOT_SENSORS_LEN;
        memset(pitot_sensors, 0, MAVLINK_MSG_ID_PITOT_SENSORS_LEN);
    memcpy(pitot_sensors, _MAV_PAYLOAD(msg), len);
#endif
}
