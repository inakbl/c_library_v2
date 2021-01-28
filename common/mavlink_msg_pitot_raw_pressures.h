#pragma once
// MESSAGE PITOT_RAW_PRESSURES PACKING

#define MAVLINK_MSG_ID_PITOT_RAW_PRESSURES 618

MAVPACKED(
typedef struct __mavlink_pitot_raw_pressures_t {
 float pitot_1_pressure[6]; /*< [m/s]  Pressure rack pitot 1 */
 float pitot_2_pressure[6]; /*< [m/s]  Pressure rack pitot 2 */
 float pitot_3_pressure[6]; /*< [m/s]  Pressure rack pitot 3 */
}) mavlink_pitot_raw_pressures_t;

#define MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN 72
#define MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_MIN_LEN 72
#define MAVLINK_MSG_ID_618_LEN 72
#define MAVLINK_MSG_ID_618_MIN_LEN 72

#define MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_CRC 94
#define MAVLINK_MSG_ID_618_CRC 94

#define MAVLINK_MSG_PITOT_RAW_PRESSURES_FIELD_PITOT_1_PRESSURE_LEN 6
#define MAVLINK_MSG_PITOT_RAW_PRESSURES_FIELD_PITOT_2_PRESSURE_LEN 6
#define MAVLINK_MSG_PITOT_RAW_PRESSURES_FIELD_PITOT_3_PRESSURE_LEN 6

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PITOT_RAW_PRESSURES { \
    618, \
    "PITOT_RAW_PRESSURES", \
    3, \
    {  { "pitot_1_pressure", NULL, MAVLINK_TYPE_FLOAT, 6, 0, offsetof(mavlink_pitot_raw_pressures_t, pitot_1_pressure) }, \
         { "pitot_2_pressure", NULL, MAVLINK_TYPE_FLOAT, 6, 24, offsetof(mavlink_pitot_raw_pressures_t, pitot_2_pressure) }, \
         { "pitot_3_pressure", NULL, MAVLINK_TYPE_FLOAT, 6, 48, offsetof(mavlink_pitot_raw_pressures_t, pitot_3_pressure) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PITOT_RAW_PRESSURES { \
    "PITOT_RAW_PRESSURES", \
    3, \
    {  { "pitot_1_pressure", NULL, MAVLINK_TYPE_FLOAT, 6, 0, offsetof(mavlink_pitot_raw_pressures_t, pitot_1_pressure) }, \
         { "pitot_2_pressure", NULL, MAVLINK_TYPE_FLOAT, 6, 24, offsetof(mavlink_pitot_raw_pressures_t, pitot_2_pressure) }, \
         { "pitot_3_pressure", NULL, MAVLINK_TYPE_FLOAT, 6, 48, offsetof(mavlink_pitot_raw_pressures_t, pitot_3_pressure) }, \
         } \
}
#endif

/**
 * @brief Pack a pitot_raw_pressures message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pitot_1_pressure [m/s]  Pressure rack pitot 1 
 * @param pitot_2_pressure [m/s]  Pressure rack pitot 2 
 * @param pitot_3_pressure [m/s]  Pressure rack pitot 3 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pitot_raw_pressures_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *pitot_1_pressure, const float *pitot_2_pressure, const float *pitot_3_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN];

    _mav_put_float_array(buf, 0, pitot_1_pressure, 6);
    _mav_put_float_array(buf, 24, pitot_2_pressure, 6);
    _mav_put_float_array(buf, 48, pitot_3_pressure, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN);
#else
    mavlink_pitot_raw_pressures_t packet;

    mav_array_memcpy(packet.pitot_1_pressure, pitot_1_pressure, sizeof(float)*6);
    mav_array_memcpy(packet.pitot_2_pressure, pitot_2_pressure, sizeof(float)*6);
    mav_array_memcpy(packet.pitot_3_pressure, pitot_3_pressure, sizeof(float)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PITOT_RAW_PRESSURES;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_MIN_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_CRC);
}

/**
 * @brief Pack a pitot_raw_pressures message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pitot_1_pressure [m/s]  Pressure rack pitot 1 
 * @param pitot_2_pressure [m/s]  Pressure rack pitot 2 
 * @param pitot_3_pressure [m/s]  Pressure rack pitot 3 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pitot_raw_pressures_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *pitot_1_pressure,const float *pitot_2_pressure,const float *pitot_3_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN];

    _mav_put_float_array(buf, 0, pitot_1_pressure, 6);
    _mav_put_float_array(buf, 24, pitot_2_pressure, 6);
    _mav_put_float_array(buf, 48, pitot_3_pressure, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN);
#else
    mavlink_pitot_raw_pressures_t packet;

    mav_array_memcpy(packet.pitot_1_pressure, pitot_1_pressure, sizeof(float)*6);
    mav_array_memcpy(packet.pitot_2_pressure, pitot_2_pressure, sizeof(float)*6);
    mav_array_memcpy(packet.pitot_3_pressure, pitot_3_pressure, sizeof(float)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PITOT_RAW_PRESSURES;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_MIN_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_CRC);
}

/**
 * @brief Encode a pitot_raw_pressures struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pitot_raw_pressures C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pitot_raw_pressures_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pitot_raw_pressures_t* pitot_raw_pressures)
{
    return mavlink_msg_pitot_raw_pressures_pack(system_id, component_id, msg, pitot_raw_pressures->pitot_1_pressure, pitot_raw_pressures->pitot_2_pressure, pitot_raw_pressures->pitot_3_pressure);
}

/**
 * @brief Encode a pitot_raw_pressures struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pitot_raw_pressures C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pitot_raw_pressures_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pitot_raw_pressures_t* pitot_raw_pressures)
{
    return mavlink_msg_pitot_raw_pressures_pack_chan(system_id, component_id, chan, msg, pitot_raw_pressures->pitot_1_pressure, pitot_raw_pressures->pitot_2_pressure, pitot_raw_pressures->pitot_3_pressure);
}

/**
 * @brief Send a pitot_raw_pressures message
 * @param chan MAVLink channel to send the message
 *
 * @param pitot_1_pressure [m/s]  Pressure rack pitot 1 
 * @param pitot_2_pressure [m/s]  Pressure rack pitot 2 
 * @param pitot_3_pressure [m/s]  Pressure rack pitot 3 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pitot_raw_pressures_send(mavlink_channel_t chan, const float *pitot_1_pressure, const float *pitot_2_pressure, const float *pitot_3_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN];

    _mav_put_float_array(buf, 0, pitot_1_pressure, 6);
    _mav_put_float_array(buf, 24, pitot_2_pressure, 6);
    _mav_put_float_array(buf, 48, pitot_3_pressure, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES, buf, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_MIN_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_CRC);
#else
    mavlink_pitot_raw_pressures_t packet;

    mav_array_memcpy(packet.pitot_1_pressure, pitot_1_pressure, sizeof(float)*6);
    mav_array_memcpy(packet.pitot_2_pressure, pitot_2_pressure, sizeof(float)*6);
    mav_array_memcpy(packet.pitot_3_pressure, pitot_3_pressure, sizeof(float)*6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES, (const char *)&packet, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_MIN_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_CRC);
#endif
}

/**
 * @brief Send a pitot_raw_pressures message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_pitot_raw_pressures_send_struct(mavlink_channel_t chan, const mavlink_pitot_raw_pressures_t* pitot_raw_pressures)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pitot_raw_pressures_send(chan, pitot_raw_pressures->pitot_1_pressure, pitot_raw_pressures->pitot_2_pressure, pitot_raw_pressures->pitot_3_pressure);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES, (const char *)pitot_raw_pressures, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_MIN_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_CRC);
#endif
}

#if MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pitot_raw_pressures_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *pitot_1_pressure, const float *pitot_2_pressure, const float *pitot_3_pressure)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_float_array(buf, 0, pitot_1_pressure, 6);
    _mav_put_float_array(buf, 24, pitot_2_pressure, 6);
    _mav_put_float_array(buf, 48, pitot_3_pressure, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES, buf, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_MIN_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_CRC);
#else
    mavlink_pitot_raw_pressures_t *packet = (mavlink_pitot_raw_pressures_t *)msgbuf;

    mav_array_memcpy(packet->pitot_1_pressure, pitot_1_pressure, sizeof(float)*6);
    mav_array_memcpy(packet->pitot_2_pressure, pitot_2_pressure, sizeof(float)*6);
    mav_array_memcpy(packet->pitot_3_pressure, pitot_3_pressure, sizeof(float)*6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES, (const char *)packet, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_MIN_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_CRC);
#endif
}
#endif

#endif

// MESSAGE PITOT_RAW_PRESSURES UNPACKING


/**
 * @brief Get field pitot_1_pressure from pitot_raw_pressures message
 *
 * @return [m/s]  Pressure rack pitot 1 
 */
static inline uint16_t mavlink_msg_pitot_raw_pressures_get_pitot_1_pressure(const mavlink_message_t* msg, float *pitot_1_pressure)
{
    return _MAV_RETURN_float_array(msg, pitot_1_pressure, 6,  0);
}

/**
 * @brief Get field pitot_2_pressure from pitot_raw_pressures message
 *
 * @return [m/s]  Pressure rack pitot 2 
 */
static inline uint16_t mavlink_msg_pitot_raw_pressures_get_pitot_2_pressure(const mavlink_message_t* msg, float *pitot_2_pressure)
{
    return _MAV_RETURN_float_array(msg, pitot_2_pressure, 6,  24);
}

/**
 * @brief Get field pitot_3_pressure from pitot_raw_pressures message
 *
 * @return [m/s]  Pressure rack pitot 3 
 */
static inline uint16_t mavlink_msg_pitot_raw_pressures_get_pitot_3_pressure(const mavlink_message_t* msg, float *pitot_3_pressure)
{
    return _MAV_RETURN_float_array(msg, pitot_3_pressure, 6,  48);
}

/**
 * @brief Decode a pitot_raw_pressures message into a struct
 *
 * @param msg The message to decode
 * @param pitot_raw_pressures C-struct to decode the message contents into
 */
static inline void mavlink_msg_pitot_raw_pressures_decode(const mavlink_message_t* msg, mavlink_pitot_raw_pressures_t* pitot_raw_pressures)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pitot_raw_pressures_get_pitot_1_pressure(msg, pitot_raw_pressures->pitot_1_pressure);
    mavlink_msg_pitot_raw_pressures_get_pitot_2_pressure(msg, pitot_raw_pressures->pitot_2_pressure);
    mavlink_msg_pitot_raw_pressures_get_pitot_3_pressure(msg, pitot_raw_pressures->pitot_3_pressure);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN? msg->len : MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN;
        memset(pitot_raw_pressures, 0, MAVLINK_MSG_ID_PITOT_RAW_PRESSURES_LEN);
    memcpy(pitot_raw_pressures, _MAV_PAYLOAD(msg), len);
#endif
}
