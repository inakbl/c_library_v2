#pragma once
// MESSAGE RUDDER_ANGLE PACKING

#define MAVLINK_MSG_ID_RUDDER_ANGLE 619


typedef struct __mavlink_rudder_angle_t {
 float rudder_angle; /*< [rad]   Rudder angle (deg) */
} mavlink_rudder_angle_t;

#define MAVLINK_MSG_ID_RUDDER_ANGLE_LEN 4
#define MAVLINK_MSG_ID_RUDDER_ANGLE_MIN_LEN 4
#define MAVLINK_MSG_ID_619_LEN 4
#define MAVLINK_MSG_ID_619_MIN_LEN 4

#define MAVLINK_MSG_ID_RUDDER_ANGLE_CRC 121
#define MAVLINK_MSG_ID_619_CRC 121



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_RUDDER_ANGLE { \
    619, \
    "RUDDER_ANGLE", \
    1, \
    {  { "rudder_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rudder_angle_t, rudder_angle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_RUDDER_ANGLE { \
    "RUDDER_ANGLE", \
    1, \
    {  { "rudder_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_rudder_angle_t, rudder_angle) }, \
         } \
}
#endif

/**
 * @brief Pack a rudder_angle message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rudder_angle [rad]   Rudder angle (deg) 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rudder_angle_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RUDDER_ANGLE_LEN];
    _mav_put_float(buf, 0, rudder_angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RUDDER_ANGLE_LEN);
#else
    mavlink_rudder_angle_t packet;
    packet.rudder_angle = rudder_angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RUDDER_ANGLE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RUDDER_ANGLE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_RUDDER_ANGLE_MIN_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_CRC);
}

/**
 * @brief Pack a rudder_angle message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rudder_angle [rad]   Rudder angle (deg) 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rudder_angle_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RUDDER_ANGLE_LEN];
    _mav_put_float(buf, 0, rudder_angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_RUDDER_ANGLE_LEN);
#else
    mavlink_rudder_angle_t packet;
    packet.rudder_angle = rudder_angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_RUDDER_ANGLE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_RUDDER_ANGLE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_RUDDER_ANGLE_MIN_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_CRC);
}

/**
 * @brief Encode a rudder_angle struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rudder_angle C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rudder_angle_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rudder_angle_t* rudder_angle)
{
    return mavlink_msg_rudder_angle_pack(system_id, component_id, msg, rudder_angle->rudder_angle);
}

/**
 * @brief Encode a rudder_angle struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rudder_angle C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rudder_angle_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_rudder_angle_t* rudder_angle)
{
    return mavlink_msg_rudder_angle_pack_chan(system_id, component_id, chan, msg, rudder_angle->rudder_angle);
}

/**
 * @brief Send a rudder_angle message
 * @param chan MAVLink channel to send the message
 *
 * @param rudder_angle [rad]   Rudder angle (deg) 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rudder_angle_send(mavlink_channel_t chan, float rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_RUDDER_ANGLE_LEN];
    _mav_put_float(buf, 0, rudder_angle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RUDDER_ANGLE, buf, MAVLINK_MSG_ID_RUDDER_ANGLE_MIN_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_CRC);
#else
    mavlink_rudder_angle_t packet;
    packet.rudder_angle = rudder_angle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RUDDER_ANGLE, (const char *)&packet, MAVLINK_MSG_ID_RUDDER_ANGLE_MIN_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_CRC);
#endif
}

/**
 * @brief Send a rudder_angle message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_rudder_angle_send_struct(mavlink_channel_t chan, const mavlink_rudder_angle_t* rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_rudder_angle_send(chan, rudder_angle->rudder_angle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RUDDER_ANGLE, (const char *)rudder_angle, MAVLINK_MSG_ID_RUDDER_ANGLE_MIN_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_CRC);
#endif
}

#if MAVLINK_MSG_ID_RUDDER_ANGLE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_rudder_angle_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, rudder_angle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RUDDER_ANGLE, buf, MAVLINK_MSG_ID_RUDDER_ANGLE_MIN_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_CRC);
#else
    mavlink_rudder_angle_t *packet = (mavlink_rudder_angle_t *)msgbuf;
    packet->rudder_angle = rudder_angle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RUDDER_ANGLE, (const char *)packet, MAVLINK_MSG_ID_RUDDER_ANGLE_MIN_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_LEN, MAVLINK_MSG_ID_RUDDER_ANGLE_CRC);
#endif
}
#endif

#endif

// MESSAGE RUDDER_ANGLE UNPACKING


/**
 * @brief Get field rudder_angle from rudder_angle message
 *
 * @return [rad]   Rudder angle (deg) 
 */
static inline float mavlink_msg_rudder_angle_get_rudder_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a rudder_angle message into a struct
 *
 * @param msg The message to decode
 * @param rudder_angle C-struct to decode the message contents into
 */
static inline void mavlink_msg_rudder_angle_decode(const mavlink_message_t* msg, mavlink_rudder_angle_t* rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    rudder_angle->rudder_angle = mavlink_msg_rudder_angle_get_rudder_angle(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_RUDDER_ANGLE_LEN? msg->len : MAVLINK_MSG_ID_RUDDER_ANGLE_LEN;
        memset(rudder_angle, 0, MAVLINK_MSG_ID_RUDDER_ANGLE_LEN);
    memcpy(rudder_angle, _MAV_PAYLOAD(msg), len);
#endif
}
