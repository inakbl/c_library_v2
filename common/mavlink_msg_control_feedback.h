#pragma once
// MESSAGE CONTROL_FEEDBACK PACKING

#define MAVLINK_MSG_ID_CONTROL_FEEDBACK 603


typedef struct __mavlink_control_feedback_t {
 float act_angle; /*< [deg]  actual angle read */
} mavlink_control_feedback_t;

#define MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN 4
#define MAVLINK_MSG_ID_CONTROL_FEEDBACK_MIN_LEN 4
#define MAVLINK_MSG_ID_603_LEN 4
#define MAVLINK_MSG_ID_603_MIN_LEN 4

#define MAVLINK_MSG_ID_CONTROL_FEEDBACK_CRC 36
#define MAVLINK_MSG_ID_603_CRC 36



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_CONTROL_FEEDBACK { \
    603, \
    "CONTROL_FEEDBACK", \
    1, \
    {  { "act_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_control_feedback_t, act_angle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_CONTROL_FEEDBACK { \
    "CONTROL_FEEDBACK", \
    1, \
    {  { "act_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_control_feedback_t, act_angle) }, \
         } \
}
#endif

/**
 * @brief Pack a control_feedback message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param act_angle [deg]  actual angle read 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_feedback_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float act_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN];
    _mav_put_float(buf, 0, act_angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN);
#else
    mavlink_control_feedback_t packet;
    packet.act_angle = act_angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONTROL_FEEDBACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_CONTROL_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_CRC);
}

/**
 * @brief Pack a control_feedback message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param act_angle [deg]  actual angle read 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_control_feedback_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float act_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN];
    _mav_put_float(buf, 0, act_angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN);
#else
    mavlink_control_feedback_t packet;
    packet.act_angle = act_angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_CONTROL_FEEDBACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_CONTROL_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_CRC);
}

/**
 * @brief Encode a control_feedback struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param control_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_feedback_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_control_feedback_t* control_feedback)
{
    return mavlink_msg_control_feedback_pack(system_id, component_id, msg, control_feedback->act_angle);
}

/**
 * @brief Encode a control_feedback struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param control_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_control_feedback_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_control_feedback_t* control_feedback)
{
    return mavlink_msg_control_feedback_pack_chan(system_id, component_id, chan, msg, control_feedback->act_angle);
}

/**
 * @brief Send a control_feedback message
 * @param chan MAVLink channel to send the message
 *
 * @param act_angle [deg]  actual angle read 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_control_feedback_send(mavlink_channel_t chan, float act_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN];
    _mav_put_float(buf, 0, act_angle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_FEEDBACK, buf, MAVLINK_MSG_ID_CONTROL_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_CRC);
#else
    mavlink_control_feedback_t packet;
    packet.act_angle = act_angle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_FEEDBACK, (const char *)&packet, MAVLINK_MSG_ID_CONTROL_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_CRC);
#endif
}

/**
 * @brief Send a control_feedback message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_control_feedback_send_struct(mavlink_channel_t chan, const mavlink_control_feedback_t* control_feedback)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_control_feedback_send(chan, control_feedback->act_angle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_FEEDBACK, (const char *)control_feedback, MAVLINK_MSG_ID_CONTROL_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_control_feedback_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float act_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, act_angle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_FEEDBACK, buf, MAVLINK_MSG_ID_CONTROL_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_CRC);
#else
    mavlink_control_feedback_t *packet = (mavlink_control_feedback_t *)msgbuf;
    packet->act_angle = act_angle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_CONTROL_FEEDBACK, (const char *)packet, MAVLINK_MSG_ID_CONTROL_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN, MAVLINK_MSG_ID_CONTROL_FEEDBACK_CRC);
#endif
}
#endif

#endif

// MESSAGE CONTROL_FEEDBACK UNPACKING


/**
 * @brief Get field act_angle from control_feedback message
 *
 * @return [deg]  actual angle read 
 */
static inline float mavlink_msg_control_feedback_get_act_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a control_feedback message into a struct
 *
 * @param msg The message to decode
 * @param control_feedback C-struct to decode the message contents into
 */
static inline void mavlink_msg_control_feedback_decode(const mavlink_message_t* msg, mavlink_control_feedback_t* control_feedback)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    control_feedback->act_angle = mavlink_msg_control_feedback_get_act_angle(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN? msg->len : MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN;
        memset(control_feedback, 0, MAVLINK_MSG_ID_CONTROL_FEEDBACK_LEN);
    memcpy(control_feedback, _MAV_PAYLOAD(msg), len);
#endif
}
