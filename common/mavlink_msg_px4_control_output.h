#pragma once
// MESSAGE PX4_CONTROL_OUTPUT PACKING

#define MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT 604

MAVPACKED(
typedef struct __mavlink_px4_control_output_t {
 float wing_angle; /*< [deg]  control output from pressure control */
}) mavlink_px4_control_output_t;

#define MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN 4
#define MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_MIN_LEN 4
#define MAVLINK_MSG_ID_604_LEN 4
#define MAVLINK_MSG_ID_604_MIN_LEN 4

#define MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_CRC 80
#define MAVLINK_MSG_ID_604_CRC 80



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PX4_CONTROL_OUTPUT { \
    604, \
    "PX4_CONTROL_OUTPUT", \
    1, \
    {  { "wing_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_px4_control_output_t, wing_angle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PX4_CONTROL_OUTPUT { \
    "PX4_CONTROL_OUTPUT", \
    1, \
    {  { "wing_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_px4_control_output_t, wing_angle) }, \
         } \
}
#endif

/**
 * @brief Pack a px4_control_output message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param wing_angle [deg]  control output from pressure control 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4_control_output_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float wing_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN];
    _mav_put_float(buf, 0, wing_angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN);
#else
    mavlink_px4_control_output_t packet;
    packet.wing_angle = wing_angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_CRC);
}

/**
 * @brief Pack a px4_control_output message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param wing_angle [deg]  control output from pressure control 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4_control_output_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float wing_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN];
    _mav_put_float(buf, 0, wing_angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN);
#else
    mavlink_px4_control_output_t packet;
    packet.wing_angle = wing_angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_CRC);
}

/**
 * @brief Encode a px4_control_output struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param px4_control_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4_control_output_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_px4_control_output_t* px4_control_output)
{
    return mavlink_msg_px4_control_output_pack(system_id, component_id, msg, px4_control_output->wing_angle);
}

/**
 * @brief Encode a px4_control_output struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param px4_control_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4_control_output_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_px4_control_output_t* px4_control_output)
{
    return mavlink_msg_px4_control_output_pack_chan(system_id, component_id, chan, msg, px4_control_output->wing_angle);
}

/**
 * @brief Send a px4_control_output message
 * @param chan MAVLink channel to send the message
 *
 * @param wing_angle [deg]  control output from pressure control 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_px4_control_output_send(mavlink_channel_t chan, float wing_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN];
    _mav_put_float(buf, 0, wing_angle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT, buf, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_CRC);
#else
    mavlink_px4_control_output_t packet;
    packet.wing_angle = wing_angle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT, (const char *)&packet, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_CRC);
#endif
}

/**
 * @brief Send a px4_control_output message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_px4_control_output_send_struct(mavlink_channel_t chan, const mavlink_px4_control_output_t* px4_control_output)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_px4_control_output_send(chan, px4_control_output->wing_angle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT, (const char *)px4_control_output, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_CRC);
#endif
}

#if MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_px4_control_output_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float wing_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, wing_angle);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT, buf, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_CRC);
#else
    mavlink_px4_control_output_t *packet = (mavlink_px4_control_output_t *)msgbuf;
    packet->wing_angle = wing_angle;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT, (const char *)packet, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_CRC);
#endif
}
#endif

#endif

// MESSAGE PX4_CONTROL_OUTPUT UNPACKING


/**
 * @brief Get field wing_angle from px4_control_output message
 *
 * @return [deg]  control output from pressure control 
 */
static inline float mavlink_msg_px4_control_output_get_wing_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a px4_control_output message into a struct
 *
 * @param msg The message to decode
 * @param px4_control_output C-struct to decode the message contents into
 */
static inline void mavlink_msg_px4_control_output_decode(const mavlink_message_t* msg, mavlink_px4_control_output_t* px4_control_output)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    px4_control_output->wing_angle = mavlink_msg_px4_control_output_get_wing_angle(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN? msg->len : MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN;
        memset(px4_control_output, 0, MAVLINK_MSG_ID_PX4_CONTROL_OUTPUT_LEN);
    memcpy(px4_control_output, _MAV_PAYLOAD(msg), len);
#endif
}
