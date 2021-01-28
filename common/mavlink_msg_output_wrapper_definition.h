#pragma once
// MESSAGE OUTPUT_WRAPPER_DEFINITION PACKING

#define MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION 601

MAVPACKED(
typedef struct __mavlink_output_wrapper_definition_t {
 double alfaActua; /*< [deg]  Angle to be set to the wing */
}) mavlink_output_wrapper_definition_t;

#define MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN 8
#define MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_MIN_LEN 8
#define MAVLINK_MSG_ID_601_LEN 8
#define MAVLINK_MSG_ID_601_MIN_LEN 8

#define MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_CRC 240
#define MAVLINK_MSG_ID_601_CRC 240



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_OUTPUT_WRAPPER_DEFINITION { \
    601, \
    "OUTPUT_WRAPPER_DEFINITION", \
    1, \
    {  { "alfaActua", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_output_wrapper_definition_t, alfaActua) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_OUTPUT_WRAPPER_DEFINITION { \
    "OUTPUT_WRAPPER_DEFINITION", \
    1, \
    {  { "alfaActua", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_output_wrapper_definition_t, alfaActua) }, \
         } \
}
#endif

/**
 * @brief Pack a output_wrapper_definition message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param alfaActua [deg]  Angle to be set to the wing 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_output_wrapper_definition_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               double alfaActua)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN];
    _mav_put_double(buf, 0, alfaActua);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN);
#else
    mavlink_output_wrapper_definition_t packet;
    packet.alfaActua = alfaActua;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_CRC);
}

/**
 * @brief Pack a output_wrapper_definition message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param alfaActua [deg]  Angle to be set to the wing 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_output_wrapper_definition_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   double alfaActua)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN];
    _mav_put_double(buf, 0, alfaActua);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN);
#else
    mavlink_output_wrapper_definition_t packet;
    packet.alfaActua = alfaActua;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_CRC);
}

/**
 * @brief Encode a output_wrapper_definition struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param output_wrapper_definition C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_output_wrapper_definition_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_output_wrapper_definition_t* output_wrapper_definition)
{
    return mavlink_msg_output_wrapper_definition_pack(system_id, component_id, msg, output_wrapper_definition->alfaActua);
}

/**
 * @brief Encode a output_wrapper_definition struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param output_wrapper_definition C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_output_wrapper_definition_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_output_wrapper_definition_t* output_wrapper_definition)
{
    return mavlink_msg_output_wrapper_definition_pack_chan(system_id, component_id, chan, msg, output_wrapper_definition->alfaActua);
}

/**
 * @brief Send a output_wrapper_definition message
 * @param chan MAVLink channel to send the message
 *
 * @param alfaActua [deg]  Angle to be set to the wing 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_output_wrapper_definition_send(mavlink_channel_t chan, double alfaActua)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN];
    _mav_put_double(buf, 0, alfaActua);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION, buf, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_CRC);
#else
    mavlink_output_wrapper_definition_t packet;
    packet.alfaActua = alfaActua;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION, (const char *)&packet, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_CRC);
#endif
}

/**
 * @brief Send a output_wrapper_definition message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_output_wrapper_definition_send_struct(mavlink_channel_t chan, const mavlink_output_wrapper_definition_t* output_wrapper_definition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_output_wrapper_definition_send(chan, output_wrapper_definition->alfaActua);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION, (const char *)output_wrapper_definition, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_output_wrapper_definition_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  double alfaActua)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_double(buf, 0, alfaActua);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION, buf, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_CRC);
#else
    mavlink_output_wrapper_definition_t *packet = (mavlink_output_wrapper_definition_t *)msgbuf;
    packet->alfaActua = alfaActua;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION, (const char *)packet, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_CRC);
#endif
}
#endif

#endif

// MESSAGE OUTPUT_WRAPPER_DEFINITION UNPACKING


/**
 * @brief Get field alfaActua from output_wrapper_definition message
 *
 * @return [deg]  Angle to be set to the wing 
 */
static inline double mavlink_msg_output_wrapper_definition_get_alfaActua(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Decode a output_wrapper_definition message into a struct
 *
 * @param msg The message to decode
 * @param output_wrapper_definition C-struct to decode the message contents into
 */
static inline void mavlink_msg_output_wrapper_definition_decode(const mavlink_message_t* msg, mavlink_output_wrapper_definition_t* output_wrapper_definition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    output_wrapper_definition->alfaActua = mavlink_msg_output_wrapper_definition_get_alfaActua(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN? msg->len : MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN;
        memset(output_wrapper_definition, 0, MAVLINK_MSG_ID_OUTPUT_WRAPPER_DEFINITION_LEN);
    memcpy(output_wrapper_definition, _MAV_PAYLOAD(msg), len);
#endif
}
