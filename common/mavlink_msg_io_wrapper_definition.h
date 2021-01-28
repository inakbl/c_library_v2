#pragma once
// MESSAGE IO_WRAPPER_DEFINITION PACKING

#define MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION 605

MAVPACKED(
typedef struct __mavlink_io_wrapper_definition_t {
 float in_4; /*<   */
 float out_4; /*<   */
 uint16_t in_3; /*<   */
 uint16_t out_3; /*<   */
 uint8_t in_1; /*<   */
 uint8_t in_2; /*<   */
 uint8_t out_1; /*<   */
 uint8_t out_2; /*<   */
}) mavlink_io_wrapper_definition_t;

#define MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN 16
#define MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_MIN_LEN 16
#define MAVLINK_MSG_ID_605_LEN 16
#define MAVLINK_MSG_ID_605_MIN_LEN 16

#define MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_CRC 153
#define MAVLINK_MSG_ID_605_CRC 153



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_IO_WRAPPER_DEFINITION { \
    605, \
    "IO_WRAPPER_DEFINITION", \
    8, \
    {  { "in_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_io_wrapper_definition_t, in_1) }, \
         { "in_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_io_wrapper_definition_t, in_2) }, \
         { "in_3", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_io_wrapper_definition_t, in_3) }, \
         { "in_4", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_io_wrapper_definition_t, in_4) }, \
         { "out_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_io_wrapper_definition_t, out_1) }, \
         { "out_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_io_wrapper_definition_t, out_2) }, \
         { "out_3", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_io_wrapper_definition_t, out_3) }, \
         { "out_4", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_io_wrapper_definition_t, out_4) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_IO_WRAPPER_DEFINITION { \
    "IO_WRAPPER_DEFINITION", \
    8, \
    {  { "in_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_io_wrapper_definition_t, in_1) }, \
         { "in_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_io_wrapper_definition_t, in_2) }, \
         { "in_3", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_io_wrapper_definition_t, in_3) }, \
         { "in_4", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_io_wrapper_definition_t, in_4) }, \
         { "out_1", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_io_wrapper_definition_t, out_1) }, \
         { "out_2", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_io_wrapper_definition_t, out_2) }, \
         { "out_3", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_io_wrapper_definition_t, out_3) }, \
         { "out_4", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_io_wrapper_definition_t, out_4) }, \
         } \
}
#endif

/**
 * @brief Pack a io_wrapper_definition message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param in_1   
 * @param in_2   
 * @param in_3   
 * @param in_4   
 * @param out_1   
 * @param out_2   
 * @param out_3   
 * @param out_4   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_io_wrapper_definition_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t in_1, uint8_t in_2, uint16_t in_3, float in_4, uint8_t out_1, uint8_t out_2, uint16_t out_3, float out_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN];
    _mav_put_float(buf, 0, in_4);
    _mav_put_float(buf, 4, out_4);
    _mav_put_uint16_t(buf, 8, in_3);
    _mav_put_uint16_t(buf, 10, out_3);
    _mav_put_uint8_t(buf, 12, in_1);
    _mav_put_uint8_t(buf, 13, in_2);
    _mav_put_uint8_t(buf, 14, out_1);
    _mav_put_uint8_t(buf, 15, out_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN);
#else
    mavlink_io_wrapper_definition_t packet;
    packet.in_4 = in_4;
    packet.out_4 = out_4;
    packet.in_3 = in_3;
    packet.out_3 = out_3;
    packet.in_1 = in_1;
    packet.in_2 = in_2;
    packet.out_1 = out_1;
    packet.out_2 = out_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_CRC);
}

/**
 * @brief Pack a io_wrapper_definition message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param in_1   
 * @param in_2   
 * @param in_3   
 * @param in_4   
 * @param out_1   
 * @param out_2   
 * @param out_3   
 * @param out_4   
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_io_wrapper_definition_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t in_1,uint8_t in_2,uint16_t in_3,float in_4,uint8_t out_1,uint8_t out_2,uint16_t out_3,float out_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN];
    _mav_put_float(buf, 0, in_4);
    _mav_put_float(buf, 4, out_4);
    _mav_put_uint16_t(buf, 8, in_3);
    _mav_put_uint16_t(buf, 10, out_3);
    _mav_put_uint8_t(buf, 12, in_1);
    _mav_put_uint8_t(buf, 13, in_2);
    _mav_put_uint8_t(buf, 14, out_1);
    _mav_put_uint8_t(buf, 15, out_2);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN);
#else
    mavlink_io_wrapper_definition_t packet;
    packet.in_4 = in_4;
    packet.out_4 = out_4;
    packet.in_3 = in_3;
    packet.out_3 = out_3;
    packet.in_1 = in_1;
    packet.in_2 = in_2;
    packet.out_1 = out_1;
    packet.out_2 = out_2;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_CRC);
}

/**
 * @brief Encode a io_wrapper_definition struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param io_wrapper_definition C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_io_wrapper_definition_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_io_wrapper_definition_t* io_wrapper_definition)
{
    return mavlink_msg_io_wrapper_definition_pack(system_id, component_id, msg, io_wrapper_definition->in_1, io_wrapper_definition->in_2, io_wrapper_definition->in_3, io_wrapper_definition->in_4, io_wrapper_definition->out_1, io_wrapper_definition->out_2, io_wrapper_definition->out_3, io_wrapper_definition->out_4);
}

/**
 * @brief Encode a io_wrapper_definition struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param io_wrapper_definition C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_io_wrapper_definition_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_io_wrapper_definition_t* io_wrapper_definition)
{
    return mavlink_msg_io_wrapper_definition_pack_chan(system_id, component_id, chan, msg, io_wrapper_definition->in_1, io_wrapper_definition->in_2, io_wrapper_definition->in_3, io_wrapper_definition->in_4, io_wrapper_definition->out_1, io_wrapper_definition->out_2, io_wrapper_definition->out_3, io_wrapper_definition->out_4);
}

/**
 * @brief Send a io_wrapper_definition message
 * @param chan MAVLink channel to send the message
 *
 * @param in_1   
 * @param in_2   
 * @param in_3   
 * @param in_4   
 * @param out_1   
 * @param out_2   
 * @param out_3   
 * @param out_4   
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_io_wrapper_definition_send(mavlink_channel_t chan, uint8_t in_1, uint8_t in_2, uint16_t in_3, float in_4, uint8_t out_1, uint8_t out_2, uint16_t out_3, float out_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN];
    _mav_put_float(buf, 0, in_4);
    _mav_put_float(buf, 4, out_4);
    _mav_put_uint16_t(buf, 8, in_3);
    _mav_put_uint16_t(buf, 10, out_3);
    _mav_put_uint8_t(buf, 12, in_1);
    _mav_put_uint8_t(buf, 13, in_2);
    _mav_put_uint8_t(buf, 14, out_1);
    _mav_put_uint8_t(buf, 15, out_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION, buf, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_CRC);
#else
    mavlink_io_wrapper_definition_t packet;
    packet.in_4 = in_4;
    packet.out_4 = out_4;
    packet.in_3 = in_3;
    packet.out_3 = out_3;
    packet.in_1 = in_1;
    packet.in_2 = in_2;
    packet.out_1 = out_1;
    packet.out_2 = out_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION, (const char *)&packet, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_CRC);
#endif
}

/**
 * @brief Send a io_wrapper_definition message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_io_wrapper_definition_send_struct(mavlink_channel_t chan, const mavlink_io_wrapper_definition_t* io_wrapper_definition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_io_wrapper_definition_send(chan, io_wrapper_definition->in_1, io_wrapper_definition->in_2, io_wrapper_definition->in_3, io_wrapper_definition->in_4, io_wrapper_definition->out_1, io_wrapper_definition->out_2, io_wrapper_definition->out_3, io_wrapper_definition->out_4);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION, (const char *)io_wrapper_definition, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_io_wrapper_definition_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t in_1, uint8_t in_2, uint16_t in_3, float in_4, uint8_t out_1, uint8_t out_2, uint16_t out_3, float out_4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, in_4);
    _mav_put_float(buf, 4, out_4);
    _mav_put_uint16_t(buf, 8, in_3);
    _mav_put_uint16_t(buf, 10, out_3);
    _mav_put_uint8_t(buf, 12, in_1);
    _mav_put_uint8_t(buf, 13, in_2);
    _mav_put_uint8_t(buf, 14, out_1);
    _mav_put_uint8_t(buf, 15, out_2);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION, buf, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_CRC);
#else
    mavlink_io_wrapper_definition_t *packet = (mavlink_io_wrapper_definition_t *)msgbuf;
    packet->in_4 = in_4;
    packet->out_4 = out_4;
    packet->in_3 = in_3;
    packet->out_3 = out_3;
    packet->in_1 = in_1;
    packet->in_2 = in_2;
    packet->out_1 = out_1;
    packet->out_2 = out_2;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION, (const char *)packet, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_CRC);
#endif
}
#endif

#endif

// MESSAGE IO_WRAPPER_DEFINITION UNPACKING


/**
 * @brief Get field in_1 from io_wrapper_definition message
 *
 * @return   
 */
static inline uint8_t mavlink_msg_io_wrapper_definition_get_in_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field in_2 from io_wrapper_definition message
 *
 * @return   
 */
static inline uint8_t mavlink_msg_io_wrapper_definition_get_in_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field in_3 from io_wrapper_definition message
 *
 * @return   
 */
static inline uint16_t mavlink_msg_io_wrapper_definition_get_in_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field in_4 from io_wrapper_definition message
 *
 * @return   
 */
static inline float mavlink_msg_io_wrapper_definition_get_in_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field out_1 from io_wrapper_definition message
 *
 * @return   
 */
static inline uint8_t mavlink_msg_io_wrapper_definition_get_out_1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field out_2 from io_wrapper_definition message
 *
 * @return   
 */
static inline uint8_t mavlink_msg_io_wrapper_definition_get_out_2(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field out_3 from io_wrapper_definition message
 *
 * @return   
 */
static inline uint16_t mavlink_msg_io_wrapper_definition_get_out_3(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field out_4 from io_wrapper_definition message
 *
 * @return   
 */
static inline float mavlink_msg_io_wrapper_definition_get_out_4(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Decode a io_wrapper_definition message into a struct
 *
 * @param msg The message to decode
 * @param io_wrapper_definition C-struct to decode the message contents into
 */
static inline void mavlink_msg_io_wrapper_definition_decode(const mavlink_message_t* msg, mavlink_io_wrapper_definition_t* io_wrapper_definition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    io_wrapper_definition->in_4 = mavlink_msg_io_wrapper_definition_get_in_4(msg);
    io_wrapper_definition->out_4 = mavlink_msg_io_wrapper_definition_get_out_4(msg);
    io_wrapper_definition->in_3 = mavlink_msg_io_wrapper_definition_get_in_3(msg);
    io_wrapper_definition->out_3 = mavlink_msg_io_wrapper_definition_get_out_3(msg);
    io_wrapper_definition->in_1 = mavlink_msg_io_wrapper_definition_get_in_1(msg);
    io_wrapper_definition->in_2 = mavlink_msg_io_wrapper_definition_get_in_2(msg);
    io_wrapper_definition->out_1 = mavlink_msg_io_wrapper_definition_get_out_1(msg);
    io_wrapper_definition->out_2 = mavlink_msg_io_wrapper_definition_get_out_2(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN? msg->len : MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN;
        memset(io_wrapper_definition, 0, MAVLINK_MSG_ID_IO_WRAPPER_DEFINITION_LEN);
    memcpy(io_wrapper_definition, _MAV_PAYLOAD(msg), len);
#endif
}
