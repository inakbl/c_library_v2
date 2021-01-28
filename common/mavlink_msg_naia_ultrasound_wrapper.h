#pragma once
// MESSAGE NAIA_ULTRASOUND_WRAPPER PACKING

#define MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER 622

MAVPACKED(
typedef struct __mavlink_naia_ultrasound_wrapper_t {
 double ultrasound_1_measurement; /*< [m] Measurement obtained by ultrasound 1 sensor.*/
 double ultrasound_2_measurement; /*< [m] Measurement obtained by ultrasound 2 sensor.*/
 double ultrasound_3_measurement; /*< [m] Measurement obtained by ultrasound 3 sensor.*/
 int8_t ultrasound_1_valid; /*< [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.*/
 int8_t ultrasound_2_valid; /*< [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.*/
 int8_t ultrasound_3_valid; /*< [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.*/
}) mavlink_naia_ultrasound_wrapper_t;

#define MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN 27
#define MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_MIN_LEN 27
#define MAVLINK_MSG_ID_622_LEN 27
#define MAVLINK_MSG_ID_622_MIN_LEN 27

#define MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_CRC 210
#define MAVLINK_MSG_ID_622_CRC 210



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAIA_ULTRASOUND_WRAPPER { \
    622, \
    "NAIA_ULTRASOUND_WRAPPER", \
    6, \
    {  { "ultrasound_1_measurement", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_naia_ultrasound_wrapper_t, ultrasound_1_measurement) }, \
         { "ultrasound_2_measurement", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_naia_ultrasound_wrapper_t, ultrasound_2_measurement) }, \
         { "ultrasound_3_measurement", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_naia_ultrasound_wrapper_t, ultrasound_3_measurement) }, \
         { "ultrasound_1_valid", NULL, MAVLINK_TYPE_INT8_T, 0, 24, offsetof(mavlink_naia_ultrasound_wrapper_t, ultrasound_1_valid) }, \
         { "ultrasound_2_valid", NULL, MAVLINK_TYPE_INT8_T, 0, 25, offsetof(mavlink_naia_ultrasound_wrapper_t, ultrasound_2_valid) }, \
         { "ultrasound_3_valid", NULL, MAVLINK_TYPE_INT8_T, 0, 26, offsetof(mavlink_naia_ultrasound_wrapper_t, ultrasound_3_valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAIA_ULTRASOUND_WRAPPER { \
    "NAIA_ULTRASOUND_WRAPPER", \
    6, \
    {  { "ultrasound_1_measurement", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_naia_ultrasound_wrapper_t, ultrasound_1_measurement) }, \
         { "ultrasound_2_measurement", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_naia_ultrasound_wrapper_t, ultrasound_2_measurement) }, \
         { "ultrasound_3_measurement", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_naia_ultrasound_wrapper_t, ultrasound_3_measurement) }, \
         { "ultrasound_1_valid", NULL, MAVLINK_TYPE_INT8_T, 0, 24, offsetof(mavlink_naia_ultrasound_wrapper_t, ultrasound_1_valid) }, \
         { "ultrasound_2_valid", NULL, MAVLINK_TYPE_INT8_T, 0, 25, offsetof(mavlink_naia_ultrasound_wrapper_t, ultrasound_2_valid) }, \
         { "ultrasound_3_valid", NULL, MAVLINK_TYPE_INT8_T, 0, 26, offsetof(mavlink_naia_ultrasound_wrapper_t, ultrasound_3_valid) }, \
         } \
}
#endif

/**
 * @brief Pack a naia_ultrasound_wrapper message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param ultrasound_1_measurement [m] Measurement obtained by ultrasound 1 sensor.
 * @param ultrasound_2_measurement [m] Measurement obtained by ultrasound 2 sensor.
 * @param ultrasound_3_measurement [m] Measurement obtained by ultrasound 3 sensor.
 * @param ultrasound_1_valid [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.
 * @param ultrasound_2_valid [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.
 * @param ultrasound_3_valid [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_ultrasound_wrapper_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               double ultrasound_1_measurement, double ultrasound_2_measurement, double ultrasound_3_measurement, int8_t ultrasound_1_valid, int8_t ultrasound_2_valid, int8_t ultrasound_3_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN];
    _mav_put_double(buf, 0, ultrasound_1_measurement);
    _mav_put_double(buf, 8, ultrasound_2_measurement);
    _mav_put_double(buf, 16, ultrasound_3_measurement);
    _mav_put_int8_t(buf, 24, ultrasound_1_valid);
    _mav_put_int8_t(buf, 25, ultrasound_2_valid);
    _mav_put_int8_t(buf, 26, ultrasound_3_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN);
#else
    mavlink_naia_ultrasound_wrapper_t packet;
    packet.ultrasound_1_measurement = ultrasound_1_measurement;
    packet.ultrasound_2_measurement = ultrasound_2_measurement;
    packet.ultrasound_3_measurement = ultrasound_3_measurement;
    packet.ultrasound_1_valid = ultrasound_1_valid;
    packet.ultrasound_2_valid = ultrasound_2_valid;
    packet.ultrasound_3_valid = ultrasound_3_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_MIN_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_CRC);
}

/**
 * @brief Pack a naia_ultrasound_wrapper message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ultrasound_1_measurement [m] Measurement obtained by ultrasound 1 sensor.
 * @param ultrasound_2_measurement [m] Measurement obtained by ultrasound 2 sensor.
 * @param ultrasound_3_measurement [m] Measurement obtained by ultrasound 3 sensor.
 * @param ultrasound_1_valid [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.
 * @param ultrasound_2_valid [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.
 * @param ultrasound_3_valid [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_ultrasound_wrapper_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   double ultrasound_1_measurement,double ultrasound_2_measurement,double ultrasound_3_measurement,int8_t ultrasound_1_valid,int8_t ultrasound_2_valid,int8_t ultrasound_3_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN];
    _mav_put_double(buf, 0, ultrasound_1_measurement);
    _mav_put_double(buf, 8, ultrasound_2_measurement);
    _mav_put_double(buf, 16, ultrasound_3_measurement);
    _mav_put_int8_t(buf, 24, ultrasound_1_valid);
    _mav_put_int8_t(buf, 25, ultrasound_2_valid);
    _mav_put_int8_t(buf, 26, ultrasound_3_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN);
#else
    mavlink_naia_ultrasound_wrapper_t packet;
    packet.ultrasound_1_measurement = ultrasound_1_measurement;
    packet.ultrasound_2_measurement = ultrasound_2_measurement;
    packet.ultrasound_3_measurement = ultrasound_3_measurement;
    packet.ultrasound_1_valid = ultrasound_1_valid;
    packet.ultrasound_2_valid = ultrasound_2_valid;
    packet.ultrasound_3_valid = ultrasound_3_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_MIN_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_CRC);
}

/**
 * @brief Encode a naia_ultrasound_wrapper struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param naia_ultrasound_wrapper C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_ultrasound_wrapper_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_naia_ultrasound_wrapper_t* naia_ultrasound_wrapper)
{
    return mavlink_msg_naia_ultrasound_wrapper_pack(system_id, component_id, msg, naia_ultrasound_wrapper->ultrasound_1_measurement, naia_ultrasound_wrapper->ultrasound_2_measurement, naia_ultrasound_wrapper->ultrasound_3_measurement, naia_ultrasound_wrapper->ultrasound_1_valid, naia_ultrasound_wrapper->ultrasound_2_valid, naia_ultrasound_wrapper->ultrasound_3_valid);
}

/**
 * @brief Encode a naia_ultrasound_wrapper struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param naia_ultrasound_wrapper C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_ultrasound_wrapper_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_naia_ultrasound_wrapper_t* naia_ultrasound_wrapper)
{
    return mavlink_msg_naia_ultrasound_wrapper_pack_chan(system_id, component_id, chan, msg, naia_ultrasound_wrapper->ultrasound_1_measurement, naia_ultrasound_wrapper->ultrasound_2_measurement, naia_ultrasound_wrapper->ultrasound_3_measurement, naia_ultrasound_wrapper->ultrasound_1_valid, naia_ultrasound_wrapper->ultrasound_2_valid, naia_ultrasound_wrapper->ultrasound_3_valid);
}

/**
 * @brief Send a naia_ultrasound_wrapper message
 * @param chan MAVLink channel to send the message
 *
 * @param ultrasound_1_measurement [m] Measurement obtained by ultrasound 1 sensor.
 * @param ultrasound_2_measurement [m] Measurement obtained by ultrasound 2 sensor.
 * @param ultrasound_3_measurement [m] Measurement obtained by ultrasound 3 sensor.
 * @param ultrasound_1_valid [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.
 * @param ultrasound_2_valid [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.
 * @param ultrasound_3_valid [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_naia_ultrasound_wrapper_send(mavlink_channel_t chan, double ultrasound_1_measurement, double ultrasound_2_measurement, double ultrasound_3_measurement, int8_t ultrasound_1_valid, int8_t ultrasound_2_valid, int8_t ultrasound_3_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN];
    _mav_put_double(buf, 0, ultrasound_1_measurement);
    _mav_put_double(buf, 8, ultrasound_2_measurement);
    _mav_put_double(buf, 16, ultrasound_3_measurement);
    _mav_put_int8_t(buf, 24, ultrasound_1_valid);
    _mav_put_int8_t(buf, 25, ultrasound_2_valid);
    _mav_put_int8_t(buf, 26, ultrasound_3_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER, buf, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_MIN_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_CRC);
#else
    mavlink_naia_ultrasound_wrapper_t packet;
    packet.ultrasound_1_measurement = ultrasound_1_measurement;
    packet.ultrasound_2_measurement = ultrasound_2_measurement;
    packet.ultrasound_3_measurement = ultrasound_3_measurement;
    packet.ultrasound_1_valid = ultrasound_1_valid;
    packet.ultrasound_2_valid = ultrasound_2_valid;
    packet.ultrasound_3_valid = ultrasound_3_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER, (const char *)&packet, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_MIN_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_CRC);
#endif
}

/**
 * @brief Send a naia_ultrasound_wrapper message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_naia_ultrasound_wrapper_send_struct(mavlink_channel_t chan, const mavlink_naia_ultrasound_wrapper_t* naia_ultrasound_wrapper)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_naia_ultrasound_wrapper_send(chan, naia_ultrasound_wrapper->ultrasound_1_measurement, naia_ultrasound_wrapper->ultrasound_2_measurement, naia_ultrasound_wrapper->ultrasound_3_measurement, naia_ultrasound_wrapper->ultrasound_1_valid, naia_ultrasound_wrapper->ultrasound_2_valid, naia_ultrasound_wrapper->ultrasound_3_valid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER, (const char *)naia_ultrasound_wrapper, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_MIN_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_naia_ultrasound_wrapper_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  double ultrasound_1_measurement, double ultrasound_2_measurement, double ultrasound_3_measurement, int8_t ultrasound_1_valid, int8_t ultrasound_2_valid, int8_t ultrasound_3_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_double(buf, 0, ultrasound_1_measurement);
    _mav_put_double(buf, 8, ultrasound_2_measurement);
    _mav_put_double(buf, 16, ultrasound_3_measurement);
    _mav_put_int8_t(buf, 24, ultrasound_1_valid);
    _mav_put_int8_t(buf, 25, ultrasound_2_valid);
    _mav_put_int8_t(buf, 26, ultrasound_3_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER, buf, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_MIN_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_CRC);
#else
    mavlink_naia_ultrasound_wrapper_t *packet = (mavlink_naia_ultrasound_wrapper_t *)msgbuf;
    packet->ultrasound_1_measurement = ultrasound_1_measurement;
    packet->ultrasound_2_measurement = ultrasound_2_measurement;
    packet->ultrasound_3_measurement = ultrasound_3_measurement;
    packet->ultrasound_1_valid = ultrasound_1_valid;
    packet->ultrasound_2_valid = ultrasound_2_valid;
    packet->ultrasound_3_valid = ultrasound_3_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER, (const char *)packet, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_MIN_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_CRC);
#endif
}
#endif

#endif

// MESSAGE NAIA_ULTRASOUND_WRAPPER UNPACKING


/**
 * @brief Get field ultrasound_1_measurement from naia_ultrasound_wrapper message
 *
 * @return [m] Measurement obtained by ultrasound 1 sensor.
 */
static inline double mavlink_msg_naia_ultrasound_wrapper_get_ultrasound_1_measurement(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field ultrasound_2_measurement from naia_ultrasound_wrapper message
 *
 * @return [m] Measurement obtained by ultrasound 2 sensor.
 */
static inline double mavlink_msg_naia_ultrasound_wrapper_get_ultrasound_2_measurement(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field ultrasound_3_measurement from naia_ultrasound_wrapper message
 *
 * @return [m] Measurement obtained by ultrasound 3 sensor.
 */
static inline double mavlink_msg_naia_ultrasound_wrapper_get_ultrasound_3_measurement(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field ultrasound_1_valid from naia_ultrasound_wrapper message
 *
 * @return [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.
 */
static inline int8_t mavlink_msg_naia_ultrasound_wrapper_get_ultrasound_1_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  24);
}

/**
 * @brief Get field ultrasound_2_valid from naia_ultrasound_wrapper message
 *
 * @return [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.
 */
static inline int8_t mavlink_msg_naia_ultrasound_wrapper_get_ultrasound_2_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  25);
}

/**
 * @brief Get field ultrasound_3_valid from naia_ultrasound_wrapper message
 *
 * @return [m] 1: beyond upper limit, 0: OK (within limits), -1: comms error.
 */
static inline int8_t mavlink_msg_naia_ultrasound_wrapper_get_ultrasound_3_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int8_t(msg,  26);
}

/**
 * @brief Decode a naia_ultrasound_wrapper message into a struct
 *
 * @param msg The message to decode
 * @param naia_ultrasound_wrapper C-struct to decode the message contents into
 */
static inline void mavlink_msg_naia_ultrasound_wrapper_decode(const mavlink_message_t* msg, mavlink_naia_ultrasound_wrapper_t* naia_ultrasound_wrapper)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    naia_ultrasound_wrapper->ultrasound_1_measurement = mavlink_msg_naia_ultrasound_wrapper_get_ultrasound_1_measurement(msg);
    naia_ultrasound_wrapper->ultrasound_2_measurement = mavlink_msg_naia_ultrasound_wrapper_get_ultrasound_2_measurement(msg);
    naia_ultrasound_wrapper->ultrasound_3_measurement = mavlink_msg_naia_ultrasound_wrapper_get_ultrasound_3_measurement(msg);
    naia_ultrasound_wrapper->ultrasound_1_valid = mavlink_msg_naia_ultrasound_wrapper_get_ultrasound_1_valid(msg);
    naia_ultrasound_wrapper->ultrasound_2_valid = mavlink_msg_naia_ultrasound_wrapper_get_ultrasound_2_valid(msg);
    naia_ultrasound_wrapper->ultrasound_3_valid = mavlink_msg_naia_ultrasound_wrapper_get_ultrasound_3_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN? msg->len : MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN;
        memset(naia_ultrasound_wrapper, 0, MAVLINK_MSG_ID_NAIA_ULTRASOUND_WRAPPER_LEN);
    memcpy(naia_ultrasound_wrapper, _MAV_PAYLOAD(msg), len);
#endif
}
