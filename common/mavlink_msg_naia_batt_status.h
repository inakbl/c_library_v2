#pragma once
// MESSAGE NAIA_BATT_STATUS PACKING

#define MAVLINK_MSG_ID_NAIA_BATT_STATUS 621

MAVPACKED(
typedef struct __mavlink_naia_batt_status_t {
 uint64_t error_count[3]; /*<  */
 uint32_t voltage[3]; /*< [mV] */
 uint32_t current[3]; /*<  */
 uint32_t termistor_1[3]; /*< [deg] */
 uint32_t termistor_2[3]; /*< [deg] */
 uint8_t batt_id[3]; /*<  */
}) mavlink_naia_batt_status_t;

#define MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN 75
#define MAVLINK_MSG_ID_NAIA_BATT_STATUS_MIN_LEN 75
#define MAVLINK_MSG_ID_621_LEN 75
#define MAVLINK_MSG_ID_621_MIN_LEN 75

#define MAVLINK_MSG_ID_NAIA_BATT_STATUS_CRC 96
#define MAVLINK_MSG_ID_621_CRC 96

#define MAVLINK_MSG_NAIA_BATT_STATUS_FIELD_ERROR_COUNT_LEN 3
#define MAVLINK_MSG_NAIA_BATT_STATUS_FIELD_VOLTAGE_LEN 3
#define MAVLINK_MSG_NAIA_BATT_STATUS_FIELD_CURRENT_LEN 3
#define MAVLINK_MSG_NAIA_BATT_STATUS_FIELD_TERMISTOR_1_LEN 3
#define MAVLINK_MSG_NAIA_BATT_STATUS_FIELD_TERMISTOR_2_LEN 3
#define MAVLINK_MSG_NAIA_BATT_STATUS_FIELD_BATT_ID_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAIA_BATT_STATUS { \
    621, \
    "NAIA_BATT_STATUS", \
    6, \
    {  { "batt_id", NULL, MAVLINK_TYPE_UINT8_T, 3, 72, offsetof(mavlink_naia_batt_status_t, batt_id) }, \
         { "error_count", NULL, MAVLINK_TYPE_UINT64_T, 3, 0, offsetof(mavlink_naia_batt_status_t, error_count) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT32_T, 3, 24, offsetof(mavlink_naia_batt_status_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_UINT32_T, 3, 36, offsetof(mavlink_naia_batt_status_t, current) }, \
         { "termistor_1", NULL, MAVLINK_TYPE_UINT32_T, 3, 48, offsetof(mavlink_naia_batt_status_t, termistor_1) }, \
         { "termistor_2", NULL, MAVLINK_TYPE_UINT32_T, 3, 60, offsetof(mavlink_naia_batt_status_t, termistor_2) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAIA_BATT_STATUS { \
    "NAIA_BATT_STATUS", \
    6, \
    {  { "batt_id", NULL, MAVLINK_TYPE_UINT8_T, 3, 72, offsetof(mavlink_naia_batt_status_t, batt_id) }, \
         { "error_count", NULL, MAVLINK_TYPE_UINT64_T, 3, 0, offsetof(mavlink_naia_batt_status_t, error_count) }, \
         { "voltage", NULL, MAVLINK_TYPE_UINT32_T, 3, 24, offsetof(mavlink_naia_batt_status_t, voltage) }, \
         { "current", NULL, MAVLINK_TYPE_UINT32_T, 3, 36, offsetof(mavlink_naia_batt_status_t, current) }, \
         { "termistor_1", NULL, MAVLINK_TYPE_UINT32_T, 3, 48, offsetof(mavlink_naia_batt_status_t, termistor_1) }, \
         { "termistor_2", NULL, MAVLINK_TYPE_UINT32_T, 3, 60, offsetof(mavlink_naia_batt_status_t, termistor_2) }, \
         } \
}
#endif

/**
 * @brief Pack a naia_batt_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param batt_id  
 * @param error_count  
 * @param voltage [mV] 
 * @param current  
 * @param termistor_1 [deg] 
 * @param termistor_2 [deg] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_batt_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const uint8_t *batt_id, const uint64_t *error_count, const uint32_t *voltage, const uint32_t *current, const uint32_t *termistor_1, const uint32_t *termistor_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN];

    _mav_put_uint64_t_array(buf, 0, error_count, 3);
    _mav_put_uint32_t_array(buf, 24, voltage, 3);
    _mav_put_uint32_t_array(buf, 36, current, 3);
    _mav_put_uint32_t_array(buf, 48, termistor_1, 3);
    _mav_put_uint32_t_array(buf, 60, termistor_2, 3);
    _mav_put_uint8_t_array(buf, 72, batt_id, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN);
#else
    mavlink_naia_batt_status_t packet;

    mav_array_memcpy(packet.error_count, error_count, sizeof(uint64_t)*3);
    mav_array_memcpy(packet.voltage, voltage, sizeof(uint32_t)*3);
    mav_array_memcpy(packet.current, current, sizeof(uint32_t)*3);
    mav_array_memcpy(packet.termistor_1, termistor_1, sizeof(uint32_t)*3);
    mav_array_memcpy(packet.termistor_2, termistor_2, sizeof(uint32_t)*3);
    mav_array_memcpy(packet.batt_id, batt_id, sizeof(uint8_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_BATT_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAIA_BATT_STATUS_MIN_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_CRC);
}

/**
 * @brief Pack a naia_batt_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param batt_id  
 * @param error_count  
 * @param voltage [mV] 
 * @param current  
 * @param termistor_1 [deg] 
 * @param termistor_2 [deg] 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_batt_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const uint8_t *batt_id,const uint64_t *error_count,const uint32_t *voltage,const uint32_t *current,const uint32_t *termistor_1,const uint32_t *termistor_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN];

    _mav_put_uint64_t_array(buf, 0, error_count, 3);
    _mav_put_uint32_t_array(buf, 24, voltage, 3);
    _mav_put_uint32_t_array(buf, 36, current, 3);
    _mav_put_uint32_t_array(buf, 48, termistor_1, 3);
    _mav_put_uint32_t_array(buf, 60, termistor_2, 3);
    _mav_put_uint8_t_array(buf, 72, batt_id, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN);
#else
    mavlink_naia_batt_status_t packet;

    mav_array_memcpy(packet.error_count, error_count, sizeof(uint64_t)*3);
    mav_array_memcpy(packet.voltage, voltage, sizeof(uint32_t)*3);
    mav_array_memcpy(packet.current, current, sizeof(uint32_t)*3);
    mav_array_memcpy(packet.termistor_1, termistor_1, sizeof(uint32_t)*3);
    mav_array_memcpy(packet.termistor_2, termistor_2, sizeof(uint32_t)*3);
    mav_array_memcpy(packet.batt_id, batt_id, sizeof(uint8_t)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_BATT_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAIA_BATT_STATUS_MIN_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_CRC);
}

/**
 * @brief Encode a naia_batt_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param naia_batt_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_batt_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_naia_batt_status_t* naia_batt_status)
{
    return mavlink_msg_naia_batt_status_pack(system_id, component_id, msg, naia_batt_status->batt_id, naia_batt_status->error_count, naia_batt_status->voltage, naia_batt_status->current, naia_batt_status->termistor_1, naia_batt_status->termistor_2);
}

/**
 * @brief Encode a naia_batt_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param naia_batt_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_batt_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_naia_batt_status_t* naia_batt_status)
{
    return mavlink_msg_naia_batt_status_pack_chan(system_id, component_id, chan, msg, naia_batt_status->batt_id, naia_batt_status->error_count, naia_batt_status->voltage, naia_batt_status->current, naia_batt_status->termistor_1, naia_batt_status->termistor_2);
}

/**
 * @brief Send a naia_batt_status message
 * @param chan MAVLink channel to send the message
 *
 * @param batt_id  
 * @param error_count  
 * @param voltage [mV] 
 * @param current  
 * @param termistor_1 [deg] 
 * @param termistor_2 [deg] 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_naia_batt_status_send(mavlink_channel_t chan, const uint8_t *batt_id, const uint64_t *error_count, const uint32_t *voltage, const uint32_t *current, const uint32_t *termistor_1, const uint32_t *termistor_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN];

    _mav_put_uint64_t_array(buf, 0, error_count, 3);
    _mav_put_uint32_t_array(buf, 24, voltage, 3);
    _mav_put_uint32_t_array(buf, 36, current, 3);
    _mav_put_uint32_t_array(buf, 48, termistor_1, 3);
    _mav_put_uint32_t_array(buf, 60, termistor_2, 3);
    _mav_put_uint8_t_array(buf, 72, batt_id, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_BATT_STATUS, buf, MAVLINK_MSG_ID_NAIA_BATT_STATUS_MIN_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_CRC);
#else
    mavlink_naia_batt_status_t packet;

    mav_array_memcpy(packet.error_count, error_count, sizeof(uint64_t)*3);
    mav_array_memcpy(packet.voltage, voltage, sizeof(uint32_t)*3);
    mav_array_memcpy(packet.current, current, sizeof(uint32_t)*3);
    mav_array_memcpy(packet.termistor_1, termistor_1, sizeof(uint32_t)*3);
    mav_array_memcpy(packet.termistor_2, termistor_2, sizeof(uint32_t)*3);
    mav_array_memcpy(packet.batt_id, batt_id, sizeof(uint8_t)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_BATT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_NAIA_BATT_STATUS_MIN_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_CRC);
#endif
}

/**
 * @brief Send a naia_batt_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_naia_batt_status_send_struct(mavlink_channel_t chan, const mavlink_naia_batt_status_t* naia_batt_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_naia_batt_status_send(chan, naia_batt_status->batt_id, naia_batt_status->error_count, naia_batt_status->voltage, naia_batt_status->current, naia_batt_status->termistor_1, naia_batt_status->termistor_2);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_BATT_STATUS, (const char *)naia_batt_status, MAVLINK_MSG_ID_NAIA_BATT_STATUS_MIN_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_naia_batt_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const uint8_t *batt_id, const uint64_t *error_count, const uint32_t *voltage, const uint32_t *current, const uint32_t *termistor_1, const uint32_t *termistor_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;

    _mav_put_uint64_t_array(buf, 0, error_count, 3);
    _mav_put_uint32_t_array(buf, 24, voltage, 3);
    _mav_put_uint32_t_array(buf, 36, current, 3);
    _mav_put_uint32_t_array(buf, 48, termistor_1, 3);
    _mav_put_uint32_t_array(buf, 60, termistor_2, 3);
    _mav_put_uint8_t_array(buf, 72, batt_id, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_BATT_STATUS, buf, MAVLINK_MSG_ID_NAIA_BATT_STATUS_MIN_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_CRC);
#else
    mavlink_naia_batt_status_t *packet = (mavlink_naia_batt_status_t *)msgbuf;

    mav_array_memcpy(packet->error_count, error_count, sizeof(uint64_t)*3);
    mav_array_memcpy(packet->voltage, voltage, sizeof(uint32_t)*3);
    mav_array_memcpy(packet->current, current, sizeof(uint32_t)*3);
    mav_array_memcpy(packet->termistor_1, termistor_1, sizeof(uint32_t)*3);
    mav_array_memcpy(packet->termistor_2, termistor_2, sizeof(uint32_t)*3);
    mav_array_memcpy(packet->batt_id, batt_id, sizeof(uint8_t)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_BATT_STATUS, (const char *)packet, MAVLINK_MSG_ID_NAIA_BATT_STATUS_MIN_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN, MAVLINK_MSG_ID_NAIA_BATT_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE NAIA_BATT_STATUS UNPACKING


/**
 * @brief Get field batt_id from naia_batt_status message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_naia_batt_status_get_batt_id(const mavlink_message_t* msg, uint8_t *batt_id)
{
    return _MAV_RETURN_uint8_t_array(msg, batt_id, 3,  72);
}

/**
 * @brief Get field error_count from naia_batt_status message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_naia_batt_status_get_error_count(const mavlink_message_t* msg, uint64_t *error_count)
{
    return _MAV_RETURN_uint64_t_array(msg, error_count, 3,  0);
}

/**
 * @brief Get field voltage from naia_batt_status message
 *
 * @return [mV] 
 */
static inline uint16_t mavlink_msg_naia_batt_status_get_voltage(const mavlink_message_t* msg, uint32_t *voltage)
{
    return _MAV_RETURN_uint32_t_array(msg, voltage, 3,  24);
}

/**
 * @brief Get field current from naia_batt_status message
 *
 * @return  
 */
static inline uint16_t mavlink_msg_naia_batt_status_get_current(const mavlink_message_t* msg, uint32_t *current)
{
    return _MAV_RETURN_uint32_t_array(msg, current, 3,  36);
}

/**
 * @brief Get field termistor_1 from naia_batt_status message
 *
 * @return [deg] 
 */
static inline uint16_t mavlink_msg_naia_batt_status_get_termistor_1(const mavlink_message_t* msg, uint32_t *termistor_1)
{
    return _MAV_RETURN_uint32_t_array(msg, termistor_1, 3,  48);
}

/**
 * @brief Get field termistor_2 from naia_batt_status message
 *
 * @return [deg] 
 */
static inline uint16_t mavlink_msg_naia_batt_status_get_termistor_2(const mavlink_message_t* msg, uint32_t *termistor_2)
{
    return _MAV_RETURN_uint32_t_array(msg, termistor_2, 3,  60);
}

/**
 * @brief Decode a naia_batt_status message into a struct
 *
 * @param msg The message to decode
 * @param naia_batt_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_naia_batt_status_decode(const mavlink_message_t* msg, mavlink_naia_batt_status_t* naia_batt_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_naia_batt_status_get_error_count(msg, naia_batt_status->error_count);
    mavlink_msg_naia_batt_status_get_voltage(msg, naia_batt_status->voltage);
    mavlink_msg_naia_batt_status_get_current(msg, naia_batt_status->current);
    mavlink_msg_naia_batt_status_get_termistor_1(msg, naia_batt_status->termistor_1);
    mavlink_msg_naia_batt_status_get_termistor_2(msg, naia_batt_status->termistor_2);
    mavlink_msg_naia_batt_status_get_batt_id(msg, naia_batt_status->batt_id);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN? msg->len : MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN;
        memset(naia_batt_status, 0, MAVLINK_MSG_ID_NAIA_BATT_STATUS_LEN);
    memcpy(naia_batt_status, _MAV_PAYLOAD(msg), len);
#endif
}
