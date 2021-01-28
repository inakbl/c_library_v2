#pragma once
// MESSAGE X_728_COMMS PACKING

#define MAVLINK_MSG_ID_X_728_COMMS 626


typedef struct __mavlink_x_728_comms_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 float voltage; /*< [V]  UPS batteries voltage. */
 float capacity; /*<   UPS batteries remaining capacity. */
 uint8_t power_adapter; /*<   0: Disconnected, 1: Connected, 2: Error. */
} mavlink_x_728_comms_t;

#define MAVLINK_MSG_ID_X_728_COMMS_LEN 17
#define MAVLINK_MSG_ID_X_728_COMMS_MIN_LEN 17
#define MAVLINK_MSG_ID_626_LEN 17
#define MAVLINK_MSG_ID_626_MIN_LEN 17

#define MAVLINK_MSG_ID_X_728_COMMS_CRC 6
#define MAVLINK_MSG_ID_626_CRC 6



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_X_728_COMMS { \
    626, \
    "X_728_COMMS", \
    4, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_x_728_comms_t, timestamp) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_x_728_comms_t, voltage) }, \
         { "capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_x_728_comms_t, capacity) }, \
         { "power_adapter", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_x_728_comms_t, power_adapter) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_X_728_COMMS { \
    "X_728_COMMS", \
    4, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_x_728_comms_t, timestamp) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_x_728_comms_t, voltage) }, \
         { "capacity", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_x_728_comms_t, capacity) }, \
         { "power_adapter", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_x_728_comms_t, power_adapter) }, \
         } \
}
#endif

/**
 * @brief Pack a x_728_comms message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param voltage [V]  UPS batteries voltage. 
 * @param capacity   UPS batteries remaining capacity. 
 * @param power_adapter   0: Disconnected, 1: Connected, 2: Error. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_x_728_comms_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, float voltage, float capacity, uint8_t power_adapter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_X_728_COMMS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, voltage);
    _mav_put_float(buf, 12, capacity);
    _mav_put_uint8_t(buf, 16, power_adapter);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_X_728_COMMS_LEN);
#else
    mavlink_x_728_comms_t packet;
    packet.timestamp = timestamp;
    packet.voltage = voltage;
    packet.capacity = capacity;
    packet.power_adapter = power_adapter;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_X_728_COMMS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_X_728_COMMS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_X_728_COMMS_MIN_LEN, MAVLINK_MSG_ID_X_728_COMMS_LEN, MAVLINK_MSG_ID_X_728_COMMS_CRC);
}

/**
 * @brief Pack a x_728_comms message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param voltage [V]  UPS batteries voltage. 
 * @param capacity   UPS batteries remaining capacity. 
 * @param power_adapter   0: Disconnected, 1: Connected, 2: Error. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_x_728_comms_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,float voltage,float capacity,uint8_t power_adapter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_X_728_COMMS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, voltage);
    _mav_put_float(buf, 12, capacity);
    _mav_put_uint8_t(buf, 16, power_adapter);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_X_728_COMMS_LEN);
#else
    mavlink_x_728_comms_t packet;
    packet.timestamp = timestamp;
    packet.voltage = voltage;
    packet.capacity = capacity;
    packet.power_adapter = power_adapter;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_X_728_COMMS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_X_728_COMMS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_X_728_COMMS_MIN_LEN, MAVLINK_MSG_ID_X_728_COMMS_LEN, MAVLINK_MSG_ID_X_728_COMMS_CRC);
}

/**
 * @brief Encode a x_728_comms struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param x_728_comms C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_x_728_comms_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_x_728_comms_t* x_728_comms)
{
    return mavlink_msg_x_728_comms_pack(system_id, component_id, msg, x_728_comms->timestamp, x_728_comms->voltage, x_728_comms->capacity, x_728_comms->power_adapter);
}

/**
 * @brief Encode a x_728_comms struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param x_728_comms C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_x_728_comms_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_x_728_comms_t* x_728_comms)
{
    return mavlink_msg_x_728_comms_pack_chan(system_id, component_id, chan, msg, x_728_comms->timestamp, x_728_comms->voltage, x_728_comms->capacity, x_728_comms->power_adapter);
}

/**
 * @brief Send a x_728_comms message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param voltage [V]  UPS batteries voltage. 
 * @param capacity   UPS batteries remaining capacity. 
 * @param power_adapter   0: Disconnected, 1: Connected, 2: Error. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_x_728_comms_send(mavlink_channel_t chan, uint64_t timestamp, float voltage, float capacity, uint8_t power_adapter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_X_728_COMMS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, voltage);
    _mav_put_float(buf, 12, capacity);
    _mav_put_uint8_t(buf, 16, power_adapter);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_X_728_COMMS, buf, MAVLINK_MSG_ID_X_728_COMMS_MIN_LEN, MAVLINK_MSG_ID_X_728_COMMS_LEN, MAVLINK_MSG_ID_X_728_COMMS_CRC);
#else
    mavlink_x_728_comms_t packet;
    packet.timestamp = timestamp;
    packet.voltage = voltage;
    packet.capacity = capacity;
    packet.power_adapter = power_adapter;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_X_728_COMMS, (const char *)&packet, MAVLINK_MSG_ID_X_728_COMMS_MIN_LEN, MAVLINK_MSG_ID_X_728_COMMS_LEN, MAVLINK_MSG_ID_X_728_COMMS_CRC);
#endif
}

/**
 * @brief Send a x_728_comms message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_x_728_comms_send_struct(mavlink_channel_t chan, const mavlink_x_728_comms_t* x_728_comms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_x_728_comms_send(chan, x_728_comms->timestamp, x_728_comms->voltage, x_728_comms->capacity, x_728_comms->power_adapter);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_X_728_COMMS, (const char *)x_728_comms, MAVLINK_MSG_ID_X_728_COMMS_MIN_LEN, MAVLINK_MSG_ID_X_728_COMMS_LEN, MAVLINK_MSG_ID_X_728_COMMS_CRC);
#endif
}

#if MAVLINK_MSG_ID_X_728_COMMS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_x_728_comms_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, float voltage, float capacity, uint8_t power_adapter)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_float(buf, 8, voltage);
    _mav_put_float(buf, 12, capacity);
    _mav_put_uint8_t(buf, 16, power_adapter);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_X_728_COMMS, buf, MAVLINK_MSG_ID_X_728_COMMS_MIN_LEN, MAVLINK_MSG_ID_X_728_COMMS_LEN, MAVLINK_MSG_ID_X_728_COMMS_CRC);
#else
    mavlink_x_728_comms_t *packet = (mavlink_x_728_comms_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->voltage = voltage;
    packet->capacity = capacity;
    packet->power_adapter = power_adapter;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_X_728_COMMS, (const char *)packet, MAVLINK_MSG_ID_X_728_COMMS_MIN_LEN, MAVLINK_MSG_ID_X_728_COMMS_LEN, MAVLINK_MSG_ID_X_728_COMMS_CRC);
#endif
}
#endif

#endif

// MESSAGE X_728_COMMS UNPACKING


/**
 * @brief Get field timestamp from x_728_comms message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_x_728_comms_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field voltage from x_728_comms message
 *
 * @return [V]  UPS batteries voltage. 
 */
static inline float mavlink_msg_x_728_comms_get_voltage(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field capacity from x_728_comms message
 *
 * @return   UPS batteries remaining capacity. 
 */
static inline float mavlink_msg_x_728_comms_get_capacity(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field power_adapter from x_728_comms message
 *
 * @return   0: Disconnected, 1: Connected, 2: Error. 
 */
static inline uint8_t mavlink_msg_x_728_comms_get_power_adapter(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Decode a x_728_comms message into a struct
 *
 * @param msg The message to decode
 * @param x_728_comms C-struct to decode the message contents into
 */
static inline void mavlink_msg_x_728_comms_decode(const mavlink_message_t* msg, mavlink_x_728_comms_t* x_728_comms)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    x_728_comms->timestamp = mavlink_msg_x_728_comms_get_timestamp(msg);
    x_728_comms->voltage = mavlink_msg_x_728_comms_get_voltage(msg);
    x_728_comms->capacity = mavlink_msg_x_728_comms_get_capacity(msg);
    x_728_comms->power_adapter = mavlink_msg_x_728_comms_get_power_adapter(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_X_728_COMMS_LEN? msg->len : MAVLINK_MSG_ID_X_728_COMMS_LEN;
        memset(x_728_comms, 0, MAVLINK_MSG_ID_X_728_COMMS_LEN);
    memcpy(x_728_comms, _MAV_PAYLOAD(msg), len);
#endif
}
