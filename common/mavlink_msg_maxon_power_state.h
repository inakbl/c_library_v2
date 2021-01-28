#pragma once
// MESSAGE MAXON_POWER_STATE PACKING

#define MAVLINK_MSG_ID_MAXON_POWER_STATE 625


typedef struct __mavlink_maxon_power_state_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 int32_t current[5]; /*< [mA]  Current as reported by Epos devices. */
 float voltage[5]; /*< [mV]  Voltage as reported by Epos devices. */
 float temperature[5]; /*< [deg]  Temperature as reported by Epos devices. */
} mavlink_maxon_power_state_t;

#define MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN 68
#define MAVLINK_MSG_ID_MAXON_POWER_STATE_MIN_LEN 68
#define MAVLINK_MSG_ID_625_LEN 68
#define MAVLINK_MSG_ID_625_MIN_LEN 68

#define MAVLINK_MSG_ID_MAXON_POWER_STATE_CRC 234
#define MAVLINK_MSG_ID_625_CRC 234

#define MAVLINK_MSG_MAXON_POWER_STATE_FIELD_CURRENT_LEN 5
#define MAVLINK_MSG_MAXON_POWER_STATE_FIELD_VOLTAGE_LEN 5
#define MAVLINK_MSG_MAXON_POWER_STATE_FIELD_TEMPERATURE_LEN 5

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_MAXON_POWER_STATE { \
    625, \
    "MAXON_POWER_STATE", \
    4, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_maxon_power_state_t, timestamp) }, \
         { "current", NULL, MAVLINK_TYPE_INT32_T, 5, 8, offsetof(mavlink_maxon_power_state_t, current) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 5, 28, offsetof(mavlink_maxon_power_state_t, voltage) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 5, 48, offsetof(mavlink_maxon_power_state_t, temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_MAXON_POWER_STATE { \
    "MAXON_POWER_STATE", \
    4, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_maxon_power_state_t, timestamp) }, \
         { "current", NULL, MAVLINK_TYPE_INT32_T, 5, 8, offsetof(mavlink_maxon_power_state_t, current) }, \
         { "voltage", NULL, MAVLINK_TYPE_FLOAT, 5, 28, offsetof(mavlink_maxon_power_state_t, voltage) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 5, 48, offsetof(mavlink_maxon_power_state_t, temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a maxon_power_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param current [mA]  Current as reported by Epos devices. 
 * @param voltage [mV]  Voltage as reported by Epos devices. 
 * @param temperature [deg]  Temperature as reported by Epos devices. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_maxon_power_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, const int32_t *current, const float *voltage, const float *temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t_array(buf, 8, current, 5);
    _mav_put_float_array(buf, 28, voltage, 5);
    _mav_put_float_array(buf, 48, temperature, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN);
#else
    mavlink_maxon_power_state_t packet;
    packet.timestamp = timestamp;
    mav_array_memcpy(packet.current, current, sizeof(int32_t)*5);
    mav_array_memcpy(packet.voltage, voltage, sizeof(float)*5);
    mav_array_memcpy(packet.temperature, temperature, sizeof(float)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAXON_POWER_STATE;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAXON_POWER_STATE_MIN_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_CRC);
}

/**
 * @brief Pack a maxon_power_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param current [mA]  Current as reported by Epos devices. 
 * @param voltage [mV]  Voltage as reported by Epos devices. 
 * @param temperature [deg]  Temperature as reported by Epos devices. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_maxon_power_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,const int32_t *current,const float *voltage,const float *temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t_array(buf, 8, current, 5);
    _mav_put_float_array(buf, 28, voltage, 5);
    _mav_put_float_array(buf, 48, temperature, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN);
#else
    mavlink_maxon_power_state_t packet;
    packet.timestamp = timestamp;
    mav_array_memcpy(packet.current, current, sizeof(int32_t)*5);
    mav_array_memcpy(packet.voltage, voltage, sizeof(float)*5);
    mav_array_memcpy(packet.temperature, temperature, sizeof(float)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_MAXON_POWER_STATE;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAXON_POWER_STATE_MIN_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_CRC);
}

/**
 * @brief Encode a maxon_power_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param maxon_power_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_maxon_power_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_maxon_power_state_t* maxon_power_state)
{
    return mavlink_msg_maxon_power_state_pack(system_id, component_id, msg, maxon_power_state->timestamp, maxon_power_state->current, maxon_power_state->voltage, maxon_power_state->temperature);
}

/**
 * @brief Encode a maxon_power_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param maxon_power_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_maxon_power_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_maxon_power_state_t* maxon_power_state)
{
    return mavlink_msg_maxon_power_state_pack_chan(system_id, component_id, chan, msg, maxon_power_state->timestamp, maxon_power_state->current, maxon_power_state->voltage, maxon_power_state->temperature);
}

/**
 * @brief Send a maxon_power_state message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param current [mA]  Current as reported by Epos devices. 
 * @param voltage [mV]  Voltage as reported by Epos devices. 
 * @param temperature [deg]  Temperature as reported by Epos devices. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_maxon_power_state_send(mavlink_channel_t chan, uint64_t timestamp, const int32_t *current, const float *voltage, const float *temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t_array(buf, 8, current, 5);
    _mav_put_float_array(buf, 28, voltage, 5);
    _mav_put_float_array(buf, 48, temperature, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAXON_POWER_STATE, buf, MAVLINK_MSG_ID_MAXON_POWER_STATE_MIN_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_CRC);
#else
    mavlink_maxon_power_state_t packet;
    packet.timestamp = timestamp;
    mav_array_memcpy(packet.current, current, sizeof(int32_t)*5);
    mav_array_memcpy(packet.voltage, voltage, sizeof(float)*5);
    mav_array_memcpy(packet.temperature, temperature, sizeof(float)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAXON_POWER_STATE, (const char *)&packet, MAVLINK_MSG_ID_MAXON_POWER_STATE_MIN_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_CRC);
#endif
}

/**
 * @brief Send a maxon_power_state message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_maxon_power_state_send_struct(mavlink_channel_t chan, const mavlink_maxon_power_state_t* maxon_power_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_maxon_power_state_send(chan, maxon_power_state->timestamp, maxon_power_state->current, maxon_power_state->voltage, maxon_power_state->temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAXON_POWER_STATE, (const char *)maxon_power_state, MAVLINK_MSG_ID_MAXON_POWER_STATE_MIN_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_CRC);
#endif
}

#if MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_maxon_power_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, const int32_t *current, const float *voltage, const float *temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_int32_t_array(buf, 8, current, 5);
    _mav_put_float_array(buf, 28, voltage, 5);
    _mav_put_float_array(buf, 48, temperature, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAXON_POWER_STATE, buf, MAVLINK_MSG_ID_MAXON_POWER_STATE_MIN_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_CRC);
#else
    mavlink_maxon_power_state_t *packet = (mavlink_maxon_power_state_t *)msgbuf;
    packet->timestamp = timestamp;
    mav_array_memcpy(packet->current, current, sizeof(int32_t)*5);
    mav_array_memcpy(packet->voltage, voltage, sizeof(float)*5);
    mav_array_memcpy(packet->temperature, temperature, sizeof(float)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAXON_POWER_STATE, (const char *)packet, MAVLINK_MSG_ID_MAXON_POWER_STATE_MIN_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN, MAVLINK_MSG_ID_MAXON_POWER_STATE_CRC);
#endif
}
#endif

#endif

// MESSAGE MAXON_POWER_STATE UNPACKING


/**
 * @brief Get field timestamp from maxon_power_state message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_maxon_power_state_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field current from maxon_power_state message
 *
 * @return [mA]  Current as reported by Epos devices. 
 */
static inline uint16_t mavlink_msg_maxon_power_state_get_current(const mavlink_message_t* msg, int32_t *current)
{
    return _MAV_RETURN_int32_t_array(msg, current, 5,  8);
}

/**
 * @brief Get field voltage from maxon_power_state message
 *
 * @return [mV]  Voltage as reported by Epos devices. 
 */
static inline uint16_t mavlink_msg_maxon_power_state_get_voltage(const mavlink_message_t* msg, float *voltage)
{
    return _MAV_RETURN_float_array(msg, voltage, 5,  28);
}

/**
 * @brief Get field temperature from maxon_power_state message
 *
 * @return [deg]  Temperature as reported by Epos devices. 
 */
static inline uint16_t mavlink_msg_maxon_power_state_get_temperature(const mavlink_message_t* msg, float *temperature)
{
    return _MAV_RETURN_float_array(msg, temperature, 5,  48);
}

/**
 * @brief Decode a maxon_power_state message into a struct
 *
 * @param msg The message to decode
 * @param maxon_power_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_maxon_power_state_decode(const mavlink_message_t* msg, mavlink_maxon_power_state_t* maxon_power_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    maxon_power_state->timestamp = mavlink_msg_maxon_power_state_get_timestamp(msg);
    mavlink_msg_maxon_power_state_get_current(msg, maxon_power_state->current);
    mavlink_msg_maxon_power_state_get_voltage(msg, maxon_power_state->voltage);
    mavlink_msg_maxon_power_state_get_temperature(msg, maxon_power_state->temperature);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN? msg->len : MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN;
        memset(maxon_power_state, 0, MAVLINK_MSG_ID_MAXON_POWER_STATE_LEN);
    memcpy(maxon_power_state, _MAV_PAYLOAD(msg), len);
#endif
}
