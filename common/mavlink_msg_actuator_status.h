#pragma once
// MESSAGE ACTUATOR_STATUS PACKING

#define MAVLINK_MSG_ID_ACTUATOR_STATUS 608


typedef struct __mavlink_actuator_status_t {
 uint64_t index; /*<   Cycle number. */
 uint64_t timestamp; /*< [us]  Timestamp. */
 uint16_t actuator_error[5]; /*<   Current actuator error value. */
 int8_t switchedon[5]; /*<   Whether the actuator is enabled. Bit 0x0002 of status word. */
 int8_t torqueon[5]; /*<   Whether the torque is enabled. Bit 0x1000 of status word. */
 int8_t operative[5]; /*<   Whether the actuator is operative. Bit 0x0004 of status word. */
 int8_t quickstopped[5]; /*<   Whether the actuator is quickstopped. Bit 0x0020 of status word. */
 int8_t faulted[5]; /*<   Whether the actuator is faulted. Bit 0x0008 of status word. */
 uint8_t target_state[5]; /*<   The uavcan driver state machine state. */
 int8_t actuator_idle[5]; /*<   Number of consecutive cycles not communicating. */
 uint8_t motors_state; /*<   Whether actuators are enabled or disabled. 0: Stopped 1: Running. */
} mavlink_actuator_status_t;

#define MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN 62
#define MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN 62
#define MAVLINK_MSG_ID_608_LEN 62
#define MAVLINK_MSG_ID_608_MIN_LEN 62

#define MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC 234
#define MAVLINK_MSG_ID_608_CRC 234

#define MAVLINK_MSG_ACTUATOR_STATUS_FIELD_ACTUATOR_ERROR_LEN 5
#define MAVLINK_MSG_ACTUATOR_STATUS_FIELD_SWITCHEDON_LEN 5
#define MAVLINK_MSG_ACTUATOR_STATUS_FIELD_TORQUEON_LEN 5
#define MAVLINK_MSG_ACTUATOR_STATUS_FIELD_OPERATIVE_LEN 5
#define MAVLINK_MSG_ACTUATOR_STATUS_FIELD_QUICKSTOPPED_LEN 5
#define MAVLINK_MSG_ACTUATOR_STATUS_FIELD_FAULTED_LEN 5
#define MAVLINK_MSG_ACTUATOR_STATUS_FIELD_TARGET_STATE_LEN 5
#define MAVLINK_MSG_ACTUATOR_STATUS_FIELD_ACTUATOR_IDLE_LEN 5

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ACTUATOR_STATUS { \
    608, \
    "ACTUATOR_STATUS", \
    11, \
    {  { "index", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_actuator_status_t, index) }, \
         { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_actuator_status_t, timestamp) }, \
         { "switchedon", NULL, MAVLINK_TYPE_INT8_T, 5, 26, offsetof(mavlink_actuator_status_t, switchedon) }, \
         { "torqueon", NULL, MAVLINK_TYPE_INT8_T, 5, 31, offsetof(mavlink_actuator_status_t, torqueon) }, \
         { "operative", NULL, MAVLINK_TYPE_INT8_T, 5, 36, offsetof(mavlink_actuator_status_t, operative) }, \
         { "quickstopped", NULL, MAVLINK_TYPE_INT8_T, 5, 41, offsetof(mavlink_actuator_status_t, quickstopped) }, \
         { "faulted", NULL, MAVLINK_TYPE_INT8_T, 5, 46, offsetof(mavlink_actuator_status_t, faulted) }, \
         { "target_state", NULL, MAVLINK_TYPE_UINT8_T, 5, 51, offsetof(mavlink_actuator_status_t, target_state) }, \
         { "actuator_idle", NULL, MAVLINK_TYPE_INT8_T, 5, 56, offsetof(mavlink_actuator_status_t, actuator_idle) }, \
         { "actuator_error", NULL, MAVLINK_TYPE_UINT16_T, 5, 16, offsetof(mavlink_actuator_status_t, actuator_error) }, \
         { "motors_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_actuator_status_t, motors_state) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ACTUATOR_STATUS { \
    "ACTUATOR_STATUS", \
    11, \
    {  { "index", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_actuator_status_t, index) }, \
         { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_actuator_status_t, timestamp) }, \
         { "switchedon", NULL, MAVLINK_TYPE_INT8_T, 5, 26, offsetof(mavlink_actuator_status_t, switchedon) }, \
         { "torqueon", NULL, MAVLINK_TYPE_INT8_T, 5, 31, offsetof(mavlink_actuator_status_t, torqueon) }, \
         { "operative", NULL, MAVLINK_TYPE_INT8_T, 5, 36, offsetof(mavlink_actuator_status_t, operative) }, \
         { "quickstopped", NULL, MAVLINK_TYPE_INT8_T, 5, 41, offsetof(mavlink_actuator_status_t, quickstopped) }, \
         { "faulted", NULL, MAVLINK_TYPE_INT8_T, 5, 46, offsetof(mavlink_actuator_status_t, faulted) }, \
         { "target_state", NULL, MAVLINK_TYPE_UINT8_T, 5, 51, offsetof(mavlink_actuator_status_t, target_state) }, \
         { "actuator_idle", NULL, MAVLINK_TYPE_INT8_T, 5, 56, offsetof(mavlink_actuator_status_t, actuator_idle) }, \
         { "actuator_error", NULL, MAVLINK_TYPE_UINT16_T, 5, 16, offsetof(mavlink_actuator_status_t, actuator_error) }, \
         { "motors_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 61, offsetof(mavlink_actuator_status_t, motors_state) }, \
         } \
}
#endif

/**
 * @brief Pack a actuator_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param index   Cycle number. 
 * @param timestamp [us]  Timestamp. 
 * @param switchedon   Whether the actuator is enabled. Bit 0x0002 of status word. 
 * @param torqueon   Whether the torque is enabled. Bit 0x1000 of status word. 
 * @param operative   Whether the actuator is operative. Bit 0x0004 of status word. 
 * @param quickstopped   Whether the actuator is quickstopped. Bit 0x0020 of status word. 
 * @param faulted   Whether the actuator is faulted. Bit 0x0008 of status word. 
 * @param target_state   The uavcan driver state machine state. 
 * @param actuator_idle   Number of consecutive cycles not communicating. 
 * @param actuator_error   Current actuator error value. 
 * @param motors_state   Whether actuators are enabled or disabled. 0: Stopped 1: Running. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t index, uint64_t timestamp, const int8_t *switchedon, const int8_t *torqueon, const int8_t *operative, const int8_t *quickstopped, const int8_t *faulted, const uint8_t *target_state, const int8_t *actuator_idle, const uint16_t *actuator_error, uint8_t motors_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, index);
    _mav_put_uint64_t(buf, 8, timestamp);
    _mav_put_uint8_t(buf, 61, motors_state);
    _mav_put_uint16_t_array(buf, 16, actuator_error, 5);
    _mav_put_int8_t_array(buf, 26, switchedon, 5);
    _mav_put_int8_t_array(buf, 31, torqueon, 5);
    _mav_put_int8_t_array(buf, 36, operative, 5);
    _mav_put_int8_t_array(buf, 41, quickstopped, 5);
    _mav_put_int8_t_array(buf, 46, faulted, 5);
    _mav_put_uint8_t_array(buf, 51, target_state, 5);
    _mav_put_int8_t_array(buf, 56, actuator_idle, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
#else
    mavlink_actuator_status_t packet;
    packet.index = index;
    packet.timestamp = timestamp;
    packet.motors_state = motors_state;
    mav_array_memcpy(packet.actuator_error, actuator_error, sizeof(uint16_t)*5);
    mav_array_memcpy(packet.switchedon, switchedon, sizeof(int8_t)*5);
    mav_array_memcpy(packet.torqueon, torqueon, sizeof(int8_t)*5);
    mav_array_memcpy(packet.operative, operative, sizeof(int8_t)*5);
    mav_array_memcpy(packet.quickstopped, quickstopped, sizeof(int8_t)*5);
    mav_array_memcpy(packet.faulted, faulted, sizeof(int8_t)*5);
    mav_array_memcpy(packet.target_state, target_state, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.actuator_idle, actuator_idle, sizeof(int8_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTUATOR_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
}

/**
 * @brief Pack a actuator_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param index   Cycle number. 
 * @param timestamp [us]  Timestamp. 
 * @param switchedon   Whether the actuator is enabled. Bit 0x0002 of status word. 
 * @param torqueon   Whether the torque is enabled. Bit 0x1000 of status word. 
 * @param operative   Whether the actuator is operative. Bit 0x0004 of status word. 
 * @param quickstopped   Whether the actuator is quickstopped. Bit 0x0020 of status word. 
 * @param faulted   Whether the actuator is faulted. Bit 0x0008 of status word. 
 * @param target_state   The uavcan driver state machine state. 
 * @param actuator_idle   Number of consecutive cycles not communicating. 
 * @param actuator_error   Current actuator error value. 
 * @param motors_state   Whether actuators are enabled or disabled. 0: Stopped 1: Running. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t index,uint64_t timestamp,const int8_t *switchedon,const int8_t *torqueon,const int8_t *operative,const int8_t *quickstopped,const int8_t *faulted,const uint8_t *target_state,const int8_t *actuator_idle,const uint16_t *actuator_error,uint8_t motors_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, index);
    _mav_put_uint64_t(buf, 8, timestamp);
    _mav_put_uint8_t(buf, 61, motors_state);
    _mav_put_uint16_t_array(buf, 16, actuator_error, 5);
    _mav_put_int8_t_array(buf, 26, switchedon, 5);
    _mav_put_int8_t_array(buf, 31, torqueon, 5);
    _mav_put_int8_t_array(buf, 36, operative, 5);
    _mav_put_int8_t_array(buf, 41, quickstopped, 5);
    _mav_put_int8_t_array(buf, 46, faulted, 5);
    _mav_put_uint8_t_array(buf, 51, target_state, 5);
    _mav_put_int8_t_array(buf, 56, actuator_idle, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
#else
    mavlink_actuator_status_t packet;
    packet.index = index;
    packet.timestamp = timestamp;
    packet.motors_state = motors_state;
    mav_array_memcpy(packet.actuator_error, actuator_error, sizeof(uint16_t)*5);
    mav_array_memcpy(packet.switchedon, switchedon, sizeof(int8_t)*5);
    mav_array_memcpy(packet.torqueon, torqueon, sizeof(int8_t)*5);
    mav_array_memcpy(packet.operative, operative, sizeof(int8_t)*5);
    mav_array_memcpy(packet.quickstopped, quickstopped, sizeof(int8_t)*5);
    mav_array_memcpy(packet.faulted, faulted, sizeof(int8_t)*5);
    mav_array_memcpy(packet.target_state, target_state, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.actuator_idle, actuator_idle, sizeof(int8_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ACTUATOR_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
}

/**
 * @brief Encode a actuator_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param actuator_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_actuator_status_t* actuator_status)
{
    return mavlink_msg_actuator_status_pack(system_id, component_id, msg, actuator_status->index, actuator_status->timestamp, actuator_status->switchedon, actuator_status->torqueon, actuator_status->operative, actuator_status->quickstopped, actuator_status->faulted, actuator_status->target_state, actuator_status->actuator_idle, actuator_status->actuator_error, actuator_status->motors_state);
}

/**
 * @brief Encode a actuator_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param actuator_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_actuator_status_t* actuator_status)
{
    return mavlink_msg_actuator_status_pack_chan(system_id, component_id, chan, msg, actuator_status->index, actuator_status->timestamp, actuator_status->switchedon, actuator_status->torqueon, actuator_status->operative, actuator_status->quickstopped, actuator_status->faulted, actuator_status->target_state, actuator_status->actuator_idle, actuator_status->actuator_error, actuator_status->motors_state);
}

/**
 * @brief Send a actuator_status message
 * @param chan MAVLink channel to send the message
 *
 * @param index   Cycle number. 
 * @param timestamp [us]  Timestamp. 
 * @param switchedon   Whether the actuator is enabled. Bit 0x0002 of status word. 
 * @param torqueon   Whether the torque is enabled. Bit 0x1000 of status word. 
 * @param operative   Whether the actuator is operative. Bit 0x0004 of status word. 
 * @param quickstopped   Whether the actuator is quickstopped. Bit 0x0020 of status word. 
 * @param faulted   Whether the actuator is faulted. Bit 0x0008 of status word. 
 * @param target_state   The uavcan driver state machine state. 
 * @param actuator_idle   Number of consecutive cycles not communicating. 
 * @param actuator_error   Current actuator error value. 
 * @param motors_state   Whether actuators are enabled or disabled. 0: Stopped 1: Running. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_actuator_status_send(mavlink_channel_t chan, uint64_t index, uint64_t timestamp, const int8_t *switchedon, const int8_t *torqueon, const int8_t *operative, const int8_t *quickstopped, const int8_t *faulted, const uint8_t *target_state, const int8_t *actuator_idle, const uint16_t *actuator_error, uint8_t motors_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, index);
    _mav_put_uint64_t(buf, 8, timestamp);
    _mav_put_uint8_t(buf, 61, motors_state);
    _mav_put_uint16_t_array(buf, 16, actuator_error, 5);
    _mav_put_int8_t_array(buf, 26, switchedon, 5);
    _mav_put_int8_t_array(buf, 31, torqueon, 5);
    _mav_put_int8_t_array(buf, 36, operative, 5);
    _mav_put_int8_t_array(buf, 41, quickstopped, 5);
    _mav_put_int8_t_array(buf, 46, faulted, 5);
    _mav_put_uint8_t_array(buf, 51, target_state, 5);
    _mav_put_int8_t_array(buf, 56, actuator_idle, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_STATUS, buf, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
#else
    mavlink_actuator_status_t packet;
    packet.index = index;
    packet.timestamp = timestamp;
    packet.motors_state = motors_state;
    mav_array_memcpy(packet.actuator_error, actuator_error, sizeof(uint16_t)*5);
    mav_array_memcpy(packet.switchedon, switchedon, sizeof(int8_t)*5);
    mav_array_memcpy(packet.torqueon, torqueon, sizeof(int8_t)*5);
    mav_array_memcpy(packet.operative, operative, sizeof(int8_t)*5);
    mav_array_memcpy(packet.quickstopped, quickstopped, sizeof(int8_t)*5);
    mav_array_memcpy(packet.faulted, faulted, sizeof(int8_t)*5);
    mav_array_memcpy(packet.target_state, target_state, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.actuator_idle, actuator_idle, sizeof(int8_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_STATUS, (const char *)&packet, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
#endif
}

/**
 * @brief Send a actuator_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_actuator_status_send_struct(mavlink_channel_t chan, const mavlink_actuator_status_t* actuator_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_actuator_status_send(chan, actuator_status->index, actuator_status->timestamp, actuator_status->switchedon, actuator_status->torqueon, actuator_status->operative, actuator_status->quickstopped, actuator_status->faulted, actuator_status->target_state, actuator_status->actuator_idle, actuator_status->actuator_error, actuator_status->motors_state);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_STATUS, (const char *)actuator_status, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_actuator_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t index, uint64_t timestamp, const int8_t *switchedon, const int8_t *torqueon, const int8_t *operative, const int8_t *quickstopped, const int8_t *faulted, const uint8_t *target_state, const int8_t *actuator_idle, const uint16_t *actuator_error, uint8_t motors_state)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, index);
    _mav_put_uint64_t(buf, 8, timestamp);
    _mav_put_uint8_t(buf, 61, motors_state);
    _mav_put_uint16_t_array(buf, 16, actuator_error, 5);
    _mav_put_int8_t_array(buf, 26, switchedon, 5);
    _mav_put_int8_t_array(buf, 31, torqueon, 5);
    _mav_put_int8_t_array(buf, 36, operative, 5);
    _mav_put_int8_t_array(buf, 41, quickstopped, 5);
    _mav_put_int8_t_array(buf, 46, faulted, 5);
    _mav_put_uint8_t_array(buf, 51, target_state, 5);
    _mav_put_int8_t_array(buf, 56, actuator_idle, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_STATUS, buf, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
#else
    mavlink_actuator_status_t *packet = (mavlink_actuator_status_t *)msgbuf;
    packet->index = index;
    packet->timestamp = timestamp;
    packet->motors_state = motors_state;
    mav_array_memcpy(packet->actuator_error, actuator_error, sizeof(uint16_t)*5);
    mav_array_memcpy(packet->switchedon, switchedon, sizeof(int8_t)*5);
    mav_array_memcpy(packet->torqueon, torqueon, sizeof(int8_t)*5);
    mav_array_memcpy(packet->operative, operative, sizeof(int8_t)*5);
    mav_array_memcpy(packet->quickstopped, quickstopped, sizeof(int8_t)*5);
    mav_array_memcpy(packet->faulted, faulted, sizeof(int8_t)*5);
    mav_array_memcpy(packet->target_state, target_state, sizeof(uint8_t)*5);
    mav_array_memcpy(packet->actuator_idle, actuator_idle, sizeof(int8_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_STATUS, (const char *)packet, MAVLINK_MSG_ID_ACTUATOR_STATUS_MIN_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN, MAVLINK_MSG_ID_ACTUATOR_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE ACTUATOR_STATUS UNPACKING


/**
 * @brief Get field index from actuator_status message
 *
 * @return   Cycle number. 
 */
static inline uint64_t mavlink_msg_actuator_status_get_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field timestamp from actuator_status message
 *
 * @return [us]  Timestamp. 
 */
static inline uint64_t mavlink_msg_actuator_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field switchedon from actuator_status message
 *
 * @return   Whether the actuator is enabled. Bit 0x0002 of status word. 
 */
static inline uint16_t mavlink_msg_actuator_status_get_switchedon(const mavlink_message_t* msg, int8_t *switchedon)
{
    return _MAV_RETURN_int8_t_array(msg, switchedon, 5,  26);
}

/**
 * @brief Get field torqueon from actuator_status message
 *
 * @return   Whether the torque is enabled. Bit 0x1000 of status word. 
 */
static inline uint16_t mavlink_msg_actuator_status_get_torqueon(const mavlink_message_t* msg, int8_t *torqueon)
{
    return _MAV_RETURN_int8_t_array(msg, torqueon, 5,  31);
}

/**
 * @brief Get field operative from actuator_status message
 *
 * @return   Whether the actuator is operative. Bit 0x0004 of status word. 
 */
static inline uint16_t mavlink_msg_actuator_status_get_operative(const mavlink_message_t* msg, int8_t *operative)
{
    return _MAV_RETURN_int8_t_array(msg, operative, 5,  36);
}

/**
 * @brief Get field quickstopped from actuator_status message
 *
 * @return   Whether the actuator is quickstopped. Bit 0x0020 of status word. 
 */
static inline uint16_t mavlink_msg_actuator_status_get_quickstopped(const mavlink_message_t* msg, int8_t *quickstopped)
{
    return _MAV_RETURN_int8_t_array(msg, quickstopped, 5,  41);
}

/**
 * @brief Get field faulted from actuator_status message
 *
 * @return   Whether the actuator is faulted. Bit 0x0008 of status word. 
 */
static inline uint16_t mavlink_msg_actuator_status_get_faulted(const mavlink_message_t* msg, int8_t *faulted)
{
    return _MAV_RETURN_int8_t_array(msg, faulted, 5,  46);
}

/**
 * @brief Get field target_state from actuator_status message
 *
 * @return   The uavcan driver state machine state. 
 */
static inline uint16_t mavlink_msg_actuator_status_get_target_state(const mavlink_message_t* msg, uint8_t *target_state)
{
    return _MAV_RETURN_uint8_t_array(msg, target_state, 5,  51);
}

/**
 * @brief Get field actuator_idle from actuator_status message
 *
 * @return   Number of consecutive cycles not communicating. 
 */
static inline uint16_t mavlink_msg_actuator_status_get_actuator_idle(const mavlink_message_t* msg, int8_t *actuator_idle)
{
    return _MAV_RETURN_int8_t_array(msg, actuator_idle, 5,  56);
}

/**
 * @brief Get field actuator_error from actuator_status message
 *
 * @return   Current actuator error value. 
 */
static inline uint16_t mavlink_msg_actuator_status_get_actuator_error(const mavlink_message_t* msg, uint16_t *actuator_error)
{
    return _MAV_RETURN_uint16_t_array(msg, actuator_error, 5,  16);
}

/**
 * @brief Get field motors_state from actuator_status message
 *
 * @return   Whether actuators are enabled or disabled. 0: Stopped 1: Running. 
 */
static inline uint8_t mavlink_msg_actuator_status_get_motors_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  61);
}

/**
 * @brief Decode a actuator_status message into a struct
 *
 * @param msg The message to decode
 * @param actuator_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_actuator_status_decode(const mavlink_message_t* msg, mavlink_actuator_status_t* actuator_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    actuator_status->index = mavlink_msg_actuator_status_get_index(msg);
    actuator_status->timestamp = mavlink_msg_actuator_status_get_timestamp(msg);
    mavlink_msg_actuator_status_get_actuator_error(msg, actuator_status->actuator_error);
    mavlink_msg_actuator_status_get_switchedon(msg, actuator_status->switchedon);
    mavlink_msg_actuator_status_get_torqueon(msg, actuator_status->torqueon);
    mavlink_msg_actuator_status_get_operative(msg, actuator_status->operative);
    mavlink_msg_actuator_status_get_quickstopped(msg, actuator_status->quickstopped);
    mavlink_msg_actuator_status_get_faulted(msg, actuator_status->faulted);
    mavlink_msg_actuator_status_get_target_state(msg, actuator_status->target_state);
    mavlink_msg_actuator_status_get_actuator_idle(msg, actuator_status->actuator_idle);
    actuator_status->motors_state = mavlink_msg_actuator_status_get_motors_state(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN? msg->len : MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN;
        memset(actuator_status, 0, MAVLINK_MSG_ID_ACTUATOR_STATUS_LEN);
    memcpy(actuator_status, _MAV_PAYLOAD(msg), len);
#endif
}
