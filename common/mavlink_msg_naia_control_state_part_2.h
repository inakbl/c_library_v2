#pragma once
// MESSAGE NAIA_CONTROL_STATE_PART_2 PACKING

#define MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2 613


typedef struct __mavlink_naia_control_state_part_2_t {
 uint64_t index; /*<   Message index. */
 float position[6]; /*<   Copy of the input position. */
 float velocity[6]; /*<   Copy of the input velocity. */
 float acceleration[6]; /*<   Copy of the input acceleration. */
 float angular_position[6]; /*<   Copy of the input angular_position. */
 float angular_velocity[6]; /*<   Copy of the input angular_velocity. */
 float angular_acceleration[6]; /*<   Copy of the input angular_acceleration. */
} mavlink_naia_control_state_part_2_t;

#define MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN 152
#define MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_MIN_LEN 152
#define MAVLINK_MSG_ID_613_LEN 152
#define MAVLINK_MSG_ID_613_MIN_LEN 152

#define MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_CRC 40
#define MAVLINK_MSG_ID_613_CRC 40

#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_2_FIELD_POSITION_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_2_FIELD_VELOCITY_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_2_FIELD_ACCELERATION_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_2_FIELD_ANGULAR_POSITION_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_2_FIELD_ANGULAR_VELOCITY_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_2_FIELD_ANGULAR_ACCELERATION_LEN 6

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAIA_CONTROL_STATE_PART_2 { \
    613, \
    "NAIA_CONTROL_STATE_PART_2", \
    7, \
    {  { "index", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_naia_control_state_part_2_t, index) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 6, 8, offsetof(mavlink_naia_control_state_part_2_t, position) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 6, 32, offsetof(mavlink_naia_control_state_part_2_t, velocity) }, \
         { "acceleration", NULL, MAVLINK_TYPE_FLOAT, 6, 56, offsetof(mavlink_naia_control_state_part_2_t, acceleration) }, \
         { "angular_position", NULL, MAVLINK_TYPE_FLOAT, 6, 80, offsetof(mavlink_naia_control_state_part_2_t, angular_position) }, \
         { "angular_velocity", NULL, MAVLINK_TYPE_FLOAT, 6, 104, offsetof(mavlink_naia_control_state_part_2_t, angular_velocity) }, \
         { "angular_acceleration", NULL, MAVLINK_TYPE_FLOAT, 6, 128, offsetof(mavlink_naia_control_state_part_2_t, angular_acceleration) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAIA_CONTROL_STATE_PART_2 { \
    "NAIA_CONTROL_STATE_PART_2", \
    7, \
    {  { "index", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_naia_control_state_part_2_t, index) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 6, 8, offsetof(mavlink_naia_control_state_part_2_t, position) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 6, 32, offsetof(mavlink_naia_control_state_part_2_t, velocity) }, \
         { "acceleration", NULL, MAVLINK_TYPE_FLOAT, 6, 56, offsetof(mavlink_naia_control_state_part_2_t, acceleration) }, \
         { "angular_position", NULL, MAVLINK_TYPE_FLOAT, 6, 80, offsetof(mavlink_naia_control_state_part_2_t, angular_position) }, \
         { "angular_velocity", NULL, MAVLINK_TYPE_FLOAT, 6, 104, offsetof(mavlink_naia_control_state_part_2_t, angular_velocity) }, \
         { "angular_acceleration", NULL, MAVLINK_TYPE_FLOAT, 6, 128, offsetof(mavlink_naia_control_state_part_2_t, angular_acceleration) }, \
         } \
}
#endif

/**
 * @brief Pack a naia_control_state_part_2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param index   Message index. 
 * @param position   Copy of the input position. 
 * @param velocity   Copy of the input velocity. 
 * @param acceleration   Copy of the input acceleration. 
 * @param angular_position   Copy of the input angular_position. 
 * @param angular_velocity   Copy of the input angular_velocity. 
 * @param angular_acceleration   Copy of the input angular_acceleration. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_control_state_part_2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t index, const float *position, const float *velocity, const float *acceleration, const float *angular_position, const float *angular_velocity, const float *angular_acceleration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN];
    _mav_put_uint64_t(buf, 0, index);
    _mav_put_float_array(buf, 8, position, 6);
    _mav_put_float_array(buf, 32, velocity, 6);
    _mav_put_float_array(buf, 56, acceleration, 6);
    _mav_put_float_array(buf, 80, angular_position, 6);
    _mav_put_float_array(buf, 104, angular_velocity, 6);
    _mav_put_float_array(buf, 128, angular_acceleration, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN);
#else
    mavlink_naia_control_state_part_2_t packet;
    packet.index = index;
    mav_array_memcpy(packet.position, position, sizeof(float)*6);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*6);
    mav_array_memcpy(packet.acceleration, acceleration, sizeof(float)*6);
    mav_array_memcpy(packet.angular_position, angular_position, sizeof(float)*6);
    mav_array_memcpy(packet.angular_velocity, angular_velocity, sizeof(float)*6);
    mav_array_memcpy(packet.angular_acceleration, angular_acceleration, sizeof(float)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_CRC);
}

/**
 * @brief Pack a naia_control_state_part_2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param index   Message index. 
 * @param position   Copy of the input position. 
 * @param velocity   Copy of the input velocity. 
 * @param acceleration   Copy of the input acceleration. 
 * @param angular_position   Copy of the input angular_position. 
 * @param angular_velocity   Copy of the input angular_velocity. 
 * @param angular_acceleration   Copy of the input angular_acceleration. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_control_state_part_2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t index,const float *position,const float *velocity,const float *acceleration,const float *angular_position,const float *angular_velocity,const float *angular_acceleration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN];
    _mav_put_uint64_t(buf, 0, index);
    _mav_put_float_array(buf, 8, position, 6);
    _mav_put_float_array(buf, 32, velocity, 6);
    _mav_put_float_array(buf, 56, acceleration, 6);
    _mav_put_float_array(buf, 80, angular_position, 6);
    _mav_put_float_array(buf, 104, angular_velocity, 6);
    _mav_put_float_array(buf, 128, angular_acceleration, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN);
#else
    mavlink_naia_control_state_part_2_t packet;
    packet.index = index;
    mav_array_memcpy(packet.position, position, sizeof(float)*6);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*6);
    mav_array_memcpy(packet.acceleration, acceleration, sizeof(float)*6);
    mav_array_memcpy(packet.angular_position, angular_position, sizeof(float)*6);
    mav_array_memcpy(packet.angular_velocity, angular_velocity, sizeof(float)*6);
    mav_array_memcpy(packet.angular_acceleration, angular_acceleration, sizeof(float)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_CRC);
}

/**
 * @brief Encode a naia_control_state_part_2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param naia_control_state_part_2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_control_state_part_2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_naia_control_state_part_2_t* naia_control_state_part_2)
{
    return mavlink_msg_naia_control_state_part_2_pack(system_id, component_id, msg, naia_control_state_part_2->index, naia_control_state_part_2->position, naia_control_state_part_2->velocity, naia_control_state_part_2->acceleration, naia_control_state_part_2->angular_position, naia_control_state_part_2->angular_velocity, naia_control_state_part_2->angular_acceleration);
}

/**
 * @brief Encode a naia_control_state_part_2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param naia_control_state_part_2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_control_state_part_2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_naia_control_state_part_2_t* naia_control_state_part_2)
{
    return mavlink_msg_naia_control_state_part_2_pack_chan(system_id, component_id, chan, msg, naia_control_state_part_2->index, naia_control_state_part_2->position, naia_control_state_part_2->velocity, naia_control_state_part_2->acceleration, naia_control_state_part_2->angular_position, naia_control_state_part_2->angular_velocity, naia_control_state_part_2->angular_acceleration);
}

/**
 * @brief Send a naia_control_state_part_2 message
 * @param chan MAVLink channel to send the message
 *
 * @param index   Message index. 
 * @param position   Copy of the input position. 
 * @param velocity   Copy of the input velocity. 
 * @param acceleration   Copy of the input acceleration. 
 * @param angular_position   Copy of the input angular_position. 
 * @param angular_velocity   Copy of the input angular_velocity. 
 * @param angular_acceleration   Copy of the input angular_acceleration. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_naia_control_state_part_2_send(mavlink_channel_t chan, uint64_t index, const float *position, const float *velocity, const float *acceleration, const float *angular_position, const float *angular_velocity, const float *angular_acceleration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN];
    _mav_put_uint64_t(buf, 0, index);
    _mav_put_float_array(buf, 8, position, 6);
    _mav_put_float_array(buf, 32, velocity, 6);
    _mav_put_float_array(buf, 56, acceleration, 6);
    _mav_put_float_array(buf, 80, angular_position, 6);
    _mav_put_float_array(buf, 104, angular_velocity, 6);
    _mav_put_float_array(buf, 128, angular_acceleration, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2, buf, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_CRC);
#else
    mavlink_naia_control_state_part_2_t packet;
    packet.index = index;
    mav_array_memcpy(packet.position, position, sizeof(float)*6);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*6);
    mav_array_memcpy(packet.acceleration, acceleration, sizeof(float)*6);
    mav_array_memcpy(packet.angular_position, angular_position, sizeof(float)*6);
    mav_array_memcpy(packet.angular_velocity, angular_velocity, sizeof(float)*6);
    mav_array_memcpy(packet.angular_acceleration, angular_acceleration, sizeof(float)*6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2, (const char *)&packet, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_CRC);
#endif
}

/**
 * @brief Send a naia_control_state_part_2 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_naia_control_state_part_2_send_struct(mavlink_channel_t chan, const mavlink_naia_control_state_part_2_t* naia_control_state_part_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_naia_control_state_part_2_send(chan, naia_control_state_part_2->index, naia_control_state_part_2->position, naia_control_state_part_2->velocity, naia_control_state_part_2->acceleration, naia_control_state_part_2->angular_position, naia_control_state_part_2->angular_velocity, naia_control_state_part_2->angular_acceleration);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2, (const char *)naia_control_state_part_2, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_naia_control_state_part_2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t index, const float *position, const float *velocity, const float *acceleration, const float *angular_position, const float *angular_velocity, const float *angular_acceleration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, index);
    _mav_put_float_array(buf, 8, position, 6);
    _mav_put_float_array(buf, 32, velocity, 6);
    _mav_put_float_array(buf, 56, acceleration, 6);
    _mav_put_float_array(buf, 80, angular_position, 6);
    _mav_put_float_array(buf, 104, angular_velocity, 6);
    _mav_put_float_array(buf, 128, angular_acceleration, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2, buf, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_CRC);
#else
    mavlink_naia_control_state_part_2_t *packet = (mavlink_naia_control_state_part_2_t *)msgbuf;
    packet->index = index;
    mav_array_memcpy(packet->position, position, sizeof(float)*6);
    mav_array_memcpy(packet->velocity, velocity, sizeof(float)*6);
    mav_array_memcpy(packet->acceleration, acceleration, sizeof(float)*6);
    mav_array_memcpy(packet->angular_position, angular_position, sizeof(float)*6);
    mav_array_memcpy(packet->angular_velocity, angular_velocity, sizeof(float)*6);
    mav_array_memcpy(packet->angular_acceleration, angular_acceleration, sizeof(float)*6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2, (const char *)packet, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_CRC);
#endif
}
#endif

#endif

// MESSAGE NAIA_CONTROL_STATE_PART_2 UNPACKING


/**
 * @brief Get field index from naia_control_state_part_2 message
 *
 * @return   Message index. 
 */
static inline uint64_t mavlink_msg_naia_control_state_part_2_get_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field position from naia_control_state_part_2 message
 *
 * @return   Copy of the input position. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_2_get_position(const mavlink_message_t* msg, float *position)
{
    return _MAV_RETURN_float_array(msg, position, 6,  8);
}

/**
 * @brief Get field velocity from naia_control_state_part_2 message
 *
 * @return   Copy of the input velocity. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_2_get_velocity(const mavlink_message_t* msg, float *velocity)
{
    return _MAV_RETURN_float_array(msg, velocity, 6,  32);
}

/**
 * @brief Get field acceleration from naia_control_state_part_2 message
 *
 * @return   Copy of the input acceleration. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_2_get_acceleration(const mavlink_message_t* msg, float *acceleration)
{
    return _MAV_RETURN_float_array(msg, acceleration, 6,  56);
}

/**
 * @brief Get field angular_position from naia_control_state_part_2 message
 *
 * @return   Copy of the input angular_position. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_2_get_angular_position(const mavlink_message_t* msg, float *angular_position)
{
    return _MAV_RETURN_float_array(msg, angular_position, 6,  80);
}

/**
 * @brief Get field angular_velocity from naia_control_state_part_2 message
 *
 * @return   Copy of the input angular_velocity. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_2_get_angular_velocity(const mavlink_message_t* msg, float *angular_velocity)
{
    return _MAV_RETURN_float_array(msg, angular_velocity, 6,  104);
}

/**
 * @brief Get field angular_acceleration from naia_control_state_part_2 message
 *
 * @return   Copy of the input angular_acceleration. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_2_get_angular_acceleration(const mavlink_message_t* msg, float *angular_acceleration)
{
    return _MAV_RETURN_float_array(msg, angular_acceleration, 6,  128);
}

/**
 * @brief Decode a naia_control_state_part_2 message into a struct
 *
 * @param msg The message to decode
 * @param naia_control_state_part_2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_naia_control_state_part_2_decode(const mavlink_message_t* msg, mavlink_naia_control_state_part_2_t* naia_control_state_part_2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    naia_control_state_part_2->index = mavlink_msg_naia_control_state_part_2_get_index(msg);
    mavlink_msg_naia_control_state_part_2_get_position(msg, naia_control_state_part_2->position);
    mavlink_msg_naia_control_state_part_2_get_velocity(msg, naia_control_state_part_2->velocity);
    mavlink_msg_naia_control_state_part_2_get_acceleration(msg, naia_control_state_part_2->acceleration);
    mavlink_msg_naia_control_state_part_2_get_angular_position(msg, naia_control_state_part_2->angular_position);
    mavlink_msg_naia_control_state_part_2_get_angular_velocity(msg, naia_control_state_part_2->angular_velocity);
    mavlink_msg_naia_control_state_part_2_get_angular_acceleration(msg, naia_control_state_part_2->angular_acceleration);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN? msg->len : MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN;
        memset(naia_control_state_part_2, 0, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_2_LEN);
    memcpy(naia_control_state_part_2, _MAV_PAYLOAD(msg), len);
#endif
}
