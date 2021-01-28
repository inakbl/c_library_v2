#pragma once
// MESSAGE SBG_SHIP_MOTION PACKING

#define MAVLINK_MSG_ID_SBG_SHIP_MOTION 627


typedef struct __mavlink_sbg_ship_motion_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 uint32_t time_stamp; /*< [us] Time since sensor is powered up us.*/
 float ship_motion[3]; /*< [m] [Surge, Sway, Heave (positive down)] at main location (in m). Surge and Sway are not fulfilled.*/
 float acceleration[3]; /*< [m/s2] [Longitudinal, Lateral, Vertical (positive down)] acceleration (in m/s2).*/
 float velocity[3]; /*< [m/s] [Longitudinal, Lateral, Vertical (positive down)] velocity (in m/s).*/
 uint16_t heave_period; /*< [s] Main heave period in seconds. s float 4 4.*/
 uint8_t heave_valid; /*<  True after heave convergence time. False in following conditions: - Turn occurred and no velocity aiding is available - Heave reached higher/lower limits - If a step is detected and filter has to re-converge - If internal failure.*/
 uint8_t heave_vel_aided; /*<  True if heave output is compensated for transient accelerations*/
 uint8_t period_available; /*<  True if the swell period is provided in this output*/
 uint8_t period_valid; /*<  True if the period returned is assumed to be valid or not.*/
} mavlink_sbg_ship_motion_t;

#define MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN 54
#define MAVLINK_MSG_ID_SBG_SHIP_MOTION_MIN_LEN 54
#define MAVLINK_MSG_ID_627_LEN 54
#define MAVLINK_MSG_ID_627_MIN_LEN 54

#define MAVLINK_MSG_ID_SBG_SHIP_MOTION_CRC 123
#define MAVLINK_MSG_ID_627_CRC 123

#define MAVLINK_MSG_SBG_SHIP_MOTION_FIELD_SHIP_MOTION_LEN 3
#define MAVLINK_MSG_SBG_SHIP_MOTION_FIELD_ACCELERATION_LEN 3
#define MAVLINK_MSG_SBG_SHIP_MOTION_FIELD_VELOCITY_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SBG_SHIP_MOTION { \
    627, \
    "SBG_SHIP_MOTION", \
    10, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_ship_motion_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_ship_motion_t, time_stamp) }, \
         { "heave_period", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_sbg_ship_motion_t, heave_period) }, \
         { "ship_motion", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_sbg_ship_motion_t, ship_motion) }, \
         { "acceleration", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_sbg_ship_motion_t, acceleration) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_sbg_ship_motion_t, velocity) }, \
         { "heave_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_sbg_ship_motion_t, heave_valid) }, \
         { "heave_vel_aided", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_sbg_ship_motion_t, heave_vel_aided) }, \
         { "period_available", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_sbg_ship_motion_t, period_available) }, \
         { "period_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_sbg_ship_motion_t, period_valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SBG_SHIP_MOTION { \
    "SBG_SHIP_MOTION", \
    10, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_ship_motion_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_ship_motion_t, time_stamp) }, \
         { "heave_period", NULL, MAVLINK_TYPE_UINT16_T, 0, 48, offsetof(mavlink_sbg_ship_motion_t, heave_period) }, \
         { "ship_motion", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_sbg_ship_motion_t, ship_motion) }, \
         { "acceleration", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_sbg_ship_motion_t, acceleration) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_sbg_ship_motion_t, velocity) }, \
         { "heave_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_sbg_ship_motion_t, heave_valid) }, \
         { "heave_vel_aided", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_sbg_ship_motion_t, heave_vel_aided) }, \
         { "period_available", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_sbg_ship_motion_t, period_available) }, \
         { "period_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_sbg_ship_motion_t, period_valid) }, \
         } \
}
#endif

/**
 * @brief Pack a sbg_ship_motion message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param heave_period [s] Main heave period in seconds. s float 4 4.
 * @param ship_motion [m] [Surge, Sway, Heave (positive down)] at main location (in m). Surge and Sway are not fulfilled.
 * @param acceleration [m/s2] [Longitudinal, Lateral, Vertical (positive down)] acceleration (in m/s2).
 * @param velocity [m/s] [Longitudinal, Lateral, Vertical (positive down)] velocity (in m/s).
 * @param heave_valid  True after heave convergence time. False in following conditions: - Turn occurred and no velocity aiding is available - Heave reached higher/lower limits - If a step is detected and filter has to re-converge - If internal failure.
 * @param heave_vel_aided  True if heave output is compensated for transient accelerations
 * @param period_available  True if the swell period is provided in this output
 * @param period_valid  True if the period returned is assumed to be valid or not.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_ship_motion_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t time_stamp, uint16_t heave_period, const float *ship_motion, const float *acceleration, const float *velocity, uint8_t heave_valid, uint8_t heave_vel_aided, uint8_t period_available, uint8_t period_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint16_t(buf, 48, heave_period);
    _mav_put_uint8_t(buf, 50, heave_valid);
    _mav_put_uint8_t(buf, 51, heave_vel_aided);
    _mav_put_uint8_t(buf, 52, period_available);
    _mav_put_uint8_t(buf, 53, period_valid);
    _mav_put_float_array(buf, 12, ship_motion, 3);
    _mav_put_float_array(buf, 24, acceleration, 3);
    _mav_put_float_array(buf, 36, velocity, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN);
#else
    mavlink_sbg_ship_motion_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.heave_period = heave_period;
    packet.heave_valid = heave_valid;
    packet.heave_vel_aided = heave_vel_aided;
    packet.period_available = period_available;
    packet.period_valid = period_valid;
    mav_array_memcpy(packet.ship_motion, ship_motion, sizeof(float)*3);
    mav_array_memcpy(packet.acceleration, acceleration, sizeof(float)*3);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_SHIP_MOTION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SBG_SHIP_MOTION_MIN_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_CRC);
}

/**
 * @brief Pack a sbg_ship_motion message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param heave_period [s] Main heave period in seconds. s float 4 4.
 * @param ship_motion [m] [Surge, Sway, Heave (positive down)] at main location (in m). Surge and Sway are not fulfilled.
 * @param acceleration [m/s2] [Longitudinal, Lateral, Vertical (positive down)] acceleration (in m/s2).
 * @param velocity [m/s] [Longitudinal, Lateral, Vertical (positive down)] velocity (in m/s).
 * @param heave_valid  True after heave convergence time. False in following conditions: - Turn occurred and no velocity aiding is available - Heave reached higher/lower limits - If a step is detected and filter has to re-converge - If internal failure.
 * @param heave_vel_aided  True if heave output is compensated for transient accelerations
 * @param period_available  True if the swell period is provided in this output
 * @param period_valid  True if the period returned is assumed to be valid or not.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_ship_motion_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t time_stamp,uint16_t heave_period,const float *ship_motion,const float *acceleration,const float *velocity,uint8_t heave_valid,uint8_t heave_vel_aided,uint8_t period_available,uint8_t period_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint16_t(buf, 48, heave_period);
    _mav_put_uint8_t(buf, 50, heave_valid);
    _mav_put_uint8_t(buf, 51, heave_vel_aided);
    _mav_put_uint8_t(buf, 52, period_available);
    _mav_put_uint8_t(buf, 53, period_valid);
    _mav_put_float_array(buf, 12, ship_motion, 3);
    _mav_put_float_array(buf, 24, acceleration, 3);
    _mav_put_float_array(buf, 36, velocity, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN);
#else
    mavlink_sbg_ship_motion_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.heave_period = heave_period;
    packet.heave_valid = heave_valid;
    packet.heave_vel_aided = heave_vel_aided;
    packet.period_available = period_available;
    packet.period_valid = period_valid;
    mav_array_memcpy(packet.ship_motion, ship_motion, sizeof(float)*3);
    mav_array_memcpy(packet.acceleration, acceleration, sizeof(float)*3);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_SHIP_MOTION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SBG_SHIP_MOTION_MIN_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_CRC);
}

/**
 * @brief Encode a sbg_ship_motion struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbg_ship_motion C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_ship_motion_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbg_ship_motion_t* sbg_ship_motion)
{
    return mavlink_msg_sbg_ship_motion_pack(system_id, component_id, msg, sbg_ship_motion->timestamp, sbg_ship_motion->time_stamp, sbg_ship_motion->heave_period, sbg_ship_motion->ship_motion, sbg_ship_motion->acceleration, sbg_ship_motion->velocity, sbg_ship_motion->heave_valid, sbg_ship_motion->heave_vel_aided, sbg_ship_motion->period_available, sbg_ship_motion->period_valid);
}

/**
 * @brief Encode a sbg_ship_motion struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sbg_ship_motion C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_ship_motion_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sbg_ship_motion_t* sbg_ship_motion)
{
    return mavlink_msg_sbg_ship_motion_pack_chan(system_id, component_id, chan, msg, sbg_ship_motion->timestamp, sbg_ship_motion->time_stamp, sbg_ship_motion->heave_period, sbg_ship_motion->ship_motion, sbg_ship_motion->acceleration, sbg_ship_motion->velocity, sbg_ship_motion->heave_valid, sbg_ship_motion->heave_vel_aided, sbg_ship_motion->period_available, sbg_ship_motion->period_valid);
}

/**
 * @brief Send a sbg_ship_motion message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param heave_period [s] Main heave period in seconds. s float 4 4.
 * @param ship_motion [m] [Surge, Sway, Heave (positive down)] at main location (in m). Surge and Sway are not fulfilled.
 * @param acceleration [m/s2] [Longitudinal, Lateral, Vertical (positive down)] acceleration (in m/s2).
 * @param velocity [m/s] [Longitudinal, Lateral, Vertical (positive down)] velocity (in m/s).
 * @param heave_valid  True after heave convergence time. False in following conditions: - Turn occurred and no velocity aiding is available - Heave reached higher/lower limits - If a step is detected and filter has to re-converge - If internal failure.
 * @param heave_vel_aided  True if heave output is compensated for transient accelerations
 * @param period_available  True if the swell period is provided in this output
 * @param period_valid  True if the period returned is assumed to be valid or not.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sbg_ship_motion_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t time_stamp, uint16_t heave_period, const float *ship_motion, const float *acceleration, const float *velocity, uint8_t heave_valid, uint8_t heave_vel_aided, uint8_t period_available, uint8_t period_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint16_t(buf, 48, heave_period);
    _mav_put_uint8_t(buf, 50, heave_valid);
    _mav_put_uint8_t(buf, 51, heave_vel_aided);
    _mav_put_uint8_t(buf, 52, period_available);
    _mav_put_uint8_t(buf, 53, period_valid);
    _mav_put_float_array(buf, 12, ship_motion, 3);
    _mav_put_float_array(buf, 24, acceleration, 3);
    _mav_put_float_array(buf, 36, velocity, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_SHIP_MOTION, buf, MAVLINK_MSG_ID_SBG_SHIP_MOTION_MIN_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_CRC);
#else
    mavlink_sbg_ship_motion_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.heave_period = heave_period;
    packet.heave_valid = heave_valid;
    packet.heave_vel_aided = heave_vel_aided;
    packet.period_available = period_available;
    packet.period_valid = period_valid;
    mav_array_memcpy(packet.ship_motion, ship_motion, sizeof(float)*3);
    mav_array_memcpy(packet.acceleration, acceleration, sizeof(float)*3);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_SHIP_MOTION, (const char *)&packet, MAVLINK_MSG_ID_SBG_SHIP_MOTION_MIN_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_CRC);
#endif
}

/**
 * @brief Send a sbg_ship_motion message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sbg_ship_motion_send_struct(mavlink_channel_t chan, const mavlink_sbg_ship_motion_t* sbg_ship_motion)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sbg_ship_motion_send(chan, sbg_ship_motion->timestamp, sbg_ship_motion->time_stamp, sbg_ship_motion->heave_period, sbg_ship_motion->ship_motion, sbg_ship_motion->acceleration, sbg_ship_motion->velocity, sbg_ship_motion->heave_valid, sbg_ship_motion->heave_vel_aided, sbg_ship_motion->period_available, sbg_ship_motion->period_valid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_SHIP_MOTION, (const char *)sbg_ship_motion, MAVLINK_MSG_ID_SBG_SHIP_MOTION_MIN_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_CRC);
#endif
}

#if MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sbg_ship_motion_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t time_stamp, uint16_t heave_period, const float *ship_motion, const float *acceleration, const float *velocity, uint8_t heave_valid, uint8_t heave_vel_aided, uint8_t period_available, uint8_t period_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint16_t(buf, 48, heave_period);
    _mav_put_uint8_t(buf, 50, heave_valid);
    _mav_put_uint8_t(buf, 51, heave_vel_aided);
    _mav_put_uint8_t(buf, 52, period_available);
    _mav_put_uint8_t(buf, 53, period_valid);
    _mav_put_float_array(buf, 12, ship_motion, 3);
    _mav_put_float_array(buf, 24, acceleration, 3);
    _mav_put_float_array(buf, 36, velocity, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_SHIP_MOTION, buf, MAVLINK_MSG_ID_SBG_SHIP_MOTION_MIN_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_CRC);
#else
    mavlink_sbg_ship_motion_t *packet = (mavlink_sbg_ship_motion_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->time_stamp = time_stamp;
    packet->heave_period = heave_period;
    packet->heave_valid = heave_valid;
    packet->heave_vel_aided = heave_vel_aided;
    packet->period_available = period_available;
    packet->period_valid = period_valid;
    mav_array_memcpy(packet->ship_motion, ship_motion, sizeof(float)*3);
    mav_array_memcpy(packet->acceleration, acceleration, sizeof(float)*3);
    mav_array_memcpy(packet->velocity, velocity, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_SHIP_MOTION, (const char *)packet, MAVLINK_MSG_ID_SBG_SHIP_MOTION_MIN_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN, MAVLINK_MSG_ID_SBG_SHIP_MOTION_CRC);
#endif
}
#endif

#endif

// MESSAGE SBG_SHIP_MOTION UNPACKING


/**
 * @brief Get field timestamp from sbg_ship_motion message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_sbg_ship_motion_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_stamp from sbg_ship_motion message
 *
 * @return [us] Time since sensor is powered up us.
 */
static inline uint32_t mavlink_msg_sbg_ship_motion_get_time_stamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field heave_period from sbg_ship_motion message
 *
 * @return [s] Main heave period in seconds. s float 4 4.
 */
static inline uint16_t mavlink_msg_sbg_ship_motion_get_heave_period(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  48);
}

/**
 * @brief Get field ship_motion from sbg_ship_motion message
 *
 * @return [m] [Surge, Sway, Heave (positive down)] at main location (in m). Surge and Sway are not fulfilled.
 */
static inline uint16_t mavlink_msg_sbg_ship_motion_get_ship_motion(const mavlink_message_t* msg, float *ship_motion)
{
    return _MAV_RETURN_float_array(msg, ship_motion, 3,  12);
}

/**
 * @brief Get field acceleration from sbg_ship_motion message
 *
 * @return [m/s2] [Longitudinal, Lateral, Vertical (positive down)] acceleration (in m/s2).
 */
static inline uint16_t mavlink_msg_sbg_ship_motion_get_acceleration(const mavlink_message_t* msg, float *acceleration)
{
    return _MAV_RETURN_float_array(msg, acceleration, 3,  24);
}

/**
 * @brief Get field velocity from sbg_ship_motion message
 *
 * @return [m/s] [Longitudinal, Lateral, Vertical (positive down)] velocity (in m/s).
 */
static inline uint16_t mavlink_msg_sbg_ship_motion_get_velocity(const mavlink_message_t* msg, float *velocity)
{
    return _MAV_RETURN_float_array(msg, velocity, 3,  36);
}

/**
 * @brief Get field heave_valid from sbg_ship_motion message
 *
 * @return  True after heave convergence time. False in following conditions: - Turn occurred and no velocity aiding is available - Heave reached higher/lower limits - If a step is detected and filter has to re-converge - If internal failure.
 */
static inline uint8_t mavlink_msg_sbg_ship_motion_get_heave_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field heave_vel_aided from sbg_ship_motion message
 *
 * @return  True if heave output is compensated for transient accelerations
 */
static inline uint8_t mavlink_msg_sbg_ship_motion_get_heave_vel_aided(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Get field period_available from sbg_ship_motion message
 *
 * @return  True if the swell period is provided in this output
 */
static inline uint8_t mavlink_msg_sbg_ship_motion_get_period_available(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field period_valid from sbg_ship_motion message
 *
 * @return  True if the period returned is assumed to be valid or not.
 */
static inline uint8_t mavlink_msg_sbg_ship_motion_get_period_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Decode a sbg_ship_motion message into a struct
 *
 * @param msg The message to decode
 * @param sbg_ship_motion C-struct to decode the message contents into
 */
static inline void mavlink_msg_sbg_ship_motion_decode(const mavlink_message_t* msg, mavlink_sbg_ship_motion_t* sbg_ship_motion)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sbg_ship_motion->timestamp = mavlink_msg_sbg_ship_motion_get_timestamp(msg);
    sbg_ship_motion->time_stamp = mavlink_msg_sbg_ship_motion_get_time_stamp(msg);
    mavlink_msg_sbg_ship_motion_get_ship_motion(msg, sbg_ship_motion->ship_motion);
    mavlink_msg_sbg_ship_motion_get_acceleration(msg, sbg_ship_motion->acceleration);
    mavlink_msg_sbg_ship_motion_get_velocity(msg, sbg_ship_motion->velocity);
    sbg_ship_motion->heave_period = mavlink_msg_sbg_ship_motion_get_heave_period(msg);
    sbg_ship_motion->heave_valid = mavlink_msg_sbg_ship_motion_get_heave_valid(msg);
    sbg_ship_motion->heave_vel_aided = mavlink_msg_sbg_ship_motion_get_heave_vel_aided(msg);
    sbg_ship_motion->period_available = mavlink_msg_sbg_ship_motion_get_period_available(msg);
    sbg_ship_motion->period_valid = mavlink_msg_sbg_ship_motion_get_period_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN? msg->len : MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN;
        memset(sbg_ship_motion, 0, MAVLINK_MSG_ID_SBG_SHIP_MOTION_LEN);
    memcpy(sbg_ship_motion, _MAV_PAYLOAD(msg), len);
#endif
}
