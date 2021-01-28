#pragma once
// MESSAGE SBG_GPS_VEL PACKING

#define MAVLINK_MSG_ID_SBG_GPS_VEL 636


typedef struct __mavlink_sbg_gps_vel_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 uint32_t time_stamp; /*< [us] Time since sensor is powered up us.*/
 uint32_t gps_tow; /*< [ms] GPS Time of Week ms.*/
 float vel[3]; /*< [m/s] Velocity in [North, East, Down] direction m/s.*/
 float vel_acc[3]; /*< [m/s] 1 sgima Accuracy in [North, East, Down] direction m/s.*/
 float course; /*<  True direction of motion over ground (0 to 360 deg).*/
 float course_acc; /*<  1 sgima course accuracy (0 to 360 deg).*/
 uint8_t vel_status; /*<  The raw GPS velocity status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 LIMIT.*/
 uint8_t vel_type; /*<  The raw GPS velocity type. 0 VEL_NO_SOLUTION, 1 VEL_UNKNOWN_TYPE, 2 VEL_DOPPLER, 3 VEL_DIFFERENTIAL.*/
} mavlink_sbg_gps_vel_t;

#define MAVLINK_MSG_ID_SBG_GPS_VEL_LEN 50
#define MAVLINK_MSG_ID_SBG_GPS_VEL_MIN_LEN 50
#define MAVLINK_MSG_ID_636_LEN 50
#define MAVLINK_MSG_ID_636_MIN_LEN 50

#define MAVLINK_MSG_ID_SBG_GPS_VEL_CRC 196
#define MAVLINK_MSG_ID_636_CRC 196

#define MAVLINK_MSG_SBG_GPS_VEL_FIELD_VEL_LEN 3
#define MAVLINK_MSG_SBG_GPS_VEL_FIELD_VEL_ACC_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SBG_GPS_VEL { \
    636, \
    "SBG_GPS_VEL", \
    9, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_gps_vel_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_gps_vel_t, time_stamp) }, \
         { "vel_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_sbg_gps_vel_t, vel_status) }, \
         { "vel_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_sbg_gps_vel_t, vel_type) }, \
         { "gps_tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_sbg_gps_vel_t, gps_tow) }, \
         { "vel", NULL, MAVLINK_TYPE_FLOAT, 3, 16, offsetof(mavlink_sbg_gps_vel_t, vel) }, \
         { "vel_acc", NULL, MAVLINK_TYPE_FLOAT, 3, 28, offsetof(mavlink_sbg_gps_vel_t, vel_acc) }, \
         { "course", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sbg_gps_vel_t, course) }, \
         { "course_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_sbg_gps_vel_t, course_acc) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SBG_GPS_VEL { \
    "SBG_GPS_VEL", \
    9, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_gps_vel_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_gps_vel_t, time_stamp) }, \
         { "vel_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_sbg_gps_vel_t, vel_status) }, \
         { "vel_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_sbg_gps_vel_t, vel_type) }, \
         { "gps_tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_sbg_gps_vel_t, gps_tow) }, \
         { "vel", NULL, MAVLINK_TYPE_FLOAT, 3, 16, offsetof(mavlink_sbg_gps_vel_t, vel) }, \
         { "vel_acc", NULL, MAVLINK_TYPE_FLOAT, 3, 28, offsetof(mavlink_sbg_gps_vel_t, vel_acc) }, \
         { "course", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sbg_gps_vel_t, course) }, \
         { "course_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_sbg_gps_vel_t, course_acc) }, \
         } \
}
#endif

/**
 * @brief Pack a sbg_gps_vel message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param vel_status  The raw GPS velocity status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 LIMIT.
 * @param vel_type  The raw GPS velocity type. 0 VEL_NO_SOLUTION, 1 VEL_UNKNOWN_TYPE, 2 VEL_DOPPLER, 3 VEL_DIFFERENTIAL.
 * @param gps_tow [ms] GPS Time of Week ms.
 * @param vel [m/s] Velocity in [North, East, Down] direction m/s.
 * @param vel_acc [m/s] 1 sgima Accuracy in [North, East, Down] direction m/s.
 * @param course  True direction of motion over ground (0 to 360 deg).
 * @param course_acc  1 sgima course accuracy (0 to 360 deg).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_gps_vel_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t time_stamp, uint8_t vel_status, uint8_t vel_type, uint32_t gps_tow, const float *vel, const float *vel_acc, float course, float course_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_GPS_VEL_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, gps_tow);
    _mav_put_float(buf, 40, course);
    _mav_put_float(buf, 44, course_acc);
    _mav_put_uint8_t(buf, 48, vel_status);
    _mav_put_uint8_t(buf, 49, vel_type);
    _mav_put_float_array(buf, 16, vel, 3);
    _mav_put_float_array(buf, 28, vel_acc, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_GPS_VEL_LEN);
#else
    mavlink_sbg_gps_vel_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.gps_tow = gps_tow;
    packet.course = course;
    packet.course_acc = course_acc;
    packet.vel_status = vel_status;
    packet.vel_type = vel_type;
    mav_array_memcpy(packet.vel, vel, sizeof(float)*3);
    mav_array_memcpy(packet.vel_acc, vel_acc, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_GPS_VEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_GPS_VEL;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SBG_GPS_VEL_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_CRC);
}

/**
 * @brief Pack a sbg_gps_vel message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param vel_status  The raw GPS velocity status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 LIMIT.
 * @param vel_type  The raw GPS velocity type. 0 VEL_NO_SOLUTION, 1 VEL_UNKNOWN_TYPE, 2 VEL_DOPPLER, 3 VEL_DIFFERENTIAL.
 * @param gps_tow [ms] GPS Time of Week ms.
 * @param vel [m/s] Velocity in [North, East, Down] direction m/s.
 * @param vel_acc [m/s] 1 sgima Accuracy in [North, East, Down] direction m/s.
 * @param course  True direction of motion over ground (0 to 360 deg).
 * @param course_acc  1 sgima course accuracy (0 to 360 deg).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_gps_vel_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t time_stamp,uint8_t vel_status,uint8_t vel_type,uint32_t gps_tow,const float *vel,const float *vel_acc,float course,float course_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_GPS_VEL_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, gps_tow);
    _mav_put_float(buf, 40, course);
    _mav_put_float(buf, 44, course_acc);
    _mav_put_uint8_t(buf, 48, vel_status);
    _mav_put_uint8_t(buf, 49, vel_type);
    _mav_put_float_array(buf, 16, vel, 3);
    _mav_put_float_array(buf, 28, vel_acc, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_GPS_VEL_LEN);
#else
    mavlink_sbg_gps_vel_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.gps_tow = gps_tow;
    packet.course = course;
    packet.course_acc = course_acc;
    packet.vel_status = vel_status;
    packet.vel_type = vel_type;
    mav_array_memcpy(packet.vel, vel, sizeof(float)*3);
    mav_array_memcpy(packet.vel_acc, vel_acc, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_GPS_VEL_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_GPS_VEL;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SBG_GPS_VEL_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_CRC);
}

/**
 * @brief Encode a sbg_gps_vel struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbg_gps_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_gps_vel_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbg_gps_vel_t* sbg_gps_vel)
{
    return mavlink_msg_sbg_gps_vel_pack(system_id, component_id, msg, sbg_gps_vel->timestamp, sbg_gps_vel->time_stamp, sbg_gps_vel->vel_status, sbg_gps_vel->vel_type, sbg_gps_vel->gps_tow, sbg_gps_vel->vel, sbg_gps_vel->vel_acc, sbg_gps_vel->course, sbg_gps_vel->course_acc);
}

/**
 * @brief Encode a sbg_gps_vel struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sbg_gps_vel C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_gps_vel_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sbg_gps_vel_t* sbg_gps_vel)
{
    return mavlink_msg_sbg_gps_vel_pack_chan(system_id, component_id, chan, msg, sbg_gps_vel->timestamp, sbg_gps_vel->time_stamp, sbg_gps_vel->vel_status, sbg_gps_vel->vel_type, sbg_gps_vel->gps_tow, sbg_gps_vel->vel, sbg_gps_vel->vel_acc, sbg_gps_vel->course, sbg_gps_vel->course_acc);
}

/**
 * @brief Send a sbg_gps_vel message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param vel_status  The raw GPS velocity status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 LIMIT.
 * @param vel_type  The raw GPS velocity type. 0 VEL_NO_SOLUTION, 1 VEL_UNKNOWN_TYPE, 2 VEL_DOPPLER, 3 VEL_DIFFERENTIAL.
 * @param gps_tow [ms] GPS Time of Week ms.
 * @param vel [m/s] Velocity in [North, East, Down] direction m/s.
 * @param vel_acc [m/s] 1 sgima Accuracy in [North, East, Down] direction m/s.
 * @param course  True direction of motion over ground (0 to 360 deg).
 * @param course_acc  1 sgima course accuracy (0 to 360 deg).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sbg_gps_vel_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t time_stamp, uint8_t vel_status, uint8_t vel_type, uint32_t gps_tow, const float *vel, const float *vel_acc, float course, float course_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_GPS_VEL_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, gps_tow);
    _mav_put_float(buf, 40, course);
    _mav_put_float(buf, 44, course_acc);
    _mav_put_uint8_t(buf, 48, vel_status);
    _mav_put_uint8_t(buf, 49, vel_type);
    _mav_put_float_array(buf, 16, vel, 3);
    _mav_put_float_array(buf, 28, vel_acc, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_VEL, buf, MAVLINK_MSG_ID_SBG_GPS_VEL_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_CRC);
#else
    mavlink_sbg_gps_vel_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.gps_tow = gps_tow;
    packet.course = course;
    packet.course_acc = course_acc;
    packet.vel_status = vel_status;
    packet.vel_type = vel_type;
    mav_array_memcpy(packet.vel, vel, sizeof(float)*3);
    mav_array_memcpy(packet.vel_acc, vel_acc, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_VEL, (const char *)&packet, MAVLINK_MSG_ID_SBG_GPS_VEL_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_CRC);
#endif
}

/**
 * @brief Send a sbg_gps_vel message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sbg_gps_vel_send_struct(mavlink_channel_t chan, const mavlink_sbg_gps_vel_t* sbg_gps_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sbg_gps_vel_send(chan, sbg_gps_vel->timestamp, sbg_gps_vel->time_stamp, sbg_gps_vel->vel_status, sbg_gps_vel->vel_type, sbg_gps_vel->gps_tow, sbg_gps_vel->vel, sbg_gps_vel->vel_acc, sbg_gps_vel->course, sbg_gps_vel->course_acc);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_VEL, (const char *)sbg_gps_vel, MAVLINK_MSG_ID_SBG_GPS_VEL_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_CRC);
#endif
}

#if MAVLINK_MSG_ID_SBG_GPS_VEL_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sbg_gps_vel_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t time_stamp, uint8_t vel_status, uint8_t vel_type, uint32_t gps_tow, const float *vel, const float *vel_acc, float course, float course_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, gps_tow);
    _mav_put_float(buf, 40, course);
    _mav_put_float(buf, 44, course_acc);
    _mav_put_uint8_t(buf, 48, vel_status);
    _mav_put_uint8_t(buf, 49, vel_type);
    _mav_put_float_array(buf, 16, vel, 3);
    _mav_put_float_array(buf, 28, vel_acc, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_VEL, buf, MAVLINK_MSG_ID_SBG_GPS_VEL_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_CRC);
#else
    mavlink_sbg_gps_vel_t *packet = (mavlink_sbg_gps_vel_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->time_stamp = time_stamp;
    packet->gps_tow = gps_tow;
    packet->course = course;
    packet->course_acc = course_acc;
    packet->vel_status = vel_status;
    packet->vel_type = vel_type;
    mav_array_memcpy(packet->vel, vel, sizeof(float)*3);
    mav_array_memcpy(packet->vel_acc, vel_acc, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_VEL, (const char *)packet, MAVLINK_MSG_ID_SBG_GPS_VEL_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_LEN, MAVLINK_MSG_ID_SBG_GPS_VEL_CRC);
#endif
}
#endif

#endif

// MESSAGE SBG_GPS_VEL UNPACKING


/**
 * @brief Get field timestamp from sbg_gps_vel message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_sbg_gps_vel_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_stamp from sbg_gps_vel message
 *
 * @return [us] Time since sensor is powered up us.
 */
static inline uint32_t mavlink_msg_sbg_gps_vel_get_time_stamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field vel_status from sbg_gps_vel message
 *
 * @return  The raw GPS velocity status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 LIMIT.
 */
static inline uint8_t mavlink_msg_sbg_gps_vel_get_vel_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field vel_type from sbg_gps_vel message
 *
 * @return  The raw GPS velocity type. 0 VEL_NO_SOLUTION, 1 VEL_UNKNOWN_TYPE, 2 VEL_DOPPLER, 3 VEL_DIFFERENTIAL.
 */
static inline uint8_t mavlink_msg_sbg_gps_vel_get_vel_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field gps_tow from sbg_gps_vel message
 *
 * @return [ms] GPS Time of Week ms.
 */
static inline uint32_t mavlink_msg_sbg_gps_vel_get_gps_tow(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field vel from sbg_gps_vel message
 *
 * @return [m/s] Velocity in [North, East, Down] direction m/s.
 */
static inline uint16_t mavlink_msg_sbg_gps_vel_get_vel(const mavlink_message_t* msg, float *vel)
{
    return _MAV_RETURN_float_array(msg, vel, 3,  16);
}

/**
 * @brief Get field vel_acc from sbg_gps_vel message
 *
 * @return [m/s] 1 sgima Accuracy in [North, East, Down] direction m/s.
 */
static inline uint16_t mavlink_msg_sbg_gps_vel_get_vel_acc(const mavlink_message_t* msg, float *vel_acc)
{
    return _MAV_RETURN_float_array(msg, vel_acc, 3,  28);
}

/**
 * @brief Get field course from sbg_gps_vel message
 *
 * @return  True direction of motion over ground (0 to 360 deg).
 */
static inline float mavlink_msg_sbg_gps_vel_get_course(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field course_acc from sbg_gps_vel message
 *
 * @return  1 sgima course accuracy (0 to 360 deg).
 */
static inline float mavlink_msg_sbg_gps_vel_get_course_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Decode a sbg_gps_vel message into a struct
 *
 * @param msg The message to decode
 * @param sbg_gps_vel C-struct to decode the message contents into
 */
static inline void mavlink_msg_sbg_gps_vel_decode(const mavlink_message_t* msg, mavlink_sbg_gps_vel_t* sbg_gps_vel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sbg_gps_vel->timestamp = mavlink_msg_sbg_gps_vel_get_timestamp(msg);
    sbg_gps_vel->time_stamp = mavlink_msg_sbg_gps_vel_get_time_stamp(msg);
    sbg_gps_vel->gps_tow = mavlink_msg_sbg_gps_vel_get_gps_tow(msg);
    mavlink_msg_sbg_gps_vel_get_vel(msg, sbg_gps_vel->vel);
    mavlink_msg_sbg_gps_vel_get_vel_acc(msg, sbg_gps_vel->vel_acc);
    sbg_gps_vel->course = mavlink_msg_sbg_gps_vel_get_course(msg);
    sbg_gps_vel->course_acc = mavlink_msg_sbg_gps_vel_get_course_acc(msg);
    sbg_gps_vel->vel_status = mavlink_msg_sbg_gps_vel_get_vel_status(msg);
    sbg_gps_vel->vel_type = mavlink_msg_sbg_gps_vel_get_vel_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SBG_GPS_VEL_LEN? msg->len : MAVLINK_MSG_ID_SBG_GPS_VEL_LEN;
        memset(sbg_gps_vel, 0, MAVLINK_MSG_ID_SBG_GPS_VEL_LEN);
    memcpy(sbg_gps_vel, _MAV_PAYLOAD(msg), len);
#endif
}
