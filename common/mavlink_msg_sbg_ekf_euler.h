#pragma once
// MESSAGE SBG_EKF_EULER PACKING

#define MAVLINK_MSG_ID_SBG_EKF_EULER 629

MAVPACKED(
typedef struct __mavlink_sbg_ekf_euler_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 uint32_t time_stamp; /*< [us] Time since sensor is powered up us.*/
 float angle[3]; /*< [rad] [Roll, Pitch, Yaw (heading)] angle rad.*/
 float accuracy[3]; /*< [rad] [Roll, Pitch, Yaw (heading)] angle accuracy (1 sigma) rad.*/
 uint8_t solution_mode; /*<  Defines the Kalman filter computation mode (see the table 4 below). 0 UNINITIALIZED The Kalman filter is not initialized and the returned data are all invalid. 1 VERTICAL_GYRO The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely. 2 AHRS A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely. 3 NAV_VELOCITY The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation. 4 NAV_POSITION Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided.*/
 uint8_t attitude_valid; /*<  True if Attitude data is reliable (Roll/Pitch error less than 0,5 deg).*/
 uint8_t heading_valid; /*<  True if Heading data is reliable (Heading error less than 1 deg).*/
 uint8_t velocity_valid; /*<  True if Velocity data is reliable (velocity error less than 1.5 m/s).*/
 uint8_t position_valid; /*<  True if Position data is reliable (Position error less than 10m).*/
 uint8_t vert_ref_used; /*<  True if vertical reference is used in solution (data used and valid since 3s).*/
 uint8_t mag_ref_used; /*<  True if magnetometer is used in solution (data used and valid since 3s).*/
 uint8_t gps1_vel_used; /*<  True if GPS velocity is used in solution (data used and valid since 3s).*/
 uint8_t gps1_pos_used; /*<  True if GPS Position is used in solution (data used and valid since 3s).*/
 uint8_t gps1_course_used; /*<  True if GPS Course is used in solution (data used and valid since 3s).*/
 uint8_t gps1_hdt_used; /*<  True if GPS True Heading is used in solution (data used and valid since 3s).*/
 uint8_t gps2_vel_used; /*<  True if GPS2 velocity is used in solution (data used and valid since 3s).*/
 uint8_t gps2_pos_used; /*<  True if GPS2 Position is used in solution (data used and valid since 3s).*/
 uint8_t gps2_course_used; /*<  True if GPS2 Course is used in solution (data used and valid since 3s).*/
 uint8_t gps2_hdt_used; /*<  True if GPS2 True Heading is used in solution (data used and valid since 3s).*/
 uint8_t odo_used; /*<  True if Odometer is used in solution (data used and valid since 3s).*/
}) mavlink_sbg_ekf_euler_t;

#define MAVLINK_MSG_ID_SBG_EKF_EULER_LEN 52
#define MAVLINK_MSG_ID_SBG_EKF_EULER_MIN_LEN 52
#define MAVLINK_MSG_ID_629_LEN 52
#define MAVLINK_MSG_ID_629_MIN_LEN 52

#define MAVLINK_MSG_ID_SBG_EKF_EULER_CRC 132
#define MAVLINK_MSG_ID_629_CRC 132

#define MAVLINK_MSG_SBG_EKF_EULER_FIELD_ANGLE_LEN 3
#define MAVLINK_MSG_SBG_EKF_EULER_FIELD_ACCURACY_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SBG_EKF_EULER { \
    629, \
    "SBG_EKF_EULER", \
    20, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_ekf_euler_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_ekf_euler_t, time_stamp) }, \
         { "angle", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_sbg_ekf_euler_t, angle) }, \
         { "accuracy", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_sbg_ekf_euler_t, accuracy) }, \
         { "solution_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_sbg_ekf_euler_t, solution_mode) }, \
         { "attitude_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_sbg_ekf_euler_t, attitude_valid) }, \
         { "heading_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_sbg_ekf_euler_t, heading_valid) }, \
         { "velocity_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_sbg_ekf_euler_t, velocity_valid) }, \
         { "position_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_sbg_ekf_euler_t, position_valid) }, \
         { "vert_ref_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_sbg_ekf_euler_t, vert_ref_used) }, \
         { "mag_ref_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_sbg_ekf_euler_t, mag_ref_used) }, \
         { "gps1_vel_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_sbg_ekf_euler_t, gps1_vel_used) }, \
         { "gps1_pos_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_sbg_ekf_euler_t, gps1_pos_used) }, \
         { "gps1_course_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_sbg_ekf_euler_t, gps1_course_used) }, \
         { "gps1_hdt_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_sbg_ekf_euler_t, gps1_hdt_used) }, \
         { "gps2_vel_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 47, offsetof(mavlink_sbg_ekf_euler_t, gps2_vel_used) }, \
         { "gps2_pos_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_sbg_ekf_euler_t, gps2_pos_used) }, \
         { "gps2_course_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_sbg_ekf_euler_t, gps2_course_used) }, \
         { "gps2_hdt_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_sbg_ekf_euler_t, gps2_hdt_used) }, \
         { "odo_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_sbg_ekf_euler_t, odo_used) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SBG_EKF_EULER { \
    "SBG_EKF_EULER", \
    20, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_ekf_euler_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_ekf_euler_t, time_stamp) }, \
         { "angle", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_sbg_ekf_euler_t, angle) }, \
         { "accuracy", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_sbg_ekf_euler_t, accuracy) }, \
         { "solution_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_sbg_ekf_euler_t, solution_mode) }, \
         { "attitude_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_sbg_ekf_euler_t, attitude_valid) }, \
         { "heading_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_sbg_ekf_euler_t, heading_valid) }, \
         { "velocity_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_sbg_ekf_euler_t, velocity_valid) }, \
         { "position_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_sbg_ekf_euler_t, position_valid) }, \
         { "vert_ref_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_sbg_ekf_euler_t, vert_ref_used) }, \
         { "mag_ref_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_sbg_ekf_euler_t, mag_ref_used) }, \
         { "gps1_vel_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_sbg_ekf_euler_t, gps1_vel_used) }, \
         { "gps1_pos_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_sbg_ekf_euler_t, gps1_pos_used) }, \
         { "gps1_course_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 45, offsetof(mavlink_sbg_ekf_euler_t, gps1_course_used) }, \
         { "gps1_hdt_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 46, offsetof(mavlink_sbg_ekf_euler_t, gps1_hdt_used) }, \
         { "gps2_vel_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 47, offsetof(mavlink_sbg_ekf_euler_t, gps2_vel_used) }, \
         { "gps2_pos_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_sbg_ekf_euler_t, gps2_pos_used) }, \
         { "gps2_course_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_sbg_ekf_euler_t, gps2_course_used) }, \
         { "gps2_hdt_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_sbg_ekf_euler_t, gps2_hdt_used) }, \
         { "odo_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_sbg_ekf_euler_t, odo_used) }, \
         } \
}
#endif

/**
 * @brief Pack a sbg_ekf_euler message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param angle [rad] [Roll, Pitch, Yaw (heading)] angle rad.
 * @param accuracy [rad] [Roll, Pitch, Yaw (heading)] angle accuracy (1 sigma) rad.
 * @param solution_mode  Defines the Kalman filter computation mode (see the table 4 below). 0 UNINITIALIZED The Kalman filter is not initialized and the returned data are all invalid. 1 VERTICAL_GYRO The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely. 2 AHRS A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely. 3 NAV_VELOCITY The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation. 4 NAV_POSITION Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided.
 * @param attitude_valid  True if Attitude data is reliable (Roll/Pitch error less than 0,5 deg).
 * @param heading_valid  True if Heading data is reliable (Heading error less than 1 deg).
 * @param velocity_valid  True if Velocity data is reliable (velocity error less than 1.5 m/s).
 * @param position_valid  True if Position data is reliable (Position error less than 10m).
 * @param vert_ref_used  True if vertical reference is used in solution (data used and valid since 3s).
 * @param mag_ref_used  True if magnetometer is used in solution (data used and valid since 3s).
 * @param gps1_vel_used  True if GPS velocity is used in solution (data used and valid since 3s).
 * @param gps1_pos_used  True if GPS Position is used in solution (data used and valid since 3s).
 * @param gps1_course_used  True if GPS Course is used in solution (data used and valid since 3s).
 * @param gps1_hdt_used  True if GPS True Heading is used in solution (data used and valid since 3s).
 * @param gps2_vel_used  True if GPS2 velocity is used in solution (data used and valid since 3s).
 * @param gps2_pos_used  True if GPS2 Position is used in solution (data used and valid since 3s).
 * @param gps2_course_used  True if GPS2 Course is used in solution (data used and valid since 3s).
 * @param gps2_hdt_used  True if GPS2 True Heading is used in solution (data used and valid since 3s).
 * @param odo_used  True if Odometer is used in solution (data used and valid since 3s).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_ekf_euler_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t time_stamp, const float *angle, const float *accuracy, uint8_t solution_mode, uint8_t attitude_valid, uint8_t heading_valid, uint8_t velocity_valid, uint8_t position_valid, uint8_t vert_ref_used, uint8_t mag_ref_used, uint8_t gps1_vel_used, uint8_t gps1_pos_used, uint8_t gps1_course_used, uint8_t gps1_hdt_used, uint8_t gps2_vel_used, uint8_t gps2_pos_used, uint8_t gps2_course_used, uint8_t gps2_hdt_used, uint8_t odo_used)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_EKF_EULER_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint8_t(buf, 36, solution_mode);
    _mav_put_uint8_t(buf, 37, attitude_valid);
    _mav_put_uint8_t(buf, 38, heading_valid);
    _mav_put_uint8_t(buf, 39, velocity_valid);
    _mav_put_uint8_t(buf, 40, position_valid);
    _mav_put_uint8_t(buf, 41, vert_ref_used);
    _mav_put_uint8_t(buf, 42, mag_ref_used);
    _mav_put_uint8_t(buf, 43, gps1_vel_used);
    _mav_put_uint8_t(buf, 44, gps1_pos_used);
    _mav_put_uint8_t(buf, 45, gps1_course_used);
    _mav_put_uint8_t(buf, 46, gps1_hdt_used);
    _mav_put_uint8_t(buf, 47, gps2_vel_used);
    _mav_put_uint8_t(buf, 48, gps2_pos_used);
    _mav_put_uint8_t(buf, 49, gps2_course_used);
    _mav_put_uint8_t(buf, 50, gps2_hdt_used);
    _mav_put_uint8_t(buf, 51, odo_used);
    _mav_put_float_array(buf, 12, angle, 3);
    _mav_put_float_array(buf, 24, accuracy, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_EKF_EULER_LEN);
#else
    mavlink_sbg_ekf_euler_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.solution_mode = solution_mode;
    packet.attitude_valid = attitude_valid;
    packet.heading_valid = heading_valid;
    packet.velocity_valid = velocity_valid;
    packet.position_valid = position_valid;
    packet.vert_ref_used = vert_ref_used;
    packet.mag_ref_used = mag_ref_used;
    packet.gps1_vel_used = gps1_vel_used;
    packet.gps1_pos_used = gps1_pos_used;
    packet.gps1_course_used = gps1_course_used;
    packet.gps1_hdt_used = gps1_hdt_used;
    packet.gps2_vel_used = gps2_vel_used;
    packet.gps2_pos_used = gps2_pos_used;
    packet.gps2_course_used = gps2_course_used;
    packet.gps2_hdt_used = gps2_hdt_used;
    packet.odo_used = odo_used;
    mav_array_memcpy(packet.angle, angle, sizeof(float)*3);
    mav_array_memcpy(packet.accuracy, accuracy, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_EKF_EULER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_EKF_EULER;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SBG_EKF_EULER_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_CRC);
}

/**
 * @brief Pack a sbg_ekf_euler message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param angle [rad] [Roll, Pitch, Yaw (heading)] angle rad.
 * @param accuracy [rad] [Roll, Pitch, Yaw (heading)] angle accuracy (1 sigma) rad.
 * @param solution_mode  Defines the Kalman filter computation mode (see the table 4 below). 0 UNINITIALIZED The Kalman filter is not initialized and the returned data are all invalid. 1 VERTICAL_GYRO The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely. 2 AHRS A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely. 3 NAV_VELOCITY The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation. 4 NAV_POSITION Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided.
 * @param attitude_valid  True if Attitude data is reliable (Roll/Pitch error less than 0,5 deg).
 * @param heading_valid  True if Heading data is reliable (Heading error less than 1 deg).
 * @param velocity_valid  True if Velocity data is reliable (velocity error less than 1.5 m/s).
 * @param position_valid  True if Position data is reliable (Position error less than 10m).
 * @param vert_ref_used  True if vertical reference is used in solution (data used and valid since 3s).
 * @param mag_ref_used  True if magnetometer is used in solution (data used and valid since 3s).
 * @param gps1_vel_used  True if GPS velocity is used in solution (data used and valid since 3s).
 * @param gps1_pos_used  True if GPS Position is used in solution (data used and valid since 3s).
 * @param gps1_course_used  True if GPS Course is used in solution (data used and valid since 3s).
 * @param gps1_hdt_used  True if GPS True Heading is used in solution (data used and valid since 3s).
 * @param gps2_vel_used  True if GPS2 velocity is used in solution (data used and valid since 3s).
 * @param gps2_pos_used  True if GPS2 Position is used in solution (data used and valid since 3s).
 * @param gps2_course_used  True if GPS2 Course is used in solution (data used and valid since 3s).
 * @param gps2_hdt_used  True if GPS2 True Heading is used in solution (data used and valid since 3s).
 * @param odo_used  True if Odometer is used in solution (data used and valid since 3s).
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_ekf_euler_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t time_stamp,const float *angle,const float *accuracy,uint8_t solution_mode,uint8_t attitude_valid,uint8_t heading_valid,uint8_t velocity_valid,uint8_t position_valid,uint8_t vert_ref_used,uint8_t mag_ref_used,uint8_t gps1_vel_used,uint8_t gps1_pos_used,uint8_t gps1_course_used,uint8_t gps1_hdt_used,uint8_t gps2_vel_used,uint8_t gps2_pos_used,uint8_t gps2_course_used,uint8_t gps2_hdt_used,uint8_t odo_used)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_EKF_EULER_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint8_t(buf, 36, solution_mode);
    _mav_put_uint8_t(buf, 37, attitude_valid);
    _mav_put_uint8_t(buf, 38, heading_valid);
    _mav_put_uint8_t(buf, 39, velocity_valid);
    _mav_put_uint8_t(buf, 40, position_valid);
    _mav_put_uint8_t(buf, 41, vert_ref_used);
    _mav_put_uint8_t(buf, 42, mag_ref_used);
    _mav_put_uint8_t(buf, 43, gps1_vel_used);
    _mav_put_uint8_t(buf, 44, gps1_pos_used);
    _mav_put_uint8_t(buf, 45, gps1_course_used);
    _mav_put_uint8_t(buf, 46, gps1_hdt_used);
    _mav_put_uint8_t(buf, 47, gps2_vel_used);
    _mav_put_uint8_t(buf, 48, gps2_pos_used);
    _mav_put_uint8_t(buf, 49, gps2_course_used);
    _mav_put_uint8_t(buf, 50, gps2_hdt_used);
    _mav_put_uint8_t(buf, 51, odo_used);
    _mav_put_float_array(buf, 12, angle, 3);
    _mav_put_float_array(buf, 24, accuracy, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_EKF_EULER_LEN);
#else
    mavlink_sbg_ekf_euler_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.solution_mode = solution_mode;
    packet.attitude_valid = attitude_valid;
    packet.heading_valid = heading_valid;
    packet.velocity_valid = velocity_valid;
    packet.position_valid = position_valid;
    packet.vert_ref_used = vert_ref_used;
    packet.mag_ref_used = mag_ref_used;
    packet.gps1_vel_used = gps1_vel_used;
    packet.gps1_pos_used = gps1_pos_used;
    packet.gps1_course_used = gps1_course_used;
    packet.gps1_hdt_used = gps1_hdt_used;
    packet.gps2_vel_used = gps2_vel_used;
    packet.gps2_pos_used = gps2_pos_used;
    packet.gps2_course_used = gps2_course_used;
    packet.gps2_hdt_used = gps2_hdt_used;
    packet.odo_used = odo_used;
    mav_array_memcpy(packet.angle, angle, sizeof(float)*3);
    mav_array_memcpy(packet.accuracy, accuracy, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_EKF_EULER_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_EKF_EULER;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SBG_EKF_EULER_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_CRC);
}

/**
 * @brief Encode a sbg_ekf_euler struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbg_ekf_euler C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_ekf_euler_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbg_ekf_euler_t* sbg_ekf_euler)
{
    return mavlink_msg_sbg_ekf_euler_pack(system_id, component_id, msg, sbg_ekf_euler->timestamp, sbg_ekf_euler->time_stamp, sbg_ekf_euler->angle, sbg_ekf_euler->accuracy, sbg_ekf_euler->solution_mode, sbg_ekf_euler->attitude_valid, sbg_ekf_euler->heading_valid, sbg_ekf_euler->velocity_valid, sbg_ekf_euler->position_valid, sbg_ekf_euler->vert_ref_used, sbg_ekf_euler->mag_ref_used, sbg_ekf_euler->gps1_vel_used, sbg_ekf_euler->gps1_pos_used, sbg_ekf_euler->gps1_course_used, sbg_ekf_euler->gps1_hdt_used, sbg_ekf_euler->gps2_vel_used, sbg_ekf_euler->gps2_pos_used, sbg_ekf_euler->gps2_course_used, sbg_ekf_euler->gps2_hdt_used, sbg_ekf_euler->odo_used);
}

/**
 * @brief Encode a sbg_ekf_euler struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sbg_ekf_euler C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_ekf_euler_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sbg_ekf_euler_t* sbg_ekf_euler)
{
    return mavlink_msg_sbg_ekf_euler_pack_chan(system_id, component_id, chan, msg, sbg_ekf_euler->timestamp, sbg_ekf_euler->time_stamp, sbg_ekf_euler->angle, sbg_ekf_euler->accuracy, sbg_ekf_euler->solution_mode, sbg_ekf_euler->attitude_valid, sbg_ekf_euler->heading_valid, sbg_ekf_euler->velocity_valid, sbg_ekf_euler->position_valid, sbg_ekf_euler->vert_ref_used, sbg_ekf_euler->mag_ref_used, sbg_ekf_euler->gps1_vel_used, sbg_ekf_euler->gps1_pos_used, sbg_ekf_euler->gps1_course_used, sbg_ekf_euler->gps1_hdt_used, sbg_ekf_euler->gps2_vel_used, sbg_ekf_euler->gps2_pos_used, sbg_ekf_euler->gps2_course_used, sbg_ekf_euler->gps2_hdt_used, sbg_ekf_euler->odo_used);
}

/**
 * @brief Send a sbg_ekf_euler message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param angle [rad] [Roll, Pitch, Yaw (heading)] angle rad.
 * @param accuracy [rad] [Roll, Pitch, Yaw (heading)] angle accuracy (1 sigma) rad.
 * @param solution_mode  Defines the Kalman filter computation mode (see the table 4 below). 0 UNINITIALIZED The Kalman filter is not initialized and the returned data are all invalid. 1 VERTICAL_GYRO The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely. 2 AHRS A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely. 3 NAV_VELOCITY The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation. 4 NAV_POSITION Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided.
 * @param attitude_valid  True if Attitude data is reliable (Roll/Pitch error less than 0,5 deg).
 * @param heading_valid  True if Heading data is reliable (Heading error less than 1 deg).
 * @param velocity_valid  True if Velocity data is reliable (velocity error less than 1.5 m/s).
 * @param position_valid  True if Position data is reliable (Position error less than 10m).
 * @param vert_ref_used  True if vertical reference is used in solution (data used and valid since 3s).
 * @param mag_ref_used  True if magnetometer is used in solution (data used and valid since 3s).
 * @param gps1_vel_used  True if GPS velocity is used in solution (data used and valid since 3s).
 * @param gps1_pos_used  True if GPS Position is used in solution (data used and valid since 3s).
 * @param gps1_course_used  True if GPS Course is used in solution (data used and valid since 3s).
 * @param gps1_hdt_used  True if GPS True Heading is used in solution (data used and valid since 3s).
 * @param gps2_vel_used  True if GPS2 velocity is used in solution (data used and valid since 3s).
 * @param gps2_pos_used  True if GPS2 Position is used in solution (data used and valid since 3s).
 * @param gps2_course_used  True if GPS2 Course is used in solution (data used and valid since 3s).
 * @param gps2_hdt_used  True if GPS2 True Heading is used in solution (data used and valid since 3s).
 * @param odo_used  True if Odometer is used in solution (data used and valid since 3s).
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sbg_ekf_euler_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t time_stamp, const float *angle, const float *accuracy, uint8_t solution_mode, uint8_t attitude_valid, uint8_t heading_valid, uint8_t velocity_valid, uint8_t position_valid, uint8_t vert_ref_used, uint8_t mag_ref_used, uint8_t gps1_vel_used, uint8_t gps1_pos_used, uint8_t gps1_course_used, uint8_t gps1_hdt_used, uint8_t gps2_vel_used, uint8_t gps2_pos_used, uint8_t gps2_course_used, uint8_t gps2_hdt_used, uint8_t odo_used)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_EKF_EULER_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint8_t(buf, 36, solution_mode);
    _mav_put_uint8_t(buf, 37, attitude_valid);
    _mav_put_uint8_t(buf, 38, heading_valid);
    _mav_put_uint8_t(buf, 39, velocity_valid);
    _mav_put_uint8_t(buf, 40, position_valid);
    _mav_put_uint8_t(buf, 41, vert_ref_used);
    _mav_put_uint8_t(buf, 42, mag_ref_used);
    _mav_put_uint8_t(buf, 43, gps1_vel_used);
    _mav_put_uint8_t(buf, 44, gps1_pos_used);
    _mav_put_uint8_t(buf, 45, gps1_course_used);
    _mav_put_uint8_t(buf, 46, gps1_hdt_used);
    _mav_put_uint8_t(buf, 47, gps2_vel_used);
    _mav_put_uint8_t(buf, 48, gps2_pos_used);
    _mav_put_uint8_t(buf, 49, gps2_course_used);
    _mav_put_uint8_t(buf, 50, gps2_hdt_used);
    _mav_put_uint8_t(buf, 51, odo_used);
    _mav_put_float_array(buf, 12, angle, 3);
    _mav_put_float_array(buf, 24, accuracy, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_EKF_EULER, buf, MAVLINK_MSG_ID_SBG_EKF_EULER_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_CRC);
#else
    mavlink_sbg_ekf_euler_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.solution_mode = solution_mode;
    packet.attitude_valid = attitude_valid;
    packet.heading_valid = heading_valid;
    packet.velocity_valid = velocity_valid;
    packet.position_valid = position_valid;
    packet.vert_ref_used = vert_ref_used;
    packet.mag_ref_used = mag_ref_used;
    packet.gps1_vel_used = gps1_vel_used;
    packet.gps1_pos_used = gps1_pos_used;
    packet.gps1_course_used = gps1_course_used;
    packet.gps1_hdt_used = gps1_hdt_used;
    packet.gps2_vel_used = gps2_vel_used;
    packet.gps2_pos_used = gps2_pos_used;
    packet.gps2_course_used = gps2_course_used;
    packet.gps2_hdt_used = gps2_hdt_used;
    packet.odo_used = odo_used;
    mav_array_memcpy(packet.angle, angle, sizeof(float)*3);
    mav_array_memcpy(packet.accuracy, accuracy, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_EKF_EULER, (const char *)&packet, MAVLINK_MSG_ID_SBG_EKF_EULER_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_CRC);
#endif
}

/**
 * @brief Send a sbg_ekf_euler message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sbg_ekf_euler_send_struct(mavlink_channel_t chan, const mavlink_sbg_ekf_euler_t* sbg_ekf_euler)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sbg_ekf_euler_send(chan, sbg_ekf_euler->timestamp, sbg_ekf_euler->time_stamp, sbg_ekf_euler->angle, sbg_ekf_euler->accuracy, sbg_ekf_euler->solution_mode, sbg_ekf_euler->attitude_valid, sbg_ekf_euler->heading_valid, sbg_ekf_euler->velocity_valid, sbg_ekf_euler->position_valid, sbg_ekf_euler->vert_ref_used, sbg_ekf_euler->mag_ref_used, sbg_ekf_euler->gps1_vel_used, sbg_ekf_euler->gps1_pos_used, sbg_ekf_euler->gps1_course_used, sbg_ekf_euler->gps1_hdt_used, sbg_ekf_euler->gps2_vel_used, sbg_ekf_euler->gps2_pos_used, sbg_ekf_euler->gps2_course_used, sbg_ekf_euler->gps2_hdt_used, sbg_ekf_euler->odo_used);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_EKF_EULER, (const char *)sbg_ekf_euler, MAVLINK_MSG_ID_SBG_EKF_EULER_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_CRC);
#endif
}

#if MAVLINK_MSG_ID_SBG_EKF_EULER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sbg_ekf_euler_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t time_stamp, const float *angle, const float *accuracy, uint8_t solution_mode, uint8_t attitude_valid, uint8_t heading_valid, uint8_t velocity_valid, uint8_t position_valid, uint8_t vert_ref_used, uint8_t mag_ref_used, uint8_t gps1_vel_used, uint8_t gps1_pos_used, uint8_t gps1_course_used, uint8_t gps1_hdt_used, uint8_t gps2_vel_used, uint8_t gps2_pos_used, uint8_t gps2_course_used, uint8_t gps2_hdt_used, uint8_t odo_used)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint8_t(buf, 36, solution_mode);
    _mav_put_uint8_t(buf, 37, attitude_valid);
    _mav_put_uint8_t(buf, 38, heading_valid);
    _mav_put_uint8_t(buf, 39, velocity_valid);
    _mav_put_uint8_t(buf, 40, position_valid);
    _mav_put_uint8_t(buf, 41, vert_ref_used);
    _mav_put_uint8_t(buf, 42, mag_ref_used);
    _mav_put_uint8_t(buf, 43, gps1_vel_used);
    _mav_put_uint8_t(buf, 44, gps1_pos_used);
    _mav_put_uint8_t(buf, 45, gps1_course_used);
    _mav_put_uint8_t(buf, 46, gps1_hdt_used);
    _mav_put_uint8_t(buf, 47, gps2_vel_used);
    _mav_put_uint8_t(buf, 48, gps2_pos_used);
    _mav_put_uint8_t(buf, 49, gps2_course_used);
    _mav_put_uint8_t(buf, 50, gps2_hdt_used);
    _mav_put_uint8_t(buf, 51, odo_used);
    _mav_put_float_array(buf, 12, angle, 3);
    _mav_put_float_array(buf, 24, accuracy, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_EKF_EULER, buf, MAVLINK_MSG_ID_SBG_EKF_EULER_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_CRC);
#else
    mavlink_sbg_ekf_euler_t *packet = (mavlink_sbg_ekf_euler_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->time_stamp = time_stamp;
    packet->solution_mode = solution_mode;
    packet->attitude_valid = attitude_valid;
    packet->heading_valid = heading_valid;
    packet->velocity_valid = velocity_valid;
    packet->position_valid = position_valid;
    packet->vert_ref_used = vert_ref_used;
    packet->mag_ref_used = mag_ref_used;
    packet->gps1_vel_used = gps1_vel_used;
    packet->gps1_pos_used = gps1_pos_used;
    packet->gps1_course_used = gps1_course_used;
    packet->gps1_hdt_used = gps1_hdt_used;
    packet->gps2_vel_used = gps2_vel_used;
    packet->gps2_pos_used = gps2_pos_used;
    packet->gps2_course_used = gps2_course_used;
    packet->gps2_hdt_used = gps2_hdt_used;
    packet->odo_used = odo_used;
    mav_array_memcpy(packet->angle, angle, sizeof(float)*3);
    mav_array_memcpy(packet->accuracy, accuracy, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_EKF_EULER, (const char *)packet, MAVLINK_MSG_ID_SBG_EKF_EULER_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_LEN, MAVLINK_MSG_ID_SBG_EKF_EULER_CRC);
#endif
}
#endif

#endif

// MESSAGE SBG_EKF_EULER UNPACKING


/**
 * @brief Get field timestamp from sbg_ekf_euler message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_sbg_ekf_euler_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_stamp from sbg_ekf_euler message
 *
 * @return [us] Time since sensor is powered up us.
 */
static inline uint32_t mavlink_msg_sbg_ekf_euler_get_time_stamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field angle from sbg_ekf_euler message
 *
 * @return [rad] [Roll, Pitch, Yaw (heading)] angle rad.
 */
static inline uint16_t mavlink_msg_sbg_ekf_euler_get_angle(const mavlink_message_t* msg, float *angle)
{
    return _MAV_RETURN_float_array(msg, angle, 3,  12);
}

/**
 * @brief Get field accuracy from sbg_ekf_euler message
 *
 * @return [rad] [Roll, Pitch, Yaw (heading)] angle accuracy (1 sigma) rad.
 */
static inline uint16_t mavlink_msg_sbg_ekf_euler_get_accuracy(const mavlink_message_t* msg, float *accuracy)
{
    return _MAV_RETURN_float_array(msg, accuracy, 3,  24);
}

/**
 * @brief Get field solution_mode from sbg_ekf_euler message
 *
 * @return  Defines the Kalman filter computation mode (see the table 4 below). 0 UNINITIALIZED The Kalman filter is not initialized and the returned data are all invalid. 1 VERTICAL_GYRO The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely. 2 AHRS A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely. 3 NAV_VELOCITY The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation. 4 NAV_POSITION Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided.
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_solution_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field attitude_valid from sbg_ekf_euler message
 *
 * @return  True if Attitude data is reliable (Roll/Pitch error less than 0,5 deg).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_attitude_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field heading_valid from sbg_ekf_euler message
 *
 * @return  True if Heading data is reliable (Heading error less than 1 deg).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_heading_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field velocity_valid from sbg_ekf_euler message
 *
 * @return  True if Velocity data is reliable (velocity error less than 1.5 m/s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_velocity_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field position_valid from sbg_ekf_euler message
 *
 * @return  True if Position data is reliable (Position error less than 10m).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_position_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field vert_ref_used from sbg_ekf_euler message
 *
 * @return  True if vertical reference is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_vert_ref_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field mag_ref_used from sbg_ekf_euler message
 *
 * @return  True if magnetometer is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_mag_ref_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field gps1_vel_used from sbg_ekf_euler message
 *
 * @return  True if GPS velocity is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_gps1_vel_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field gps1_pos_used from sbg_ekf_euler message
 *
 * @return  True if GPS Position is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_gps1_pos_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Get field gps1_course_used from sbg_ekf_euler message
 *
 * @return  True if GPS Course is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_gps1_course_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  45);
}

/**
 * @brief Get field gps1_hdt_used from sbg_ekf_euler message
 *
 * @return  True if GPS True Heading is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_gps1_hdt_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  46);
}

/**
 * @brief Get field gps2_vel_used from sbg_ekf_euler message
 *
 * @return  True if GPS2 velocity is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_gps2_vel_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  47);
}

/**
 * @brief Get field gps2_pos_used from sbg_ekf_euler message
 *
 * @return  True if GPS2 Position is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_gps2_pos_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field gps2_course_used from sbg_ekf_euler message
 *
 * @return  True if GPS2 Course is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_gps2_course_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field gps2_hdt_used from sbg_ekf_euler message
 *
 * @return  True if GPS2 True Heading is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_gps2_hdt_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field odo_used from sbg_ekf_euler message
 *
 * @return  True if Odometer is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_euler_get_odo_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Decode a sbg_ekf_euler message into a struct
 *
 * @param msg The message to decode
 * @param sbg_ekf_euler C-struct to decode the message contents into
 */
static inline void mavlink_msg_sbg_ekf_euler_decode(const mavlink_message_t* msg, mavlink_sbg_ekf_euler_t* sbg_ekf_euler)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sbg_ekf_euler->timestamp = mavlink_msg_sbg_ekf_euler_get_timestamp(msg);
    sbg_ekf_euler->time_stamp = mavlink_msg_sbg_ekf_euler_get_time_stamp(msg);
    mavlink_msg_sbg_ekf_euler_get_angle(msg, sbg_ekf_euler->angle);
    mavlink_msg_sbg_ekf_euler_get_accuracy(msg, sbg_ekf_euler->accuracy);
    sbg_ekf_euler->solution_mode = mavlink_msg_sbg_ekf_euler_get_solution_mode(msg);
    sbg_ekf_euler->attitude_valid = mavlink_msg_sbg_ekf_euler_get_attitude_valid(msg);
    sbg_ekf_euler->heading_valid = mavlink_msg_sbg_ekf_euler_get_heading_valid(msg);
    sbg_ekf_euler->velocity_valid = mavlink_msg_sbg_ekf_euler_get_velocity_valid(msg);
    sbg_ekf_euler->position_valid = mavlink_msg_sbg_ekf_euler_get_position_valid(msg);
    sbg_ekf_euler->vert_ref_used = mavlink_msg_sbg_ekf_euler_get_vert_ref_used(msg);
    sbg_ekf_euler->mag_ref_used = mavlink_msg_sbg_ekf_euler_get_mag_ref_used(msg);
    sbg_ekf_euler->gps1_vel_used = mavlink_msg_sbg_ekf_euler_get_gps1_vel_used(msg);
    sbg_ekf_euler->gps1_pos_used = mavlink_msg_sbg_ekf_euler_get_gps1_pos_used(msg);
    sbg_ekf_euler->gps1_course_used = mavlink_msg_sbg_ekf_euler_get_gps1_course_used(msg);
    sbg_ekf_euler->gps1_hdt_used = mavlink_msg_sbg_ekf_euler_get_gps1_hdt_used(msg);
    sbg_ekf_euler->gps2_vel_used = mavlink_msg_sbg_ekf_euler_get_gps2_vel_used(msg);
    sbg_ekf_euler->gps2_pos_used = mavlink_msg_sbg_ekf_euler_get_gps2_pos_used(msg);
    sbg_ekf_euler->gps2_course_used = mavlink_msg_sbg_ekf_euler_get_gps2_course_used(msg);
    sbg_ekf_euler->gps2_hdt_used = mavlink_msg_sbg_ekf_euler_get_gps2_hdt_used(msg);
    sbg_ekf_euler->odo_used = mavlink_msg_sbg_ekf_euler_get_odo_used(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SBG_EKF_EULER_LEN? msg->len : MAVLINK_MSG_ID_SBG_EKF_EULER_LEN;
        memset(sbg_ekf_euler, 0, MAVLINK_MSG_ID_SBG_EKF_EULER_LEN);
    memcpy(sbg_ekf_euler, _MAV_PAYLOAD(msg), len);
#endif
}
