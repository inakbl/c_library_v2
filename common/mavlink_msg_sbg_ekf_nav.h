#pragma once
// MESSAGE SBG_EKF_NAV PACKING

#define MAVLINK_MSG_ID_SBG_EKF_NAV 632

MAVPACKED(
typedef struct __mavlink_sbg_ekf_nav_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 uint32_t time_stamp; /*< [us] Time since sensor is powered up us.*/
 float velocity[3]; /*< [m/s] Velocity (North, East, Down) direction m/s.*/
 float velocity_accuracy[3]; /*< [m/s] 1 sigma Velocity in (North, East, Down) direction accuracy m/s.*/
 float position[3]; /*< [deg] [Latitude (deg), Longitude (deg), Altitude (above Mean Sea Level in meters)].*/
 float undulation; /*<  Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude) (Height above Ellipsoid = altitude + undulation).*/
 float position_accuracy[3]; /*< [m] Latitude, Longitude, Vertical accuracy m (1 sigma).*/
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
}) mavlink_sbg_ekf_nav_t;

#define MAVLINK_MSG_ID_SBG_EKF_NAV_LEN 80
#define MAVLINK_MSG_ID_SBG_EKF_NAV_MIN_LEN 80
#define MAVLINK_MSG_ID_632_LEN 80
#define MAVLINK_MSG_ID_632_MIN_LEN 80

#define MAVLINK_MSG_ID_SBG_EKF_NAV_CRC 125
#define MAVLINK_MSG_ID_632_CRC 125

#define MAVLINK_MSG_SBG_EKF_NAV_FIELD_VELOCITY_LEN 3
#define MAVLINK_MSG_SBG_EKF_NAV_FIELD_VELOCITY_ACCURACY_LEN 3
#define MAVLINK_MSG_SBG_EKF_NAV_FIELD_POSITION_LEN 3
#define MAVLINK_MSG_SBG_EKF_NAV_FIELD_POSITION_ACCURACY_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SBG_EKF_NAV { \
    632, \
    "SBG_EKF_NAV", \
    23, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_ekf_nav_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_ekf_nav_t, time_stamp) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_sbg_ekf_nav_t, velocity) }, \
         { "velocity_accuracy", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_sbg_ekf_nav_t, velocity_accuracy) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_sbg_ekf_nav_t, position) }, \
         { "undulation", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_sbg_ekf_nav_t, undulation) }, \
         { "position_accuracy", NULL, MAVLINK_TYPE_FLOAT, 3, 52, offsetof(mavlink_sbg_ekf_nav_t, position_accuracy) }, \
         { "solution_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_sbg_ekf_nav_t, solution_mode) }, \
         { "attitude_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 65, offsetof(mavlink_sbg_ekf_nav_t, attitude_valid) }, \
         { "heading_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 66, offsetof(mavlink_sbg_ekf_nav_t, heading_valid) }, \
         { "velocity_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 67, offsetof(mavlink_sbg_ekf_nav_t, velocity_valid) }, \
         { "position_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 68, offsetof(mavlink_sbg_ekf_nav_t, position_valid) }, \
         { "vert_ref_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 69, offsetof(mavlink_sbg_ekf_nav_t, vert_ref_used) }, \
         { "mag_ref_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_sbg_ekf_nav_t, mag_ref_used) }, \
         { "gps1_vel_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 71, offsetof(mavlink_sbg_ekf_nav_t, gps1_vel_used) }, \
         { "gps1_pos_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_sbg_ekf_nav_t, gps1_pos_used) }, \
         { "gps1_course_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 73, offsetof(mavlink_sbg_ekf_nav_t, gps1_course_used) }, \
         { "gps1_hdt_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 74, offsetof(mavlink_sbg_ekf_nav_t, gps1_hdt_used) }, \
         { "gps2_vel_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 75, offsetof(mavlink_sbg_ekf_nav_t, gps2_vel_used) }, \
         { "gps2_pos_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 76, offsetof(mavlink_sbg_ekf_nav_t, gps2_pos_used) }, \
         { "gps2_course_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 77, offsetof(mavlink_sbg_ekf_nav_t, gps2_course_used) }, \
         { "gps2_hdt_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 78, offsetof(mavlink_sbg_ekf_nav_t, gps2_hdt_used) }, \
         { "odo_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 79, offsetof(mavlink_sbg_ekf_nav_t, odo_used) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SBG_EKF_NAV { \
    "SBG_EKF_NAV", \
    23, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_ekf_nav_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_ekf_nav_t, time_stamp) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_sbg_ekf_nav_t, velocity) }, \
         { "velocity_accuracy", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_sbg_ekf_nav_t, velocity_accuracy) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_sbg_ekf_nav_t, position) }, \
         { "undulation", NULL, MAVLINK_TYPE_FLOAT, 0, 48, offsetof(mavlink_sbg_ekf_nav_t, undulation) }, \
         { "position_accuracy", NULL, MAVLINK_TYPE_FLOAT, 3, 52, offsetof(mavlink_sbg_ekf_nav_t, position_accuracy) }, \
         { "solution_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_sbg_ekf_nav_t, solution_mode) }, \
         { "attitude_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 65, offsetof(mavlink_sbg_ekf_nav_t, attitude_valid) }, \
         { "heading_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 66, offsetof(mavlink_sbg_ekf_nav_t, heading_valid) }, \
         { "velocity_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 67, offsetof(mavlink_sbg_ekf_nav_t, velocity_valid) }, \
         { "position_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 68, offsetof(mavlink_sbg_ekf_nav_t, position_valid) }, \
         { "vert_ref_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 69, offsetof(mavlink_sbg_ekf_nav_t, vert_ref_used) }, \
         { "mag_ref_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_sbg_ekf_nav_t, mag_ref_used) }, \
         { "gps1_vel_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 71, offsetof(mavlink_sbg_ekf_nav_t, gps1_vel_used) }, \
         { "gps1_pos_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_sbg_ekf_nav_t, gps1_pos_used) }, \
         { "gps1_course_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 73, offsetof(mavlink_sbg_ekf_nav_t, gps1_course_used) }, \
         { "gps1_hdt_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 74, offsetof(mavlink_sbg_ekf_nav_t, gps1_hdt_used) }, \
         { "gps2_vel_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 75, offsetof(mavlink_sbg_ekf_nav_t, gps2_vel_used) }, \
         { "gps2_pos_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 76, offsetof(mavlink_sbg_ekf_nav_t, gps2_pos_used) }, \
         { "gps2_course_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 77, offsetof(mavlink_sbg_ekf_nav_t, gps2_course_used) }, \
         { "gps2_hdt_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 78, offsetof(mavlink_sbg_ekf_nav_t, gps2_hdt_used) }, \
         { "odo_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 79, offsetof(mavlink_sbg_ekf_nav_t, odo_used) }, \
         } \
}
#endif

/**
 * @brief Pack a sbg_ekf_nav message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param velocity [m/s] Velocity (North, East, Down) direction m/s.
 * @param velocity_accuracy [m/s] 1 sigma Velocity in (North, East, Down) direction accuracy m/s.
 * @param position [deg] [Latitude (deg), Longitude (deg), Altitude (above Mean Sea Level in meters)].
 * @param undulation  Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude) (Height above Ellipsoid = altitude + undulation).
 * @param position_accuracy [m] Latitude, Longitude, Vertical accuracy m (1 sigma).
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
static inline uint16_t mavlink_msg_sbg_ekf_nav_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t time_stamp, const float *velocity, const float *velocity_accuracy, const float *position, float undulation, const float *position_accuracy, uint8_t solution_mode, uint8_t attitude_valid, uint8_t heading_valid, uint8_t velocity_valid, uint8_t position_valid, uint8_t vert_ref_used, uint8_t mag_ref_used, uint8_t gps1_vel_used, uint8_t gps1_pos_used, uint8_t gps1_course_used, uint8_t gps1_hdt_used, uint8_t gps2_vel_used, uint8_t gps2_pos_used, uint8_t gps2_course_used, uint8_t gps2_hdt_used, uint8_t odo_used)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_EKF_NAV_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_float(buf, 48, undulation);
    _mav_put_uint8_t(buf, 64, solution_mode);
    _mav_put_uint8_t(buf, 65, attitude_valid);
    _mav_put_uint8_t(buf, 66, heading_valid);
    _mav_put_uint8_t(buf, 67, velocity_valid);
    _mav_put_uint8_t(buf, 68, position_valid);
    _mav_put_uint8_t(buf, 69, vert_ref_used);
    _mav_put_uint8_t(buf, 70, mag_ref_used);
    _mav_put_uint8_t(buf, 71, gps1_vel_used);
    _mav_put_uint8_t(buf, 72, gps1_pos_used);
    _mav_put_uint8_t(buf, 73, gps1_course_used);
    _mav_put_uint8_t(buf, 74, gps1_hdt_used);
    _mav_put_uint8_t(buf, 75, gps2_vel_used);
    _mav_put_uint8_t(buf, 76, gps2_pos_used);
    _mav_put_uint8_t(buf, 77, gps2_course_used);
    _mav_put_uint8_t(buf, 78, gps2_hdt_used);
    _mav_put_uint8_t(buf, 79, odo_used);
    _mav_put_float_array(buf, 12, velocity, 3);
    _mav_put_float_array(buf, 24, velocity_accuracy, 3);
    _mav_put_float_array(buf, 36, position, 3);
    _mav_put_float_array(buf, 52, position_accuracy, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_EKF_NAV_LEN);
#else
    mavlink_sbg_ekf_nav_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.undulation = undulation;
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
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet.velocity_accuracy, velocity_accuracy, sizeof(float)*3);
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.position_accuracy, position_accuracy, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_EKF_NAV_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_EKF_NAV;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SBG_EKF_NAV_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_CRC);
}

/**
 * @brief Pack a sbg_ekf_nav message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param velocity [m/s] Velocity (North, East, Down) direction m/s.
 * @param velocity_accuracy [m/s] 1 sigma Velocity in (North, East, Down) direction accuracy m/s.
 * @param position [deg] [Latitude (deg), Longitude (deg), Altitude (above Mean Sea Level in meters)].
 * @param undulation  Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude) (Height above Ellipsoid = altitude + undulation).
 * @param position_accuracy [m] Latitude, Longitude, Vertical accuracy m (1 sigma).
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
static inline uint16_t mavlink_msg_sbg_ekf_nav_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t time_stamp,const float *velocity,const float *velocity_accuracy,const float *position,float undulation,const float *position_accuracy,uint8_t solution_mode,uint8_t attitude_valid,uint8_t heading_valid,uint8_t velocity_valid,uint8_t position_valid,uint8_t vert_ref_used,uint8_t mag_ref_used,uint8_t gps1_vel_used,uint8_t gps1_pos_used,uint8_t gps1_course_used,uint8_t gps1_hdt_used,uint8_t gps2_vel_used,uint8_t gps2_pos_used,uint8_t gps2_course_used,uint8_t gps2_hdt_used,uint8_t odo_used)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_EKF_NAV_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_float(buf, 48, undulation);
    _mav_put_uint8_t(buf, 64, solution_mode);
    _mav_put_uint8_t(buf, 65, attitude_valid);
    _mav_put_uint8_t(buf, 66, heading_valid);
    _mav_put_uint8_t(buf, 67, velocity_valid);
    _mav_put_uint8_t(buf, 68, position_valid);
    _mav_put_uint8_t(buf, 69, vert_ref_used);
    _mav_put_uint8_t(buf, 70, mag_ref_used);
    _mav_put_uint8_t(buf, 71, gps1_vel_used);
    _mav_put_uint8_t(buf, 72, gps1_pos_used);
    _mav_put_uint8_t(buf, 73, gps1_course_used);
    _mav_put_uint8_t(buf, 74, gps1_hdt_used);
    _mav_put_uint8_t(buf, 75, gps2_vel_used);
    _mav_put_uint8_t(buf, 76, gps2_pos_used);
    _mav_put_uint8_t(buf, 77, gps2_course_used);
    _mav_put_uint8_t(buf, 78, gps2_hdt_used);
    _mav_put_uint8_t(buf, 79, odo_used);
    _mav_put_float_array(buf, 12, velocity, 3);
    _mav_put_float_array(buf, 24, velocity_accuracy, 3);
    _mav_put_float_array(buf, 36, position, 3);
    _mav_put_float_array(buf, 52, position_accuracy, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_EKF_NAV_LEN);
#else
    mavlink_sbg_ekf_nav_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.undulation = undulation;
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
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet.velocity_accuracy, velocity_accuracy, sizeof(float)*3);
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.position_accuracy, position_accuracy, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_EKF_NAV_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_EKF_NAV;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SBG_EKF_NAV_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_CRC);
}

/**
 * @brief Encode a sbg_ekf_nav struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbg_ekf_nav C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_ekf_nav_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbg_ekf_nav_t* sbg_ekf_nav)
{
    return mavlink_msg_sbg_ekf_nav_pack(system_id, component_id, msg, sbg_ekf_nav->timestamp, sbg_ekf_nav->time_stamp, sbg_ekf_nav->velocity, sbg_ekf_nav->velocity_accuracy, sbg_ekf_nav->position, sbg_ekf_nav->undulation, sbg_ekf_nav->position_accuracy, sbg_ekf_nav->solution_mode, sbg_ekf_nav->attitude_valid, sbg_ekf_nav->heading_valid, sbg_ekf_nav->velocity_valid, sbg_ekf_nav->position_valid, sbg_ekf_nav->vert_ref_used, sbg_ekf_nav->mag_ref_used, sbg_ekf_nav->gps1_vel_used, sbg_ekf_nav->gps1_pos_used, sbg_ekf_nav->gps1_course_used, sbg_ekf_nav->gps1_hdt_used, sbg_ekf_nav->gps2_vel_used, sbg_ekf_nav->gps2_pos_used, sbg_ekf_nav->gps2_course_used, sbg_ekf_nav->gps2_hdt_used, sbg_ekf_nav->odo_used);
}

/**
 * @brief Encode a sbg_ekf_nav struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sbg_ekf_nav C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_ekf_nav_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sbg_ekf_nav_t* sbg_ekf_nav)
{
    return mavlink_msg_sbg_ekf_nav_pack_chan(system_id, component_id, chan, msg, sbg_ekf_nav->timestamp, sbg_ekf_nav->time_stamp, sbg_ekf_nav->velocity, sbg_ekf_nav->velocity_accuracy, sbg_ekf_nav->position, sbg_ekf_nav->undulation, sbg_ekf_nav->position_accuracy, sbg_ekf_nav->solution_mode, sbg_ekf_nav->attitude_valid, sbg_ekf_nav->heading_valid, sbg_ekf_nav->velocity_valid, sbg_ekf_nav->position_valid, sbg_ekf_nav->vert_ref_used, sbg_ekf_nav->mag_ref_used, sbg_ekf_nav->gps1_vel_used, sbg_ekf_nav->gps1_pos_used, sbg_ekf_nav->gps1_course_used, sbg_ekf_nav->gps1_hdt_used, sbg_ekf_nav->gps2_vel_used, sbg_ekf_nav->gps2_pos_used, sbg_ekf_nav->gps2_course_used, sbg_ekf_nav->gps2_hdt_used, sbg_ekf_nav->odo_used);
}

/**
 * @brief Send a sbg_ekf_nav message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param velocity [m/s] Velocity (North, East, Down) direction m/s.
 * @param velocity_accuracy [m/s] 1 sigma Velocity in (North, East, Down) direction accuracy m/s.
 * @param position [deg] [Latitude (deg), Longitude (deg), Altitude (above Mean Sea Level in meters)].
 * @param undulation  Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude) (Height above Ellipsoid = altitude + undulation).
 * @param position_accuracy [m] Latitude, Longitude, Vertical accuracy m (1 sigma).
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

static inline void mavlink_msg_sbg_ekf_nav_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t time_stamp, const float *velocity, const float *velocity_accuracy, const float *position, float undulation, const float *position_accuracy, uint8_t solution_mode, uint8_t attitude_valid, uint8_t heading_valid, uint8_t velocity_valid, uint8_t position_valid, uint8_t vert_ref_used, uint8_t mag_ref_used, uint8_t gps1_vel_used, uint8_t gps1_pos_used, uint8_t gps1_course_used, uint8_t gps1_hdt_used, uint8_t gps2_vel_used, uint8_t gps2_pos_used, uint8_t gps2_course_used, uint8_t gps2_hdt_used, uint8_t odo_used)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_EKF_NAV_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_float(buf, 48, undulation);
    _mav_put_uint8_t(buf, 64, solution_mode);
    _mav_put_uint8_t(buf, 65, attitude_valid);
    _mav_put_uint8_t(buf, 66, heading_valid);
    _mav_put_uint8_t(buf, 67, velocity_valid);
    _mav_put_uint8_t(buf, 68, position_valid);
    _mav_put_uint8_t(buf, 69, vert_ref_used);
    _mav_put_uint8_t(buf, 70, mag_ref_used);
    _mav_put_uint8_t(buf, 71, gps1_vel_used);
    _mav_put_uint8_t(buf, 72, gps1_pos_used);
    _mav_put_uint8_t(buf, 73, gps1_course_used);
    _mav_put_uint8_t(buf, 74, gps1_hdt_used);
    _mav_put_uint8_t(buf, 75, gps2_vel_used);
    _mav_put_uint8_t(buf, 76, gps2_pos_used);
    _mav_put_uint8_t(buf, 77, gps2_course_used);
    _mav_put_uint8_t(buf, 78, gps2_hdt_used);
    _mav_put_uint8_t(buf, 79, odo_used);
    _mav_put_float_array(buf, 12, velocity, 3);
    _mav_put_float_array(buf, 24, velocity_accuracy, 3);
    _mav_put_float_array(buf, 36, position, 3);
    _mav_put_float_array(buf, 52, position_accuracy, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_EKF_NAV, buf, MAVLINK_MSG_ID_SBG_EKF_NAV_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_CRC);
#else
    mavlink_sbg_ekf_nav_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.undulation = undulation;
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
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet.velocity_accuracy, velocity_accuracy, sizeof(float)*3);
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.position_accuracy, position_accuracy, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_EKF_NAV, (const char *)&packet, MAVLINK_MSG_ID_SBG_EKF_NAV_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_CRC);
#endif
}

/**
 * @brief Send a sbg_ekf_nav message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sbg_ekf_nav_send_struct(mavlink_channel_t chan, const mavlink_sbg_ekf_nav_t* sbg_ekf_nav)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sbg_ekf_nav_send(chan, sbg_ekf_nav->timestamp, sbg_ekf_nav->time_stamp, sbg_ekf_nav->velocity, sbg_ekf_nav->velocity_accuracy, sbg_ekf_nav->position, sbg_ekf_nav->undulation, sbg_ekf_nav->position_accuracy, sbg_ekf_nav->solution_mode, sbg_ekf_nav->attitude_valid, sbg_ekf_nav->heading_valid, sbg_ekf_nav->velocity_valid, sbg_ekf_nav->position_valid, sbg_ekf_nav->vert_ref_used, sbg_ekf_nav->mag_ref_used, sbg_ekf_nav->gps1_vel_used, sbg_ekf_nav->gps1_pos_used, sbg_ekf_nav->gps1_course_used, sbg_ekf_nav->gps1_hdt_used, sbg_ekf_nav->gps2_vel_used, sbg_ekf_nav->gps2_pos_used, sbg_ekf_nav->gps2_course_used, sbg_ekf_nav->gps2_hdt_used, sbg_ekf_nav->odo_used);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_EKF_NAV, (const char *)sbg_ekf_nav, MAVLINK_MSG_ID_SBG_EKF_NAV_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_CRC);
#endif
}

#if MAVLINK_MSG_ID_SBG_EKF_NAV_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sbg_ekf_nav_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t time_stamp, const float *velocity, const float *velocity_accuracy, const float *position, float undulation, const float *position_accuracy, uint8_t solution_mode, uint8_t attitude_valid, uint8_t heading_valid, uint8_t velocity_valid, uint8_t position_valid, uint8_t vert_ref_used, uint8_t mag_ref_used, uint8_t gps1_vel_used, uint8_t gps1_pos_used, uint8_t gps1_course_used, uint8_t gps1_hdt_used, uint8_t gps2_vel_used, uint8_t gps2_pos_used, uint8_t gps2_course_used, uint8_t gps2_hdt_used, uint8_t odo_used)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_float(buf, 48, undulation);
    _mav_put_uint8_t(buf, 64, solution_mode);
    _mav_put_uint8_t(buf, 65, attitude_valid);
    _mav_put_uint8_t(buf, 66, heading_valid);
    _mav_put_uint8_t(buf, 67, velocity_valid);
    _mav_put_uint8_t(buf, 68, position_valid);
    _mav_put_uint8_t(buf, 69, vert_ref_used);
    _mav_put_uint8_t(buf, 70, mag_ref_used);
    _mav_put_uint8_t(buf, 71, gps1_vel_used);
    _mav_put_uint8_t(buf, 72, gps1_pos_used);
    _mav_put_uint8_t(buf, 73, gps1_course_used);
    _mav_put_uint8_t(buf, 74, gps1_hdt_used);
    _mav_put_uint8_t(buf, 75, gps2_vel_used);
    _mav_put_uint8_t(buf, 76, gps2_pos_used);
    _mav_put_uint8_t(buf, 77, gps2_course_used);
    _mav_put_uint8_t(buf, 78, gps2_hdt_used);
    _mav_put_uint8_t(buf, 79, odo_used);
    _mav_put_float_array(buf, 12, velocity, 3);
    _mav_put_float_array(buf, 24, velocity_accuracy, 3);
    _mav_put_float_array(buf, 36, position, 3);
    _mav_put_float_array(buf, 52, position_accuracy, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_EKF_NAV, buf, MAVLINK_MSG_ID_SBG_EKF_NAV_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_CRC);
#else
    mavlink_sbg_ekf_nav_t *packet = (mavlink_sbg_ekf_nav_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->time_stamp = time_stamp;
    packet->undulation = undulation;
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
    mav_array_memcpy(packet->velocity, velocity, sizeof(float)*3);
    mav_array_memcpy(packet->velocity_accuracy, velocity_accuracy, sizeof(float)*3);
    mav_array_memcpy(packet->position, position, sizeof(float)*3);
    mav_array_memcpy(packet->position_accuracy, position_accuracy, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_EKF_NAV, (const char *)packet, MAVLINK_MSG_ID_SBG_EKF_NAV_MIN_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_LEN, MAVLINK_MSG_ID_SBG_EKF_NAV_CRC);
#endif
}
#endif

#endif

// MESSAGE SBG_EKF_NAV UNPACKING


/**
 * @brief Get field timestamp from sbg_ekf_nav message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_sbg_ekf_nav_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_stamp from sbg_ekf_nav message
 *
 * @return [us] Time since sensor is powered up us.
 */
static inline uint32_t mavlink_msg_sbg_ekf_nav_get_time_stamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field velocity from sbg_ekf_nav message
 *
 * @return [m/s] Velocity (North, East, Down) direction m/s.
 */
static inline uint16_t mavlink_msg_sbg_ekf_nav_get_velocity(const mavlink_message_t* msg, float *velocity)
{
    return _MAV_RETURN_float_array(msg, velocity, 3,  12);
}

/**
 * @brief Get field velocity_accuracy from sbg_ekf_nav message
 *
 * @return [m/s] 1 sigma Velocity in (North, East, Down) direction accuracy m/s.
 */
static inline uint16_t mavlink_msg_sbg_ekf_nav_get_velocity_accuracy(const mavlink_message_t* msg, float *velocity_accuracy)
{
    return _MAV_RETURN_float_array(msg, velocity_accuracy, 3,  24);
}

/**
 * @brief Get field position from sbg_ekf_nav message
 *
 * @return [deg] [Latitude (deg), Longitude (deg), Altitude (above Mean Sea Level in meters)].
 */
static inline uint16_t mavlink_msg_sbg_ekf_nav_get_position(const mavlink_message_t* msg, float *position)
{
    return _MAV_RETURN_float_array(msg, position, 3,  36);
}

/**
 * @brief Get field undulation from sbg_ekf_nav message
 *
 * @return  Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude) (Height above Ellipsoid = altitude + undulation).
 */
static inline float mavlink_msg_sbg_ekf_nav_get_undulation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  48);
}

/**
 * @brief Get field position_accuracy from sbg_ekf_nav message
 *
 * @return [m] Latitude, Longitude, Vertical accuracy m (1 sigma).
 */
static inline uint16_t mavlink_msg_sbg_ekf_nav_get_position_accuracy(const mavlink_message_t* msg, float *position_accuracy)
{
    return _MAV_RETURN_float_array(msg, position_accuracy, 3,  52);
}

/**
 * @brief Get field solution_mode from sbg_ekf_nav message
 *
 * @return  Defines the Kalman filter computation mode (see the table 4 below). 0 UNINITIALIZED The Kalman filter is not initialized and the returned data are all invalid. 1 VERTICAL_GYRO The Kalman filter only rely on a vertical reference to compute roll and pitch angles. Heading and navigation data drift freely. 2 AHRS A heading reference is available, the Kalman filter provides full orientation but navigation data drift freely. 3 NAV_VELOCITY The Kalman filter computes orientation and velocity. Position is freely integrated from velocity estimation. 4 NAV_POSITION Nominal mode, the Kalman filter computes all parameters (attitude, velocity, position). Absolute position is provided.
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_solution_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  64);
}

/**
 * @brief Get field attitude_valid from sbg_ekf_nav message
 *
 * @return  True if Attitude data is reliable (Roll/Pitch error less than 0,5 deg).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_attitude_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  65);
}

/**
 * @brief Get field heading_valid from sbg_ekf_nav message
 *
 * @return  True if Heading data is reliable (Heading error less than 1 deg).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_heading_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  66);
}

/**
 * @brief Get field velocity_valid from sbg_ekf_nav message
 *
 * @return  True if Velocity data is reliable (velocity error less than 1.5 m/s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_velocity_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  67);
}

/**
 * @brief Get field position_valid from sbg_ekf_nav message
 *
 * @return  True if Position data is reliable (Position error less than 10m).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_position_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  68);
}

/**
 * @brief Get field vert_ref_used from sbg_ekf_nav message
 *
 * @return  True if vertical reference is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_vert_ref_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  69);
}

/**
 * @brief Get field mag_ref_used from sbg_ekf_nav message
 *
 * @return  True if magnetometer is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_mag_ref_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  70);
}

/**
 * @brief Get field gps1_vel_used from sbg_ekf_nav message
 *
 * @return  True if GPS velocity is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_gps1_vel_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  71);
}

/**
 * @brief Get field gps1_pos_used from sbg_ekf_nav message
 *
 * @return  True if GPS Position is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_gps1_pos_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  72);
}

/**
 * @brief Get field gps1_course_used from sbg_ekf_nav message
 *
 * @return  True if GPS Course is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_gps1_course_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  73);
}

/**
 * @brief Get field gps1_hdt_used from sbg_ekf_nav message
 *
 * @return  True if GPS True Heading is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_gps1_hdt_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  74);
}

/**
 * @brief Get field gps2_vel_used from sbg_ekf_nav message
 *
 * @return  True if GPS2 velocity is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_gps2_vel_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  75);
}

/**
 * @brief Get field gps2_pos_used from sbg_ekf_nav message
 *
 * @return  True if GPS2 Position is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_gps2_pos_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  76);
}

/**
 * @brief Get field gps2_course_used from sbg_ekf_nav message
 *
 * @return  True if GPS2 Course is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_gps2_course_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  77);
}

/**
 * @brief Get field gps2_hdt_used from sbg_ekf_nav message
 *
 * @return  True if GPS2 True Heading is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_gps2_hdt_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  78);
}

/**
 * @brief Get field odo_used from sbg_ekf_nav message
 *
 * @return  True if Odometer is used in solution (data used and valid since 3s).
 */
static inline uint8_t mavlink_msg_sbg_ekf_nav_get_odo_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  79);
}

/**
 * @brief Decode a sbg_ekf_nav message into a struct
 *
 * @param msg The message to decode
 * @param sbg_ekf_nav C-struct to decode the message contents into
 */
static inline void mavlink_msg_sbg_ekf_nav_decode(const mavlink_message_t* msg, mavlink_sbg_ekf_nav_t* sbg_ekf_nav)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sbg_ekf_nav->timestamp = mavlink_msg_sbg_ekf_nav_get_timestamp(msg);
    sbg_ekf_nav->time_stamp = mavlink_msg_sbg_ekf_nav_get_time_stamp(msg);
    mavlink_msg_sbg_ekf_nav_get_velocity(msg, sbg_ekf_nav->velocity);
    mavlink_msg_sbg_ekf_nav_get_velocity_accuracy(msg, sbg_ekf_nav->velocity_accuracy);
    mavlink_msg_sbg_ekf_nav_get_position(msg, sbg_ekf_nav->position);
    sbg_ekf_nav->undulation = mavlink_msg_sbg_ekf_nav_get_undulation(msg);
    mavlink_msg_sbg_ekf_nav_get_position_accuracy(msg, sbg_ekf_nav->position_accuracy);
    sbg_ekf_nav->solution_mode = mavlink_msg_sbg_ekf_nav_get_solution_mode(msg);
    sbg_ekf_nav->attitude_valid = mavlink_msg_sbg_ekf_nav_get_attitude_valid(msg);
    sbg_ekf_nav->heading_valid = mavlink_msg_sbg_ekf_nav_get_heading_valid(msg);
    sbg_ekf_nav->velocity_valid = mavlink_msg_sbg_ekf_nav_get_velocity_valid(msg);
    sbg_ekf_nav->position_valid = mavlink_msg_sbg_ekf_nav_get_position_valid(msg);
    sbg_ekf_nav->vert_ref_used = mavlink_msg_sbg_ekf_nav_get_vert_ref_used(msg);
    sbg_ekf_nav->mag_ref_used = mavlink_msg_sbg_ekf_nav_get_mag_ref_used(msg);
    sbg_ekf_nav->gps1_vel_used = mavlink_msg_sbg_ekf_nav_get_gps1_vel_used(msg);
    sbg_ekf_nav->gps1_pos_used = mavlink_msg_sbg_ekf_nav_get_gps1_pos_used(msg);
    sbg_ekf_nav->gps1_course_used = mavlink_msg_sbg_ekf_nav_get_gps1_course_used(msg);
    sbg_ekf_nav->gps1_hdt_used = mavlink_msg_sbg_ekf_nav_get_gps1_hdt_used(msg);
    sbg_ekf_nav->gps2_vel_used = mavlink_msg_sbg_ekf_nav_get_gps2_vel_used(msg);
    sbg_ekf_nav->gps2_pos_used = mavlink_msg_sbg_ekf_nav_get_gps2_pos_used(msg);
    sbg_ekf_nav->gps2_course_used = mavlink_msg_sbg_ekf_nav_get_gps2_course_used(msg);
    sbg_ekf_nav->gps2_hdt_used = mavlink_msg_sbg_ekf_nav_get_gps2_hdt_used(msg);
    sbg_ekf_nav->odo_used = mavlink_msg_sbg_ekf_nav_get_odo_used(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SBG_EKF_NAV_LEN? msg->len : MAVLINK_MSG_ID_SBG_EKF_NAV_LEN;
        memset(sbg_ekf_nav, 0, MAVLINK_MSG_ID_SBG_EKF_NAV_LEN);
    memcpy(sbg_ekf_nav, _MAV_PAYLOAD(msg), len);
#endif
}
