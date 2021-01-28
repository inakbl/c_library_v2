#pragma once
// MESSAGE SBG_IMU_DATA PACKING

#define MAVLINK_MSG_ID_SBG_IMU_DATA 630


typedef struct __mavlink_sbg_imu_data_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 uint32_t time_stamp; /*< [us] Time since sensor is powered up us.*/
 float accel[3]; /*< [m/s2] Filtered Accelerometer - X, Y, Z axis m/s2.*/
 float gyro[3]; /*< [rad/s] Filtered Gyroscope - X, Y, Z axis rad/s.*/
 float temp; /*<  Internal Temperature degC.*/
 float delta_vel[3]; /*< [m/s2] Sculling output - X, Y, Z axis m/s2.*/
 float delta_angle[3]; /*< [rad/s] Coning output - X, Y, Z axis rad/s.*/
 uint8_t imu_com; /*<  True if the communication with the IMU is ok..*/
 uint8_t imu_status; /*<  True if internal IMU passes Built In Test (Calibration, CPU).*/
 uint8_t imu_accel_x; /*<  True if accelerometer X passes Built In Test.*/
 uint8_t imu_accel_y; /*<  True if accelerometer Y passes Built In Test.*/
 uint8_t imu_accel_z; /*<  True if accelerometer Z passes Built In Test.*/
 uint8_t imu_gyro_x; /*<  True if gyroscope X passes Built In Test.*/
 uint8_t imu_gyro_y; /*<  True if gyroscope Y passes Built In Test.*/
 uint8_t imu_gyro_z; /*<  True if gyroscope Z passes Built In Test.*/
 uint8_t imu_accels_in_range; /*<  True if accelerometers are within operating range.*/
 uint8_t imu_gyros_in_range; /*<  True if gyroscopes are within operating range.*/
} mavlink_sbg_imu_data_t;

#define MAVLINK_MSG_ID_SBG_IMU_DATA_LEN 74
#define MAVLINK_MSG_ID_SBG_IMU_DATA_MIN_LEN 74
#define MAVLINK_MSG_ID_630_LEN 74
#define MAVLINK_MSG_ID_630_MIN_LEN 74

#define MAVLINK_MSG_ID_SBG_IMU_DATA_CRC 27
#define MAVLINK_MSG_ID_630_CRC 27

#define MAVLINK_MSG_SBG_IMU_DATA_FIELD_ACCEL_LEN 3
#define MAVLINK_MSG_SBG_IMU_DATA_FIELD_GYRO_LEN 3
#define MAVLINK_MSG_SBG_IMU_DATA_FIELD_DELTA_VEL_LEN 3
#define MAVLINK_MSG_SBG_IMU_DATA_FIELD_DELTA_ANGLE_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SBG_IMU_DATA { \
    630, \
    "SBG_IMU_DATA", \
    17, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_imu_data_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_imu_data_t, time_stamp) }, \
         { "imu_com", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_sbg_imu_data_t, imu_com) }, \
         { "imu_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 65, offsetof(mavlink_sbg_imu_data_t, imu_status) }, \
         { "imu_accel_x", NULL, MAVLINK_TYPE_UINT8_T, 0, 66, offsetof(mavlink_sbg_imu_data_t, imu_accel_x) }, \
         { "imu_accel_y", NULL, MAVLINK_TYPE_UINT8_T, 0, 67, offsetof(mavlink_sbg_imu_data_t, imu_accel_y) }, \
         { "imu_accel_z", NULL, MAVLINK_TYPE_UINT8_T, 0, 68, offsetof(mavlink_sbg_imu_data_t, imu_accel_z) }, \
         { "imu_gyro_x", NULL, MAVLINK_TYPE_UINT8_T, 0, 69, offsetof(mavlink_sbg_imu_data_t, imu_gyro_x) }, \
         { "imu_gyro_y", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_sbg_imu_data_t, imu_gyro_y) }, \
         { "imu_gyro_z", NULL, MAVLINK_TYPE_UINT8_T, 0, 71, offsetof(mavlink_sbg_imu_data_t, imu_gyro_z) }, \
         { "imu_accels_in_range", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_sbg_imu_data_t, imu_accels_in_range) }, \
         { "imu_gyros_in_range", NULL, MAVLINK_TYPE_UINT8_T, 0, 73, offsetof(mavlink_sbg_imu_data_t, imu_gyros_in_range) }, \
         { "accel", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_sbg_imu_data_t, accel) }, \
         { "gyro", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_sbg_imu_data_t, gyro) }, \
         { "temp", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sbg_imu_data_t, temp) }, \
         { "delta_vel", NULL, MAVLINK_TYPE_FLOAT, 3, 40, offsetof(mavlink_sbg_imu_data_t, delta_vel) }, \
         { "delta_angle", NULL, MAVLINK_TYPE_FLOAT, 3, 52, offsetof(mavlink_sbg_imu_data_t, delta_angle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SBG_IMU_DATA { \
    "SBG_IMU_DATA", \
    17, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_imu_data_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_imu_data_t, time_stamp) }, \
         { "imu_com", NULL, MAVLINK_TYPE_UINT8_T, 0, 64, offsetof(mavlink_sbg_imu_data_t, imu_com) }, \
         { "imu_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 65, offsetof(mavlink_sbg_imu_data_t, imu_status) }, \
         { "imu_accel_x", NULL, MAVLINK_TYPE_UINT8_T, 0, 66, offsetof(mavlink_sbg_imu_data_t, imu_accel_x) }, \
         { "imu_accel_y", NULL, MAVLINK_TYPE_UINT8_T, 0, 67, offsetof(mavlink_sbg_imu_data_t, imu_accel_y) }, \
         { "imu_accel_z", NULL, MAVLINK_TYPE_UINT8_T, 0, 68, offsetof(mavlink_sbg_imu_data_t, imu_accel_z) }, \
         { "imu_gyro_x", NULL, MAVLINK_TYPE_UINT8_T, 0, 69, offsetof(mavlink_sbg_imu_data_t, imu_gyro_x) }, \
         { "imu_gyro_y", NULL, MAVLINK_TYPE_UINT8_T, 0, 70, offsetof(mavlink_sbg_imu_data_t, imu_gyro_y) }, \
         { "imu_gyro_z", NULL, MAVLINK_TYPE_UINT8_T, 0, 71, offsetof(mavlink_sbg_imu_data_t, imu_gyro_z) }, \
         { "imu_accels_in_range", NULL, MAVLINK_TYPE_UINT8_T, 0, 72, offsetof(mavlink_sbg_imu_data_t, imu_accels_in_range) }, \
         { "imu_gyros_in_range", NULL, MAVLINK_TYPE_UINT8_T, 0, 73, offsetof(mavlink_sbg_imu_data_t, imu_gyros_in_range) }, \
         { "accel", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_sbg_imu_data_t, accel) }, \
         { "gyro", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_sbg_imu_data_t, gyro) }, \
         { "temp", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sbg_imu_data_t, temp) }, \
         { "delta_vel", NULL, MAVLINK_TYPE_FLOAT, 3, 40, offsetof(mavlink_sbg_imu_data_t, delta_vel) }, \
         { "delta_angle", NULL, MAVLINK_TYPE_FLOAT, 3, 52, offsetof(mavlink_sbg_imu_data_t, delta_angle) }, \
         } \
}
#endif

/**
 * @brief Pack a sbg_imu_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param imu_com  True if the communication with the IMU is ok..
 * @param imu_status  True if internal IMU passes Built In Test (Calibration, CPU).
 * @param imu_accel_x  True if accelerometer X passes Built In Test.
 * @param imu_accel_y  True if accelerometer Y passes Built In Test.
 * @param imu_accel_z  True if accelerometer Z passes Built In Test.
 * @param imu_gyro_x  True if gyroscope X passes Built In Test.
 * @param imu_gyro_y  True if gyroscope Y passes Built In Test.
 * @param imu_gyro_z  True if gyroscope Z passes Built In Test.
 * @param imu_accels_in_range  True if accelerometers are within operating range.
 * @param imu_gyros_in_range  True if gyroscopes are within operating range.
 * @param accel [m/s2] Filtered Accelerometer - X, Y, Z axis m/s2.
 * @param gyro [rad/s] Filtered Gyroscope - X, Y, Z axis rad/s.
 * @param temp  Internal Temperature degC.
 * @param delta_vel [m/s2] Sculling output - X, Y, Z axis m/s2.
 * @param delta_angle [rad/s] Coning output - X, Y, Z axis rad/s.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_imu_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t time_stamp, uint8_t imu_com, uint8_t imu_status, uint8_t imu_accel_x, uint8_t imu_accel_y, uint8_t imu_accel_z, uint8_t imu_gyro_x, uint8_t imu_gyro_y, uint8_t imu_gyro_z, uint8_t imu_accels_in_range, uint8_t imu_gyros_in_range, const float *accel, const float *gyro, float temp, const float *delta_vel, const float *delta_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_IMU_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_float(buf, 36, temp);
    _mav_put_uint8_t(buf, 64, imu_com);
    _mav_put_uint8_t(buf, 65, imu_status);
    _mav_put_uint8_t(buf, 66, imu_accel_x);
    _mav_put_uint8_t(buf, 67, imu_accel_y);
    _mav_put_uint8_t(buf, 68, imu_accel_z);
    _mav_put_uint8_t(buf, 69, imu_gyro_x);
    _mav_put_uint8_t(buf, 70, imu_gyro_y);
    _mav_put_uint8_t(buf, 71, imu_gyro_z);
    _mav_put_uint8_t(buf, 72, imu_accels_in_range);
    _mav_put_uint8_t(buf, 73, imu_gyros_in_range);
    _mav_put_float_array(buf, 12, accel, 3);
    _mav_put_float_array(buf, 24, gyro, 3);
    _mav_put_float_array(buf, 40, delta_vel, 3);
    _mav_put_float_array(buf, 52, delta_angle, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_IMU_DATA_LEN);
#else
    mavlink_sbg_imu_data_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.temp = temp;
    packet.imu_com = imu_com;
    packet.imu_status = imu_status;
    packet.imu_accel_x = imu_accel_x;
    packet.imu_accel_y = imu_accel_y;
    packet.imu_accel_z = imu_accel_z;
    packet.imu_gyro_x = imu_gyro_x;
    packet.imu_gyro_y = imu_gyro_y;
    packet.imu_gyro_z = imu_gyro_z;
    packet.imu_accels_in_range = imu_accels_in_range;
    packet.imu_gyros_in_range = imu_gyros_in_range;
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
    mav_array_memcpy(packet.gyro, gyro, sizeof(float)*3);
    mav_array_memcpy(packet.delta_vel, delta_vel, sizeof(float)*3);
    mav_array_memcpy(packet.delta_angle, delta_angle, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_IMU_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_IMU_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SBG_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_CRC);
}

/**
 * @brief Pack a sbg_imu_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param imu_com  True if the communication with the IMU is ok..
 * @param imu_status  True if internal IMU passes Built In Test (Calibration, CPU).
 * @param imu_accel_x  True if accelerometer X passes Built In Test.
 * @param imu_accel_y  True if accelerometer Y passes Built In Test.
 * @param imu_accel_z  True if accelerometer Z passes Built In Test.
 * @param imu_gyro_x  True if gyroscope X passes Built In Test.
 * @param imu_gyro_y  True if gyroscope Y passes Built In Test.
 * @param imu_gyro_z  True if gyroscope Z passes Built In Test.
 * @param imu_accels_in_range  True if accelerometers are within operating range.
 * @param imu_gyros_in_range  True if gyroscopes are within operating range.
 * @param accel [m/s2] Filtered Accelerometer - X, Y, Z axis m/s2.
 * @param gyro [rad/s] Filtered Gyroscope - X, Y, Z axis rad/s.
 * @param temp  Internal Temperature degC.
 * @param delta_vel [m/s2] Sculling output - X, Y, Z axis m/s2.
 * @param delta_angle [rad/s] Coning output - X, Y, Z axis rad/s.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_imu_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t time_stamp,uint8_t imu_com,uint8_t imu_status,uint8_t imu_accel_x,uint8_t imu_accel_y,uint8_t imu_accel_z,uint8_t imu_gyro_x,uint8_t imu_gyro_y,uint8_t imu_gyro_z,uint8_t imu_accels_in_range,uint8_t imu_gyros_in_range,const float *accel,const float *gyro,float temp,const float *delta_vel,const float *delta_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_IMU_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_float(buf, 36, temp);
    _mav_put_uint8_t(buf, 64, imu_com);
    _mav_put_uint8_t(buf, 65, imu_status);
    _mav_put_uint8_t(buf, 66, imu_accel_x);
    _mav_put_uint8_t(buf, 67, imu_accel_y);
    _mav_put_uint8_t(buf, 68, imu_accel_z);
    _mav_put_uint8_t(buf, 69, imu_gyro_x);
    _mav_put_uint8_t(buf, 70, imu_gyro_y);
    _mav_put_uint8_t(buf, 71, imu_gyro_z);
    _mav_put_uint8_t(buf, 72, imu_accels_in_range);
    _mav_put_uint8_t(buf, 73, imu_gyros_in_range);
    _mav_put_float_array(buf, 12, accel, 3);
    _mav_put_float_array(buf, 24, gyro, 3);
    _mav_put_float_array(buf, 40, delta_vel, 3);
    _mav_put_float_array(buf, 52, delta_angle, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_IMU_DATA_LEN);
#else
    mavlink_sbg_imu_data_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.temp = temp;
    packet.imu_com = imu_com;
    packet.imu_status = imu_status;
    packet.imu_accel_x = imu_accel_x;
    packet.imu_accel_y = imu_accel_y;
    packet.imu_accel_z = imu_accel_z;
    packet.imu_gyro_x = imu_gyro_x;
    packet.imu_gyro_y = imu_gyro_y;
    packet.imu_gyro_z = imu_gyro_z;
    packet.imu_accels_in_range = imu_accels_in_range;
    packet.imu_gyros_in_range = imu_gyros_in_range;
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
    mav_array_memcpy(packet.gyro, gyro, sizeof(float)*3);
    mav_array_memcpy(packet.delta_vel, delta_vel, sizeof(float)*3);
    mav_array_memcpy(packet.delta_angle, delta_angle, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_IMU_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_IMU_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SBG_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_CRC);
}

/**
 * @brief Encode a sbg_imu_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbg_imu_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_imu_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbg_imu_data_t* sbg_imu_data)
{
    return mavlink_msg_sbg_imu_data_pack(system_id, component_id, msg, sbg_imu_data->timestamp, sbg_imu_data->time_stamp, sbg_imu_data->imu_com, sbg_imu_data->imu_status, sbg_imu_data->imu_accel_x, sbg_imu_data->imu_accel_y, sbg_imu_data->imu_accel_z, sbg_imu_data->imu_gyro_x, sbg_imu_data->imu_gyro_y, sbg_imu_data->imu_gyro_z, sbg_imu_data->imu_accels_in_range, sbg_imu_data->imu_gyros_in_range, sbg_imu_data->accel, sbg_imu_data->gyro, sbg_imu_data->temp, sbg_imu_data->delta_vel, sbg_imu_data->delta_angle);
}

/**
 * @brief Encode a sbg_imu_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sbg_imu_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_imu_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sbg_imu_data_t* sbg_imu_data)
{
    return mavlink_msg_sbg_imu_data_pack_chan(system_id, component_id, chan, msg, sbg_imu_data->timestamp, sbg_imu_data->time_stamp, sbg_imu_data->imu_com, sbg_imu_data->imu_status, sbg_imu_data->imu_accel_x, sbg_imu_data->imu_accel_y, sbg_imu_data->imu_accel_z, sbg_imu_data->imu_gyro_x, sbg_imu_data->imu_gyro_y, sbg_imu_data->imu_gyro_z, sbg_imu_data->imu_accels_in_range, sbg_imu_data->imu_gyros_in_range, sbg_imu_data->accel, sbg_imu_data->gyro, sbg_imu_data->temp, sbg_imu_data->delta_vel, sbg_imu_data->delta_angle);
}

/**
 * @brief Send a sbg_imu_data message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param imu_com  True if the communication with the IMU is ok..
 * @param imu_status  True if internal IMU passes Built In Test (Calibration, CPU).
 * @param imu_accel_x  True if accelerometer X passes Built In Test.
 * @param imu_accel_y  True if accelerometer Y passes Built In Test.
 * @param imu_accel_z  True if accelerometer Z passes Built In Test.
 * @param imu_gyro_x  True if gyroscope X passes Built In Test.
 * @param imu_gyro_y  True if gyroscope Y passes Built In Test.
 * @param imu_gyro_z  True if gyroscope Z passes Built In Test.
 * @param imu_accels_in_range  True if accelerometers are within operating range.
 * @param imu_gyros_in_range  True if gyroscopes are within operating range.
 * @param accel [m/s2] Filtered Accelerometer - X, Y, Z axis m/s2.
 * @param gyro [rad/s] Filtered Gyroscope - X, Y, Z axis rad/s.
 * @param temp  Internal Temperature degC.
 * @param delta_vel [m/s2] Sculling output - X, Y, Z axis m/s2.
 * @param delta_angle [rad/s] Coning output - X, Y, Z axis rad/s.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sbg_imu_data_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t time_stamp, uint8_t imu_com, uint8_t imu_status, uint8_t imu_accel_x, uint8_t imu_accel_y, uint8_t imu_accel_z, uint8_t imu_gyro_x, uint8_t imu_gyro_y, uint8_t imu_gyro_z, uint8_t imu_accels_in_range, uint8_t imu_gyros_in_range, const float *accel, const float *gyro, float temp, const float *delta_vel, const float *delta_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_IMU_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_float(buf, 36, temp);
    _mav_put_uint8_t(buf, 64, imu_com);
    _mav_put_uint8_t(buf, 65, imu_status);
    _mav_put_uint8_t(buf, 66, imu_accel_x);
    _mav_put_uint8_t(buf, 67, imu_accel_y);
    _mav_put_uint8_t(buf, 68, imu_accel_z);
    _mav_put_uint8_t(buf, 69, imu_gyro_x);
    _mav_put_uint8_t(buf, 70, imu_gyro_y);
    _mav_put_uint8_t(buf, 71, imu_gyro_z);
    _mav_put_uint8_t(buf, 72, imu_accels_in_range);
    _mav_put_uint8_t(buf, 73, imu_gyros_in_range);
    _mav_put_float_array(buf, 12, accel, 3);
    _mav_put_float_array(buf, 24, gyro, 3);
    _mav_put_float_array(buf, 40, delta_vel, 3);
    _mav_put_float_array(buf, 52, delta_angle, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_IMU_DATA, buf, MAVLINK_MSG_ID_SBG_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_CRC);
#else
    mavlink_sbg_imu_data_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.temp = temp;
    packet.imu_com = imu_com;
    packet.imu_status = imu_status;
    packet.imu_accel_x = imu_accel_x;
    packet.imu_accel_y = imu_accel_y;
    packet.imu_accel_z = imu_accel_z;
    packet.imu_gyro_x = imu_gyro_x;
    packet.imu_gyro_y = imu_gyro_y;
    packet.imu_gyro_z = imu_gyro_z;
    packet.imu_accels_in_range = imu_accels_in_range;
    packet.imu_gyros_in_range = imu_gyros_in_range;
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
    mav_array_memcpy(packet.gyro, gyro, sizeof(float)*3);
    mav_array_memcpy(packet.delta_vel, delta_vel, sizeof(float)*3);
    mav_array_memcpy(packet.delta_angle, delta_angle, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_IMU_DATA, (const char *)&packet, MAVLINK_MSG_ID_SBG_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_CRC);
#endif
}

/**
 * @brief Send a sbg_imu_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sbg_imu_data_send_struct(mavlink_channel_t chan, const mavlink_sbg_imu_data_t* sbg_imu_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sbg_imu_data_send(chan, sbg_imu_data->timestamp, sbg_imu_data->time_stamp, sbg_imu_data->imu_com, sbg_imu_data->imu_status, sbg_imu_data->imu_accel_x, sbg_imu_data->imu_accel_y, sbg_imu_data->imu_accel_z, sbg_imu_data->imu_gyro_x, sbg_imu_data->imu_gyro_y, sbg_imu_data->imu_gyro_z, sbg_imu_data->imu_accels_in_range, sbg_imu_data->imu_gyros_in_range, sbg_imu_data->accel, sbg_imu_data->gyro, sbg_imu_data->temp, sbg_imu_data->delta_vel, sbg_imu_data->delta_angle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_IMU_DATA, (const char *)sbg_imu_data, MAVLINK_MSG_ID_SBG_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_SBG_IMU_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sbg_imu_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t time_stamp, uint8_t imu_com, uint8_t imu_status, uint8_t imu_accel_x, uint8_t imu_accel_y, uint8_t imu_accel_z, uint8_t imu_gyro_x, uint8_t imu_gyro_y, uint8_t imu_gyro_z, uint8_t imu_accels_in_range, uint8_t imu_gyros_in_range, const float *accel, const float *gyro, float temp, const float *delta_vel, const float *delta_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_float(buf, 36, temp);
    _mav_put_uint8_t(buf, 64, imu_com);
    _mav_put_uint8_t(buf, 65, imu_status);
    _mav_put_uint8_t(buf, 66, imu_accel_x);
    _mav_put_uint8_t(buf, 67, imu_accel_y);
    _mav_put_uint8_t(buf, 68, imu_accel_z);
    _mav_put_uint8_t(buf, 69, imu_gyro_x);
    _mav_put_uint8_t(buf, 70, imu_gyro_y);
    _mav_put_uint8_t(buf, 71, imu_gyro_z);
    _mav_put_uint8_t(buf, 72, imu_accels_in_range);
    _mav_put_uint8_t(buf, 73, imu_gyros_in_range);
    _mav_put_float_array(buf, 12, accel, 3);
    _mav_put_float_array(buf, 24, gyro, 3);
    _mav_put_float_array(buf, 40, delta_vel, 3);
    _mav_put_float_array(buf, 52, delta_angle, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_IMU_DATA, buf, MAVLINK_MSG_ID_SBG_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_CRC);
#else
    mavlink_sbg_imu_data_t *packet = (mavlink_sbg_imu_data_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->time_stamp = time_stamp;
    packet->temp = temp;
    packet->imu_com = imu_com;
    packet->imu_status = imu_status;
    packet->imu_accel_x = imu_accel_x;
    packet->imu_accel_y = imu_accel_y;
    packet->imu_accel_z = imu_accel_z;
    packet->imu_gyro_x = imu_gyro_x;
    packet->imu_gyro_y = imu_gyro_y;
    packet->imu_gyro_z = imu_gyro_z;
    packet->imu_accels_in_range = imu_accels_in_range;
    packet->imu_gyros_in_range = imu_gyros_in_range;
    mav_array_memcpy(packet->accel, accel, sizeof(float)*3);
    mav_array_memcpy(packet->gyro, gyro, sizeof(float)*3);
    mav_array_memcpy(packet->delta_vel, delta_vel, sizeof(float)*3);
    mav_array_memcpy(packet->delta_angle, delta_angle, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_IMU_DATA, (const char *)packet, MAVLINK_MSG_ID_SBG_IMU_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_LEN, MAVLINK_MSG_ID_SBG_IMU_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE SBG_IMU_DATA UNPACKING


/**
 * @brief Get field timestamp from sbg_imu_data message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_sbg_imu_data_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_stamp from sbg_imu_data message
 *
 * @return [us] Time since sensor is powered up us.
 */
static inline uint32_t mavlink_msg_sbg_imu_data_get_time_stamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field imu_com from sbg_imu_data message
 *
 * @return  True if the communication with the IMU is ok..
 */
static inline uint8_t mavlink_msg_sbg_imu_data_get_imu_com(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  64);
}

/**
 * @brief Get field imu_status from sbg_imu_data message
 *
 * @return  True if internal IMU passes Built In Test (Calibration, CPU).
 */
static inline uint8_t mavlink_msg_sbg_imu_data_get_imu_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  65);
}

/**
 * @brief Get field imu_accel_x from sbg_imu_data message
 *
 * @return  True if accelerometer X passes Built In Test.
 */
static inline uint8_t mavlink_msg_sbg_imu_data_get_imu_accel_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  66);
}

/**
 * @brief Get field imu_accel_y from sbg_imu_data message
 *
 * @return  True if accelerometer Y passes Built In Test.
 */
static inline uint8_t mavlink_msg_sbg_imu_data_get_imu_accel_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  67);
}

/**
 * @brief Get field imu_accel_z from sbg_imu_data message
 *
 * @return  True if accelerometer Z passes Built In Test.
 */
static inline uint8_t mavlink_msg_sbg_imu_data_get_imu_accel_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  68);
}

/**
 * @brief Get field imu_gyro_x from sbg_imu_data message
 *
 * @return  True if gyroscope X passes Built In Test.
 */
static inline uint8_t mavlink_msg_sbg_imu_data_get_imu_gyro_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  69);
}

/**
 * @brief Get field imu_gyro_y from sbg_imu_data message
 *
 * @return  True if gyroscope Y passes Built In Test.
 */
static inline uint8_t mavlink_msg_sbg_imu_data_get_imu_gyro_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  70);
}

/**
 * @brief Get field imu_gyro_z from sbg_imu_data message
 *
 * @return  True if gyroscope Z passes Built In Test.
 */
static inline uint8_t mavlink_msg_sbg_imu_data_get_imu_gyro_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  71);
}

/**
 * @brief Get field imu_accels_in_range from sbg_imu_data message
 *
 * @return  True if accelerometers are within operating range.
 */
static inline uint8_t mavlink_msg_sbg_imu_data_get_imu_accels_in_range(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  72);
}

/**
 * @brief Get field imu_gyros_in_range from sbg_imu_data message
 *
 * @return  True if gyroscopes are within operating range.
 */
static inline uint8_t mavlink_msg_sbg_imu_data_get_imu_gyros_in_range(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  73);
}

/**
 * @brief Get field accel from sbg_imu_data message
 *
 * @return [m/s2] Filtered Accelerometer - X, Y, Z axis m/s2.
 */
static inline uint16_t mavlink_msg_sbg_imu_data_get_accel(const mavlink_message_t* msg, float *accel)
{
    return _MAV_RETURN_float_array(msg, accel, 3,  12);
}

/**
 * @brief Get field gyro from sbg_imu_data message
 *
 * @return [rad/s] Filtered Gyroscope - X, Y, Z axis rad/s.
 */
static inline uint16_t mavlink_msg_sbg_imu_data_get_gyro(const mavlink_message_t* msg, float *gyro)
{
    return _MAV_RETURN_float_array(msg, gyro, 3,  24);
}

/**
 * @brief Get field temp from sbg_imu_data message
 *
 * @return  Internal Temperature degC.
 */
static inline float mavlink_msg_sbg_imu_data_get_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field delta_vel from sbg_imu_data message
 *
 * @return [m/s2] Sculling output - X, Y, Z axis m/s2.
 */
static inline uint16_t mavlink_msg_sbg_imu_data_get_delta_vel(const mavlink_message_t* msg, float *delta_vel)
{
    return _MAV_RETURN_float_array(msg, delta_vel, 3,  40);
}

/**
 * @brief Get field delta_angle from sbg_imu_data message
 *
 * @return [rad/s] Coning output - X, Y, Z axis rad/s.
 */
static inline uint16_t mavlink_msg_sbg_imu_data_get_delta_angle(const mavlink_message_t* msg, float *delta_angle)
{
    return _MAV_RETURN_float_array(msg, delta_angle, 3,  52);
}

/**
 * @brief Decode a sbg_imu_data message into a struct
 *
 * @param msg The message to decode
 * @param sbg_imu_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_sbg_imu_data_decode(const mavlink_message_t* msg, mavlink_sbg_imu_data_t* sbg_imu_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sbg_imu_data->timestamp = mavlink_msg_sbg_imu_data_get_timestamp(msg);
    sbg_imu_data->time_stamp = mavlink_msg_sbg_imu_data_get_time_stamp(msg);
    mavlink_msg_sbg_imu_data_get_accel(msg, sbg_imu_data->accel);
    mavlink_msg_sbg_imu_data_get_gyro(msg, sbg_imu_data->gyro);
    sbg_imu_data->temp = mavlink_msg_sbg_imu_data_get_temp(msg);
    mavlink_msg_sbg_imu_data_get_delta_vel(msg, sbg_imu_data->delta_vel);
    mavlink_msg_sbg_imu_data_get_delta_angle(msg, sbg_imu_data->delta_angle);
    sbg_imu_data->imu_com = mavlink_msg_sbg_imu_data_get_imu_com(msg);
    sbg_imu_data->imu_status = mavlink_msg_sbg_imu_data_get_imu_status(msg);
    sbg_imu_data->imu_accel_x = mavlink_msg_sbg_imu_data_get_imu_accel_x(msg);
    sbg_imu_data->imu_accel_y = mavlink_msg_sbg_imu_data_get_imu_accel_y(msg);
    sbg_imu_data->imu_accel_z = mavlink_msg_sbg_imu_data_get_imu_accel_z(msg);
    sbg_imu_data->imu_gyro_x = mavlink_msg_sbg_imu_data_get_imu_gyro_x(msg);
    sbg_imu_data->imu_gyro_y = mavlink_msg_sbg_imu_data_get_imu_gyro_y(msg);
    sbg_imu_data->imu_gyro_z = mavlink_msg_sbg_imu_data_get_imu_gyro_z(msg);
    sbg_imu_data->imu_accels_in_range = mavlink_msg_sbg_imu_data_get_imu_accels_in_range(msg);
    sbg_imu_data->imu_gyros_in_range = mavlink_msg_sbg_imu_data_get_imu_gyros_in_range(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SBG_IMU_DATA_LEN? msg->len : MAVLINK_MSG_ID_SBG_IMU_DATA_LEN;
        memset(sbg_imu_data, 0, MAVLINK_MSG_ID_SBG_IMU_DATA_LEN);
    memcpy(sbg_imu_data, _MAV_PAYLOAD(msg), len);
#endif
}
