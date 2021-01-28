#pragma once
// MESSAGE SBG_DATA PACKING

#define MAVLINK_MSG_ID_SBG_DATA 607


typedef struct __mavlink_sbg_data_t {
 float acc_z; /*< [m/s/s]  Vertical acceleration from ship motion profile (positive down) */
 float vel_z; /*< [m/s]  Vertical velocity from ship motion profile (positive down)  */
 float acc_x; /*< [m/s/s]  X acceleration (filtered) */
 float acc_y; /*< [m/s/s]  Y acceleration (filtered) */
 float gyro_x; /*< [rad/s]  Filtered Gyroscope - X axis rad/s  */
 float gyro_y; /*< [rad/s]  Filtered Gyroscope - Y axis rad/s  */
 float gyro_z; /*< [rad/s]  Filtered Gyroscope - Z axis rad/s  */
 float ekf_roll; /*< [rad]  Roll angle filtered with SBG EKF */
 float ekf_pitch; /*< [rad]  Roll angle filtered with SBG EKF */
 float ekf_yaw; /*< [rad]  Roll angle filtered with SBG EKF */
 float temp; /*< [cdegC]  Internal Temperature degC  */
 float heave; /*< [m]  Heave (positive down) at main location (in m)  */
 uint8_t main_power; /*<    (BOOL) True when main power supply is OK. */
 uint8_t imu_power; /*<     (BOOL) True when IMU power supply is OK. */
 uint8_t gps_power; /*<     (BOOL) Set to True when GPS power supply is OK. */
 uint8_t settings; /*<      (BOOL) True if settings were correctly loaded */
 uint8_t temperature; /*<   (BOOL) True when temperature is within specified limits. */
 uint8_t port_a; /*<        (BOOL) PORT A: False in case of low level communication error. */
 uint8_t imu_com; /*<       (BOOL) True if the communication with the IMU is ok. */
 uint8_t imu_status; /*<    (BOOL) True if internal IMU passes Built In Test (Calibration, CPU) */
 uint8_t heave_valid; /*<    (BOOL) True after heave convergence time. */
} mavlink_sbg_data_t;

#define MAVLINK_MSG_ID_SBG_DATA_LEN 57
#define MAVLINK_MSG_ID_SBG_DATA_MIN_LEN 57
#define MAVLINK_MSG_ID_607_LEN 57
#define MAVLINK_MSG_ID_607_MIN_LEN 57

#define MAVLINK_MSG_ID_SBG_DATA_CRC 243
#define MAVLINK_MSG_ID_607_CRC 243



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SBG_DATA { \
    607, \
    "SBG_DATA", \
    21, \
    {  { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sbg_data_t, acc_z) }, \
         { "vel_z", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sbg_data_t, vel_z) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sbg_data_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sbg_data_t, acc_y) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sbg_data_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sbg_data_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sbg_data_t, gyro_z) }, \
         { "ekf_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sbg_data_t, ekf_roll) }, \
         { "ekf_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sbg_data_t, ekf_pitch) }, \
         { "ekf_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sbg_data_t, ekf_yaw) }, \
         { "temp", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sbg_data_t, temp) }, \
         { "heave", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_sbg_data_t, heave) }, \
         { "main_power", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_sbg_data_t, main_power) }, \
         { "imu_power", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_sbg_data_t, imu_power) }, \
         { "gps_power", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_sbg_data_t, gps_power) }, \
         { "settings", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_sbg_data_t, settings) }, \
         { "temperature", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_sbg_data_t, temperature) }, \
         { "port_a", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_sbg_data_t, port_a) }, \
         { "imu_com", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_sbg_data_t, imu_com) }, \
         { "imu_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_sbg_data_t, imu_status) }, \
         { "heave_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_sbg_data_t, heave_valid) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SBG_DATA { \
    "SBG_DATA", \
    21, \
    {  { "acc_z", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_sbg_data_t, acc_z) }, \
         { "vel_z", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_sbg_data_t, vel_z) }, \
         { "acc_x", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_sbg_data_t, acc_x) }, \
         { "acc_y", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sbg_data_t, acc_y) }, \
         { "gyro_x", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sbg_data_t, gyro_x) }, \
         { "gyro_y", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sbg_data_t, gyro_y) }, \
         { "gyro_z", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sbg_data_t, gyro_z) }, \
         { "ekf_roll", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sbg_data_t, ekf_roll) }, \
         { "ekf_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_sbg_data_t, ekf_pitch) }, \
         { "ekf_yaw", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_sbg_data_t, ekf_yaw) }, \
         { "temp", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_sbg_data_t, temp) }, \
         { "heave", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_sbg_data_t, heave) }, \
         { "main_power", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_sbg_data_t, main_power) }, \
         { "imu_power", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_sbg_data_t, imu_power) }, \
         { "gps_power", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_sbg_data_t, gps_power) }, \
         { "settings", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_sbg_data_t, settings) }, \
         { "temperature", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_sbg_data_t, temperature) }, \
         { "port_a", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_sbg_data_t, port_a) }, \
         { "imu_com", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_sbg_data_t, imu_com) }, \
         { "imu_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_sbg_data_t, imu_status) }, \
         { "heave_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 56, offsetof(mavlink_sbg_data_t, heave_valid) }, \
         } \
}
#endif

/**
 * @brief Pack a sbg_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param acc_z [m/s/s]  Vertical acceleration from ship motion profile (positive down) 
 * @param vel_z [m/s]  Vertical velocity from ship motion profile (positive down)  
 * @param acc_x [m/s/s]  X acceleration (filtered) 
 * @param acc_y [m/s/s]  Y acceleration (filtered) 
 * @param gyro_x [rad/s]  Filtered Gyroscope - X axis rad/s  
 * @param gyro_y [rad/s]  Filtered Gyroscope - Y axis rad/s  
 * @param gyro_z [rad/s]  Filtered Gyroscope - Z axis rad/s  
 * @param ekf_roll [rad]  Roll angle filtered with SBG EKF 
 * @param ekf_pitch [rad]  Roll angle filtered with SBG EKF 
 * @param ekf_yaw [rad]  Roll angle filtered with SBG EKF 
 * @param temp [cdegC]  Internal Temperature degC  
 * @param heave [m]  Heave (positive down) at main location (in m)  
 * @param main_power    (BOOL) True when main power supply is OK. 
 * @param imu_power     (BOOL) True when IMU power supply is OK. 
 * @param gps_power     (BOOL) Set to True when GPS power supply is OK. 
 * @param settings      (BOOL) True if settings were correctly loaded 
 * @param temperature   (BOOL) True when temperature is within specified limits. 
 * @param port_a        (BOOL) PORT A: False in case of low level communication error. 
 * @param imu_com       (BOOL) True if the communication with the IMU is ok. 
 * @param imu_status    (BOOL) True if internal IMU passes Built In Test (Calibration, CPU) 
 * @param heave_valid    (BOOL) True after heave convergence time. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float acc_z, float vel_z, float acc_x, float acc_y, float gyro_x, float gyro_y, float gyro_z, float ekf_roll, float ekf_pitch, float ekf_yaw, float temp, float heave, uint8_t main_power, uint8_t imu_power, uint8_t gps_power, uint8_t settings, uint8_t temperature, uint8_t port_a, uint8_t imu_com, uint8_t imu_status, uint8_t heave_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_DATA_LEN];
    _mav_put_float(buf, 0, acc_z);
    _mav_put_float(buf, 4, vel_z);
    _mav_put_float(buf, 8, acc_x);
    _mav_put_float(buf, 12, acc_y);
    _mav_put_float(buf, 16, gyro_x);
    _mav_put_float(buf, 20, gyro_y);
    _mav_put_float(buf, 24, gyro_z);
    _mav_put_float(buf, 28, ekf_roll);
    _mav_put_float(buf, 32, ekf_pitch);
    _mav_put_float(buf, 36, ekf_yaw);
    _mav_put_float(buf, 40, temp);
    _mav_put_float(buf, 44, heave);
    _mav_put_uint8_t(buf, 48, main_power);
    _mav_put_uint8_t(buf, 49, imu_power);
    _mav_put_uint8_t(buf, 50, gps_power);
    _mav_put_uint8_t(buf, 51, settings);
    _mav_put_uint8_t(buf, 52, temperature);
    _mav_put_uint8_t(buf, 53, port_a);
    _mav_put_uint8_t(buf, 54, imu_com);
    _mav_put_uint8_t(buf, 55, imu_status);
    _mav_put_uint8_t(buf, 56, heave_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_DATA_LEN);
#else
    mavlink_sbg_data_t packet;
    packet.acc_z = acc_z;
    packet.vel_z = vel_z;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.ekf_roll = ekf_roll;
    packet.ekf_pitch = ekf_pitch;
    packet.ekf_yaw = ekf_yaw;
    packet.temp = temp;
    packet.heave = heave;
    packet.main_power = main_power;
    packet.imu_power = imu_power;
    packet.gps_power = gps_power;
    packet.settings = settings;
    packet.temperature = temperature;
    packet.port_a = port_a;
    packet.imu_com = imu_com;
    packet.imu_status = imu_status;
    packet.heave_valid = heave_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SBG_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_DATA_LEN, MAVLINK_MSG_ID_SBG_DATA_CRC);
}

/**
 * @brief Pack a sbg_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param acc_z [m/s/s]  Vertical acceleration from ship motion profile (positive down) 
 * @param vel_z [m/s]  Vertical velocity from ship motion profile (positive down)  
 * @param acc_x [m/s/s]  X acceleration (filtered) 
 * @param acc_y [m/s/s]  Y acceleration (filtered) 
 * @param gyro_x [rad/s]  Filtered Gyroscope - X axis rad/s  
 * @param gyro_y [rad/s]  Filtered Gyroscope - Y axis rad/s  
 * @param gyro_z [rad/s]  Filtered Gyroscope - Z axis rad/s  
 * @param ekf_roll [rad]  Roll angle filtered with SBG EKF 
 * @param ekf_pitch [rad]  Roll angle filtered with SBG EKF 
 * @param ekf_yaw [rad]  Roll angle filtered with SBG EKF 
 * @param temp [cdegC]  Internal Temperature degC  
 * @param heave [m]  Heave (positive down) at main location (in m)  
 * @param main_power    (BOOL) True when main power supply is OK. 
 * @param imu_power     (BOOL) True when IMU power supply is OK. 
 * @param gps_power     (BOOL) Set to True when GPS power supply is OK. 
 * @param settings      (BOOL) True if settings were correctly loaded 
 * @param temperature   (BOOL) True when temperature is within specified limits. 
 * @param port_a        (BOOL) PORT A: False in case of low level communication error. 
 * @param imu_com       (BOOL) True if the communication with the IMU is ok. 
 * @param imu_status    (BOOL) True if internal IMU passes Built In Test (Calibration, CPU) 
 * @param heave_valid    (BOOL) True after heave convergence time. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float acc_z,float vel_z,float acc_x,float acc_y,float gyro_x,float gyro_y,float gyro_z,float ekf_roll,float ekf_pitch,float ekf_yaw,float temp,float heave,uint8_t main_power,uint8_t imu_power,uint8_t gps_power,uint8_t settings,uint8_t temperature,uint8_t port_a,uint8_t imu_com,uint8_t imu_status,uint8_t heave_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_DATA_LEN];
    _mav_put_float(buf, 0, acc_z);
    _mav_put_float(buf, 4, vel_z);
    _mav_put_float(buf, 8, acc_x);
    _mav_put_float(buf, 12, acc_y);
    _mav_put_float(buf, 16, gyro_x);
    _mav_put_float(buf, 20, gyro_y);
    _mav_put_float(buf, 24, gyro_z);
    _mav_put_float(buf, 28, ekf_roll);
    _mav_put_float(buf, 32, ekf_pitch);
    _mav_put_float(buf, 36, ekf_yaw);
    _mav_put_float(buf, 40, temp);
    _mav_put_float(buf, 44, heave);
    _mav_put_uint8_t(buf, 48, main_power);
    _mav_put_uint8_t(buf, 49, imu_power);
    _mav_put_uint8_t(buf, 50, gps_power);
    _mav_put_uint8_t(buf, 51, settings);
    _mav_put_uint8_t(buf, 52, temperature);
    _mav_put_uint8_t(buf, 53, port_a);
    _mav_put_uint8_t(buf, 54, imu_com);
    _mav_put_uint8_t(buf, 55, imu_status);
    _mav_put_uint8_t(buf, 56, heave_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_DATA_LEN);
#else
    mavlink_sbg_data_t packet;
    packet.acc_z = acc_z;
    packet.vel_z = vel_z;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.ekf_roll = ekf_roll;
    packet.ekf_pitch = ekf_pitch;
    packet.ekf_yaw = ekf_yaw;
    packet.temp = temp;
    packet.heave = heave;
    packet.main_power = main_power;
    packet.imu_power = imu_power;
    packet.gps_power = gps_power;
    packet.settings = settings;
    packet.temperature = temperature;
    packet.port_a = port_a;
    packet.imu_com = imu_com;
    packet.imu_status = imu_status;
    packet.heave_valid = heave_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SBG_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_DATA_LEN, MAVLINK_MSG_ID_SBG_DATA_CRC);
}

/**
 * @brief Encode a sbg_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbg_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbg_data_t* sbg_data)
{
    return mavlink_msg_sbg_data_pack(system_id, component_id, msg, sbg_data->acc_z, sbg_data->vel_z, sbg_data->acc_x, sbg_data->acc_y, sbg_data->gyro_x, sbg_data->gyro_y, sbg_data->gyro_z, sbg_data->ekf_roll, sbg_data->ekf_pitch, sbg_data->ekf_yaw, sbg_data->temp, sbg_data->heave, sbg_data->main_power, sbg_data->imu_power, sbg_data->gps_power, sbg_data->settings, sbg_data->temperature, sbg_data->port_a, sbg_data->imu_com, sbg_data->imu_status, sbg_data->heave_valid);
}

/**
 * @brief Encode a sbg_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sbg_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sbg_data_t* sbg_data)
{
    return mavlink_msg_sbg_data_pack_chan(system_id, component_id, chan, msg, sbg_data->acc_z, sbg_data->vel_z, sbg_data->acc_x, sbg_data->acc_y, sbg_data->gyro_x, sbg_data->gyro_y, sbg_data->gyro_z, sbg_data->ekf_roll, sbg_data->ekf_pitch, sbg_data->ekf_yaw, sbg_data->temp, sbg_data->heave, sbg_data->main_power, sbg_data->imu_power, sbg_data->gps_power, sbg_data->settings, sbg_data->temperature, sbg_data->port_a, sbg_data->imu_com, sbg_data->imu_status, sbg_data->heave_valid);
}

/**
 * @brief Send a sbg_data message
 * @param chan MAVLink channel to send the message
 *
 * @param acc_z [m/s/s]  Vertical acceleration from ship motion profile (positive down) 
 * @param vel_z [m/s]  Vertical velocity from ship motion profile (positive down)  
 * @param acc_x [m/s/s]  X acceleration (filtered) 
 * @param acc_y [m/s/s]  Y acceleration (filtered) 
 * @param gyro_x [rad/s]  Filtered Gyroscope - X axis rad/s  
 * @param gyro_y [rad/s]  Filtered Gyroscope - Y axis rad/s  
 * @param gyro_z [rad/s]  Filtered Gyroscope - Z axis rad/s  
 * @param ekf_roll [rad]  Roll angle filtered with SBG EKF 
 * @param ekf_pitch [rad]  Roll angle filtered with SBG EKF 
 * @param ekf_yaw [rad]  Roll angle filtered with SBG EKF 
 * @param temp [cdegC]  Internal Temperature degC  
 * @param heave [m]  Heave (positive down) at main location (in m)  
 * @param main_power    (BOOL) True when main power supply is OK. 
 * @param imu_power     (BOOL) True when IMU power supply is OK. 
 * @param gps_power     (BOOL) Set to True when GPS power supply is OK. 
 * @param settings      (BOOL) True if settings were correctly loaded 
 * @param temperature   (BOOL) True when temperature is within specified limits. 
 * @param port_a        (BOOL) PORT A: False in case of low level communication error. 
 * @param imu_com       (BOOL) True if the communication with the IMU is ok. 
 * @param imu_status    (BOOL) True if internal IMU passes Built In Test (Calibration, CPU) 
 * @param heave_valid    (BOOL) True after heave convergence time. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sbg_data_send(mavlink_channel_t chan, float acc_z, float vel_z, float acc_x, float acc_y, float gyro_x, float gyro_y, float gyro_z, float ekf_roll, float ekf_pitch, float ekf_yaw, float temp, float heave, uint8_t main_power, uint8_t imu_power, uint8_t gps_power, uint8_t settings, uint8_t temperature, uint8_t port_a, uint8_t imu_com, uint8_t imu_status, uint8_t heave_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_DATA_LEN];
    _mav_put_float(buf, 0, acc_z);
    _mav_put_float(buf, 4, vel_z);
    _mav_put_float(buf, 8, acc_x);
    _mav_put_float(buf, 12, acc_y);
    _mav_put_float(buf, 16, gyro_x);
    _mav_put_float(buf, 20, gyro_y);
    _mav_put_float(buf, 24, gyro_z);
    _mav_put_float(buf, 28, ekf_roll);
    _mav_put_float(buf, 32, ekf_pitch);
    _mav_put_float(buf, 36, ekf_yaw);
    _mav_put_float(buf, 40, temp);
    _mav_put_float(buf, 44, heave);
    _mav_put_uint8_t(buf, 48, main_power);
    _mav_put_uint8_t(buf, 49, imu_power);
    _mav_put_uint8_t(buf, 50, gps_power);
    _mav_put_uint8_t(buf, 51, settings);
    _mav_put_uint8_t(buf, 52, temperature);
    _mav_put_uint8_t(buf, 53, port_a);
    _mav_put_uint8_t(buf, 54, imu_com);
    _mav_put_uint8_t(buf, 55, imu_status);
    _mav_put_uint8_t(buf, 56, heave_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_DATA, buf, MAVLINK_MSG_ID_SBG_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_DATA_LEN, MAVLINK_MSG_ID_SBG_DATA_CRC);
#else
    mavlink_sbg_data_t packet;
    packet.acc_z = acc_z;
    packet.vel_z = vel_z;
    packet.acc_x = acc_x;
    packet.acc_y = acc_y;
    packet.gyro_x = gyro_x;
    packet.gyro_y = gyro_y;
    packet.gyro_z = gyro_z;
    packet.ekf_roll = ekf_roll;
    packet.ekf_pitch = ekf_pitch;
    packet.ekf_yaw = ekf_yaw;
    packet.temp = temp;
    packet.heave = heave;
    packet.main_power = main_power;
    packet.imu_power = imu_power;
    packet.gps_power = gps_power;
    packet.settings = settings;
    packet.temperature = temperature;
    packet.port_a = port_a;
    packet.imu_com = imu_com;
    packet.imu_status = imu_status;
    packet.heave_valid = heave_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_DATA, (const char *)&packet, MAVLINK_MSG_ID_SBG_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_DATA_LEN, MAVLINK_MSG_ID_SBG_DATA_CRC);
#endif
}

/**
 * @brief Send a sbg_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sbg_data_send_struct(mavlink_channel_t chan, const mavlink_sbg_data_t* sbg_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sbg_data_send(chan, sbg_data->acc_z, sbg_data->vel_z, sbg_data->acc_x, sbg_data->acc_y, sbg_data->gyro_x, sbg_data->gyro_y, sbg_data->gyro_z, sbg_data->ekf_roll, sbg_data->ekf_pitch, sbg_data->ekf_yaw, sbg_data->temp, sbg_data->heave, sbg_data->main_power, sbg_data->imu_power, sbg_data->gps_power, sbg_data->settings, sbg_data->temperature, sbg_data->port_a, sbg_data->imu_com, sbg_data->imu_status, sbg_data->heave_valid);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_DATA, (const char *)sbg_data, MAVLINK_MSG_ID_SBG_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_DATA_LEN, MAVLINK_MSG_ID_SBG_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_SBG_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sbg_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float acc_z, float vel_z, float acc_x, float acc_y, float gyro_x, float gyro_y, float gyro_z, float ekf_roll, float ekf_pitch, float ekf_yaw, float temp, float heave, uint8_t main_power, uint8_t imu_power, uint8_t gps_power, uint8_t settings, uint8_t temperature, uint8_t port_a, uint8_t imu_com, uint8_t imu_status, uint8_t heave_valid)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, acc_z);
    _mav_put_float(buf, 4, vel_z);
    _mav_put_float(buf, 8, acc_x);
    _mav_put_float(buf, 12, acc_y);
    _mav_put_float(buf, 16, gyro_x);
    _mav_put_float(buf, 20, gyro_y);
    _mav_put_float(buf, 24, gyro_z);
    _mav_put_float(buf, 28, ekf_roll);
    _mav_put_float(buf, 32, ekf_pitch);
    _mav_put_float(buf, 36, ekf_yaw);
    _mav_put_float(buf, 40, temp);
    _mav_put_float(buf, 44, heave);
    _mav_put_uint8_t(buf, 48, main_power);
    _mav_put_uint8_t(buf, 49, imu_power);
    _mav_put_uint8_t(buf, 50, gps_power);
    _mav_put_uint8_t(buf, 51, settings);
    _mav_put_uint8_t(buf, 52, temperature);
    _mav_put_uint8_t(buf, 53, port_a);
    _mav_put_uint8_t(buf, 54, imu_com);
    _mav_put_uint8_t(buf, 55, imu_status);
    _mav_put_uint8_t(buf, 56, heave_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_DATA, buf, MAVLINK_MSG_ID_SBG_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_DATA_LEN, MAVLINK_MSG_ID_SBG_DATA_CRC);
#else
    mavlink_sbg_data_t *packet = (mavlink_sbg_data_t *)msgbuf;
    packet->acc_z = acc_z;
    packet->vel_z = vel_z;
    packet->acc_x = acc_x;
    packet->acc_y = acc_y;
    packet->gyro_x = gyro_x;
    packet->gyro_y = gyro_y;
    packet->gyro_z = gyro_z;
    packet->ekf_roll = ekf_roll;
    packet->ekf_pitch = ekf_pitch;
    packet->ekf_yaw = ekf_yaw;
    packet->temp = temp;
    packet->heave = heave;
    packet->main_power = main_power;
    packet->imu_power = imu_power;
    packet->gps_power = gps_power;
    packet->settings = settings;
    packet->temperature = temperature;
    packet->port_a = port_a;
    packet->imu_com = imu_com;
    packet->imu_status = imu_status;
    packet->heave_valid = heave_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_DATA, (const char *)packet, MAVLINK_MSG_ID_SBG_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_DATA_LEN, MAVLINK_MSG_ID_SBG_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE SBG_DATA UNPACKING


/**
 * @brief Get field acc_z from sbg_data message
 *
 * @return [m/s/s]  Vertical acceleration from ship motion profile (positive down) 
 */
static inline float mavlink_msg_sbg_data_get_acc_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field vel_z from sbg_data message
 *
 * @return [m/s]  Vertical velocity from ship motion profile (positive down)  
 */
static inline float mavlink_msg_sbg_data_get_vel_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field acc_x from sbg_data message
 *
 * @return [m/s/s]  X acceleration (filtered) 
 */
static inline float mavlink_msg_sbg_data_get_acc_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field acc_y from sbg_data message
 *
 * @return [m/s/s]  Y acceleration (filtered) 
 */
static inline float mavlink_msg_sbg_data_get_acc_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field gyro_x from sbg_data message
 *
 * @return [rad/s]  Filtered Gyroscope - X axis rad/s  
 */
static inline float mavlink_msg_sbg_data_get_gyro_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field gyro_y from sbg_data message
 *
 * @return [rad/s]  Filtered Gyroscope - Y axis rad/s  
 */
static inline float mavlink_msg_sbg_data_get_gyro_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field gyro_z from sbg_data message
 *
 * @return [rad/s]  Filtered Gyroscope - Z axis rad/s  
 */
static inline float mavlink_msg_sbg_data_get_gyro_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field ekf_roll from sbg_data message
 *
 * @return [rad]  Roll angle filtered with SBG EKF 
 */
static inline float mavlink_msg_sbg_data_get_ekf_roll(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field ekf_pitch from sbg_data message
 *
 * @return [rad]  Roll angle filtered with SBG EKF 
 */
static inline float mavlink_msg_sbg_data_get_ekf_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field ekf_yaw from sbg_data message
 *
 * @return [rad]  Roll angle filtered with SBG EKF 
 */
static inline float mavlink_msg_sbg_data_get_ekf_yaw(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field temp from sbg_data message
 *
 * @return [cdegC]  Internal Temperature degC  
 */
static inline float mavlink_msg_sbg_data_get_temp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field heave from sbg_data message
 *
 * @return [m]  Heave (positive down) at main location (in m)  
 */
static inline float mavlink_msg_sbg_data_get_heave(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field main_power from sbg_data message
 *
 * @return    (BOOL) True when main power supply is OK. 
 */
static inline uint8_t mavlink_msg_sbg_data_get_main_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field imu_power from sbg_data message
 *
 * @return     (BOOL) True when IMU power supply is OK. 
 */
static inline uint8_t mavlink_msg_sbg_data_get_imu_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field gps_power from sbg_data message
 *
 * @return     (BOOL) Set to True when GPS power supply is OK. 
 */
static inline uint8_t mavlink_msg_sbg_data_get_gps_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field settings from sbg_data message
 *
 * @return      (BOOL) True if settings were correctly loaded 
 */
static inline uint8_t mavlink_msg_sbg_data_get_settings(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Get field temperature from sbg_data message
 *
 * @return   (BOOL) True when temperature is within specified limits. 
 */
static inline uint8_t mavlink_msg_sbg_data_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field port_a from sbg_data message
 *
 * @return        (BOOL) PORT A: False in case of low level communication error. 
 */
static inline uint8_t mavlink_msg_sbg_data_get_port_a(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Get field imu_com from sbg_data message
 *
 * @return       (BOOL) True if the communication with the IMU is ok. 
 */
static inline uint8_t mavlink_msg_sbg_data_get_imu_com(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  54);
}

/**
 * @brief Get field imu_status from sbg_data message
 *
 * @return    (BOOL) True if internal IMU passes Built In Test (Calibration, CPU) 
 */
static inline uint8_t mavlink_msg_sbg_data_get_imu_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  55);
}

/**
 * @brief Get field heave_valid from sbg_data message
 *
 * @return    (BOOL) True after heave convergence time. 
 */
static inline uint8_t mavlink_msg_sbg_data_get_heave_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  56);
}

/**
 * @brief Decode a sbg_data message into a struct
 *
 * @param msg The message to decode
 * @param sbg_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_sbg_data_decode(const mavlink_message_t* msg, mavlink_sbg_data_t* sbg_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sbg_data->acc_z = mavlink_msg_sbg_data_get_acc_z(msg);
    sbg_data->vel_z = mavlink_msg_sbg_data_get_vel_z(msg);
    sbg_data->acc_x = mavlink_msg_sbg_data_get_acc_x(msg);
    sbg_data->acc_y = mavlink_msg_sbg_data_get_acc_y(msg);
    sbg_data->gyro_x = mavlink_msg_sbg_data_get_gyro_x(msg);
    sbg_data->gyro_y = mavlink_msg_sbg_data_get_gyro_y(msg);
    sbg_data->gyro_z = mavlink_msg_sbg_data_get_gyro_z(msg);
    sbg_data->ekf_roll = mavlink_msg_sbg_data_get_ekf_roll(msg);
    sbg_data->ekf_pitch = mavlink_msg_sbg_data_get_ekf_pitch(msg);
    sbg_data->ekf_yaw = mavlink_msg_sbg_data_get_ekf_yaw(msg);
    sbg_data->temp = mavlink_msg_sbg_data_get_temp(msg);
    sbg_data->heave = mavlink_msg_sbg_data_get_heave(msg);
    sbg_data->main_power = mavlink_msg_sbg_data_get_main_power(msg);
    sbg_data->imu_power = mavlink_msg_sbg_data_get_imu_power(msg);
    sbg_data->gps_power = mavlink_msg_sbg_data_get_gps_power(msg);
    sbg_data->settings = mavlink_msg_sbg_data_get_settings(msg);
    sbg_data->temperature = mavlink_msg_sbg_data_get_temperature(msg);
    sbg_data->port_a = mavlink_msg_sbg_data_get_port_a(msg);
    sbg_data->imu_com = mavlink_msg_sbg_data_get_imu_com(msg);
    sbg_data->imu_status = mavlink_msg_sbg_data_get_imu_status(msg);
    sbg_data->heave_valid = mavlink_msg_sbg_data_get_heave_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SBG_DATA_LEN? msg->len : MAVLINK_MSG_ID_SBG_DATA_LEN;
        memset(sbg_data, 0, MAVLINK_MSG_ID_SBG_DATA_LEN);
    memcpy(sbg_data, _MAV_PAYLOAD(msg), len);
#endif
}
