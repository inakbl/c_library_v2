#pragma once
// MESSAGE NAIA_CONTROL_STATE_PART_1 PACKING

#define MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1 612

MAVPACKED(
typedef struct __mavlink_naia_control_state_part_1_t {
 uint64_t timestamp; /*< [us] Timestamp. */
 uint64_t index; /*<   Message index. */
 float control_aoa_s; /*<   Control starboard angle calculated by the control loop. It will be commanded to its actuator. */
 float control_aoa_p; /*<   Control port angle calculated by the control loop. It will be commanded to its actuator. */
 float control_aoa_r; /*<   Control rear angle calculated by the control loop. It will be commanded to its actuator. */
 float control_aoa_c; /*<   Control central angle calculated by the control loop. It will be commanded to its actuator. */
 float static_pressure_s; /*<   Informs about the static pressure calculated at the starboard pitot. */
 float static_pressure_p; /*<   Informs about the static pressure calculated at the port pitot. */
 float static_pressure_r; /*<   Informs about the static pressure calculated at the rear pitot. */
 float speed_s[2]; /*<   Informs about the speed calculated at the starboard pitot and its standard deviation. Vector [2 x 1]. */
 float speed_p[2]; /*<   Informs about the speed calculated at the port pitot and its standard deviation. Vector [2 x 1]. */
 float speed_r[2]; /*<   Informs about the speed calculated at the rear pitot and its standard deviation. Vector [2 x 1]. */
 float speed_imu[2]; /*<   Informs about the speed calculated by the IMU and its standard deviation. */
 float speed_control[2]; /*<   Informs about the speed calculated by the fusion of all the sensors mounted and its standard deviation. */
 float raw_speed_control[2]; /*<   Informs about the raw speed calculated by the fusion of all the sensors mounted and its standard deviation. */
 float error_depth; /*<   Informs about the control error depth. */
 float error_heel; /*<   Informs about the control error heel. */
 float error_pitch; /*<   Informs about the control error pitch. */
 float local_depth; /*<   Informs about the local depth measured by the sensors. */
 float local_heel; /*<   Informs about the local heel measured by the sensors. */
 float local_pitch; /*<   Informs about the local pitch measured by the sensors. */
 float target_depth; /*<   Informs about the instantaneous target depth of the control. */
 float target_pitch; /*<   Informs about the instantaneous target height of the center of mass of the boat. */
 float aoa_depth[2]; /*<   Informs about the cummulative and instantaneous control signal associated with depth control. */
 float aoa_heel[2]; /*<   Informs about the cummulative and instantaneous control signal associated with heel control. */
 float aoa_pitch[2]; /*<   Informs about the cummulative and instantaneous control signal associated with pitch control. */
 float raw_pressure_s[6]; /*<   Copy of the input raw_pressure_s. */
 float raw_pressure_p[6]; /*<   Copy of the input raw_pressure_p. */
 float raw_pressure_r[6]; /*<   Copy of the input raw_pressure_r. */
 uint8_t status; /*<   Control algorithm Informs about whether the control loop is calculating new actuator positions (1) or not (0). */
}) mavlink_naia_control_state_part_1_t;

#define MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN 221
#define MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_MIN_LEN 221
#define MAVLINK_MSG_ID_612_LEN 221
#define MAVLINK_MSG_ID_612_MIN_LEN 221

#define MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_CRC 235
#define MAVLINK_MSG_ID_612_CRC 235

#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_1_FIELD_SPEED_S_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_1_FIELD_SPEED_P_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_1_FIELD_SPEED_R_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_1_FIELD_SPEED_IMU_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_1_FIELD_SPEED_CONTROL_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_1_FIELD_RAW_SPEED_CONTROL_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_1_FIELD_AOA_DEPTH_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_1_FIELD_AOA_HEEL_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_1_FIELD_AOA_PITCH_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_1_FIELD_RAW_PRESSURE_S_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_1_FIELD_RAW_PRESSURE_P_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_STATE_PART_1_FIELD_RAW_PRESSURE_R_LEN 6

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAIA_CONTROL_STATE_PART_1 { \
    612, \
    "NAIA_CONTROL_STATE_PART_1", \
    30, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_naia_control_state_part_1_t, timestamp) }, \
         { "index", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_naia_control_state_part_1_t, index) }, \
         { "control_aoa_s", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_naia_control_state_part_1_t, control_aoa_s) }, \
         { "control_aoa_p", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_naia_control_state_part_1_t, control_aoa_p) }, \
         { "control_aoa_r", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_naia_control_state_part_1_t, control_aoa_r) }, \
         { "control_aoa_c", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_naia_control_state_part_1_t, control_aoa_c) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 220, offsetof(mavlink_naia_control_state_part_1_t, status) }, \
         { "static_pressure_s", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_naia_control_state_part_1_t, static_pressure_s) }, \
         { "static_pressure_p", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_naia_control_state_part_1_t, static_pressure_p) }, \
         { "static_pressure_r", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_naia_control_state_part_1_t, static_pressure_r) }, \
         { "speed_s", NULL, MAVLINK_TYPE_FLOAT, 2, 44, offsetof(mavlink_naia_control_state_part_1_t, speed_s) }, \
         { "speed_p", NULL, MAVLINK_TYPE_FLOAT, 2, 52, offsetof(mavlink_naia_control_state_part_1_t, speed_p) }, \
         { "speed_r", NULL, MAVLINK_TYPE_FLOAT, 2, 60, offsetof(mavlink_naia_control_state_part_1_t, speed_r) }, \
         { "speed_imu", NULL, MAVLINK_TYPE_FLOAT, 2, 68, offsetof(mavlink_naia_control_state_part_1_t, speed_imu) }, \
         { "speed_control", NULL, MAVLINK_TYPE_FLOAT, 2, 76, offsetof(mavlink_naia_control_state_part_1_t, speed_control) }, \
         { "raw_speed_control", NULL, MAVLINK_TYPE_FLOAT, 2, 84, offsetof(mavlink_naia_control_state_part_1_t, raw_speed_control) }, \
         { "error_depth", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_naia_control_state_part_1_t, error_depth) }, \
         { "error_heel", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_naia_control_state_part_1_t, error_heel) }, \
         { "error_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_naia_control_state_part_1_t, error_pitch) }, \
         { "local_depth", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_naia_control_state_part_1_t, local_depth) }, \
         { "local_heel", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_naia_control_state_part_1_t, local_heel) }, \
         { "local_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_naia_control_state_part_1_t, local_pitch) }, \
         { "target_depth", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_naia_control_state_part_1_t, target_depth) }, \
         { "target_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_naia_control_state_part_1_t, target_pitch) }, \
         { "aoa_depth", NULL, MAVLINK_TYPE_FLOAT, 2, 124, offsetof(mavlink_naia_control_state_part_1_t, aoa_depth) }, \
         { "aoa_heel", NULL, MAVLINK_TYPE_FLOAT, 2, 132, offsetof(mavlink_naia_control_state_part_1_t, aoa_heel) }, \
         { "aoa_pitch", NULL, MAVLINK_TYPE_FLOAT, 2, 140, offsetof(mavlink_naia_control_state_part_1_t, aoa_pitch) }, \
         { "raw_pressure_s", NULL, MAVLINK_TYPE_FLOAT, 6, 148, offsetof(mavlink_naia_control_state_part_1_t, raw_pressure_s) }, \
         { "raw_pressure_p", NULL, MAVLINK_TYPE_FLOAT, 6, 172, offsetof(mavlink_naia_control_state_part_1_t, raw_pressure_p) }, \
         { "raw_pressure_r", NULL, MAVLINK_TYPE_FLOAT, 6, 196, offsetof(mavlink_naia_control_state_part_1_t, raw_pressure_r) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAIA_CONTROL_STATE_PART_1 { \
    "NAIA_CONTROL_STATE_PART_1", \
    30, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_naia_control_state_part_1_t, timestamp) }, \
         { "index", NULL, MAVLINK_TYPE_UINT64_T, 0, 8, offsetof(mavlink_naia_control_state_part_1_t, index) }, \
         { "control_aoa_s", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_naia_control_state_part_1_t, control_aoa_s) }, \
         { "control_aoa_p", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_naia_control_state_part_1_t, control_aoa_p) }, \
         { "control_aoa_r", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_naia_control_state_part_1_t, control_aoa_r) }, \
         { "control_aoa_c", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_naia_control_state_part_1_t, control_aoa_c) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 220, offsetof(mavlink_naia_control_state_part_1_t, status) }, \
         { "static_pressure_s", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_naia_control_state_part_1_t, static_pressure_s) }, \
         { "static_pressure_p", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_naia_control_state_part_1_t, static_pressure_p) }, \
         { "static_pressure_r", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_naia_control_state_part_1_t, static_pressure_r) }, \
         { "speed_s", NULL, MAVLINK_TYPE_FLOAT, 2, 44, offsetof(mavlink_naia_control_state_part_1_t, speed_s) }, \
         { "speed_p", NULL, MAVLINK_TYPE_FLOAT, 2, 52, offsetof(mavlink_naia_control_state_part_1_t, speed_p) }, \
         { "speed_r", NULL, MAVLINK_TYPE_FLOAT, 2, 60, offsetof(mavlink_naia_control_state_part_1_t, speed_r) }, \
         { "speed_imu", NULL, MAVLINK_TYPE_FLOAT, 2, 68, offsetof(mavlink_naia_control_state_part_1_t, speed_imu) }, \
         { "speed_control", NULL, MAVLINK_TYPE_FLOAT, 2, 76, offsetof(mavlink_naia_control_state_part_1_t, speed_control) }, \
         { "raw_speed_control", NULL, MAVLINK_TYPE_FLOAT, 2, 84, offsetof(mavlink_naia_control_state_part_1_t, raw_speed_control) }, \
         { "error_depth", NULL, MAVLINK_TYPE_FLOAT, 0, 92, offsetof(mavlink_naia_control_state_part_1_t, error_depth) }, \
         { "error_heel", NULL, MAVLINK_TYPE_FLOAT, 0, 96, offsetof(mavlink_naia_control_state_part_1_t, error_heel) }, \
         { "error_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 100, offsetof(mavlink_naia_control_state_part_1_t, error_pitch) }, \
         { "local_depth", NULL, MAVLINK_TYPE_FLOAT, 0, 104, offsetof(mavlink_naia_control_state_part_1_t, local_depth) }, \
         { "local_heel", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_naia_control_state_part_1_t, local_heel) }, \
         { "local_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_naia_control_state_part_1_t, local_pitch) }, \
         { "target_depth", NULL, MAVLINK_TYPE_FLOAT, 0, 116, offsetof(mavlink_naia_control_state_part_1_t, target_depth) }, \
         { "target_pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 120, offsetof(mavlink_naia_control_state_part_1_t, target_pitch) }, \
         { "aoa_depth", NULL, MAVLINK_TYPE_FLOAT, 2, 124, offsetof(mavlink_naia_control_state_part_1_t, aoa_depth) }, \
         { "aoa_heel", NULL, MAVLINK_TYPE_FLOAT, 2, 132, offsetof(mavlink_naia_control_state_part_1_t, aoa_heel) }, \
         { "aoa_pitch", NULL, MAVLINK_TYPE_FLOAT, 2, 140, offsetof(mavlink_naia_control_state_part_1_t, aoa_pitch) }, \
         { "raw_pressure_s", NULL, MAVLINK_TYPE_FLOAT, 6, 148, offsetof(mavlink_naia_control_state_part_1_t, raw_pressure_s) }, \
         { "raw_pressure_p", NULL, MAVLINK_TYPE_FLOAT, 6, 172, offsetof(mavlink_naia_control_state_part_1_t, raw_pressure_p) }, \
         { "raw_pressure_r", NULL, MAVLINK_TYPE_FLOAT, 6, 196, offsetof(mavlink_naia_control_state_part_1_t, raw_pressure_r) }, \
         } \
}
#endif

/**
 * @brief Pack a naia_control_state_part_1 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp. 
 * @param index   Message index. 
 * @param control_aoa_s   Control starboard angle calculated by the control loop. It will be commanded to its actuator. 
 * @param control_aoa_p   Control port angle calculated by the control loop. It will be commanded to its actuator. 
 * @param control_aoa_r   Control rear angle calculated by the control loop. It will be commanded to its actuator. 
 * @param control_aoa_c   Control central angle calculated by the control loop. It will be commanded to its actuator. 
 * @param status   Control algorithm Informs about whether the control loop is calculating new actuator positions (1) or not (0). 
 * @param static_pressure_s   Informs about the static pressure calculated at the starboard pitot. 
 * @param static_pressure_p   Informs about the static pressure calculated at the port pitot. 
 * @param static_pressure_r   Informs about the static pressure calculated at the rear pitot. 
 * @param speed_s   Informs about the speed calculated at the starboard pitot and its standard deviation. Vector [2 x 1]. 
 * @param speed_p   Informs about the speed calculated at the port pitot and its standard deviation. Vector [2 x 1]. 
 * @param speed_r   Informs about the speed calculated at the rear pitot and its standard deviation. Vector [2 x 1]. 
 * @param speed_imu   Informs about the speed calculated by the IMU and its standard deviation. 
 * @param speed_control   Informs about the speed calculated by the fusion of all the sensors mounted and its standard deviation. 
 * @param raw_speed_control   Informs about the raw speed calculated by the fusion of all the sensors mounted and its standard deviation. 
 * @param error_depth   Informs about the control error depth. 
 * @param error_heel   Informs about the control error heel. 
 * @param error_pitch   Informs about the control error pitch. 
 * @param local_depth   Informs about the local depth measured by the sensors. 
 * @param local_heel   Informs about the local heel measured by the sensors. 
 * @param local_pitch   Informs about the local pitch measured by the sensors. 
 * @param target_depth   Informs about the instantaneous target depth of the control. 
 * @param target_pitch   Informs about the instantaneous target height of the center of mass of the boat. 
 * @param aoa_depth   Informs about the cummulative and instantaneous control signal associated with depth control. 
 * @param aoa_heel   Informs about the cummulative and instantaneous control signal associated with heel control. 
 * @param aoa_pitch   Informs about the cummulative and instantaneous control signal associated with pitch control. 
 * @param raw_pressure_s   Copy of the input raw_pressure_s. 
 * @param raw_pressure_p   Copy of the input raw_pressure_p. 
 * @param raw_pressure_r   Copy of the input raw_pressure_r. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint64_t index, float control_aoa_s, float control_aoa_p, float control_aoa_r, float control_aoa_c, uint8_t status, float static_pressure_s, float static_pressure_p, float static_pressure_r, const float *speed_s, const float *speed_p, const float *speed_r, const float *speed_imu, const float *speed_control, const float *raw_speed_control, float error_depth, float error_heel, float error_pitch, float local_depth, float local_heel, float local_pitch, float target_depth, float target_pitch, const float *aoa_depth, const float *aoa_heel, const float *aoa_pitch, const float *raw_pressure_s, const float *raw_pressure_p, const float *raw_pressure_r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, index);
    _mav_put_float(buf, 16, control_aoa_s);
    _mav_put_float(buf, 20, control_aoa_p);
    _mav_put_float(buf, 24, control_aoa_r);
    _mav_put_float(buf, 28, control_aoa_c);
    _mav_put_float(buf, 32, static_pressure_s);
    _mav_put_float(buf, 36, static_pressure_p);
    _mav_put_float(buf, 40, static_pressure_r);
    _mav_put_float(buf, 92, error_depth);
    _mav_put_float(buf, 96, error_heel);
    _mav_put_float(buf, 100, error_pitch);
    _mav_put_float(buf, 104, local_depth);
    _mav_put_float(buf, 108, local_heel);
    _mav_put_float(buf, 112, local_pitch);
    _mav_put_float(buf, 116, target_depth);
    _mav_put_float(buf, 120, target_pitch);
    _mav_put_uint8_t(buf, 220, status);
    _mav_put_float_array(buf, 44, speed_s, 2);
    _mav_put_float_array(buf, 52, speed_p, 2);
    _mav_put_float_array(buf, 60, speed_r, 2);
    _mav_put_float_array(buf, 68, speed_imu, 2);
    _mav_put_float_array(buf, 76, speed_control, 2);
    _mav_put_float_array(buf, 84, raw_speed_control, 2);
    _mav_put_float_array(buf, 124, aoa_depth, 2);
    _mav_put_float_array(buf, 132, aoa_heel, 2);
    _mav_put_float_array(buf, 140, aoa_pitch, 2);
    _mav_put_float_array(buf, 148, raw_pressure_s, 6);
    _mav_put_float_array(buf, 172, raw_pressure_p, 6);
    _mav_put_float_array(buf, 196, raw_pressure_r, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN);
#else
    mavlink_naia_control_state_part_1_t packet;
    packet.timestamp = timestamp;
    packet.index = index;
    packet.control_aoa_s = control_aoa_s;
    packet.control_aoa_p = control_aoa_p;
    packet.control_aoa_r = control_aoa_r;
    packet.control_aoa_c = control_aoa_c;
    packet.static_pressure_s = static_pressure_s;
    packet.static_pressure_p = static_pressure_p;
    packet.static_pressure_r = static_pressure_r;
    packet.error_depth = error_depth;
    packet.error_heel = error_heel;
    packet.error_pitch = error_pitch;
    packet.local_depth = local_depth;
    packet.local_heel = local_heel;
    packet.local_pitch = local_pitch;
    packet.target_depth = target_depth;
    packet.target_pitch = target_pitch;
    packet.status = status;
    mav_array_memcpy(packet.speed_s, speed_s, sizeof(float)*2);
    mav_array_memcpy(packet.speed_p, speed_p, sizeof(float)*2);
    mav_array_memcpy(packet.speed_r, speed_r, sizeof(float)*2);
    mav_array_memcpy(packet.speed_imu, speed_imu, sizeof(float)*2);
    mav_array_memcpy(packet.speed_control, speed_control, sizeof(float)*2);
    mav_array_memcpy(packet.raw_speed_control, raw_speed_control, sizeof(float)*2);
    mav_array_memcpy(packet.aoa_depth, aoa_depth, sizeof(float)*2);
    mav_array_memcpy(packet.aoa_heel, aoa_heel, sizeof(float)*2);
    mav_array_memcpy(packet.aoa_pitch, aoa_pitch, sizeof(float)*2);
    mav_array_memcpy(packet.raw_pressure_s, raw_pressure_s, sizeof(float)*6);
    mav_array_memcpy(packet.raw_pressure_p, raw_pressure_p, sizeof(float)*6);
    mav_array_memcpy(packet.raw_pressure_r, raw_pressure_r, sizeof(float)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_CRC);
}

/**
 * @brief Pack a naia_control_state_part_1 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp. 
 * @param index   Message index. 
 * @param control_aoa_s   Control starboard angle calculated by the control loop. It will be commanded to its actuator. 
 * @param control_aoa_p   Control port angle calculated by the control loop. It will be commanded to its actuator. 
 * @param control_aoa_r   Control rear angle calculated by the control loop. It will be commanded to its actuator. 
 * @param control_aoa_c   Control central angle calculated by the control loop. It will be commanded to its actuator. 
 * @param status   Control algorithm Informs about whether the control loop is calculating new actuator positions (1) or not (0). 
 * @param static_pressure_s   Informs about the static pressure calculated at the starboard pitot. 
 * @param static_pressure_p   Informs about the static pressure calculated at the port pitot. 
 * @param static_pressure_r   Informs about the static pressure calculated at the rear pitot. 
 * @param speed_s   Informs about the speed calculated at the starboard pitot and its standard deviation. Vector [2 x 1]. 
 * @param speed_p   Informs about the speed calculated at the port pitot and its standard deviation. Vector [2 x 1]. 
 * @param speed_r   Informs about the speed calculated at the rear pitot and its standard deviation. Vector [2 x 1]. 
 * @param speed_imu   Informs about the speed calculated by the IMU and its standard deviation. 
 * @param speed_control   Informs about the speed calculated by the fusion of all the sensors mounted and its standard deviation. 
 * @param raw_speed_control   Informs about the raw speed calculated by the fusion of all the sensors mounted and its standard deviation. 
 * @param error_depth   Informs about the control error depth. 
 * @param error_heel   Informs about the control error heel. 
 * @param error_pitch   Informs about the control error pitch. 
 * @param local_depth   Informs about the local depth measured by the sensors. 
 * @param local_heel   Informs about the local heel measured by the sensors. 
 * @param local_pitch   Informs about the local pitch measured by the sensors. 
 * @param target_depth   Informs about the instantaneous target depth of the control. 
 * @param target_pitch   Informs about the instantaneous target height of the center of mass of the boat. 
 * @param aoa_depth   Informs about the cummulative and instantaneous control signal associated with depth control. 
 * @param aoa_heel   Informs about the cummulative and instantaneous control signal associated with heel control. 
 * @param aoa_pitch   Informs about the cummulative and instantaneous control signal associated with pitch control. 
 * @param raw_pressure_s   Copy of the input raw_pressure_s. 
 * @param raw_pressure_p   Copy of the input raw_pressure_p. 
 * @param raw_pressure_r   Copy of the input raw_pressure_r. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint64_t index,float control_aoa_s,float control_aoa_p,float control_aoa_r,float control_aoa_c,uint8_t status,float static_pressure_s,float static_pressure_p,float static_pressure_r,const float *speed_s,const float *speed_p,const float *speed_r,const float *speed_imu,const float *speed_control,const float *raw_speed_control,float error_depth,float error_heel,float error_pitch,float local_depth,float local_heel,float local_pitch,float target_depth,float target_pitch,const float *aoa_depth,const float *aoa_heel,const float *aoa_pitch,const float *raw_pressure_s,const float *raw_pressure_p,const float *raw_pressure_r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, index);
    _mav_put_float(buf, 16, control_aoa_s);
    _mav_put_float(buf, 20, control_aoa_p);
    _mav_put_float(buf, 24, control_aoa_r);
    _mav_put_float(buf, 28, control_aoa_c);
    _mav_put_float(buf, 32, static_pressure_s);
    _mav_put_float(buf, 36, static_pressure_p);
    _mav_put_float(buf, 40, static_pressure_r);
    _mav_put_float(buf, 92, error_depth);
    _mav_put_float(buf, 96, error_heel);
    _mav_put_float(buf, 100, error_pitch);
    _mav_put_float(buf, 104, local_depth);
    _mav_put_float(buf, 108, local_heel);
    _mav_put_float(buf, 112, local_pitch);
    _mav_put_float(buf, 116, target_depth);
    _mav_put_float(buf, 120, target_pitch);
    _mav_put_uint8_t(buf, 220, status);
    _mav_put_float_array(buf, 44, speed_s, 2);
    _mav_put_float_array(buf, 52, speed_p, 2);
    _mav_put_float_array(buf, 60, speed_r, 2);
    _mav_put_float_array(buf, 68, speed_imu, 2);
    _mav_put_float_array(buf, 76, speed_control, 2);
    _mav_put_float_array(buf, 84, raw_speed_control, 2);
    _mav_put_float_array(buf, 124, aoa_depth, 2);
    _mav_put_float_array(buf, 132, aoa_heel, 2);
    _mav_put_float_array(buf, 140, aoa_pitch, 2);
    _mav_put_float_array(buf, 148, raw_pressure_s, 6);
    _mav_put_float_array(buf, 172, raw_pressure_p, 6);
    _mav_put_float_array(buf, 196, raw_pressure_r, 6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN);
#else
    mavlink_naia_control_state_part_1_t packet;
    packet.timestamp = timestamp;
    packet.index = index;
    packet.control_aoa_s = control_aoa_s;
    packet.control_aoa_p = control_aoa_p;
    packet.control_aoa_r = control_aoa_r;
    packet.control_aoa_c = control_aoa_c;
    packet.static_pressure_s = static_pressure_s;
    packet.static_pressure_p = static_pressure_p;
    packet.static_pressure_r = static_pressure_r;
    packet.error_depth = error_depth;
    packet.error_heel = error_heel;
    packet.error_pitch = error_pitch;
    packet.local_depth = local_depth;
    packet.local_heel = local_heel;
    packet.local_pitch = local_pitch;
    packet.target_depth = target_depth;
    packet.target_pitch = target_pitch;
    packet.status = status;
    mav_array_memcpy(packet.speed_s, speed_s, sizeof(float)*2);
    mav_array_memcpy(packet.speed_p, speed_p, sizeof(float)*2);
    mav_array_memcpy(packet.speed_r, speed_r, sizeof(float)*2);
    mav_array_memcpy(packet.speed_imu, speed_imu, sizeof(float)*2);
    mav_array_memcpy(packet.speed_control, speed_control, sizeof(float)*2);
    mav_array_memcpy(packet.raw_speed_control, raw_speed_control, sizeof(float)*2);
    mav_array_memcpy(packet.aoa_depth, aoa_depth, sizeof(float)*2);
    mav_array_memcpy(packet.aoa_heel, aoa_heel, sizeof(float)*2);
    mav_array_memcpy(packet.aoa_pitch, aoa_pitch, sizeof(float)*2);
    mav_array_memcpy(packet.raw_pressure_s, raw_pressure_s, sizeof(float)*6);
    mav_array_memcpy(packet.raw_pressure_p, raw_pressure_p, sizeof(float)*6);
    mav_array_memcpy(packet.raw_pressure_r, raw_pressure_r, sizeof(float)*6);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_CRC);
}

/**
 * @brief Encode a naia_control_state_part_1 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param naia_control_state_part_1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_naia_control_state_part_1_t* naia_control_state_part_1)
{
    return mavlink_msg_naia_control_state_part_1_pack(system_id, component_id, msg, naia_control_state_part_1->timestamp, naia_control_state_part_1->index, naia_control_state_part_1->control_aoa_s, naia_control_state_part_1->control_aoa_p, naia_control_state_part_1->control_aoa_r, naia_control_state_part_1->control_aoa_c, naia_control_state_part_1->status, naia_control_state_part_1->static_pressure_s, naia_control_state_part_1->static_pressure_p, naia_control_state_part_1->static_pressure_r, naia_control_state_part_1->speed_s, naia_control_state_part_1->speed_p, naia_control_state_part_1->speed_r, naia_control_state_part_1->speed_imu, naia_control_state_part_1->speed_control, naia_control_state_part_1->raw_speed_control, naia_control_state_part_1->error_depth, naia_control_state_part_1->error_heel, naia_control_state_part_1->error_pitch, naia_control_state_part_1->local_depth, naia_control_state_part_1->local_heel, naia_control_state_part_1->local_pitch, naia_control_state_part_1->target_depth, naia_control_state_part_1->target_pitch, naia_control_state_part_1->aoa_depth, naia_control_state_part_1->aoa_heel, naia_control_state_part_1->aoa_pitch, naia_control_state_part_1->raw_pressure_s, naia_control_state_part_1->raw_pressure_p, naia_control_state_part_1->raw_pressure_r);
}

/**
 * @brief Encode a naia_control_state_part_1 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param naia_control_state_part_1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_naia_control_state_part_1_t* naia_control_state_part_1)
{
    return mavlink_msg_naia_control_state_part_1_pack_chan(system_id, component_id, chan, msg, naia_control_state_part_1->timestamp, naia_control_state_part_1->index, naia_control_state_part_1->control_aoa_s, naia_control_state_part_1->control_aoa_p, naia_control_state_part_1->control_aoa_r, naia_control_state_part_1->control_aoa_c, naia_control_state_part_1->status, naia_control_state_part_1->static_pressure_s, naia_control_state_part_1->static_pressure_p, naia_control_state_part_1->static_pressure_r, naia_control_state_part_1->speed_s, naia_control_state_part_1->speed_p, naia_control_state_part_1->speed_r, naia_control_state_part_1->speed_imu, naia_control_state_part_1->speed_control, naia_control_state_part_1->raw_speed_control, naia_control_state_part_1->error_depth, naia_control_state_part_1->error_heel, naia_control_state_part_1->error_pitch, naia_control_state_part_1->local_depth, naia_control_state_part_1->local_heel, naia_control_state_part_1->local_pitch, naia_control_state_part_1->target_depth, naia_control_state_part_1->target_pitch, naia_control_state_part_1->aoa_depth, naia_control_state_part_1->aoa_heel, naia_control_state_part_1->aoa_pitch, naia_control_state_part_1->raw_pressure_s, naia_control_state_part_1->raw_pressure_p, naia_control_state_part_1->raw_pressure_r);
}

/**
 * @brief Send a naia_control_state_part_1 message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp. 
 * @param index   Message index. 
 * @param control_aoa_s   Control starboard angle calculated by the control loop. It will be commanded to its actuator. 
 * @param control_aoa_p   Control port angle calculated by the control loop. It will be commanded to its actuator. 
 * @param control_aoa_r   Control rear angle calculated by the control loop. It will be commanded to its actuator. 
 * @param control_aoa_c   Control central angle calculated by the control loop. It will be commanded to its actuator. 
 * @param status   Control algorithm Informs about whether the control loop is calculating new actuator positions (1) or not (0). 
 * @param static_pressure_s   Informs about the static pressure calculated at the starboard pitot. 
 * @param static_pressure_p   Informs about the static pressure calculated at the port pitot. 
 * @param static_pressure_r   Informs about the static pressure calculated at the rear pitot. 
 * @param speed_s   Informs about the speed calculated at the starboard pitot and its standard deviation. Vector [2 x 1]. 
 * @param speed_p   Informs about the speed calculated at the port pitot and its standard deviation. Vector [2 x 1]. 
 * @param speed_r   Informs about the speed calculated at the rear pitot and its standard deviation. Vector [2 x 1]. 
 * @param speed_imu   Informs about the speed calculated by the IMU and its standard deviation. 
 * @param speed_control   Informs about the speed calculated by the fusion of all the sensors mounted and its standard deviation. 
 * @param raw_speed_control   Informs about the raw speed calculated by the fusion of all the sensors mounted and its standard deviation. 
 * @param error_depth   Informs about the control error depth. 
 * @param error_heel   Informs about the control error heel. 
 * @param error_pitch   Informs about the control error pitch. 
 * @param local_depth   Informs about the local depth measured by the sensors. 
 * @param local_heel   Informs about the local heel measured by the sensors. 
 * @param local_pitch   Informs about the local pitch measured by the sensors. 
 * @param target_depth   Informs about the instantaneous target depth of the control. 
 * @param target_pitch   Informs about the instantaneous target height of the center of mass of the boat. 
 * @param aoa_depth   Informs about the cummulative and instantaneous control signal associated with depth control. 
 * @param aoa_heel   Informs about the cummulative and instantaneous control signal associated with heel control. 
 * @param aoa_pitch   Informs about the cummulative and instantaneous control signal associated with pitch control. 
 * @param raw_pressure_s   Copy of the input raw_pressure_s. 
 * @param raw_pressure_p   Copy of the input raw_pressure_p. 
 * @param raw_pressure_r   Copy of the input raw_pressure_r. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_naia_control_state_part_1_send(mavlink_channel_t chan, uint64_t timestamp, uint64_t index, float control_aoa_s, float control_aoa_p, float control_aoa_r, float control_aoa_c, uint8_t status, float static_pressure_s, float static_pressure_p, float static_pressure_r, const float *speed_s, const float *speed_p, const float *speed_r, const float *speed_imu, const float *speed_control, const float *raw_speed_control, float error_depth, float error_heel, float error_pitch, float local_depth, float local_heel, float local_pitch, float target_depth, float target_pitch, const float *aoa_depth, const float *aoa_heel, const float *aoa_pitch, const float *raw_pressure_s, const float *raw_pressure_p, const float *raw_pressure_r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, index);
    _mav_put_float(buf, 16, control_aoa_s);
    _mav_put_float(buf, 20, control_aoa_p);
    _mav_put_float(buf, 24, control_aoa_r);
    _mav_put_float(buf, 28, control_aoa_c);
    _mav_put_float(buf, 32, static_pressure_s);
    _mav_put_float(buf, 36, static_pressure_p);
    _mav_put_float(buf, 40, static_pressure_r);
    _mav_put_float(buf, 92, error_depth);
    _mav_put_float(buf, 96, error_heel);
    _mav_put_float(buf, 100, error_pitch);
    _mav_put_float(buf, 104, local_depth);
    _mav_put_float(buf, 108, local_heel);
    _mav_put_float(buf, 112, local_pitch);
    _mav_put_float(buf, 116, target_depth);
    _mav_put_float(buf, 120, target_pitch);
    _mav_put_uint8_t(buf, 220, status);
    _mav_put_float_array(buf, 44, speed_s, 2);
    _mav_put_float_array(buf, 52, speed_p, 2);
    _mav_put_float_array(buf, 60, speed_r, 2);
    _mav_put_float_array(buf, 68, speed_imu, 2);
    _mav_put_float_array(buf, 76, speed_control, 2);
    _mav_put_float_array(buf, 84, raw_speed_control, 2);
    _mav_put_float_array(buf, 124, aoa_depth, 2);
    _mav_put_float_array(buf, 132, aoa_heel, 2);
    _mav_put_float_array(buf, 140, aoa_pitch, 2);
    _mav_put_float_array(buf, 148, raw_pressure_s, 6);
    _mav_put_float_array(buf, 172, raw_pressure_p, 6);
    _mav_put_float_array(buf, 196, raw_pressure_r, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1, buf, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_CRC);
#else
    mavlink_naia_control_state_part_1_t packet;
    packet.timestamp = timestamp;
    packet.index = index;
    packet.control_aoa_s = control_aoa_s;
    packet.control_aoa_p = control_aoa_p;
    packet.control_aoa_r = control_aoa_r;
    packet.control_aoa_c = control_aoa_c;
    packet.static_pressure_s = static_pressure_s;
    packet.static_pressure_p = static_pressure_p;
    packet.static_pressure_r = static_pressure_r;
    packet.error_depth = error_depth;
    packet.error_heel = error_heel;
    packet.error_pitch = error_pitch;
    packet.local_depth = local_depth;
    packet.local_heel = local_heel;
    packet.local_pitch = local_pitch;
    packet.target_depth = target_depth;
    packet.target_pitch = target_pitch;
    packet.status = status;
    mav_array_memcpy(packet.speed_s, speed_s, sizeof(float)*2);
    mav_array_memcpy(packet.speed_p, speed_p, sizeof(float)*2);
    mav_array_memcpy(packet.speed_r, speed_r, sizeof(float)*2);
    mav_array_memcpy(packet.speed_imu, speed_imu, sizeof(float)*2);
    mav_array_memcpy(packet.speed_control, speed_control, sizeof(float)*2);
    mav_array_memcpy(packet.raw_speed_control, raw_speed_control, sizeof(float)*2);
    mav_array_memcpy(packet.aoa_depth, aoa_depth, sizeof(float)*2);
    mav_array_memcpy(packet.aoa_heel, aoa_heel, sizeof(float)*2);
    mav_array_memcpy(packet.aoa_pitch, aoa_pitch, sizeof(float)*2);
    mav_array_memcpy(packet.raw_pressure_s, raw_pressure_s, sizeof(float)*6);
    mav_array_memcpy(packet.raw_pressure_p, raw_pressure_p, sizeof(float)*6);
    mav_array_memcpy(packet.raw_pressure_r, raw_pressure_r, sizeof(float)*6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1, (const char *)&packet, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_CRC);
#endif
}

/**
 * @brief Send a naia_control_state_part_1 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_naia_control_state_part_1_send_struct(mavlink_channel_t chan, const mavlink_naia_control_state_part_1_t* naia_control_state_part_1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_naia_control_state_part_1_send(chan, naia_control_state_part_1->timestamp, naia_control_state_part_1->index, naia_control_state_part_1->control_aoa_s, naia_control_state_part_1->control_aoa_p, naia_control_state_part_1->control_aoa_r, naia_control_state_part_1->control_aoa_c, naia_control_state_part_1->status, naia_control_state_part_1->static_pressure_s, naia_control_state_part_1->static_pressure_p, naia_control_state_part_1->static_pressure_r, naia_control_state_part_1->speed_s, naia_control_state_part_1->speed_p, naia_control_state_part_1->speed_r, naia_control_state_part_1->speed_imu, naia_control_state_part_1->speed_control, naia_control_state_part_1->raw_speed_control, naia_control_state_part_1->error_depth, naia_control_state_part_1->error_heel, naia_control_state_part_1->error_pitch, naia_control_state_part_1->local_depth, naia_control_state_part_1->local_heel, naia_control_state_part_1->local_pitch, naia_control_state_part_1->target_depth, naia_control_state_part_1->target_pitch, naia_control_state_part_1->aoa_depth, naia_control_state_part_1->aoa_heel, naia_control_state_part_1->aoa_pitch, naia_control_state_part_1->raw_pressure_s, naia_control_state_part_1->raw_pressure_p, naia_control_state_part_1->raw_pressure_r);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1, (const char *)naia_control_state_part_1, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_naia_control_state_part_1_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint64_t index, float control_aoa_s, float control_aoa_p, float control_aoa_r, float control_aoa_c, uint8_t status, float static_pressure_s, float static_pressure_p, float static_pressure_r, const float *speed_s, const float *speed_p, const float *speed_r, const float *speed_imu, const float *speed_control, const float *raw_speed_control, float error_depth, float error_heel, float error_pitch, float local_depth, float local_heel, float local_pitch, float target_depth, float target_pitch, const float *aoa_depth, const float *aoa_heel, const float *aoa_pitch, const float *raw_pressure_s, const float *raw_pressure_p, const float *raw_pressure_r)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint64_t(buf, 8, index);
    _mav_put_float(buf, 16, control_aoa_s);
    _mav_put_float(buf, 20, control_aoa_p);
    _mav_put_float(buf, 24, control_aoa_r);
    _mav_put_float(buf, 28, control_aoa_c);
    _mav_put_float(buf, 32, static_pressure_s);
    _mav_put_float(buf, 36, static_pressure_p);
    _mav_put_float(buf, 40, static_pressure_r);
    _mav_put_float(buf, 92, error_depth);
    _mav_put_float(buf, 96, error_heel);
    _mav_put_float(buf, 100, error_pitch);
    _mav_put_float(buf, 104, local_depth);
    _mav_put_float(buf, 108, local_heel);
    _mav_put_float(buf, 112, local_pitch);
    _mav_put_float(buf, 116, target_depth);
    _mav_put_float(buf, 120, target_pitch);
    _mav_put_uint8_t(buf, 220, status);
    _mav_put_float_array(buf, 44, speed_s, 2);
    _mav_put_float_array(buf, 52, speed_p, 2);
    _mav_put_float_array(buf, 60, speed_r, 2);
    _mav_put_float_array(buf, 68, speed_imu, 2);
    _mav_put_float_array(buf, 76, speed_control, 2);
    _mav_put_float_array(buf, 84, raw_speed_control, 2);
    _mav_put_float_array(buf, 124, aoa_depth, 2);
    _mav_put_float_array(buf, 132, aoa_heel, 2);
    _mav_put_float_array(buf, 140, aoa_pitch, 2);
    _mav_put_float_array(buf, 148, raw_pressure_s, 6);
    _mav_put_float_array(buf, 172, raw_pressure_p, 6);
    _mav_put_float_array(buf, 196, raw_pressure_r, 6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1, buf, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_CRC);
#else
    mavlink_naia_control_state_part_1_t *packet = (mavlink_naia_control_state_part_1_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->index = index;
    packet->control_aoa_s = control_aoa_s;
    packet->control_aoa_p = control_aoa_p;
    packet->control_aoa_r = control_aoa_r;
    packet->control_aoa_c = control_aoa_c;
    packet->static_pressure_s = static_pressure_s;
    packet->static_pressure_p = static_pressure_p;
    packet->static_pressure_r = static_pressure_r;
    packet->error_depth = error_depth;
    packet->error_heel = error_heel;
    packet->error_pitch = error_pitch;
    packet->local_depth = local_depth;
    packet->local_heel = local_heel;
    packet->local_pitch = local_pitch;
    packet->target_depth = target_depth;
    packet->target_pitch = target_pitch;
    packet->status = status;
    mav_array_memcpy(packet->speed_s, speed_s, sizeof(float)*2);
    mav_array_memcpy(packet->speed_p, speed_p, sizeof(float)*2);
    mav_array_memcpy(packet->speed_r, speed_r, sizeof(float)*2);
    mav_array_memcpy(packet->speed_imu, speed_imu, sizeof(float)*2);
    mav_array_memcpy(packet->speed_control, speed_control, sizeof(float)*2);
    mav_array_memcpy(packet->raw_speed_control, raw_speed_control, sizeof(float)*2);
    mav_array_memcpy(packet->aoa_depth, aoa_depth, sizeof(float)*2);
    mav_array_memcpy(packet->aoa_heel, aoa_heel, sizeof(float)*2);
    mav_array_memcpy(packet->aoa_pitch, aoa_pitch, sizeof(float)*2);
    mav_array_memcpy(packet->raw_pressure_s, raw_pressure_s, sizeof(float)*6);
    mav_array_memcpy(packet->raw_pressure_p, raw_pressure_p, sizeof(float)*6);
    mav_array_memcpy(packet->raw_pressure_r, raw_pressure_r, sizeof(float)*6);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1, (const char *)packet, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_CRC);
#endif
}
#endif

#endif

// MESSAGE NAIA_CONTROL_STATE_PART_1 UNPACKING


/**
 * @brief Get field timestamp from naia_control_state_part_1 message
 *
 * @return [us] Timestamp. 
 */
static inline uint64_t mavlink_msg_naia_control_state_part_1_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field index from naia_control_state_part_1 message
 *
 * @return   Message index. 
 */
static inline uint64_t mavlink_msg_naia_control_state_part_1_get_index(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  8);
}

/**
 * @brief Get field control_aoa_s from naia_control_state_part_1 message
 *
 * @return   Control starboard angle calculated by the control loop. It will be commanded to its actuator. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_control_aoa_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field control_aoa_p from naia_control_state_part_1 message
 *
 * @return   Control port angle calculated by the control loop. It will be commanded to its actuator. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_control_aoa_p(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field control_aoa_r from naia_control_state_part_1 message
 *
 * @return   Control rear angle calculated by the control loop. It will be commanded to its actuator. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_control_aoa_r(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field control_aoa_c from naia_control_state_part_1 message
 *
 * @return   Control central angle calculated by the control loop. It will be commanded to its actuator. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_control_aoa_c(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field status from naia_control_state_part_1 message
 *
 * @return   Control algorithm Informs about whether the control loop is calculating new actuator positions (1) or not (0). 
 */
static inline uint8_t mavlink_msg_naia_control_state_part_1_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  220);
}

/**
 * @brief Get field static_pressure_s from naia_control_state_part_1 message
 *
 * @return   Informs about the static pressure calculated at the starboard pitot. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_static_pressure_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field static_pressure_p from naia_control_state_part_1 message
 *
 * @return   Informs about the static pressure calculated at the port pitot. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_static_pressure_p(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field static_pressure_r from naia_control_state_part_1 message
 *
 * @return   Informs about the static pressure calculated at the rear pitot. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_static_pressure_r(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field speed_s from naia_control_state_part_1 message
 *
 * @return   Informs about the speed calculated at the starboard pitot and its standard deviation. Vector [2 x 1]. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_get_speed_s(const mavlink_message_t* msg, float *speed_s)
{
    return _MAV_RETURN_float_array(msg, speed_s, 2,  44);
}

/**
 * @brief Get field speed_p from naia_control_state_part_1 message
 *
 * @return   Informs about the speed calculated at the port pitot and its standard deviation. Vector [2 x 1]. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_get_speed_p(const mavlink_message_t* msg, float *speed_p)
{
    return _MAV_RETURN_float_array(msg, speed_p, 2,  52);
}

/**
 * @brief Get field speed_r from naia_control_state_part_1 message
 *
 * @return   Informs about the speed calculated at the rear pitot and its standard deviation. Vector [2 x 1]. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_get_speed_r(const mavlink_message_t* msg, float *speed_r)
{
    return _MAV_RETURN_float_array(msg, speed_r, 2,  60);
}

/**
 * @brief Get field speed_imu from naia_control_state_part_1 message
 *
 * @return   Informs about the speed calculated by the IMU and its standard deviation. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_get_speed_imu(const mavlink_message_t* msg, float *speed_imu)
{
    return _MAV_RETURN_float_array(msg, speed_imu, 2,  68);
}

/**
 * @brief Get field speed_control from naia_control_state_part_1 message
 *
 * @return   Informs about the speed calculated by the fusion of all the sensors mounted and its standard deviation. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_get_speed_control(const mavlink_message_t* msg, float *speed_control)
{
    return _MAV_RETURN_float_array(msg, speed_control, 2,  76);
}

/**
 * @brief Get field raw_speed_control from naia_control_state_part_1 message
 *
 * @return   Informs about the raw speed calculated by the fusion of all the sensors mounted and its standard deviation. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_get_raw_speed_control(const mavlink_message_t* msg, float *raw_speed_control)
{
    return _MAV_RETURN_float_array(msg, raw_speed_control, 2,  84);
}

/**
 * @brief Get field error_depth from naia_control_state_part_1 message
 *
 * @return   Informs about the control error depth. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_error_depth(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  92);
}

/**
 * @brief Get field error_heel from naia_control_state_part_1 message
 *
 * @return   Informs about the control error heel. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_error_heel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  96);
}

/**
 * @brief Get field error_pitch from naia_control_state_part_1 message
 *
 * @return   Informs about the control error pitch. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_error_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  100);
}

/**
 * @brief Get field local_depth from naia_control_state_part_1 message
 *
 * @return   Informs about the local depth measured by the sensors. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_local_depth(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  104);
}

/**
 * @brief Get field local_heel from naia_control_state_part_1 message
 *
 * @return   Informs about the local heel measured by the sensors. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_local_heel(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Get field local_pitch from naia_control_state_part_1 message
 *
 * @return   Informs about the local pitch measured by the sensors. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_local_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Get field target_depth from naia_control_state_part_1 message
 *
 * @return   Informs about the instantaneous target depth of the control. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_target_depth(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  116);
}

/**
 * @brief Get field target_pitch from naia_control_state_part_1 message
 *
 * @return   Informs about the instantaneous target height of the center of mass of the boat. 
 */
static inline float mavlink_msg_naia_control_state_part_1_get_target_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  120);
}

/**
 * @brief Get field aoa_depth from naia_control_state_part_1 message
 *
 * @return   Informs about the cummulative and instantaneous control signal associated with depth control. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_get_aoa_depth(const mavlink_message_t* msg, float *aoa_depth)
{
    return _MAV_RETURN_float_array(msg, aoa_depth, 2,  124);
}

/**
 * @brief Get field aoa_heel from naia_control_state_part_1 message
 *
 * @return   Informs about the cummulative and instantaneous control signal associated with heel control. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_get_aoa_heel(const mavlink_message_t* msg, float *aoa_heel)
{
    return _MAV_RETURN_float_array(msg, aoa_heel, 2,  132);
}

/**
 * @brief Get field aoa_pitch from naia_control_state_part_1 message
 *
 * @return   Informs about the cummulative and instantaneous control signal associated with pitch control. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_get_aoa_pitch(const mavlink_message_t* msg, float *aoa_pitch)
{
    return _MAV_RETURN_float_array(msg, aoa_pitch, 2,  140);
}

/**
 * @brief Get field raw_pressure_s from naia_control_state_part_1 message
 *
 * @return   Copy of the input raw_pressure_s. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_get_raw_pressure_s(const mavlink_message_t* msg, float *raw_pressure_s)
{
    return _MAV_RETURN_float_array(msg, raw_pressure_s, 6,  148);
}

/**
 * @brief Get field raw_pressure_p from naia_control_state_part_1 message
 *
 * @return   Copy of the input raw_pressure_p. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_get_raw_pressure_p(const mavlink_message_t* msg, float *raw_pressure_p)
{
    return _MAV_RETURN_float_array(msg, raw_pressure_p, 6,  172);
}

/**
 * @brief Get field raw_pressure_r from naia_control_state_part_1 message
 *
 * @return   Copy of the input raw_pressure_r. 
 */
static inline uint16_t mavlink_msg_naia_control_state_part_1_get_raw_pressure_r(const mavlink_message_t* msg, float *raw_pressure_r)
{
    return _MAV_RETURN_float_array(msg, raw_pressure_r, 6,  196);
}

/**
 * @brief Decode a naia_control_state_part_1 message into a struct
 *
 * @param msg The message to decode
 * @param naia_control_state_part_1 C-struct to decode the message contents into
 */
static inline void mavlink_msg_naia_control_state_part_1_decode(const mavlink_message_t* msg, mavlink_naia_control_state_part_1_t* naia_control_state_part_1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    naia_control_state_part_1->timestamp = mavlink_msg_naia_control_state_part_1_get_timestamp(msg);
    naia_control_state_part_1->index = mavlink_msg_naia_control_state_part_1_get_index(msg);
    naia_control_state_part_1->control_aoa_s = mavlink_msg_naia_control_state_part_1_get_control_aoa_s(msg);
    naia_control_state_part_1->control_aoa_p = mavlink_msg_naia_control_state_part_1_get_control_aoa_p(msg);
    naia_control_state_part_1->control_aoa_r = mavlink_msg_naia_control_state_part_1_get_control_aoa_r(msg);
    naia_control_state_part_1->control_aoa_c = mavlink_msg_naia_control_state_part_1_get_control_aoa_c(msg);
    naia_control_state_part_1->static_pressure_s = mavlink_msg_naia_control_state_part_1_get_static_pressure_s(msg);
    naia_control_state_part_1->static_pressure_p = mavlink_msg_naia_control_state_part_1_get_static_pressure_p(msg);
    naia_control_state_part_1->static_pressure_r = mavlink_msg_naia_control_state_part_1_get_static_pressure_r(msg);
    mavlink_msg_naia_control_state_part_1_get_speed_s(msg, naia_control_state_part_1->speed_s);
    mavlink_msg_naia_control_state_part_1_get_speed_p(msg, naia_control_state_part_1->speed_p);
    mavlink_msg_naia_control_state_part_1_get_speed_r(msg, naia_control_state_part_1->speed_r);
    mavlink_msg_naia_control_state_part_1_get_speed_imu(msg, naia_control_state_part_1->speed_imu);
    mavlink_msg_naia_control_state_part_1_get_speed_control(msg, naia_control_state_part_1->speed_control);
    mavlink_msg_naia_control_state_part_1_get_raw_speed_control(msg, naia_control_state_part_1->raw_speed_control);
    naia_control_state_part_1->error_depth = mavlink_msg_naia_control_state_part_1_get_error_depth(msg);
    naia_control_state_part_1->error_heel = mavlink_msg_naia_control_state_part_1_get_error_heel(msg);
    naia_control_state_part_1->error_pitch = mavlink_msg_naia_control_state_part_1_get_error_pitch(msg);
    naia_control_state_part_1->local_depth = mavlink_msg_naia_control_state_part_1_get_local_depth(msg);
    naia_control_state_part_1->local_heel = mavlink_msg_naia_control_state_part_1_get_local_heel(msg);
    naia_control_state_part_1->local_pitch = mavlink_msg_naia_control_state_part_1_get_local_pitch(msg);
    naia_control_state_part_1->target_depth = mavlink_msg_naia_control_state_part_1_get_target_depth(msg);
    naia_control_state_part_1->target_pitch = mavlink_msg_naia_control_state_part_1_get_target_pitch(msg);
    mavlink_msg_naia_control_state_part_1_get_aoa_depth(msg, naia_control_state_part_1->aoa_depth);
    mavlink_msg_naia_control_state_part_1_get_aoa_heel(msg, naia_control_state_part_1->aoa_heel);
    mavlink_msg_naia_control_state_part_1_get_aoa_pitch(msg, naia_control_state_part_1->aoa_pitch);
    mavlink_msg_naia_control_state_part_1_get_raw_pressure_s(msg, naia_control_state_part_1->raw_pressure_s);
    mavlink_msg_naia_control_state_part_1_get_raw_pressure_p(msg, naia_control_state_part_1->raw_pressure_p);
    mavlink_msg_naia_control_state_part_1_get_raw_pressure_r(msg, naia_control_state_part_1->raw_pressure_r);
    naia_control_state_part_1->status = mavlink_msg_naia_control_state_part_1_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN? msg->len : MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN;
        memset(naia_control_state_part_1, 0, MAVLINK_MSG_ID_NAIA_CONTROL_STATE_PART_1_LEN);
    memcpy(naia_control_state_part_1, _MAV_PAYLOAD(msg), len);
#endif
}
