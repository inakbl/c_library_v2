#pragma once
// MESSAGE NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1 PACKING

#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1 623


typedef struct __mavlink_naia_control_wrapper_input_csv_settings_part1_t {
 float gains_speeds[3]; /*< [kts]  Speeds of the boat at which the control gains have been adjusted. The vector has m elements. */
 float target_speeds[2]; /*< [kts]  Speeds of the boat at which the reference height have been adjusted. The vector has m elements. */
 float target_height[2]; /*< [mm]  Vector that defines the target height at each measured speed. */
 float target_pitch[2]; /*< [deg]  Vector that defines the target pitch at each measured speed. */
 float kp_depth[3]; /*< [deg/s]  Defines the depth proportional gain vector. */
 float kd_depth[3]; /*< [deg/s]  Defines the depth derivative gain vector. */
 float kp_pitch[3]; /*< [deg/s]  Defines the pitch proportional gain vector. */
 float kd_pitch[3]; /*< [deg/s]  Defines the pitch derivative gain vector. */
 float kp_heel[3]; /*< [deg/s]  Defines the heel proportional gain vector. */
 float kd_heel[3]; /*< [deg/s]  Defines the heel derivative gain vector. */
 float lateral_depth_coupling; /*< [deg/s]  Defines the degree of coupling between the lateral semiwings and depth control desired. */
 float lateral_pitch_coupling; /*< [deg/s]  Defines the degree of coupling between the lateral semiwings and heel control desired. */
} mavlink_naia_control_wrapper_input_csv_settings_part1_t;

#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN 116
#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_MIN_LEN 116
#define MAVLINK_MSG_ID_623_LEN 116
#define MAVLINK_MSG_ID_623_MIN_LEN 116

#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_CRC 227
#define MAVLINK_MSG_ID_623_CRC 227

#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_FIELD_GAINS_SPEEDS_LEN 3
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_FIELD_TARGET_SPEEDS_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_FIELD_TARGET_HEIGHT_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_FIELD_TARGET_PITCH_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_FIELD_KP_DEPTH_LEN 3
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_FIELD_KD_DEPTH_LEN 3
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_FIELD_KP_PITCH_LEN 3
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_FIELD_KD_PITCH_LEN 3
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_FIELD_KP_HEEL_LEN 3
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_FIELD_KD_HEEL_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1 { \
    623, \
    "NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1", \
    12, \
    {  { "gains_speeds", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, gains_speeds) }, \
         { "target_speeds", NULL, MAVLINK_TYPE_FLOAT, 2, 12, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, target_speeds) }, \
         { "target_height", NULL, MAVLINK_TYPE_FLOAT, 2, 20, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, target_height) }, \
         { "target_pitch", NULL, MAVLINK_TYPE_FLOAT, 2, 28, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, target_pitch) }, \
         { "kp_depth", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, kp_depth) }, \
         { "kd_depth", NULL, MAVLINK_TYPE_FLOAT, 3, 48, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, kd_depth) }, \
         { "kp_pitch", NULL, MAVLINK_TYPE_FLOAT, 3, 60, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, kp_pitch) }, \
         { "kd_pitch", NULL, MAVLINK_TYPE_FLOAT, 3, 72, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, kd_pitch) }, \
         { "kp_heel", NULL, MAVLINK_TYPE_FLOAT, 3, 84, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, kp_heel) }, \
         { "kd_heel", NULL, MAVLINK_TYPE_FLOAT, 3, 96, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, kd_heel) }, \
         { "lateral_depth_coupling", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, lateral_depth_coupling) }, \
         { "lateral_pitch_coupling", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, lateral_pitch_coupling) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1 { \
    "NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1", \
    12, \
    {  { "gains_speeds", NULL, MAVLINK_TYPE_FLOAT, 3, 0, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, gains_speeds) }, \
         { "target_speeds", NULL, MAVLINK_TYPE_FLOAT, 2, 12, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, target_speeds) }, \
         { "target_height", NULL, MAVLINK_TYPE_FLOAT, 2, 20, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, target_height) }, \
         { "target_pitch", NULL, MAVLINK_TYPE_FLOAT, 2, 28, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, target_pitch) }, \
         { "kp_depth", NULL, MAVLINK_TYPE_FLOAT, 3, 36, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, kp_depth) }, \
         { "kd_depth", NULL, MAVLINK_TYPE_FLOAT, 3, 48, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, kd_depth) }, \
         { "kp_pitch", NULL, MAVLINK_TYPE_FLOAT, 3, 60, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, kp_pitch) }, \
         { "kd_pitch", NULL, MAVLINK_TYPE_FLOAT, 3, 72, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, kd_pitch) }, \
         { "kp_heel", NULL, MAVLINK_TYPE_FLOAT, 3, 84, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, kp_heel) }, \
         { "kd_heel", NULL, MAVLINK_TYPE_FLOAT, 3, 96, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, kd_heel) }, \
         { "lateral_depth_coupling", NULL, MAVLINK_TYPE_FLOAT, 0, 108, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, lateral_depth_coupling) }, \
         { "lateral_pitch_coupling", NULL, MAVLINK_TYPE_FLOAT, 0, 112, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part1_t, lateral_pitch_coupling) }, \
         } \
}
#endif

/**
 * @brief Pack a naia_control_wrapper_input_csv_settings_part1 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gains_speeds [kts]  Speeds of the boat at which the control gains have been adjusted. The vector has m elements. 
 * @param target_speeds [kts]  Speeds of the boat at which the reference height have been adjusted. The vector has m elements. 
 * @param target_height [mm]  Vector that defines the target height at each measured speed. 
 * @param target_pitch [deg]  Vector that defines the target pitch at each measured speed. 
 * @param kp_depth [deg/s]  Defines the depth proportional gain vector. 
 * @param kd_depth [deg/s]  Defines the depth derivative gain vector. 
 * @param kp_pitch [deg/s]  Defines the pitch proportional gain vector. 
 * @param kd_pitch [deg/s]  Defines the pitch derivative gain vector. 
 * @param kp_heel [deg/s]  Defines the heel proportional gain vector. 
 * @param kd_heel [deg/s]  Defines the heel derivative gain vector. 
 * @param lateral_depth_coupling [deg/s]  Defines the degree of coupling between the lateral semiwings and depth control desired. 
 * @param lateral_pitch_coupling [deg/s]  Defines the degree of coupling between the lateral semiwings and heel control desired. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *gains_speeds, const float *target_speeds, const float *target_height, const float *target_pitch, const float *kp_depth, const float *kd_depth, const float *kp_pitch, const float *kd_pitch, const float *kp_heel, const float *kd_heel, float lateral_depth_coupling, float lateral_pitch_coupling)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN];
    _mav_put_float(buf, 108, lateral_depth_coupling);
    _mav_put_float(buf, 112, lateral_pitch_coupling);
    _mav_put_float_array(buf, 0, gains_speeds, 3);
    _mav_put_float_array(buf, 12, target_speeds, 2);
    _mav_put_float_array(buf, 20, target_height, 2);
    _mav_put_float_array(buf, 28, target_pitch, 2);
    _mav_put_float_array(buf, 36, kp_depth, 3);
    _mav_put_float_array(buf, 48, kd_depth, 3);
    _mav_put_float_array(buf, 60, kp_pitch, 3);
    _mav_put_float_array(buf, 72, kd_pitch, 3);
    _mav_put_float_array(buf, 84, kp_heel, 3);
    _mav_put_float_array(buf, 96, kd_heel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN);
#else
    mavlink_naia_control_wrapper_input_csv_settings_part1_t packet;
    packet.lateral_depth_coupling = lateral_depth_coupling;
    packet.lateral_pitch_coupling = lateral_pitch_coupling;
    mav_array_memcpy(packet.gains_speeds, gains_speeds, sizeof(float)*3);
    mav_array_memcpy(packet.target_speeds, target_speeds, sizeof(float)*2);
    mav_array_memcpy(packet.target_height, target_height, sizeof(float)*2);
    mav_array_memcpy(packet.target_pitch, target_pitch, sizeof(float)*2);
    mav_array_memcpy(packet.kp_depth, kp_depth, sizeof(float)*3);
    mav_array_memcpy(packet.kd_depth, kd_depth, sizeof(float)*3);
    mav_array_memcpy(packet.kp_pitch, kp_pitch, sizeof(float)*3);
    mav_array_memcpy(packet.kd_pitch, kd_pitch, sizeof(float)*3);
    mav_array_memcpy(packet.kp_heel, kp_heel, sizeof(float)*3);
    mav_array_memcpy(packet.kd_heel, kd_heel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_CRC);
}

/**
 * @brief Pack a naia_control_wrapper_input_csv_settings_part1 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gains_speeds [kts]  Speeds of the boat at which the control gains have been adjusted. The vector has m elements. 
 * @param target_speeds [kts]  Speeds of the boat at which the reference height have been adjusted. The vector has m elements. 
 * @param target_height [mm]  Vector that defines the target height at each measured speed. 
 * @param target_pitch [deg]  Vector that defines the target pitch at each measured speed. 
 * @param kp_depth [deg/s]  Defines the depth proportional gain vector. 
 * @param kd_depth [deg/s]  Defines the depth derivative gain vector. 
 * @param kp_pitch [deg/s]  Defines the pitch proportional gain vector. 
 * @param kd_pitch [deg/s]  Defines the pitch derivative gain vector. 
 * @param kp_heel [deg/s]  Defines the heel proportional gain vector. 
 * @param kd_heel [deg/s]  Defines the heel derivative gain vector. 
 * @param lateral_depth_coupling [deg/s]  Defines the degree of coupling between the lateral semiwings and depth control desired. 
 * @param lateral_pitch_coupling [deg/s]  Defines the degree of coupling between the lateral semiwings and heel control desired. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *gains_speeds,const float *target_speeds,const float *target_height,const float *target_pitch,const float *kp_depth,const float *kd_depth,const float *kp_pitch,const float *kd_pitch,const float *kp_heel,const float *kd_heel,float lateral_depth_coupling,float lateral_pitch_coupling)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN];
    _mav_put_float(buf, 108, lateral_depth_coupling);
    _mav_put_float(buf, 112, lateral_pitch_coupling);
    _mav_put_float_array(buf, 0, gains_speeds, 3);
    _mav_put_float_array(buf, 12, target_speeds, 2);
    _mav_put_float_array(buf, 20, target_height, 2);
    _mav_put_float_array(buf, 28, target_pitch, 2);
    _mav_put_float_array(buf, 36, kp_depth, 3);
    _mav_put_float_array(buf, 48, kd_depth, 3);
    _mav_put_float_array(buf, 60, kp_pitch, 3);
    _mav_put_float_array(buf, 72, kd_pitch, 3);
    _mav_put_float_array(buf, 84, kp_heel, 3);
    _mav_put_float_array(buf, 96, kd_heel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN);
#else
    mavlink_naia_control_wrapper_input_csv_settings_part1_t packet;
    packet.lateral_depth_coupling = lateral_depth_coupling;
    packet.lateral_pitch_coupling = lateral_pitch_coupling;
    mav_array_memcpy(packet.gains_speeds, gains_speeds, sizeof(float)*3);
    mav_array_memcpy(packet.target_speeds, target_speeds, sizeof(float)*2);
    mav_array_memcpy(packet.target_height, target_height, sizeof(float)*2);
    mav_array_memcpy(packet.target_pitch, target_pitch, sizeof(float)*2);
    mav_array_memcpy(packet.kp_depth, kp_depth, sizeof(float)*3);
    mav_array_memcpy(packet.kd_depth, kd_depth, sizeof(float)*3);
    mav_array_memcpy(packet.kp_pitch, kp_pitch, sizeof(float)*3);
    mav_array_memcpy(packet.kd_pitch, kd_pitch, sizeof(float)*3);
    mav_array_memcpy(packet.kp_heel, kp_heel, sizeof(float)*3);
    mav_array_memcpy(packet.kd_heel, kd_heel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_CRC);
}

/**
 * @brief Encode a naia_control_wrapper_input_csv_settings_part1 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param naia_control_wrapper_input_csv_settings_part1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_naia_control_wrapper_input_csv_settings_part1_t* naia_control_wrapper_input_csv_settings_part1)
{
    return mavlink_msg_naia_control_wrapper_input_csv_settings_part1_pack(system_id, component_id, msg, naia_control_wrapper_input_csv_settings_part1->gains_speeds, naia_control_wrapper_input_csv_settings_part1->target_speeds, naia_control_wrapper_input_csv_settings_part1->target_height, naia_control_wrapper_input_csv_settings_part1->target_pitch, naia_control_wrapper_input_csv_settings_part1->kp_depth, naia_control_wrapper_input_csv_settings_part1->kd_depth, naia_control_wrapper_input_csv_settings_part1->kp_pitch, naia_control_wrapper_input_csv_settings_part1->kd_pitch, naia_control_wrapper_input_csv_settings_part1->kp_heel, naia_control_wrapper_input_csv_settings_part1->kd_heel, naia_control_wrapper_input_csv_settings_part1->lateral_depth_coupling, naia_control_wrapper_input_csv_settings_part1->lateral_pitch_coupling);
}

/**
 * @brief Encode a naia_control_wrapper_input_csv_settings_part1 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param naia_control_wrapper_input_csv_settings_part1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_naia_control_wrapper_input_csv_settings_part1_t* naia_control_wrapper_input_csv_settings_part1)
{
    return mavlink_msg_naia_control_wrapper_input_csv_settings_part1_pack_chan(system_id, component_id, chan, msg, naia_control_wrapper_input_csv_settings_part1->gains_speeds, naia_control_wrapper_input_csv_settings_part1->target_speeds, naia_control_wrapper_input_csv_settings_part1->target_height, naia_control_wrapper_input_csv_settings_part1->target_pitch, naia_control_wrapper_input_csv_settings_part1->kp_depth, naia_control_wrapper_input_csv_settings_part1->kd_depth, naia_control_wrapper_input_csv_settings_part1->kp_pitch, naia_control_wrapper_input_csv_settings_part1->kd_pitch, naia_control_wrapper_input_csv_settings_part1->kp_heel, naia_control_wrapper_input_csv_settings_part1->kd_heel, naia_control_wrapper_input_csv_settings_part1->lateral_depth_coupling, naia_control_wrapper_input_csv_settings_part1->lateral_pitch_coupling);
}

/**
 * @brief Send a naia_control_wrapper_input_csv_settings_part1 message
 * @param chan MAVLink channel to send the message
 *
 * @param gains_speeds [kts]  Speeds of the boat at which the control gains have been adjusted. The vector has m elements. 
 * @param target_speeds [kts]  Speeds of the boat at which the reference height have been adjusted. The vector has m elements. 
 * @param target_height [mm]  Vector that defines the target height at each measured speed. 
 * @param target_pitch [deg]  Vector that defines the target pitch at each measured speed. 
 * @param kp_depth [deg/s]  Defines the depth proportional gain vector. 
 * @param kd_depth [deg/s]  Defines the depth derivative gain vector. 
 * @param kp_pitch [deg/s]  Defines the pitch proportional gain vector. 
 * @param kd_pitch [deg/s]  Defines the pitch derivative gain vector. 
 * @param kp_heel [deg/s]  Defines the heel proportional gain vector. 
 * @param kd_heel [deg/s]  Defines the heel derivative gain vector. 
 * @param lateral_depth_coupling [deg/s]  Defines the degree of coupling between the lateral semiwings and depth control desired. 
 * @param lateral_pitch_coupling [deg/s]  Defines the degree of coupling between the lateral semiwings and heel control desired. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_naia_control_wrapper_input_csv_settings_part1_send(mavlink_channel_t chan, const float *gains_speeds, const float *target_speeds, const float *target_height, const float *target_pitch, const float *kp_depth, const float *kd_depth, const float *kp_pitch, const float *kd_pitch, const float *kp_heel, const float *kd_heel, float lateral_depth_coupling, float lateral_pitch_coupling)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN];
    _mav_put_float(buf, 108, lateral_depth_coupling);
    _mav_put_float(buf, 112, lateral_pitch_coupling);
    _mav_put_float_array(buf, 0, gains_speeds, 3);
    _mav_put_float_array(buf, 12, target_speeds, 2);
    _mav_put_float_array(buf, 20, target_height, 2);
    _mav_put_float_array(buf, 28, target_pitch, 2);
    _mav_put_float_array(buf, 36, kp_depth, 3);
    _mav_put_float_array(buf, 48, kd_depth, 3);
    _mav_put_float_array(buf, 60, kp_pitch, 3);
    _mav_put_float_array(buf, 72, kd_pitch, 3);
    _mav_put_float_array(buf, 84, kp_heel, 3);
    _mav_put_float_array(buf, 96, kd_heel, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1, buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_CRC);
#else
    mavlink_naia_control_wrapper_input_csv_settings_part1_t packet;
    packet.lateral_depth_coupling = lateral_depth_coupling;
    packet.lateral_pitch_coupling = lateral_pitch_coupling;
    mav_array_memcpy(packet.gains_speeds, gains_speeds, sizeof(float)*3);
    mav_array_memcpy(packet.target_speeds, target_speeds, sizeof(float)*2);
    mav_array_memcpy(packet.target_height, target_height, sizeof(float)*2);
    mav_array_memcpy(packet.target_pitch, target_pitch, sizeof(float)*2);
    mav_array_memcpy(packet.kp_depth, kp_depth, sizeof(float)*3);
    mav_array_memcpy(packet.kd_depth, kd_depth, sizeof(float)*3);
    mav_array_memcpy(packet.kp_pitch, kp_pitch, sizeof(float)*3);
    mav_array_memcpy(packet.kd_pitch, kd_pitch, sizeof(float)*3);
    mav_array_memcpy(packet.kp_heel, kp_heel, sizeof(float)*3);
    mav_array_memcpy(packet.kd_heel, kd_heel, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1, (const char *)&packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_CRC);
#endif
}

/**
 * @brief Send a naia_control_wrapper_input_csv_settings_part1 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_naia_control_wrapper_input_csv_settings_part1_send_struct(mavlink_channel_t chan, const mavlink_naia_control_wrapper_input_csv_settings_part1_t* naia_control_wrapper_input_csv_settings_part1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_naia_control_wrapper_input_csv_settings_part1_send(chan, naia_control_wrapper_input_csv_settings_part1->gains_speeds, naia_control_wrapper_input_csv_settings_part1->target_speeds, naia_control_wrapper_input_csv_settings_part1->target_height, naia_control_wrapper_input_csv_settings_part1->target_pitch, naia_control_wrapper_input_csv_settings_part1->kp_depth, naia_control_wrapper_input_csv_settings_part1->kd_depth, naia_control_wrapper_input_csv_settings_part1->kp_pitch, naia_control_wrapper_input_csv_settings_part1->kd_pitch, naia_control_wrapper_input_csv_settings_part1->kp_heel, naia_control_wrapper_input_csv_settings_part1->kd_heel, naia_control_wrapper_input_csv_settings_part1->lateral_depth_coupling, naia_control_wrapper_input_csv_settings_part1->lateral_pitch_coupling);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1, (const char *)naia_control_wrapper_input_csv_settings_part1, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_naia_control_wrapper_input_csv_settings_part1_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *gains_speeds, const float *target_speeds, const float *target_height, const float *target_pitch, const float *kp_depth, const float *kd_depth, const float *kp_pitch, const float *kd_pitch, const float *kp_heel, const float *kd_heel, float lateral_depth_coupling, float lateral_pitch_coupling)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 108, lateral_depth_coupling);
    _mav_put_float(buf, 112, lateral_pitch_coupling);
    _mav_put_float_array(buf, 0, gains_speeds, 3);
    _mav_put_float_array(buf, 12, target_speeds, 2);
    _mav_put_float_array(buf, 20, target_height, 2);
    _mav_put_float_array(buf, 28, target_pitch, 2);
    _mav_put_float_array(buf, 36, kp_depth, 3);
    _mav_put_float_array(buf, 48, kd_depth, 3);
    _mav_put_float_array(buf, 60, kp_pitch, 3);
    _mav_put_float_array(buf, 72, kd_pitch, 3);
    _mav_put_float_array(buf, 84, kp_heel, 3);
    _mav_put_float_array(buf, 96, kd_heel, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1, buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_CRC);
#else
    mavlink_naia_control_wrapper_input_csv_settings_part1_t *packet = (mavlink_naia_control_wrapper_input_csv_settings_part1_t *)msgbuf;
    packet->lateral_depth_coupling = lateral_depth_coupling;
    packet->lateral_pitch_coupling = lateral_pitch_coupling;
    mav_array_memcpy(packet->gains_speeds, gains_speeds, sizeof(float)*3);
    mav_array_memcpy(packet->target_speeds, target_speeds, sizeof(float)*2);
    mav_array_memcpy(packet->target_height, target_height, sizeof(float)*2);
    mav_array_memcpy(packet->target_pitch, target_pitch, sizeof(float)*2);
    mav_array_memcpy(packet->kp_depth, kp_depth, sizeof(float)*3);
    mav_array_memcpy(packet->kd_depth, kd_depth, sizeof(float)*3);
    mav_array_memcpy(packet->kp_pitch, kp_pitch, sizeof(float)*3);
    mav_array_memcpy(packet->kd_pitch, kd_pitch, sizeof(float)*3);
    mav_array_memcpy(packet->kp_heel, kp_heel, sizeof(float)*3);
    mav_array_memcpy(packet->kd_heel, kd_heel, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1, (const char *)packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_CRC);
#endif
}
#endif

#endif

// MESSAGE NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1 UNPACKING


/**
 * @brief Get field gains_speeds from naia_control_wrapper_input_csv_settings_part1 message
 *
 * @return [kts]  Speeds of the boat at which the control gains have been adjusted. The vector has m elements. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_gains_speeds(const mavlink_message_t* msg, float *gains_speeds)
{
    return _MAV_RETURN_float_array(msg, gains_speeds, 3,  0);
}

/**
 * @brief Get field target_speeds from naia_control_wrapper_input_csv_settings_part1 message
 *
 * @return [kts]  Speeds of the boat at which the reference height have been adjusted. The vector has m elements. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_target_speeds(const mavlink_message_t* msg, float *target_speeds)
{
    return _MAV_RETURN_float_array(msg, target_speeds, 2,  12);
}

/**
 * @brief Get field target_height from naia_control_wrapper_input_csv_settings_part1 message
 *
 * @return [mm]  Vector that defines the target height at each measured speed. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_target_height(const mavlink_message_t* msg, float *target_height)
{
    return _MAV_RETURN_float_array(msg, target_height, 2,  20);
}

/**
 * @brief Get field target_pitch from naia_control_wrapper_input_csv_settings_part1 message
 *
 * @return [deg]  Vector that defines the target pitch at each measured speed. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_target_pitch(const mavlink_message_t* msg, float *target_pitch)
{
    return _MAV_RETURN_float_array(msg, target_pitch, 2,  28);
}

/**
 * @brief Get field kp_depth from naia_control_wrapper_input_csv_settings_part1 message
 *
 * @return [deg/s]  Defines the depth proportional gain vector. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_kp_depth(const mavlink_message_t* msg, float *kp_depth)
{
    return _MAV_RETURN_float_array(msg, kp_depth, 3,  36);
}

/**
 * @brief Get field kd_depth from naia_control_wrapper_input_csv_settings_part1 message
 *
 * @return [deg/s]  Defines the depth derivative gain vector. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_kd_depth(const mavlink_message_t* msg, float *kd_depth)
{
    return _MAV_RETURN_float_array(msg, kd_depth, 3,  48);
}

/**
 * @brief Get field kp_pitch from naia_control_wrapper_input_csv_settings_part1 message
 *
 * @return [deg/s]  Defines the pitch proportional gain vector. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_kp_pitch(const mavlink_message_t* msg, float *kp_pitch)
{
    return _MAV_RETURN_float_array(msg, kp_pitch, 3,  60);
}

/**
 * @brief Get field kd_pitch from naia_control_wrapper_input_csv_settings_part1 message
 *
 * @return [deg/s]  Defines the pitch derivative gain vector. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_kd_pitch(const mavlink_message_t* msg, float *kd_pitch)
{
    return _MAV_RETURN_float_array(msg, kd_pitch, 3,  72);
}

/**
 * @brief Get field kp_heel from naia_control_wrapper_input_csv_settings_part1 message
 *
 * @return [deg/s]  Defines the heel proportional gain vector. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_kp_heel(const mavlink_message_t* msg, float *kp_heel)
{
    return _MAV_RETURN_float_array(msg, kp_heel, 3,  84);
}

/**
 * @brief Get field kd_heel from naia_control_wrapper_input_csv_settings_part1 message
 *
 * @return [deg/s]  Defines the heel derivative gain vector. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_kd_heel(const mavlink_message_t* msg, float *kd_heel)
{
    return _MAV_RETURN_float_array(msg, kd_heel, 3,  96);
}

/**
 * @brief Get field lateral_depth_coupling from naia_control_wrapper_input_csv_settings_part1 message
 *
 * @return [deg/s]  Defines the degree of coupling between the lateral semiwings and depth control desired. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_lateral_depth_coupling(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  108);
}

/**
 * @brief Get field lateral_pitch_coupling from naia_control_wrapper_input_csv_settings_part1 message
 *
 * @return [deg/s]  Defines the degree of coupling between the lateral semiwings and heel control desired. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_lateral_pitch_coupling(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  112);
}

/**
 * @brief Decode a naia_control_wrapper_input_csv_settings_part1 message into a struct
 *
 * @param msg The message to decode
 * @param naia_control_wrapper_input_csv_settings_part1 C-struct to decode the message contents into
 */
static inline void mavlink_msg_naia_control_wrapper_input_csv_settings_part1_decode(const mavlink_message_t* msg, mavlink_naia_control_wrapper_input_csv_settings_part1_t* naia_control_wrapper_input_csv_settings_part1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_gains_speeds(msg, naia_control_wrapper_input_csv_settings_part1->gains_speeds);
    mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_target_speeds(msg, naia_control_wrapper_input_csv_settings_part1->target_speeds);
    mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_target_height(msg, naia_control_wrapper_input_csv_settings_part1->target_height);
    mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_target_pitch(msg, naia_control_wrapper_input_csv_settings_part1->target_pitch);
    mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_kp_depth(msg, naia_control_wrapper_input_csv_settings_part1->kp_depth);
    mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_kd_depth(msg, naia_control_wrapper_input_csv_settings_part1->kd_depth);
    mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_kp_pitch(msg, naia_control_wrapper_input_csv_settings_part1->kp_pitch);
    mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_kd_pitch(msg, naia_control_wrapper_input_csv_settings_part1->kd_pitch);
    mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_kp_heel(msg, naia_control_wrapper_input_csv_settings_part1->kp_heel);
    mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_kd_heel(msg, naia_control_wrapper_input_csv_settings_part1->kd_heel);
    naia_control_wrapper_input_csv_settings_part1->lateral_depth_coupling = mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_lateral_depth_coupling(msg);
    naia_control_wrapper_input_csv_settings_part1->lateral_pitch_coupling = mavlink_msg_naia_control_wrapper_input_csv_settings_part1_get_lateral_pitch_coupling(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN? msg->len : MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN;
        memset(naia_control_wrapper_input_csv_settings_part1, 0, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART1_LEN);
    memcpy(naia_control_wrapper_input_csv_settings_part1, _MAV_PAYLOAD(msg), len);
#endif
}
