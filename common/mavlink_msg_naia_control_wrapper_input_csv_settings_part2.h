#pragma once
// MESSAGE NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2 PACKING

#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2 624


typedef struct __mavlink_naia_control_wrapper_input_csv_settings_part2_t {
 float initial_aoa_sprc[4]; /*< [deg]  Forces the control to impose a desired initial angle in each control surface. */
 float derivative_time_constant; /*< [s]  Lowpass filtering factor used to estimate numerical derivatives. */
 float filter_speed; /*<   Modulates the filtering level of the control speed signal. */
 float takeoff_hysteresis_speed; /*< [kts]  Determines the speed gap of the hystereis loop. */
 float heel_threshold; /*< [mm]  Pressure threshold that smoothly triggers the heel active control. */
 float pitch_threshold[2]; /*< [mm]  Pressure threshold that smoothly triggers the pitch active control. */
 float heel_hardness; /*< [Pa-1]  Constant that control the "stiffness" of the heel active control. */
 float pitch_hardness; /*< [Pa-1]  Constant that control the "stiffness" of the pitch active control. */
 float deflection_range[2]; /*< [deg]  Vector that defines the minimum and maximum ranges of the actuator position. */
 float deflection_speed_range[2]; /*< [deg/s]  Vector that defines the minimum and maximum ranges of the actuator speed. */
 float emergency_splashdown_sample_multiplier; /*<   Defines the multiplier between samples of the commanded position during an emergency splashdown. */
 float emergency_splashdown_final_angle; /*< [deg]  Defines the target angle commanded to the actuators during an emergency splashdown. */
 float pitot_speed_std_s; /*< [m/s]  A priori estimation of the standard deviation of the speed measured by the starboard pitot. */
 float pitot_speed_std_p; /*< [m/s]  A priori estimation of the standard deviation of the speed measured by the port pitot. */
 float pitot_speed_std_r; /*< [m/s]  A priori estimation of the standard deviation of the speed measured by the rear pitot. */
 float reset; /*<   Reset the control  when this value is changed. */
} mavlink_naia_control_wrapper_input_csv_settings_part2_t;

#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN 88
#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_MIN_LEN 88
#define MAVLINK_MSG_ID_624_LEN 88
#define MAVLINK_MSG_ID_624_MIN_LEN 88

#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_CRC 10
#define MAVLINK_MSG_ID_624_CRC 10

#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_FIELD_INITIAL_AOA_SPRC_LEN 4
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_FIELD_PITCH_THRESHOLD_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_FIELD_DEFLECTION_RANGE_LEN 2
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_FIELD_DEFLECTION_SPEED_RANGE_LEN 2

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2 { \
    624, \
    "NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2", \
    16, \
    {  { "initial_aoa_sprc", NULL, MAVLINK_TYPE_FLOAT, 4, 0, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, initial_aoa_sprc) }, \
         { "derivative_time_constant", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, derivative_time_constant) }, \
         { "filter_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, filter_speed) }, \
         { "takeoff_hysteresis_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, takeoff_hysteresis_speed) }, \
         { "heel_threshold", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, heel_threshold) }, \
         { "pitch_threshold", NULL, MAVLINK_TYPE_FLOAT, 2, 32, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, pitch_threshold) }, \
         { "heel_hardness", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, heel_hardness) }, \
         { "pitch_hardness", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, pitch_hardness) }, \
         { "deflection_range", NULL, MAVLINK_TYPE_FLOAT, 2, 48, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, deflection_range) }, \
         { "deflection_speed_range", NULL, MAVLINK_TYPE_FLOAT, 2, 56, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, deflection_speed_range) }, \
         { "emergency_splashdown_sample_multiplier", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, emergency_splashdown_sample_multiplier) }, \
         { "emergency_splashdown_final_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, emergency_splashdown_final_angle) }, \
         { "pitot_speed_std_s", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, pitot_speed_std_s) }, \
         { "pitot_speed_std_p", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, pitot_speed_std_p) }, \
         { "pitot_speed_std_r", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, pitot_speed_std_r) }, \
         { "reset", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, reset) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2 { \
    "NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2", \
    16, \
    {  { "initial_aoa_sprc", NULL, MAVLINK_TYPE_FLOAT, 4, 0, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, initial_aoa_sprc) }, \
         { "derivative_time_constant", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, derivative_time_constant) }, \
         { "filter_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, filter_speed) }, \
         { "takeoff_hysteresis_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, takeoff_hysteresis_speed) }, \
         { "heel_threshold", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, heel_threshold) }, \
         { "pitch_threshold", NULL, MAVLINK_TYPE_FLOAT, 2, 32, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, pitch_threshold) }, \
         { "heel_hardness", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, heel_hardness) }, \
         { "pitch_hardness", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, pitch_hardness) }, \
         { "deflection_range", NULL, MAVLINK_TYPE_FLOAT, 2, 48, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, deflection_range) }, \
         { "deflection_speed_range", NULL, MAVLINK_TYPE_FLOAT, 2, 56, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, deflection_speed_range) }, \
         { "emergency_splashdown_sample_multiplier", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, emergency_splashdown_sample_multiplier) }, \
         { "emergency_splashdown_final_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, emergency_splashdown_final_angle) }, \
         { "pitot_speed_std_s", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, pitot_speed_std_s) }, \
         { "pitot_speed_std_p", NULL, MAVLINK_TYPE_FLOAT, 0, 76, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, pitot_speed_std_p) }, \
         { "pitot_speed_std_r", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, pitot_speed_std_r) }, \
         { "reset", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_naia_control_wrapper_input_csv_settings_part2_t, reset) }, \
         } \
}
#endif

/**
 * @brief Pack a naia_control_wrapper_input_csv_settings_part2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param initial_aoa_sprc [deg]  Forces the control to impose a desired initial angle in each control surface. 
 * @param derivative_time_constant [s]  Lowpass filtering factor used to estimate numerical derivatives. 
 * @param filter_speed   Modulates the filtering level of the control speed signal. 
 * @param takeoff_hysteresis_speed [kts]  Determines the speed gap of the hystereis loop. 
 * @param heel_threshold [mm]  Pressure threshold that smoothly triggers the heel active control. 
 * @param pitch_threshold [mm]  Pressure threshold that smoothly triggers the pitch active control. 
 * @param heel_hardness [Pa-1]  Constant that control the "stiffness" of the heel active control. 
 * @param pitch_hardness [Pa-1]  Constant that control the "stiffness" of the pitch active control. 
 * @param deflection_range [deg]  Vector that defines the minimum and maximum ranges of the actuator position. 
 * @param deflection_speed_range [deg/s]  Vector that defines the minimum and maximum ranges of the actuator speed. 
 * @param emergency_splashdown_sample_multiplier   Defines the multiplier between samples of the commanded position during an emergency splashdown. 
 * @param emergency_splashdown_final_angle [deg]  Defines the target angle commanded to the actuators during an emergency splashdown. 
 * @param pitot_speed_std_s [m/s]  A priori estimation of the standard deviation of the speed measured by the starboard pitot. 
 * @param pitot_speed_std_p [m/s]  A priori estimation of the standard deviation of the speed measured by the port pitot. 
 * @param pitot_speed_std_r [m/s]  A priori estimation of the standard deviation of the speed measured by the rear pitot. 
 * @param reset   Reset the control  when this value is changed. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *initial_aoa_sprc, float derivative_time_constant, float filter_speed, float takeoff_hysteresis_speed, float heel_threshold, const float *pitch_threshold, float heel_hardness, float pitch_hardness, const float *deflection_range, const float *deflection_speed_range, float emergency_splashdown_sample_multiplier, float emergency_splashdown_final_angle, float pitot_speed_std_s, float pitot_speed_std_p, float pitot_speed_std_r, float reset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN];
    _mav_put_float(buf, 16, derivative_time_constant);
    _mav_put_float(buf, 20, filter_speed);
    _mav_put_float(buf, 24, takeoff_hysteresis_speed);
    _mav_put_float(buf, 28, heel_threshold);
    _mav_put_float(buf, 40, heel_hardness);
    _mav_put_float(buf, 44, pitch_hardness);
    _mav_put_float(buf, 64, emergency_splashdown_sample_multiplier);
    _mav_put_float(buf, 68, emergency_splashdown_final_angle);
    _mav_put_float(buf, 72, pitot_speed_std_s);
    _mav_put_float(buf, 76, pitot_speed_std_p);
    _mav_put_float(buf, 80, pitot_speed_std_r);
    _mav_put_float(buf, 84, reset);
    _mav_put_float_array(buf, 0, initial_aoa_sprc, 4);
    _mav_put_float_array(buf, 32, pitch_threshold, 2);
    _mav_put_float_array(buf, 48, deflection_range, 2);
    _mav_put_float_array(buf, 56, deflection_speed_range, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN);
#else
    mavlink_naia_control_wrapper_input_csv_settings_part2_t packet;
    packet.derivative_time_constant = derivative_time_constant;
    packet.filter_speed = filter_speed;
    packet.takeoff_hysteresis_speed = takeoff_hysteresis_speed;
    packet.heel_threshold = heel_threshold;
    packet.heel_hardness = heel_hardness;
    packet.pitch_hardness = pitch_hardness;
    packet.emergency_splashdown_sample_multiplier = emergency_splashdown_sample_multiplier;
    packet.emergency_splashdown_final_angle = emergency_splashdown_final_angle;
    packet.pitot_speed_std_s = pitot_speed_std_s;
    packet.pitot_speed_std_p = pitot_speed_std_p;
    packet.pitot_speed_std_r = pitot_speed_std_r;
    packet.reset = reset;
    mav_array_memcpy(packet.initial_aoa_sprc, initial_aoa_sprc, sizeof(float)*4);
    mav_array_memcpy(packet.pitch_threshold, pitch_threshold, sizeof(float)*2);
    mav_array_memcpy(packet.deflection_range, deflection_range, sizeof(float)*2);
    mav_array_memcpy(packet.deflection_speed_range, deflection_speed_range, sizeof(float)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_CRC);
}

/**
 * @brief Pack a naia_control_wrapper_input_csv_settings_part2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param initial_aoa_sprc [deg]  Forces the control to impose a desired initial angle in each control surface. 
 * @param derivative_time_constant [s]  Lowpass filtering factor used to estimate numerical derivatives. 
 * @param filter_speed   Modulates the filtering level of the control speed signal. 
 * @param takeoff_hysteresis_speed [kts]  Determines the speed gap of the hystereis loop. 
 * @param heel_threshold [mm]  Pressure threshold that smoothly triggers the heel active control. 
 * @param pitch_threshold [mm]  Pressure threshold that smoothly triggers the pitch active control. 
 * @param heel_hardness [Pa-1]  Constant that control the "stiffness" of the heel active control. 
 * @param pitch_hardness [Pa-1]  Constant that control the "stiffness" of the pitch active control. 
 * @param deflection_range [deg]  Vector that defines the minimum and maximum ranges of the actuator position. 
 * @param deflection_speed_range [deg/s]  Vector that defines the minimum and maximum ranges of the actuator speed. 
 * @param emergency_splashdown_sample_multiplier   Defines the multiplier between samples of the commanded position during an emergency splashdown. 
 * @param emergency_splashdown_final_angle [deg]  Defines the target angle commanded to the actuators during an emergency splashdown. 
 * @param pitot_speed_std_s [m/s]  A priori estimation of the standard deviation of the speed measured by the starboard pitot. 
 * @param pitot_speed_std_p [m/s]  A priori estimation of the standard deviation of the speed measured by the port pitot. 
 * @param pitot_speed_std_r [m/s]  A priori estimation of the standard deviation of the speed measured by the rear pitot. 
 * @param reset   Reset the control  when this value is changed. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *initial_aoa_sprc,float derivative_time_constant,float filter_speed,float takeoff_hysteresis_speed,float heel_threshold,const float *pitch_threshold,float heel_hardness,float pitch_hardness,const float *deflection_range,const float *deflection_speed_range,float emergency_splashdown_sample_multiplier,float emergency_splashdown_final_angle,float pitot_speed_std_s,float pitot_speed_std_p,float pitot_speed_std_r,float reset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN];
    _mav_put_float(buf, 16, derivative_time_constant);
    _mav_put_float(buf, 20, filter_speed);
    _mav_put_float(buf, 24, takeoff_hysteresis_speed);
    _mav_put_float(buf, 28, heel_threshold);
    _mav_put_float(buf, 40, heel_hardness);
    _mav_put_float(buf, 44, pitch_hardness);
    _mav_put_float(buf, 64, emergency_splashdown_sample_multiplier);
    _mav_put_float(buf, 68, emergency_splashdown_final_angle);
    _mav_put_float(buf, 72, pitot_speed_std_s);
    _mav_put_float(buf, 76, pitot_speed_std_p);
    _mav_put_float(buf, 80, pitot_speed_std_r);
    _mav_put_float(buf, 84, reset);
    _mav_put_float_array(buf, 0, initial_aoa_sprc, 4);
    _mav_put_float_array(buf, 32, pitch_threshold, 2);
    _mav_put_float_array(buf, 48, deflection_range, 2);
    _mav_put_float_array(buf, 56, deflection_speed_range, 2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN);
#else
    mavlink_naia_control_wrapper_input_csv_settings_part2_t packet;
    packet.derivative_time_constant = derivative_time_constant;
    packet.filter_speed = filter_speed;
    packet.takeoff_hysteresis_speed = takeoff_hysteresis_speed;
    packet.heel_threshold = heel_threshold;
    packet.heel_hardness = heel_hardness;
    packet.pitch_hardness = pitch_hardness;
    packet.emergency_splashdown_sample_multiplier = emergency_splashdown_sample_multiplier;
    packet.emergency_splashdown_final_angle = emergency_splashdown_final_angle;
    packet.pitot_speed_std_s = pitot_speed_std_s;
    packet.pitot_speed_std_p = pitot_speed_std_p;
    packet.pitot_speed_std_r = pitot_speed_std_r;
    packet.reset = reset;
    mav_array_memcpy(packet.initial_aoa_sprc, initial_aoa_sprc, sizeof(float)*4);
    mav_array_memcpy(packet.pitch_threshold, pitch_threshold, sizeof(float)*2);
    mav_array_memcpy(packet.deflection_range, deflection_range, sizeof(float)*2);
    mav_array_memcpy(packet.deflection_speed_range, deflection_speed_range, sizeof(float)*2);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_CRC);
}

/**
 * @brief Encode a naia_control_wrapper_input_csv_settings_part2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param naia_control_wrapper_input_csv_settings_part2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_naia_control_wrapper_input_csv_settings_part2_t* naia_control_wrapper_input_csv_settings_part2)
{
    return mavlink_msg_naia_control_wrapper_input_csv_settings_part2_pack(system_id, component_id, msg, naia_control_wrapper_input_csv_settings_part2->initial_aoa_sprc, naia_control_wrapper_input_csv_settings_part2->derivative_time_constant, naia_control_wrapper_input_csv_settings_part2->filter_speed, naia_control_wrapper_input_csv_settings_part2->takeoff_hysteresis_speed, naia_control_wrapper_input_csv_settings_part2->heel_threshold, naia_control_wrapper_input_csv_settings_part2->pitch_threshold, naia_control_wrapper_input_csv_settings_part2->heel_hardness, naia_control_wrapper_input_csv_settings_part2->pitch_hardness, naia_control_wrapper_input_csv_settings_part2->deflection_range, naia_control_wrapper_input_csv_settings_part2->deflection_speed_range, naia_control_wrapper_input_csv_settings_part2->emergency_splashdown_sample_multiplier, naia_control_wrapper_input_csv_settings_part2->emergency_splashdown_final_angle, naia_control_wrapper_input_csv_settings_part2->pitot_speed_std_s, naia_control_wrapper_input_csv_settings_part2->pitot_speed_std_p, naia_control_wrapper_input_csv_settings_part2->pitot_speed_std_r, naia_control_wrapper_input_csv_settings_part2->reset);
}

/**
 * @brief Encode a naia_control_wrapper_input_csv_settings_part2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param naia_control_wrapper_input_csv_settings_part2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_naia_control_wrapper_input_csv_settings_part2_t* naia_control_wrapper_input_csv_settings_part2)
{
    return mavlink_msg_naia_control_wrapper_input_csv_settings_part2_pack_chan(system_id, component_id, chan, msg, naia_control_wrapper_input_csv_settings_part2->initial_aoa_sprc, naia_control_wrapper_input_csv_settings_part2->derivative_time_constant, naia_control_wrapper_input_csv_settings_part2->filter_speed, naia_control_wrapper_input_csv_settings_part2->takeoff_hysteresis_speed, naia_control_wrapper_input_csv_settings_part2->heel_threshold, naia_control_wrapper_input_csv_settings_part2->pitch_threshold, naia_control_wrapper_input_csv_settings_part2->heel_hardness, naia_control_wrapper_input_csv_settings_part2->pitch_hardness, naia_control_wrapper_input_csv_settings_part2->deflection_range, naia_control_wrapper_input_csv_settings_part2->deflection_speed_range, naia_control_wrapper_input_csv_settings_part2->emergency_splashdown_sample_multiplier, naia_control_wrapper_input_csv_settings_part2->emergency_splashdown_final_angle, naia_control_wrapper_input_csv_settings_part2->pitot_speed_std_s, naia_control_wrapper_input_csv_settings_part2->pitot_speed_std_p, naia_control_wrapper_input_csv_settings_part2->pitot_speed_std_r, naia_control_wrapper_input_csv_settings_part2->reset);
}

/**
 * @brief Send a naia_control_wrapper_input_csv_settings_part2 message
 * @param chan MAVLink channel to send the message
 *
 * @param initial_aoa_sprc [deg]  Forces the control to impose a desired initial angle in each control surface. 
 * @param derivative_time_constant [s]  Lowpass filtering factor used to estimate numerical derivatives. 
 * @param filter_speed   Modulates the filtering level of the control speed signal. 
 * @param takeoff_hysteresis_speed [kts]  Determines the speed gap of the hystereis loop. 
 * @param heel_threshold [mm]  Pressure threshold that smoothly triggers the heel active control. 
 * @param pitch_threshold [mm]  Pressure threshold that smoothly triggers the pitch active control. 
 * @param heel_hardness [Pa-1]  Constant that control the "stiffness" of the heel active control. 
 * @param pitch_hardness [Pa-1]  Constant that control the "stiffness" of the pitch active control. 
 * @param deflection_range [deg]  Vector that defines the minimum and maximum ranges of the actuator position. 
 * @param deflection_speed_range [deg/s]  Vector that defines the minimum and maximum ranges of the actuator speed. 
 * @param emergency_splashdown_sample_multiplier   Defines the multiplier between samples of the commanded position during an emergency splashdown. 
 * @param emergency_splashdown_final_angle [deg]  Defines the target angle commanded to the actuators during an emergency splashdown. 
 * @param pitot_speed_std_s [m/s]  A priori estimation of the standard deviation of the speed measured by the starboard pitot. 
 * @param pitot_speed_std_p [m/s]  A priori estimation of the standard deviation of the speed measured by the port pitot. 
 * @param pitot_speed_std_r [m/s]  A priori estimation of the standard deviation of the speed measured by the rear pitot. 
 * @param reset   Reset the control  when this value is changed. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_naia_control_wrapper_input_csv_settings_part2_send(mavlink_channel_t chan, const float *initial_aoa_sprc, float derivative_time_constant, float filter_speed, float takeoff_hysteresis_speed, float heel_threshold, const float *pitch_threshold, float heel_hardness, float pitch_hardness, const float *deflection_range, const float *deflection_speed_range, float emergency_splashdown_sample_multiplier, float emergency_splashdown_final_angle, float pitot_speed_std_s, float pitot_speed_std_p, float pitot_speed_std_r, float reset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN];
    _mav_put_float(buf, 16, derivative_time_constant);
    _mav_put_float(buf, 20, filter_speed);
    _mav_put_float(buf, 24, takeoff_hysteresis_speed);
    _mav_put_float(buf, 28, heel_threshold);
    _mav_put_float(buf, 40, heel_hardness);
    _mav_put_float(buf, 44, pitch_hardness);
    _mav_put_float(buf, 64, emergency_splashdown_sample_multiplier);
    _mav_put_float(buf, 68, emergency_splashdown_final_angle);
    _mav_put_float(buf, 72, pitot_speed_std_s);
    _mav_put_float(buf, 76, pitot_speed_std_p);
    _mav_put_float(buf, 80, pitot_speed_std_r);
    _mav_put_float(buf, 84, reset);
    _mav_put_float_array(buf, 0, initial_aoa_sprc, 4);
    _mav_put_float_array(buf, 32, pitch_threshold, 2);
    _mav_put_float_array(buf, 48, deflection_range, 2);
    _mav_put_float_array(buf, 56, deflection_speed_range, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2, buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_CRC);
#else
    mavlink_naia_control_wrapper_input_csv_settings_part2_t packet;
    packet.derivative_time_constant = derivative_time_constant;
    packet.filter_speed = filter_speed;
    packet.takeoff_hysteresis_speed = takeoff_hysteresis_speed;
    packet.heel_threshold = heel_threshold;
    packet.heel_hardness = heel_hardness;
    packet.pitch_hardness = pitch_hardness;
    packet.emergency_splashdown_sample_multiplier = emergency_splashdown_sample_multiplier;
    packet.emergency_splashdown_final_angle = emergency_splashdown_final_angle;
    packet.pitot_speed_std_s = pitot_speed_std_s;
    packet.pitot_speed_std_p = pitot_speed_std_p;
    packet.pitot_speed_std_r = pitot_speed_std_r;
    packet.reset = reset;
    mav_array_memcpy(packet.initial_aoa_sprc, initial_aoa_sprc, sizeof(float)*4);
    mav_array_memcpy(packet.pitch_threshold, pitch_threshold, sizeof(float)*2);
    mav_array_memcpy(packet.deflection_range, deflection_range, sizeof(float)*2);
    mav_array_memcpy(packet.deflection_speed_range, deflection_speed_range, sizeof(float)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2, (const char *)&packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_CRC);
#endif
}

/**
 * @brief Send a naia_control_wrapper_input_csv_settings_part2 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_naia_control_wrapper_input_csv_settings_part2_send_struct(mavlink_channel_t chan, const mavlink_naia_control_wrapper_input_csv_settings_part2_t* naia_control_wrapper_input_csv_settings_part2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_naia_control_wrapper_input_csv_settings_part2_send(chan, naia_control_wrapper_input_csv_settings_part2->initial_aoa_sprc, naia_control_wrapper_input_csv_settings_part2->derivative_time_constant, naia_control_wrapper_input_csv_settings_part2->filter_speed, naia_control_wrapper_input_csv_settings_part2->takeoff_hysteresis_speed, naia_control_wrapper_input_csv_settings_part2->heel_threshold, naia_control_wrapper_input_csv_settings_part2->pitch_threshold, naia_control_wrapper_input_csv_settings_part2->heel_hardness, naia_control_wrapper_input_csv_settings_part2->pitch_hardness, naia_control_wrapper_input_csv_settings_part2->deflection_range, naia_control_wrapper_input_csv_settings_part2->deflection_speed_range, naia_control_wrapper_input_csv_settings_part2->emergency_splashdown_sample_multiplier, naia_control_wrapper_input_csv_settings_part2->emergency_splashdown_final_angle, naia_control_wrapper_input_csv_settings_part2->pitot_speed_std_s, naia_control_wrapper_input_csv_settings_part2->pitot_speed_std_p, naia_control_wrapper_input_csv_settings_part2->pitot_speed_std_r, naia_control_wrapper_input_csv_settings_part2->reset);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2, (const char *)naia_control_wrapper_input_csv_settings_part2, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_naia_control_wrapper_input_csv_settings_part2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *initial_aoa_sprc, float derivative_time_constant, float filter_speed, float takeoff_hysteresis_speed, float heel_threshold, const float *pitch_threshold, float heel_hardness, float pitch_hardness, const float *deflection_range, const float *deflection_speed_range, float emergency_splashdown_sample_multiplier, float emergency_splashdown_final_angle, float pitot_speed_std_s, float pitot_speed_std_p, float pitot_speed_std_r, float reset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 16, derivative_time_constant);
    _mav_put_float(buf, 20, filter_speed);
    _mav_put_float(buf, 24, takeoff_hysteresis_speed);
    _mav_put_float(buf, 28, heel_threshold);
    _mav_put_float(buf, 40, heel_hardness);
    _mav_put_float(buf, 44, pitch_hardness);
    _mav_put_float(buf, 64, emergency_splashdown_sample_multiplier);
    _mav_put_float(buf, 68, emergency_splashdown_final_angle);
    _mav_put_float(buf, 72, pitot_speed_std_s);
    _mav_put_float(buf, 76, pitot_speed_std_p);
    _mav_put_float(buf, 80, pitot_speed_std_r);
    _mav_put_float(buf, 84, reset);
    _mav_put_float_array(buf, 0, initial_aoa_sprc, 4);
    _mav_put_float_array(buf, 32, pitch_threshold, 2);
    _mav_put_float_array(buf, 48, deflection_range, 2);
    _mav_put_float_array(buf, 56, deflection_speed_range, 2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2, buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_CRC);
#else
    mavlink_naia_control_wrapper_input_csv_settings_part2_t *packet = (mavlink_naia_control_wrapper_input_csv_settings_part2_t *)msgbuf;
    packet->derivative_time_constant = derivative_time_constant;
    packet->filter_speed = filter_speed;
    packet->takeoff_hysteresis_speed = takeoff_hysteresis_speed;
    packet->heel_threshold = heel_threshold;
    packet->heel_hardness = heel_hardness;
    packet->pitch_hardness = pitch_hardness;
    packet->emergency_splashdown_sample_multiplier = emergency_splashdown_sample_multiplier;
    packet->emergency_splashdown_final_angle = emergency_splashdown_final_angle;
    packet->pitot_speed_std_s = pitot_speed_std_s;
    packet->pitot_speed_std_p = pitot_speed_std_p;
    packet->pitot_speed_std_r = pitot_speed_std_r;
    packet->reset = reset;
    mav_array_memcpy(packet->initial_aoa_sprc, initial_aoa_sprc, sizeof(float)*4);
    mav_array_memcpy(packet->pitch_threshold, pitch_threshold, sizeof(float)*2);
    mav_array_memcpy(packet->deflection_range, deflection_range, sizeof(float)*2);
    mav_array_memcpy(packet->deflection_speed_range, deflection_speed_range, sizeof(float)*2);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2, (const char *)packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_CRC);
#endif
}
#endif

#endif

// MESSAGE NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2 UNPACKING


/**
 * @brief Get field initial_aoa_sprc from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [deg]  Forces the control to impose a desired initial angle in each control surface. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_initial_aoa_sprc(const mavlink_message_t* msg, float *initial_aoa_sprc)
{
    return _MAV_RETURN_float_array(msg, initial_aoa_sprc, 4,  0);
}

/**
 * @brief Get field derivative_time_constant from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [s]  Lowpass filtering factor used to estimate numerical derivatives. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_derivative_time_constant(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field filter_speed from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return   Modulates the filtering level of the control speed signal. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_filter_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field takeoff_hysteresis_speed from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [kts]  Determines the speed gap of the hystereis loop. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_takeoff_hysteresis_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field heel_threshold from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [mm]  Pressure threshold that smoothly triggers the heel active control. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_heel_threshold(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field pitch_threshold from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [mm]  Pressure threshold that smoothly triggers the pitch active control. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_pitch_threshold(const mavlink_message_t* msg, float *pitch_threshold)
{
    return _MAV_RETURN_float_array(msg, pitch_threshold, 2,  32);
}

/**
 * @brief Get field heel_hardness from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [Pa-1]  Constant that control the "stiffness" of the heel active control. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_heel_hardness(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field pitch_hardness from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [Pa-1]  Constant that control the "stiffness" of the pitch active control. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_pitch_hardness(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field deflection_range from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [deg]  Vector that defines the minimum and maximum ranges of the actuator position. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_deflection_range(const mavlink_message_t* msg, float *deflection_range)
{
    return _MAV_RETURN_float_array(msg, deflection_range, 2,  48);
}

/**
 * @brief Get field deflection_speed_range from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [deg/s]  Vector that defines the minimum and maximum ranges of the actuator speed. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_deflection_speed_range(const mavlink_message_t* msg, float *deflection_speed_range)
{
    return _MAV_RETURN_float_array(msg, deflection_speed_range, 2,  56);
}

/**
 * @brief Get field emergency_splashdown_sample_multiplier from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return   Defines the multiplier between samples of the commanded position during an emergency splashdown. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_emergency_splashdown_sample_multiplier(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field emergency_splashdown_final_angle from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [deg]  Defines the target angle commanded to the actuators during an emergency splashdown. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_emergency_splashdown_final_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field pitot_speed_std_s from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [m/s]  A priori estimation of the standard deviation of the speed measured by the starboard pitot. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_pitot_speed_std_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field pitot_speed_std_p from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [m/s]  A priori estimation of the standard deviation of the speed measured by the port pitot. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_pitot_speed_std_p(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  76);
}

/**
 * @brief Get field pitot_speed_std_r from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return [m/s]  A priori estimation of the standard deviation of the speed measured by the rear pitot. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_pitot_speed_std_r(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field reset from naia_control_wrapper_input_csv_settings_part2 message
 *
 * @return   Reset the control  when this value is changed. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_reset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Decode a naia_control_wrapper_input_csv_settings_part2 message into a struct
 *
 * @param msg The message to decode
 * @param naia_control_wrapper_input_csv_settings_part2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_naia_control_wrapper_input_csv_settings_part2_decode(const mavlink_message_t* msg, mavlink_naia_control_wrapper_input_csv_settings_part2_t* naia_control_wrapper_input_csv_settings_part2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_initial_aoa_sprc(msg, naia_control_wrapper_input_csv_settings_part2->initial_aoa_sprc);
    naia_control_wrapper_input_csv_settings_part2->derivative_time_constant = mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_derivative_time_constant(msg);
    naia_control_wrapper_input_csv_settings_part2->filter_speed = mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_filter_speed(msg);
    naia_control_wrapper_input_csv_settings_part2->takeoff_hysteresis_speed = mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_takeoff_hysteresis_speed(msg);
    naia_control_wrapper_input_csv_settings_part2->heel_threshold = mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_heel_threshold(msg);
    mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_pitch_threshold(msg, naia_control_wrapper_input_csv_settings_part2->pitch_threshold);
    naia_control_wrapper_input_csv_settings_part2->heel_hardness = mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_heel_hardness(msg);
    naia_control_wrapper_input_csv_settings_part2->pitch_hardness = mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_pitch_hardness(msg);
    mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_deflection_range(msg, naia_control_wrapper_input_csv_settings_part2->deflection_range);
    mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_deflection_speed_range(msg, naia_control_wrapper_input_csv_settings_part2->deflection_speed_range);
    naia_control_wrapper_input_csv_settings_part2->emergency_splashdown_sample_multiplier = mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_emergency_splashdown_sample_multiplier(msg);
    naia_control_wrapper_input_csv_settings_part2->emergency_splashdown_final_angle = mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_emergency_splashdown_final_angle(msg);
    naia_control_wrapper_input_csv_settings_part2->pitot_speed_std_s = mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_pitot_speed_std_s(msg);
    naia_control_wrapper_input_csv_settings_part2->pitot_speed_std_p = mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_pitot_speed_std_p(msg);
    naia_control_wrapper_input_csv_settings_part2->pitot_speed_std_r = mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_pitot_speed_std_r(msg);
    naia_control_wrapper_input_csv_settings_part2->reset = mavlink_msg_naia_control_wrapper_input_csv_settings_part2_get_reset(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN? msg->len : MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN;
        memset(naia_control_wrapper_input_csv_settings_part2, 0, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_SETTINGS_PART2_LEN);
    memcpy(naia_control_wrapper_input_csv_settings_part2, _MAV_PAYLOAD(msg), len);
#endif
}
