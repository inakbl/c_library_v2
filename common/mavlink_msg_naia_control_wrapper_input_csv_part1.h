#pragma once
// MESSAGE NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1 PACKING

#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1 616

MAVPACKED(
typedef struct __mavlink_naia_control_wrapper_input_csv_part1_t {
 float time; /*<   Timestamp. */
 float raw_pressure_s[6]; /*<   Array of raw (but offset corrected) pressures measured by the starboard pitot. */
 float raw_pressure_p[6]; /*<   Array of raw (but offset corrected) pressures measured by the port pitot. */
 float raw_pressure_r[6]; /*<   Array of raw (but offset corrected) pressures measured by the rear pitot. */
 float position[6]; /*<   Global coordinates defining the position of the boat. The vector has the form: [Latitude, longitude, heigth]. */
 float velocity[6]; /*<   Velocity measured by the IMU in combination with the GPS when signal is available. */
 float acceleration[6]; /*<   Acceleration measured by the  Inertial Measurement Unit (IMU). */
 float angular_position[6]; /*<   Rotational speed, measured by the IMU. */
 float angular_velocity[6]; /*<   Euler angles measured by the IMU. */
 float angular_acceleration[6]; /*<   Angular acceleration measured by the IMU. */
 float imposed_aoa_sprc[4]; /*<   Geometric wing angles imposed manually, SPRC* convention. If set to NaN, control activates; othewise, input is used. */
 float rudder_angle; /*<   Rudder angle value obtained by a Magnetic encoder. */
}) mavlink_naia_control_wrapper_input_csv_part1_t;

#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN 240
#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_MIN_LEN 240
#define MAVLINK_MSG_ID_616_LEN 240
#define MAVLINK_MSG_ID_616_MIN_LEN 240

#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_CRC 240
#define MAVLINK_MSG_ID_616_CRC 240

#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_FIELD_RAW_PRESSURE_S_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_FIELD_RAW_PRESSURE_P_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_FIELD_RAW_PRESSURE_R_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_FIELD_POSITION_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_FIELD_VELOCITY_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_FIELD_ACCELERATION_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_FIELD_ANGULAR_POSITION_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_FIELD_ANGULAR_VELOCITY_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_FIELD_ANGULAR_ACCELERATION_LEN 6
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_FIELD_IMPOSED_AOA_SPRC_LEN 4

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1 { \
    616, \
    "NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1", \
    12, \
    {  { "time", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, time) }, \
         { "raw_pressure_s", NULL, MAVLINK_TYPE_FLOAT, 6, 4, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, raw_pressure_s) }, \
         { "raw_pressure_p", NULL, MAVLINK_TYPE_FLOAT, 6, 28, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, raw_pressure_p) }, \
         { "raw_pressure_r", NULL, MAVLINK_TYPE_FLOAT, 6, 52, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, raw_pressure_r) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 6, 76, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, position) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 6, 100, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, velocity) }, \
         { "acceleration", NULL, MAVLINK_TYPE_FLOAT, 6, 124, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, acceleration) }, \
         { "angular_position", NULL, MAVLINK_TYPE_FLOAT, 6, 148, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, angular_position) }, \
         { "angular_velocity", NULL, MAVLINK_TYPE_FLOAT, 6, 172, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, angular_velocity) }, \
         { "angular_acceleration", NULL, MAVLINK_TYPE_FLOAT, 6, 196, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, angular_acceleration) }, \
         { "imposed_aoa_sprc", NULL, MAVLINK_TYPE_FLOAT, 4, 220, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, imposed_aoa_sprc) }, \
         { "rudder_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 236, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, rudder_angle) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1 { \
    "NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1", \
    12, \
    {  { "time", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, time) }, \
         { "raw_pressure_s", NULL, MAVLINK_TYPE_FLOAT, 6, 4, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, raw_pressure_s) }, \
         { "raw_pressure_p", NULL, MAVLINK_TYPE_FLOAT, 6, 28, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, raw_pressure_p) }, \
         { "raw_pressure_r", NULL, MAVLINK_TYPE_FLOAT, 6, 52, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, raw_pressure_r) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 6, 76, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, position) }, \
         { "velocity", NULL, MAVLINK_TYPE_FLOAT, 6, 100, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, velocity) }, \
         { "acceleration", NULL, MAVLINK_TYPE_FLOAT, 6, 124, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, acceleration) }, \
         { "angular_position", NULL, MAVLINK_TYPE_FLOAT, 6, 148, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, angular_position) }, \
         { "angular_velocity", NULL, MAVLINK_TYPE_FLOAT, 6, 172, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, angular_velocity) }, \
         { "angular_acceleration", NULL, MAVLINK_TYPE_FLOAT, 6, 196, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, angular_acceleration) }, \
         { "imposed_aoa_sprc", NULL, MAVLINK_TYPE_FLOAT, 4, 220, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, imposed_aoa_sprc) }, \
         { "rudder_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 236, offsetof(mavlink_naia_control_wrapper_input_csv_part1_t, rudder_angle) }, \
         } \
}
#endif

/**
 * @brief Pack a naia_control_wrapper_input_csv_part1 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time   Timestamp. 
 * @param raw_pressure_s   Array of raw (but offset corrected) pressures measured by the starboard pitot. 
 * @param raw_pressure_p   Array of raw (but offset corrected) pressures measured by the port pitot. 
 * @param raw_pressure_r   Array of raw (but offset corrected) pressures measured by the rear pitot. 
 * @param position   Global coordinates defining the position of the boat. The vector has the form: [Latitude, longitude, heigth]. 
 * @param velocity   Velocity measured by the IMU in combination with the GPS when signal is available. 
 * @param acceleration   Acceleration measured by the  Inertial Measurement Unit (IMU). 
 * @param angular_position   Rotational speed, measured by the IMU. 
 * @param angular_velocity   Euler angles measured by the IMU. 
 * @param angular_acceleration   Angular acceleration measured by the IMU. 
 * @param imposed_aoa_sprc   Geometric wing angles imposed manually, SPRC* convention. If set to NaN, control activates; othewise, input is used. 
 * @param rudder_angle   Rudder angle value obtained by a Magnetic encoder. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float time, const float *raw_pressure_s, const float *raw_pressure_p, const float *raw_pressure_r, const float *position, const float *velocity, const float *acceleration, const float *angular_position, const float *angular_velocity, const float *angular_acceleration, const float *imposed_aoa_sprc, float rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN];
    _mav_put_float(buf, 0, time);
    _mav_put_float(buf, 236, rudder_angle);
    _mav_put_float_array(buf, 4, raw_pressure_s, 6);
    _mav_put_float_array(buf, 28, raw_pressure_p, 6);
    _mav_put_float_array(buf, 52, raw_pressure_r, 6);
    _mav_put_float_array(buf, 76, position, 6);
    _mav_put_float_array(buf, 100, velocity, 6);
    _mav_put_float_array(buf, 124, acceleration, 6);
    _mav_put_float_array(buf, 148, angular_position, 6);
    _mav_put_float_array(buf, 172, angular_velocity, 6);
    _mav_put_float_array(buf, 196, angular_acceleration, 6);
    _mav_put_float_array(buf, 220, imposed_aoa_sprc, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN);
#else
    mavlink_naia_control_wrapper_input_csv_part1_t packet;
    packet.time = time;
    packet.rudder_angle = rudder_angle;
    mav_array_memcpy(packet.raw_pressure_s, raw_pressure_s, sizeof(float)*6);
    mav_array_memcpy(packet.raw_pressure_p, raw_pressure_p, sizeof(float)*6);
    mav_array_memcpy(packet.raw_pressure_r, raw_pressure_r, sizeof(float)*6);
    mav_array_memcpy(packet.position, position, sizeof(float)*6);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*6);
    mav_array_memcpy(packet.acceleration, acceleration, sizeof(float)*6);
    mav_array_memcpy(packet.angular_position, angular_position, sizeof(float)*6);
    mav_array_memcpy(packet.angular_velocity, angular_velocity, sizeof(float)*6);
    mav_array_memcpy(packet.angular_acceleration, angular_acceleration, sizeof(float)*6);
    mav_array_memcpy(packet.imposed_aoa_sprc, imposed_aoa_sprc, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_CRC);
}

/**
 * @brief Pack a naia_control_wrapper_input_csv_part1 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time   Timestamp. 
 * @param raw_pressure_s   Array of raw (but offset corrected) pressures measured by the starboard pitot. 
 * @param raw_pressure_p   Array of raw (but offset corrected) pressures measured by the port pitot. 
 * @param raw_pressure_r   Array of raw (but offset corrected) pressures measured by the rear pitot. 
 * @param position   Global coordinates defining the position of the boat. The vector has the form: [Latitude, longitude, heigth]. 
 * @param velocity   Velocity measured by the IMU in combination with the GPS when signal is available. 
 * @param acceleration   Acceleration measured by the  Inertial Measurement Unit (IMU). 
 * @param angular_position   Rotational speed, measured by the IMU. 
 * @param angular_velocity   Euler angles measured by the IMU. 
 * @param angular_acceleration   Angular acceleration measured by the IMU. 
 * @param imposed_aoa_sprc   Geometric wing angles imposed manually, SPRC* convention. If set to NaN, control activates; othewise, input is used. 
 * @param rudder_angle   Rudder angle value obtained by a Magnetic encoder. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float time,const float *raw_pressure_s,const float *raw_pressure_p,const float *raw_pressure_r,const float *position,const float *velocity,const float *acceleration,const float *angular_position,const float *angular_velocity,const float *angular_acceleration,const float *imposed_aoa_sprc,float rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN];
    _mav_put_float(buf, 0, time);
    _mav_put_float(buf, 236, rudder_angle);
    _mav_put_float_array(buf, 4, raw_pressure_s, 6);
    _mav_put_float_array(buf, 28, raw_pressure_p, 6);
    _mav_put_float_array(buf, 52, raw_pressure_r, 6);
    _mav_put_float_array(buf, 76, position, 6);
    _mav_put_float_array(buf, 100, velocity, 6);
    _mav_put_float_array(buf, 124, acceleration, 6);
    _mav_put_float_array(buf, 148, angular_position, 6);
    _mav_put_float_array(buf, 172, angular_velocity, 6);
    _mav_put_float_array(buf, 196, angular_acceleration, 6);
    _mav_put_float_array(buf, 220, imposed_aoa_sprc, 4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN);
#else
    mavlink_naia_control_wrapper_input_csv_part1_t packet;
    packet.time = time;
    packet.rudder_angle = rudder_angle;
    mav_array_memcpy(packet.raw_pressure_s, raw_pressure_s, sizeof(float)*6);
    mav_array_memcpy(packet.raw_pressure_p, raw_pressure_p, sizeof(float)*6);
    mav_array_memcpy(packet.raw_pressure_r, raw_pressure_r, sizeof(float)*6);
    mav_array_memcpy(packet.position, position, sizeof(float)*6);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*6);
    mav_array_memcpy(packet.acceleration, acceleration, sizeof(float)*6);
    mav_array_memcpy(packet.angular_position, angular_position, sizeof(float)*6);
    mav_array_memcpy(packet.angular_velocity, angular_velocity, sizeof(float)*6);
    mav_array_memcpy(packet.angular_acceleration, angular_acceleration, sizeof(float)*6);
    mav_array_memcpy(packet.imposed_aoa_sprc, imposed_aoa_sprc, sizeof(float)*4);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_CRC);
}

/**
 * @brief Encode a naia_control_wrapper_input_csv_part1 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param naia_control_wrapper_input_csv_part1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_naia_control_wrapper_input_csv_part1_t* naia_control_wrapper_input_csv_part1)
{
    return mavlink_msg_naia_control_wrapper_input_csv_part1_pack(system_id, component_id, msg, naia_control_wrapper_input_csv_part1->time, naia_control_wrapper_input_csv_part1->raw_pressure_s, naia_control_wrapper_input_csv_part1->raw_pressure_p, naia_control_wrapper_input_csv_part1->raw_pressure_r, naia_control_wrapper_input_csv_part1->position, naia_control_wrapper_input_csv_part1->velocity, naia_control_wrapper_input_csv_part1->acceleration, naia_control_wrapper_input_csv_part1->angular_position, naia_control_wrapper_input_csv_part1->angular_velocity, naia_control_wrapper_input_csv_part1->angular_acceleration, naia_control_wrapper_input_csv_part1->imposed_aoa_sprc, naia_control_wrapper_input_csv_part1->rudder_angle);
}

/**
 * @brief Encode a naia_control_wrapper_input_csv_part1 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param naia_control_wrapper_input_csv_part1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_naia_control_wrapper_input_csv_part1_t* naia_control_wrapper_input_csv_part1)
{
    return mavlink_msg_naia_control_wrapper_input_csv_part1_pack_chan(system_id, component_id, chan, msg, naia_control_wrapper_input_csv_part1->time, naia_control_wrapper_input_csv_part1->raw_pressure_s, naia_control_wrapper_input_csv_part1->raw_pressure_p, naia_control_wrapper_input_csv_part1->raw_pressure_r, naia_control_wrapper_input_csv_part1->position, naia_control_wrapper_input_csv_part1->velocity, naia_control_wrapper_input_csv_part1->acceleration, naia_control_wrapper_input_csv_part1->angular_position, naia_control_wrapper_input_csv_part1->angular_velocity, naia_control_wrapper_input_csv_part1->angular_acceleration, naia_control_wrapper_input_csv_part1->imposed_aoa_sprc, naia_control_wrapper_input_csv_part1->rudder_angle);
}

/**
 * @brief Send a naia_control_wrapper_input_csv_part1 message
 * @param chan MAVLink channel to send the message
 *
 * @param time   Timestamp. 
 * @param raw_pressure_s   Array of raw (but offset corrected) pressures measured by the starboard pitot. 
 * @param raw_pressure_p   Array of raw (but offset corrected) pressures measured by the port pitot. 
 * @param raw_pressure_r   Array of raw (but offset corrected) pressures measured by the rear pitot. 
 * @param position   Global coordinates defining the position of the boat. The vector has the form: [Latitude, longitude, heigth]. 
 * @param velocity   Velocity measured by the IMU in combination with the GPS when signal is available. 
 * @param acceleration   Acceleration measured by the  Inertial Measurement Unit (IMU). 
 * @param angular_position   Rotational speed, measured by the IMU. 
 * @param angular_velocity   Euler angles measured by the IMU. 
 * @param angular_acceleration   Angular acceleration measured by the IMU. 
 * @param imposed_aoa_sprc   Geometric wing angles imposed manually, SPRC* convention. If set to NaN, control activates; othewise, input is used. 
 * @param rudder_angle   Rudder angle value obtained by a Magnetic encoder. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_naia_control_wrapper_input_csv_part1_send(mavlink_channel_t chan, float time, const float *raw_pressure_s, const float *raw_pressure_p, const float *raw_pressure_r, const float *position, const float *velocity, const float *acceleration, const float *angular_position, const float *angular_velocity, const float *angular_acceleration, const float *imposed_aoa_sprc, float rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN];
    _mav_put_float(buf, 0, time);
    _mav_put_float(buf, 236, rudder_angle);
    _mav_put_float_array(buf, 4, raw_pressure_s, 6);
    _mav_put_float_array(buf, 28, raw_pressure_p, 6);
    _mav_put_float_array(buf, 52, raw_pressure_r, 6);
    _mav_put_float_array(buf, 76, position, 6);
    _mav_put_float_array(buf, 100, velocity, 6);
    _mav_put_float_array(buf, 124, acceleration, 6);
    _mav_put_float_array(buf, 148, angular_position, 6);
    _mav_put_float_array(buf, 172, angular_velocity, 6);
    _mav_put_float_array(buf, 196, angular_acceleration, 6);
    _mav_put_float_array(buf, 220, imposed_aoa_sprc, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1, buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_CRC);
#else
    mavlink_naia_control_wrapper_input_csv_part1_t packet;
    packet.time = time;
    packet.rudder_angle = rudder_angle;
    mav_array_memcpy(packet.raw_pressure_s, raw_pressure_s, sizeof(float)*6);
    mav_array_memcpy(packet.raw_pressure_p, raw_pressure_p, sizeof(float)*6);
    mav_array_memcpy(packet.raw_pressure_r, raw_pressure_r, sizeof(float)*6);
    mav_array_memcpy(packet.position, position, sizeof(float)*6);
    mav_array_memcpy(packet.velocity, velocity, sizeof(float)*6);
    mav_array_memcpy(packet.acceleration, acceleration, sizeof(float)*6);
    mav_array_memcpy(packet.angular_position, angular_position, sizeof(float)*6);
    mav_array_memcpy(packet.angular_velocity, angular_velocity, sizeof(float)*6);
    mav_array_memcpy(packet.angular_acceleration, angular_acceleration, sizeof(float)*6);
    mav_array_memcpy(packet.imposed_aoa_sprc, imposed_aoa_sprc, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1, (const char *)&packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_CRC);
#endif
}

/**
 * @brief Send a naia_control_wrapper_input_csv_part1 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_naia_control_wrapper_input_csv_part1_send_struct(mavlink_channel_t chan, const mavlink_naia_control_wrapper_input_csv_part1_t* naia_control_wrapper_input_csv_part1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_naia_control_wrapper_input_csv_part1_send(chan, naia_control_wrapper_input_csv_part1->time, naia_control_wrapper_input_csv_part1->raw_pressure_s, naia_control_wrapper_input_csv_part1->raw_pressure_p, naia_control_wrapper_input_csv_part1->raw_pressure_r, naia_control_wrapper_input_csv_part1->position, naia_control_wrapper_input_csv_part1->velocity, naia_control_wrapper_input_csv_part1->acceleration, naia_control_wrapper_input_csv_part1->angular_position, naia_control_wrapper_input_csv_part1->angular_velocity, naia_control_wrapper_input_csv_part1->angular_acceleration, naia_control_wrapper_input_csv_part1->imposed_aoa_sprc, naia_control_wrapper_input_csv_part1->rudder_angle);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1, (const char *)naia_control_wrapper_input_csv_part1, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_naia_control_wrapper_input_csv_part1_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float time, const float *raw_pressure_s, const float *raw_pressure_p, const float *raw_pressure_r, const float *position, const float *velocity, const float *acceleration, const float *angular_position, const float *angular_velocity, const float *angular_acceleration, const float *imposed_aoa_sprc, float rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, time);
    _mav_put_float(buf, 236, rudder_angle);
    _mav_put_float_array(buf, 4, raw_pressure_s, 6);
    _mav_put_float_array(buf, 28, raw_pressure_p, 6);
    _mav_put_float_array(buf, 52, raw_pressure_r, 6);
    _mav_put_float_array(buf, 76, position, 6);
    _mav_put_float_array(buf, 100, velocity, 6);
    _mav_put_float_array(buf, 124, acceleration, 6);
    _mav_put_float_array(buf, 148, angular_position, 6);
    _mav_put_float_array(buf, 172, angular_velocity, 6);
    _mav_put_float_array(buf, 196, angular_acceleration, 6);
    _mav_put_float_array(buf, 220, imposed_aoa_sprc, 4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1, buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_CRC);
#else
    mavlink_naia_control_wrapper_input_csv_part1_t *packet = (mavlink_naia_control_wrapper_input_csv_part1_t *)msgbuf;
    packet->time = time;
    packet->rudder_angle = rudder_angle;
    mav_array_memcpy(packet->raw_pressure_s, raw_pressure_s, sizeof(float)*6);
    mav_array_memcpy(packet->raw_pressure_p, raw_pressure_p, sizeof(float)*6);
    mav_array_memcpy(packet->raw_pressure_r, raw_pressure_r, sizeof(float)*6);
    mav_array_memcpy(packet->position, position, sizeof(float)*6);
    mav_array_memcpy(packet->velocity, velocity, sizeof(float)*6);
    mav_array_memcpy(packet->acceleration, acceleration, sizeof(float)*6);
    mav_array_memcpy(packet->angular_position, angular_position, sizeof(float)*6);
    mav_array_memcpy(packet->angular_velocity, angular_velocity, sizeof(float)*6);
    mav_array_memcpy(packet->angular_acceleration, angular_acceleration, sizeof(float)*6);
    mav_array_memcpy(packet->imposed_aoa_sprc, imposed_aoa_sprc, sizeof(float)*4);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1, (const char *)packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_CRC);
#endif
}
#endif

#endif

// MESSAGE NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1 UNPACKING


/**
 * @brief Get field time from naia_control_wrapper_input_csv_part1 message
 *
 * @return   Timestamp. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_part1_get_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field raw_pressure_s from naia_control_wrapper_input_csv_part1 message
 *
 * @return   Array of raw (but offset corrected) pressures measured by the starboard pitot. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_get_raw_pressure_s(const mavlink_message_t* msg, float *raw_pressure_s)
{
    return _MAV_RETURN_float_array(msg, raw_pressure_s, 6,  4);
}

/**
 * @brief Get field raw_pressure_p from naia_control_wrapper_input_csv_part1 message
 *
 * @return   Array of raw (but offset corrected) pressures measured by the port pitot. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_get_raw_pressure_p(const mavlink_message_t* msg, float *raw_pressure_p)
{
    return _MAV_RETURN_float_array(msg, raw_pressure_p, 6,  28);
}

/**
 * @brief Get field raw_pressure_r from naia_control_wrapper_input_csv_part1 message
 *
 * @return   Array of raw (but offset corrected) pressures measured by the rear pitot. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_get_raw_pressure_r(const mavlink_message_t* msg, float *raw_pressure_r)
{
    return _MAV_RETURN_float_array(msg, raw_pressure_r, 6,  52);
}

/**
 * @brief Get field position from naia_control_wrapper_input_csv_part1 message
 *
 * @return   Global coordinates defining the position of the boat. The vector has the form: [Latitude, longitude, heigth]. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_get_position(const mavlink_message_t* msg, float *position)
{
    return _MAV_RETURN_float_array(msg, position, 6,  76);
}

/**
 * @brief Get field velocity from naia_control_wrapper_input_csv_part1 message
 *
 * @return   Velocity measured by the IMU in combination with the GPS when signal is available. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_get_velocity(const mavlink_message_t* msg, float *velocity)
{
    return _MAV_RETURN_float_array(msg, velocity, 6,  100);
}

/**
 * @brief Get field acceleration from naia_control_wrapper_input_csv_part1 message
 *
 * @return   Acceleration measured by the  Inertial Measurement Unit (IMU). 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_get_acceleration(const mavlink_message_t* msg, float *acceleration)
{
    return _MAV_RETURN_float_array(msg, acceleration, 6,  124);
}

/**
 * @brief Get field angular_position from naia_control_wrapper_input_csv_part1 message
 *
 * @return   Rotational speed, measured by the IMU. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_get_angular_position(const mavlink_message_t* msg, float *angular_position)
{
    return _MAV_RETURN_float_array(msg, angular_position, 6,  148);
}

/**
 * @brief Get field angular_velocity from naia_control_wrapper_input_csv_part1 message
 *
 * @return   Euler angles measured by the IMU. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_get_angular_velocity(const mavlink_message_t* msg, float *angular_velocity)
{
    return _MAV_RETURN_float_array(msg, angular_velocity, 6,  172);
}

/**
 * @brief Get field angular_acceleration from naia_control_wrapper_input_csv_part1 message
 *
 * @return   Angular acceleration measured by the IMU. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_get_angular_acceleration(const mavlink_message_t* msg, float *angular_acceleration)
{
    return _MAV_RETURN_float_array(msg, angular_acceleration, 6,  196);
}

/**
 * @brief Get field imposed_aoa_sprc from naia_control_wrapper_input_csv_part1 message
 *
 * @return   Geometric wing angles imposed manually, SPRC* convention. If set to NaN, control activates; othewise, input is used. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part1_get_imposed_aoa_sprc(const mavlink_message_t* msg, float *imposed_aoa_sprc)
{
    return _MAV_RETURN_float_array(msg, imposed_aoa_sprc, 4,  220);
}

/**
 * @brief Get field rudder_angle from naia_control_wrapper_input_csv_part1 message
 *
 * @return   Rudder angle value obtained by a Magnetic encoder. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_part1_get_rudder_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  236);
}

/**
 * @brief Decode a naia_control_wrapper_input_csv_part1 message into a struct
 *
 * @param msg The message to decode
 * @param naia_control_wrapper_input_csv_part1 C-struct to decode the message contents into
 */
static inline void mavlink_msg_naia_control_wrapper_input_csv_part1_decode(const mavlink_message_t* msg, mavlink_naia_control_wrapper_input_csv_part1_t* naia_control_wrapper_input_csv_part1)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    naia_control_wrapper_input_csv_part1->time = mavlink_msg_naia_control_wrapper_input_csv_part1_get_time(msg);
    mavlink_msg_naia_control_wrapper_input_csv_part1_get_raw_pressure_s(msg, naia_control_wrapper_input_csv_part1->raw_pressure_s);
    mavlink_msg_naia_control_wrapper_input_csv_part1_get_raw_pressure_p(msg, naia_control_wrapper_input_csv_part1->raw_pressure_p);
    mavlink_msg_naia_control_wrapper_input_csv_part1_get_raw_pressure_r(msg, naia_control_wrapper_input_csv_part1->raw_pressure_r);
    mavlink_msg_naia_control_wrapper_input_csv_part1_get_position(msg, naia_control_wrapper_input_csv_part1->position);
    mavlink_msg_naia_control_wrapper_input_csv_part1_get_velocity(msg, naia_control_wrapper_input_csv_part1->velocity);
    mavlink_msg_naia_control_wrapper_input_csv_part1_get_acceleration(msg, naia_control_wrapper_input_csv_part1->acceleration);
    mavlink_msg_naia_control_wrapper_input_csv_part1_get_angular_position(msg, naia_control_wrapper_input_csv_part1->angular_position);
    mavlink_msg_naia_control_wrapper_input_csv_part1_get_angular_velocity(msg, naia_control_wrapper_input_csv_part1->angular_velocity);
    mavlink_msg_naia_control_wrapper_input_csv_part1_get_angular_acceleration(msg, naia_control_wrapper_input_csv_part1->angular_acceleration);
    mavlink_msg_naia_control_wrapper_input_csv_part1_get_imposed_aoa_sprc(msg, naia_control_wrapper_input_csv_part1->imposed_aoa_sprc);
    naia_control_wrapper_input_csv_part1->rudder_angle = mavlink_msg_naia_control_wrapper_input_csv_part1_get_rudder_angle(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN? msg->len : MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN;
        memset(naia_control_wrapper_input_csv_part1, 0, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART1_LEN);
    memcpy(naia_control_wrapper_input_csv_part1, _MAV_PAYLOAD(msg), len);
#endif
}
