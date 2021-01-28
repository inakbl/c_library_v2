#pragma once
// MESSAGE NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2 PACKING

#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2 617

MAVPACKED(
typedef struct __mavlink_naia_control_wrapper_input_csv_part2_t {
 float time; /*<   Timestamp. */
 float control_kp_gain[10]; /*<   Proportional control gain vector. The vector has  m elements. */
 float control_ki_gain[10]; /*<   Integral control gain vector. The vector has  m elements. */
 float control_kd_gain[10]; /*<   Derivative control gain vector. The vector has  m elements. */
 float control_gains_tacking[3]; /*<   Control gains set manually for tacking maneuver. The vector has 3  (P,I,D) elements. */
 float interpolation_speeds[10]; /*<   Speeds of the boat at which the control gains (control_Gains) have been adjusted. The vector has m elements. */
 float reference_height[10]; /*<   Reference height above the water surface for the control loop. The vector has m elements. */
}) mavlink_naia_control_wrapper_input_csv_part2_t;

#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN 216
#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_MIN_LEN 216
#define MAVLINK_MSG_ID_617_LEN 216
#define MAVLINK_MSG_ID_617_MIN_LEN 216

#define MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_CRC 141
#define MAVLINK_MSG_ID_617_CRC 141

#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_FIELD_CONTROL_KP_GAIN_LEN 10
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_FIELD_CONTROL_KI_GAIN_LEN 10
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_FIELD_CONTROL_KD_GAIN_LEN 10
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_FIELD_CONTROL_GAINS_TACKING_LEN 3
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_FIELD_INTERPOLATION_SPEEDS_LEN 10
#define MAVLINK_MSG_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_FIELD_REFERENCE_HEIGHT_LEN 10

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2 { \
    617, \
    "NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2", \
    7, \
    {  { "time", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, time) }, \
         { "control_kp_gain", NULL, MAVLINK_TYPE_FLOAT, 10, 4, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, control_kp_gain) }, \
         { "control_ki_gain", NULL, MAVLINK_TYPE_FLOAT, 10, 44, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, control_ki_gain) }, \
         { "control_kd_gain", NULL, MAVLINK_TYPE_FLOAT, 10, 84, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, control_kd_gain) }, \
         { "control_gains_tacking", NULL, MAVLINK_TYPE_FLOAT, 3, 124, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, control_gains_tacking) }, \
         { "interpolation_speeds", NULL, MAVLINK_TYPE_FLOAT, 10, 136, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, interpolation_speeds) }, \
         { "reference_height", NULL, MAVLINK_TYPE_FLOAT, 10, 176, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, reference_height) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2 { \
    "NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2", \
    7, \
    {  { "time", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, time) }, \
         { "control_kp_gain", NULL, MAVLINK_TYPE_FLOAT, 10, 4, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, control_kp_gain) }, \
         { "control_ki_gain", NULL, MAVLINK_TYPE_FLOAT, 10, 44, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, control_ki_gain) }, \
         { "control_kd_gain", NULL, MAVLINK_TYPE_FLOAT, 10, 84, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, control_kd_gain) }, \
         { "control_gains_tacking", NULL, MAVLINK_TYPE_FLOAT, 3, 124, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, control_gains_tacking) }, \
         { "interpolation_speeds", NULL, MAVLINK_TYPE_FLOAT, 10, 136, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, interpolation_speeds) }, \
         { "reference_height", NULL, MAVLINK_TYPE_FLOAT, 10, 176, offsetof(mavlink_naia_control_wrapper_input_csv_part2_t, reference_height) }, \
         } \
}
#endif

/**
 * @brief Pack a naia_control_wrapper_input_csv_part2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time   Timestamp. 
 * @param control_kp_gain   Proportional control gain vector. The vector has  m elements. 
 * @param control_ki_gain   Integral control gain vector. The vector has  m elements. 
 * @param control_kd_gain   Derivative control gain vector. The vector has  m elements. 
 * @param control_gains_tacking   Control gains set manually for tacking maneuver. The vector has 3  (P,I,D) elements. 
 * @param interpolation_speeds   Speeds of the boat at which the control gains (control_Gains) have been adjusted. The vector has m elements. 
 * @param reference_height   Reference height above the water surface for the control loop. The vector has m elements. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float time, const float *control_kp_gain, const float *control_ki_gain, const float *control_kd_gain, const float *control_gains_tacking, const float *interpolation_speeds, const float *reference_height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN];
    _mav_put_float(buf, 0, time);
    _mav_put_float_array(buf, 4, control_kp_gain, 10);
    _mav_put_float_array(buf, 44, control_ki_gain, 10);
    _mav_put_float_array(buf, 84, control_kd_gain, 10);
    _mav_put_float_array(buf, 124, control_gains_tacking, 3);
    _mav_put_float_array(buf, 136, interpolation_speeds, 10);
    _mav_put_float_array(buf, 176, reference_height, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN);
#else
    mavlink_naia_control_wrapper_input_csv_part2_t packet;
    packet.time = time;
    mav_array_memcpy(packet.control_kp_gain, control_kp_gain, sizeof(float)*10);
    mav_array_memcpy(packet.control_ki_gain, control_ki_gain, sizeof(float)*10);
    mav_array_memcpy(packet.control_kd_gain, control_kd_gain, sizeof(float)*10);
    mav_array_memcpy(packet.control_gains_tacking, control_gains_tacking, sizeof(float)*3);
    mav_array_memcpy(packet.interpolation_speeds, interpolation_speeds, sizeof(float)*10);
    mav_array_memcpy(packet.reference_height, reference_height, sizeof(float)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_CRC);
}

/**
 * @brief Pack a naia_control_wrapper_input_csv_part2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time   Timestamp. 
 * @param control_kp_gain   Proportional control gain vector. The vector has  m elements. 
 * @param control_ki_gain   Integral control gain vector. The vector has  m elements. 
 * @param control_kd_gain   Derivative control gain vector. The vector has  m elements. 
 * @param control_gains_tacking   Control gains set manually for tacking maneuver. The vector has 3  (P,I,D) elements. 
 * @param interpolation_speeds   Speeds of the boat at which the control gains (control_Gains) have been adjusted. The vector has m elements. 
 * @param reference_height   Reference height above the water surface for the control loop. The vector has m elements. 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float time,const float *control_kp_gain,const float *control_ki_gain,const float *control_kd_gain,const float *control_gains_tacking,const float *interpolation_speeds,const float *reference_height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN];
    _mav_put_float(buf, 0, time);
    _mav_put_float_array(buf, 4, control_kp_gain, 10);
    _mav_put_float_array(buf, 44, control_ki_gain, 10);
    _mav_put_float_array(buf, 84, control_kd_gain, 10);
    _mav_put_float_array(buf, 124, control_gains_tacking, 3);
    _mav_put_float_array(buf, 136, interpolation_speeds, 10);
    _mav_put_float_array(buf, 176, reference_height, 10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN);
#else
    mavlink_naia_control_wrapper_input_csv_part2_t packet;
    packet.time = time;
    mav_array_memcpy(packet.control_kp_gain, control_kp_gain, sizeof(float)*10);
    mav_array_memcpy(packet.control_ki_gain, control_ki_gain, sizeof(float)*10);
    mav_array_memcpy(packet.control_kd_gain, control_kd_gain, sizeof(float)*10);
    mav_array_memcpy(packet.control_gains_tacking, control_gains_tacking, sizeof(float)*3);
    mav_array_memcpy(packet.interpolation_speeds, interpolation_speeds, sizeof(float)*10);
    mav_array_memcpy(packet.reference_height, reference_height, sizeof(float)*10);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_CRC);
}

/**
 * @brief Encode a naia_control_wrapper_input_csv_part2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param naia_control_wrapper_input_csv_part2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_naia_control_wrapper_input_csv_part2_t* naia_control_wrapper_input_csv_part2)
{
    return mavlink_msg_naia_control_wrapper_input_csv_part2_pack(system_id, component_id, msg, naia_control_wrapper_input_csv_part2->time, naia_control_wrapper_input_csv_part2->control_kp_gain, naia_control_wrapper_input_csv_part2->control_ki_gain, naia_control_wrapper_input_csv_part2->control_kd_gain, naia_control_wrapper_input_csv_part2->control_gains_tacking, naia_control_wrapper_input_csv_part2->interpolation_speeds, naia_control_wrapper_input_csv_part2->reference_height);
}

/**
 * @brief Encode a naia_control_wrapper_input_csv_part2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param naia_control_wrapper_input_csv_part2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_naia_control_wrapper_input_csv_part2_t* naia_control_wrapper_input_csv_part2)
{
    return mavlink_msg_naia_control_wrapper_input_csv_part2_pack_chan(system_id, component_id, chan, msg, naia_control_wrapper_input_csv_part2->time, naia_control_wrapper_input_csv_part2->control_kp_gain, naia_control_wrapper_input_csv_part2->control_ki_gain, naia_control_wrapper_input_csv_part2->control_kd_gain, naia_control_wrapper_input_csv_part2->control_gains_tacking, naia_control_wrapper_input_csv_part2->interpolation_speeds, naia_control_wrapper_input_csv_part2->reference_height);
}

/**
 * @brief Send a naia_control_wrapper_input_csv_part2 message
 * @param chan MAVLink channel to send the message
 *
 * @param time   Timestamp. 
 * @param control_kp_gain   Proportional control gain vector. The vector has  m elements. 
 * @param control_ki_gain   Integral control gain vector. The vector has  m elements. 
 * @param control_kd_gain   Derivative control gain vector. The vector has  m elements. 
 * @param control_gains_tacking   Control gains set manually for tacking maneuver. The vector has 3  (P,I,D) elements. 
 * @param interpolation_speeds   Speeds of the boat at which the control gains (control_Gains) have been adjusted. The vector has m elements. 
 * @param reference_height   Reference height above the water surface for the control loop. The vector has m elements. 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_naia_control_wrapper_input_csv_part2_send(mavlink_channel_t chan, float time, const float *control_kp_gain, const float *control_ki_gain, const float *control_kd_gain, const float *control_gains_tacking, const float *interpolation_speeds, const float *reference_height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN];
    _mav_put_float(buf, 0, time);
    _mav_put_float_array(buf, 4, control_kp_gain, 10);
    _mav_put_float_array(buf, 44, control_ki_gain, 10);
    _mav_put_float_array(buf, 84, control_kd_gain, 10);
    _mav_put_float_array(buf, 124, control_gains_tacking, 3);
    _mav_put_float_array(buf, 136, interpolation_speeds, 10);
    _mav_put_float_array(buf, 176, reference_height, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2, buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_CRC);
#else
    mavlink_naia_control_wrapper_input_csv_part2_t packet;
    packet.time = time;
    mav_array_memcpy(packet.control_kp_gain, control_kp_gain, sizeof(float)*10);
    mav_array_memcpy(packet.control_ki_gain, control_ki_gain, sizeof(float)*10);
    mav_array_memcpy(packet.control_kd_gain, control_kd_gain, sizeof(float)*10);
    mav_array_memcpy(packet.control_gains_tacking, control_gains_tacking, sizeof(float)*3);
    mav_array_memcpy(packet.interpolation_speeds, interpolation_speeds, sizeof(float)*10);
    mav_array_memcpy(packet.reference_height, reference_height, sizeof(float)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2, (const char *)&packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_CRC);
#endif
}

/**
 * @brief Send a naia_control_wrapper_input_csv_part2 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_naia_control_wrapper_input_csv_part2_send_struct(mavlink_channel_t chan, const mavlink_naia_control_wrapper_input_csv_part2_t* naia_control_wrapper_input_csv_part2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_naia_control_wrapper_input_csv_part2_send(chan, naia_control_wrapper_input_csv_part2->time, naia_control_wrapper_input_csv_part2->control_kp_gain, naia_control_wrapper_input_csv_part2->control_ki_gain, naia_control_wrapper_input_csv_part2->control_kd_gain, naia_control_wrapper_input_csv_part2->control_gains_tacking, naia_control_wrapper_input_csv_part2->interpolation_speeds, naia_control_wrapper_input_csv_part2->reference_height);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2, (const char *)naia_control_wrapper_input_csv_part2, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_naia_control_wrapper_input_csv_part2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float time, const float *control_kp_gain, const float *control_ki_gain, const float *control_kd_gain, const float *control_gains_tacking, const float *interpolation_speeds, const float *reference_height)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, time);
    _mav_put_float_array(buf, 4, control_kp_gain, 10);
    _mav_put_float_array(buf, 44, control_ki_gain, 10);
    _mav_put_float_array(buf, 84, control_kd_gain, 10);
    _mav_put_float_array(buf, 124, control_gains_tacking, 3);
    _mav_put_float_array(buf, 136, interpolation_speeds, 10);
    _mav_put_float_array(buf, 176, reference_height, 10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2, buf, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_CRC);
#else
    mavlink_naia_control_wrapper_input_csv_part2_t *packet = (mavlink_naia_control_wrapper_input_csv_part2_t *)msgbuf;
    packet->time = time;
    mav_array_memcpy(packet->control_kp_gain, control_kp_gain, sizeof(float)*10);
    mav_array_memcpy(packet->control_ki_gain, control_ki_gain, sizeof(float)*10);
    mav_array_memcpy(packet->control_kd_gain, control_kd_gain, sizeof(float)*10);
    mav_array_memcpy(packet->control_gains_tacking, control_gains_tacking, sizeof(float)*3);
    mav_array_memcpy(packet->interpolation_speeds, interpolation_speeds, sizeof(float)*10);
    mav_array_memcpy(packet->reference_height, reference_height, sizeof(float)*10);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2, (const char *)packet, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_MIN_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_CRC);
#endif
}
#endif

#endif

// MESSAGE NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2 UNPACKING


/**
 * @brief Get field time from naia_control_wrapper_input_csv_part2 message
 *
 * @return   Timestamp. 
 */
static inline float mavlink_msg_naia_control_wrapper_input_csv_part2_get_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field control_kp_gain from naia_control_wrapper_input_csv_part2 message
 *
 * @return   Proportional control gain vector. The vector has  m elements. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part2_get_control_kp_gain(const mavlink_message_t* msg, float *control_kp_gain)
{
    return _MAV_RETURN_float_array(msg, control_kp_gain, 10,  4);
}

/**
 * @brief Get field control_ki_gain from naia_control_wrapper_input_csv_part2 message
 *
 * @return   Integral control gain vector. The vector has  m elements. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part2_get_control_ki_gain(const mavlink_message_t* msg, float *control_ki_gain)
{
    return _MAV_RETURN_float_array(msg, control_ki_gain, 10,  44);
}

/**
 * @brief Get field control_kd_gain from naia_control_wrapper_input_csv_part2 message
 *
 * @return   Derivative control gain vector. The vector has  m elements. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part2_get_control_kd_gain(const mavlink_message_t* msg, float *control_kd_gain)
{
    return _MAV_RETURN_float_array(msg, control_kd_gain, 10,  84);
}

/**
 * @brief Get field control_gains_tacking from naia_control_wrapper_input_csv_part2 message
 *
 * @return   Control gains set manually for tacking maneuver. The vector has 3  (P,I,D) elements. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part2_get_control_gains_tacking(const mavlink_message_t* msg, float *control_gains_tacking)
{
    return _MAV_RETURN_float_array(msg, control_gains_tacking, 3,  124);
}

/**
 * @brief Get field interpolation_speeds from naia_control_wrapper_input_csv_part2 message
 *
 * @return   Speeds of the boat at which the control gains (control_Gains) have been adjusted. The vector has m elements. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part2_get_interpolation_speeds(const mavlink_message_t* msg, float *interpolation_speeds)
{
    return _MAV_RETURN_float_array(msg, interpolation_speeds, 10,  136);
}

/**
 * @brief Get field reference_height from naia_control_wrapper_input_csv_part2 message
 *
 * @return   Reference height above the water surface for the control loop. The vector has m elements. 
 */
static inline uint16_t mavlink_msg_naia_control_wrapper_input_csv_part2_get_reference_height(const mavlink_message_t* msg, float *reference_height)
{
    return _MAV_RETURN_float_array(msg, reference_height, 10,  176);
}

/**
 * @brief Decode a naia_control_wrapper_input_csv_part2 message into a struct
 *
 * @param msg The message to decode
 * @param naia_control_wrapper_input_csv_part2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_naia_control_wrapper_input_csv_part2_decode(const mavlink_message_t* msg, mavlink_naia_control_wrapper_input_csv_part2_t* naia_control_wrapper_input_csv_part2)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    naia_control_wrapper_input_csv_part2->time = mavlink_msg_naia_control_wrapper_input_csv_part2_get_time(msg);
    mavlink_msg_naia_control_wrapper_input_csv_part2_get_control_kp_gain(msg, naia_control_wrapper_input_csv_part2->control_kp_gain);
    mavlink_msg_naia_control_wrapper_input_csv_part2_get_control_ki_gain(msg, naia_control_wrapper_input_csv_part2->control_ki_gain);
    mavlink_msg_naia_control_wrapper_input_csv_part2_get_control_kd_gain(msg, naia_control_wrapper_input_csv_part2->control_kd_gain);
    mavlink_msg_naia_control_wrapper_input_csv_part2_get_control_gains_tacking(msg, naia_control_wrapper_input_csv_part2->control_gains_tacking);
    mavlink_msg_naia_control_wrapper_input_csv_part2_get_interpolation_speeds(msg, naia_control_wrapper_input_csv_part2->interpolation_speeds);
    mavlink_msg_naia_control_wrapper_input_csv_part2_get_reference_height(msg, naia_control_wrapper_input_csv_part2->reference_height);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN? msg->len : MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN;
        memset(naia_control_wrapper_input_csv_part2, 0, MAVLINK_MSG_ID_NAIA_CONTROL_WRAPPER_INPUT_CSV_PART2_LEN);
    memcpy(naia_control_wrapper_input_csv_part2, _MAV_PAYLOAD(msg), len);
#endif
}
