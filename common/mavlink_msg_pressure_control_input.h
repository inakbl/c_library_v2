#pragma once
// MESSAGE PRESSURE_CONTROL_INPUT PACKING

#define MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT 602


typedef struct __mavlink_pressure_control_input_t {
 float measure_pressure; /*<   Pressure read */
 float kp; /*<   P term of PID control */
 float kd; /*<   D term of PID control */
 float ki; /*<   I term of PID control */
 float wind_up; /*<   Limit of PID integration error */
 float speed; /*< [m/s]  Boat speed */
 float k_delay; /*<   K delay */
 float height_set_point; /*< [m]  Height reference */
 float pressure_offset; /*< [Pa]  Offset in pressure */
 float equilibrium_angle; /*< [deg]  Reference angle for actuators */
 float angle_multiplier; /*<   Constant for PID control results */
 float change_control_sign; /*<   Constant for PID control results */
 uint8_t control_enabled; /*<   (BOOL) Flag to activate control */
 uint8_t enable_torque; /*<   (BOOL) Activate torque */
 uint8_t config_mode; /*<   (BOOL) Activate config mode */
} mavlink_pressure_control_input_t;

#define MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN 51
#define MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_MIN_LEN 51
#define MAVLINK_MSG_ID_602_LEN 51
#define MAVLINK_MSG_ID_602_MIN_LEN 51

#define MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_CRC 241
#define MAVLINK_MSG_ID_602_CRC 241



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PRESSURE_CONTROL_INPUT { \
    602, \
    "PRESSURE_CONTROL_INPUT", \
    15, \
    {  { "measure_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_pressure_control_input_t, measure_pressure) }, \
         { "kp", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pressure_control_input_t, kp) }, \
         { "kd", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_pressure_control_input_t, kd) }, \
         { "ki", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pressure_control_input_t, ki) }, \
         { "wind_up", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_pressure_control_input_t, wind_up) }, \
         { "control_enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_pressure_control_input_t, control_enabled) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_pressure_control_input_t, speed) }, \
         { "k_delay", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_pressure_control_input_t, k_delay) }, \
         { "height_set_point", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_pressure_control_input_t, height_set_point) }, \
         { "pressure_offset", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_pressure_control_input_t, pressure_offset) }, \
         { "equilibrium_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_pressure_control_input_t, equilibrium_angle) }, \
         { "angle_multiplier", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_pressure_control_input_t, angle_multiplier) }, \
         { "change_control_sign", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_pressure_control_input_t, change_control_sign) }, \
         { "enable_torque", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_pressure_control_input_t, enable_torque) }, \
         { "config_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_pressure_control_input_t, config_mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PRESSURE_CONTROL_INPUT { \
    "PRESSURE_CONTROL_INPUT", \
    15, \
    {  { "measure_pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_pressure_control_input_t, measure_pressure) }, \
         { "kp", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_pressure_control_input_t, kp) }, \
         { "kd", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_pressure_control_input_t, kd) }, \
         { "ki", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_pressure_control_input_t, ki) }, \
         { "wind_up", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_pressure_control_input_t, wind_up) }, \
         { "control_enabled", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_pressure_control_input_t, control_enabled) }, \
         { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_pressure_control_input_t, speed) }, \
         { "k_delay", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_pressure_control_input_t, k_delay) }, \
         { "height_set_point", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_pressure_control_input_t, height_set_point) }, \
         { "pressure_offset", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_pressure_control_input_t, pressure_offset) }, \
         { "equilibrium_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_pressure_control_input_t, equilibrium_angle) }, \
         { "angle_multiplier", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_pressure_control_input_t, angle_multiplier) }, \
         { "change_control_sign", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_pressure_control_input_t, change_control_sign) }, \
         { "enable_torque", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_pressure_control_input_t, enable_torque) }, \
         { "config_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_pressure_control_input_t, config_mode) }, \
         } \
}
#endif

/**
 * @brief Pack a pressure_control_input message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param measure_pressure   Pressure read 
 * @param kp   P term of PID control 
 * @param kd   D term of PID control 
 * @param ki   I term of PID control 
 * @param wind_up   Limit of PID integration error 
 * @param control_enabled   (BOOL) Flag to activate control 
 * @param speed [m/s]  Boat speed 
 * @param k_delay   K delay 
 * @param height_set_point [m]  Height reference 
 * @param pressure_offset [Pa]  Offset in pressure 
 * @param equilibrium_angle [deg]  Reference angle for actuators 
 * @param angle_multiplier   Constant for PID control results 
 * @param change_control_sign   Constant for PID control results 
 * @param enable_torque   (BOOL) Activate torque 
 * @param config_mode   (BOOL) Activate config mode 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pressure_control_input_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float measure_pressure, float kp, float kd, float ki, float wind_up, uint8_t control_enabled, float speed, float k_delay, float height_set_point, float pressure_offset, float equilibrium_angle, float angle_multiplier, float change_control_sign, uint8_t enable_torque, uint8_t config_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN];
    _mav_put_float(buf, 0, measure_pressure);
    _mav_put_float(buf, 4, kp);
    _mav_put_float(buf, 8, kd);
    _mav_put_float(buf, 12, ki);
    _mav_put_float(buf, 16, wind_up);
    _mav_put_float(buf, 20, speed);
    _mav_put_float(buf, 24, k_delay);
    _mav_put_float(buf, 28, height_set_point);
    _mav_put_float(buf, 32, pressure_offset);
    _mav_put_float(buf, 36, equilibrium_angle);
    _mav_put_float(buf, 40, angle_multiplier);
    _mav_put_float(buf, 44, change_control_sign);
    _mav_put_uint8_t(buf, 48, control_enabled);
    _mav_put_uint8_t(buf, 49, enable_torque);
    _mav_put_uint8_t(buf, 50, config_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN);
#else
    mavlink_pressure_control_input_t packet;
    packet.measure_pressure = measure_pressure;
    packet.kp = kp;
    packet.kd = kd;
    packet.ki = ki;
    packet.wind_up = wind_up;
    packet.speed = speed;
    packet.k_delay = k_delay;
    packet.height_set_point = height_set_point;
    packet.pressure_offset = pressure_offset;
    packet.equilibrium_angle = equilibrium_angle;
    packet.angle_multiplier = angle_multiplier;
    packet.change_control_sign = change_control_sign;
    packet.control_enabled = control_enabled;
    packet.enable_torque = enable_torque;
    packet.config_mode = config_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_CRC);
}

/**
 * @brief Pack a pressure_control_input message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param measure_pressure   Pressure read 
 * @param kp   P term of PID control 
 * @param kd   D term of PID control 
 * @param ki   I term of PID control 
 * @param wind_up   Limit of PID integration error 
 * @param control_enabled   (BOOL) Flag to activate control 
 * @param speed [m/s]  Boat speed 
 * @param k_delay   K delay 
 * @param height_set_point [m]  Height reference 
 * @param pressure_offset [Pa]  Offset in pressure 
 * @param equilibrium_angle [deg]  Reference angle for actuators 
 * @param angle_multiplier   Constant for PID control results 
 * @param change_control_sign   Constant for PID control results 
 * @param enable_torque   (BOOL) Activate torque 
 * @param config_mode   (BOOL) Activate config mode 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pressure_control_input_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float measure_pressure,float kp,float kd,float ki,float wind_up,uint8_t control_enabled,float speed,float k_delay,float height_set_point,float pressure_offset,float equilibrium_angle,float angle_multiplier,float change_control_sign,uint8_t enable_torque,uint8_t config_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN];
    _mav_put_float(buf, 0, measure_pressure);
    _mav_put_float(buf, 4, kp);
    _mav_put_float(buf, 8, kd);
    _mav_put_float(buf, 12, ki);
    _mav_put_float(buf, 16, wind_up);
    _mav_put_float(buf, 20, speed);
    _mav_put_float(buf, 24, k_delay);
    _mav_put_float(buf, 28, height_set_point);
    _mav_put_float(buf, 32, pressure_offset);
    _mav_put_float(buf, 36, equilibrium_angle);
    _mav_put_float(buf, 40, angle_multiplier);
    _mav_put_float(buf, 44, change_control_sign);
    _mav_put_uint8_t(buf, 48, control_enabled);
    _mav_put_uint8_t(buf, 49, enable_torque);
    _mav_put_uint8_t(buf, 50, config_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN);
#else
    mavlink_pressure_control_input_t packet;
    packet.measure_pressure = measure_pressure;
    packet.kp = kp;
    packet.kd = kd;
    packet.ki = ki;
    packet.wind_up = wind_up;
    packet.speed = speed;
    packet.k_delay = k_delay;
    packet.height_set_point = height_set_point;
    packet.pressure_offset = pressure_offset;
    packet.equilibrium_angle = equilibrium_angle;
    packet.angle_multiplier = angle_multiplier;
    packet.change_control_sign = change_control_sign;
    packet.control_enabled = control_enabled;
    packet.enable_torque = enable_torque;
    packet.config_mode = config_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_CRC);
}

/**
 * @brief Encode a pressure_control_input struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pressure_control_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pressure_control_input_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pressure_control_input_t* pressure_control_input)
{
    return mavlink_msg_pressure_control_input_pack(system_id, component_id, msg, pressure_control_input->measure_pressure, pressure_control_input->kp, pressure_control_input->kd, pressure_control_input->ki, pressure_control_input->wind_up, pressure_control_input->control_enabled, pressure_control_input->speed, pressure_control_input->k_delay, pressure_control_input->height_set_point, pressure_control_input->pressure_offset, pressure_control_input->equilibrium_angle, pressure_control_input->angle_multiplier, pressure_control_input->change_control_sign, pressure_control_input->enable_torque, pressure_control_input->config_mode);
}

/**
 * @brief Encode a pressure_control_input struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param pressure_control_input C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pressure_control_input_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_pressure_control_input_t* pressure_control_input)
{
    return mavlink_msg_pressure_control_input_pack_chan(system_id, component_id, chan, msg, pressure_control_input->measure_pressure, pressure_control_input->kp, pressure_control_input->kd, pressure_control_input->ki, pressure_control_input->wind_up, pressure_control_input->control_enabled, pressure_control_input->speed, pressure_control_input->k_delay, pressure_control_input->height_set_point, pressure_control_input->pressure_offset, pressure_control_input->equilibrium_angle, pressure_control_input->angle_multiplier, pressure_control_input->change_control_sign, pressure_control_input->enable_torque, pressure_control_input->config_mode);
}

/**
 * @brief Send a pressure_control_input message
 * @param chan MAVLink channel to send the message
 *
 * @param measure_pressure   Pressure read 
 * @param kp   P term of PID control 
 * @param kd   D term of PID control 
 * @param ki   I term of PID control 
 * @param wind_up   Limit of PID integration error 
 * @param control_enabled   (BOOL) Flag to activate control 
 * @param speed [m/s]  Boat speed 
 * @param k_delay   K delay 
 * @param height_set_point [m]  Height reference 
 * @param pressure_offset [Pa]  Offset in pressure 
 * @param equilibrium_angle [deg]  Reference angle for actuators 
 * @param angle_multiplier   Constant for PID control results 
 * @param change_control_sign   Constant for PID control results 
 * @param enable_torque   (BOOL) Activate torque 
 * @param config_mode   (BOOL) Activate config mode 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pressure_control_input_send(mavlink_channel_t chan, float measure_pressure, float kp, float kd, float ki, float wind_up, uint8_t control_enabled, float speed, float k_delay, float height_set_point, float pressure_offset, float equilibrium_angle, float angle_multiplier, float change_control_sign, uint8_t enable_torque, uint8_t config_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN];
    _mav_put_float(buf, 0, measure_pressure);
    _mav_put_float(buf, 4, kp);
    _mav_put_float(buf, 8, kd);
    _mav_put_float(buf, 12, ki);
    _mav_put_float(buf, 16, wind_up);
    _mav_put_float(buf, 20, speed);
    _mav_put_float(buf, 24, k_delay);
    _mav_put_float(buf, 28, height_set_point);
    _mav_put_float(buf, 32, pressure_offset);
    _mav_put_float(buf, 36, equilibrium_angle);
    _mav_put_float(buf, 40, angle_multiplier);
    _mav_put_float(buf, 44, change_control_sign);
    _mav_put_uint8_t(buf, 48, control_enabled);
    _mav_put_uint8_t(buf, 49, enable_torque);
    _mav_put_uint8_t(buf, 50, config_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT, buf, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_CRC);
#else
    mavlink_pressure_control_input_t packet;
    packet.measure_pressure = measure_pressure;
    packet.kp = kp;
    packet.kd = kd;
    packet.ki = ki;
    packet.wind_up = wind_up;
    packet.speed = speed;
    packet.k_delay = k_delay;
    packet.height_set_point = height_set_point;
    packet.pressure_offset = pressure_offset;
    packet.equilibrium_angle = equilibrium_angle;
    packet.angle_multiplier = angle_multiplier;
    packet.change_control_sign = change_control_sign;
    packet.control_enabled = control_enabled;
    packet.enable_torque = enable_torque;
    packet.config_mode = config_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT, (const char *)&packet, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_CRC);
#endif
}

/**
 * @brief Send a pressure_control_input message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_pressure_control_input_send_struct(mavlink_channel_t chan, const mavlink_pressure_control_input_t* pressure_control_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_pressure_control_input_send(chan, pressure_control_input->measure_pressure, pressure_control_input->kp, pressure_control_input->kd, pressure_control_input->ki, pressure_control_input->wind_up, pressure_control_input->control_enabled, pressure_control_input->speed, pressure_control_input->k_delay, pressure_control_input->height_set_point, pressure_control_input->pressure_offset, pressure_control_input->equilibrium_angle, pressure_control_input->angle_multiplier, pressure_control_input->change_control_sign, pressure_control_input->enable_torque, pressure_control_input->config_mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT, (const char *)pressure_control_input, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_CRC);
#endif
}

#if MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_pressure_control_input_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float measure_pressure, float kp, float kd, float ki, float wind_up, uint8_t control_enabled, float speed, float k_delay, float height_set_point, float pressure_offset, float equilibrium_angle, float angle_multiplier, float change_control_sign, uint8_t enable_torque, uint8_t config_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, measure_pressure);
    _mav_put_float(buf, 4, kp);
    _mav_put_float(buf, 8, kd);
    _mav_put_float(buf, 12, ki);
    _mav_put_float(buf, 16, wind_up);
    _mav_put_float(buf, 20, speed);
    _mav_put_float(buf, 24, k_delay);
    _mav_put_float(buf, 28, height_set_point);
    _mav_put_float(buf, 32, pressure_offset);
    _mav_put_float(buf, 36, equilibrium_angle);
    _mav_put_float(buf, 40, angle_multiplier);
    _mav_put_float(buf, 44, change_control_sign);
    _mav_put_uint8_t(buf, 48, control_enabled);
    _mav_put_uint8_t(buf, 49, enable_torque);
    _mav_put_uint8_t(buf, 50, config_mode);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT, buf, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_CRC);
#else
    mavlink_pressure_control_input_t *packet = (mavlink_pressure_control_input_t *)msgbuf;
    packet->measure_pressure = measure_pressure;
    packet->kp = kp;
    packet->kd = kd;
    packet->ki = ki;
    packet->wind_up = wind_up;
    packet->speed = speed;
    packet->k_delay = k_delay;
    packet->height_set_point = height_set_point;
    packet->pressure_offset = pressure_offset;
    packet->equilibrium_angle = equilibrium_angle;
    packet->angle_multiplier = angle_multiplier;
    packet->change_control_sign = change_control_sign;
    packet->control_enabled = control_enabled;
    packet->enable_torque = enable_torque;
    packet->config_mode = config_mode;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT, (const char *)packet, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_MIN_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_CRC);
#endif
}
#endif

#endif

// MESSAGE PRESSURE_CONTROL_INPUT UNPACKING


/**
 * @brief Get field measure_pressure from pressure_control_input message
 *
 * @return   Pressure read 
 */
static inline float mavlink_msg_pressure_control_input_get_measure_pressure(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field kp from pressure_control_input message
 *
 * @return   P term of PID control 
 */
static inline float mavlink_msg_pressure_control_input_get_kp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field kd from pressure_control_input message
 *
 * @return   D term of PID control 
 */
static inline float mavlink_msg_pressure_control_input_get_kd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field ki from pressure_control_input message
 *
 * @return   I term of PID control 
 */
static inline float mavlink_msg_pressure_control_input_get_ki(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field wind_up from pressure_control_input message
 *
 * @return   Limit of PID integration error 
 */
static inline float mavlink_msg_pressure_control_input_get_wind_up(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field control_enabled from pressure_control_input message
 *
 * @return   (BOOL) Flag to activate control 
 */
static inline uint8_t mavlink_msg_pressure_control_input_get_control_enabled(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field speed from pressure_control_input message
 *
 * @return [m/s]  Boat speed 
 */
static inline float mavlink_msg_pressure_control_input_get_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field k_delay from pressure_control_input message
 *
 * @return   K delay 
 */
static inline float mavlink_msg_pressure_control_input_get_k_delay(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field height_set_point from pressure_control_input message
 *
 * @return [m]  Height reference 
 */
static inline float mavlink_msg_pressure_control_input_get_height_set_point(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field pressure_offset from pressure_control_input message
 *
 * @return [Pa]  Offset in pressure 
 */
static inline float mavlink_msg_pressure_control_input_get_pressure_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field equilibrium_angle from pressure_control_input message
 *
 * @return [deg]  Reference angle for actuators 
 */
static inline float mavlink_msg_pressure_control_input_get_equilibrium_angle(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field angle_multiplier from pressure_control_input message
 *
 * @return   Constant for PID control results 
 */
static inline float mavlink_msg_pressure_control_input_get_angle_multiplier(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field change_control_sign from pressure_control_input message
 *
 * @return   Constant for PID control results 
 */
static inline float mavlink_msg_pressure_control_input_get_change_control_sign(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field enable_torque from pressure_control_input message
 *
 * @return   (BOOL) Activate torque 
 */
static inline uint8_t mavlink_msg_pressure_control_input_get_enable_torque(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field config_mode from pressure_control_input message
 *
 * @return   (BOOL) Activate config mode 
 */
static inline uint8_t mavlink_msg_pressure_control_input_get_config_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Decode a pressure_control_input message into a struct
 *
 * @param msg The message to decode
 * @param pressure_control_input C-struct to decode the message contents into
 */
static inline void mavlink_msg_pressure_control_input_decode(const mavlink_message_t* msg, mavlink_pressure_control_input_t* pressure_control_input)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    pressure_control_input->measure_pressure = mavlink_msg_pressure_control_input_get_measure_pressure(msg);
    pressure_control_input->kp = mavlink_msg_pressure_control_input_get_kp(msg);
    pressure_control_input->kd = mavlink_msg_pressure_control_input_get_kd(msg);
    pressure_control_input->ki = mavlink_msg_pressure_control_input_get_ki(msg);
    pressure_control_input->wind_up = mavlink_msg_pressure_control_input_get_wind_up(msg);
    pressure_control_input->speed = mavlink_msg_pressure_control_input_get_speed(msg);
    pressure_control_input->k_delay = mavlink_msg_pressure_control_input_get_k_delay(msg);
    pressure_control_input->height_set_point = mavlink_msg_pressure_control_input_get_height_set_point(msg);
    pressure_control_input->pressure_offset = mavlink_msg_pressure_control_input_get_pressure_offset(msg);
    pressure_control_input->equilibrium_angle = mavlink_msg_pressure_control_input_get_equilibrium_angle(msg);
    pressure_control_input->angle_multiplier = mavlink_msg_pressure_control_input_get_angle_multiplier(msg);
    pressure_control_input->change_control_sign = mavlink_msg_pressure_control_input_get_change_control_sign(msg);
    pressure_control_input->control_enabled = mavlink_msg_pressure_control_input_get_control_enabled(msg);
    pressure_control_input->enable_torque = mavlink_msg_pressure_control_input_get_enable_torque(msg);
    pressure_control_input->config_mode = mavlink_msg_pressure_control_input_get_config_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN? msg->len : MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN;
        memset(pressure_control_input, 0, MAVLINK_MSG_ID_PRESSURE_CONTROL_INPUT_LEN);
    memcpy(pressure_control_input, _MAV_PAYLOAD(msg), len);
#endif
}
