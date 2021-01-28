#pragma once
// MESSAGE INPUT_WRAPPER_DEFINITION PACKING

#define MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION 600


typedef struct __mavlink_input_wrapper_definition_t {
 double setpoint; /*< [m]  Desired depth (ho) */
 double ultrasound_measurement; /*< [m]  Measurement obtained by ultrasound sensor */
 double usoundMeasDataRef1; /*< [m]  Reference value when there are no waves */
 double usoundMeasDataThresh; /*< [m]  Filtering ripple */
 double kp; /*<   Proportional constant */
 double ki; /*<   Integral constant */
 double kd; /*<   Derivative constant */
 double rudderlength; /*< [m]  Length of overall rudder */
} mavlink_input_wrapper_definition_t;

#define MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN 64
#define MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_MIN_LEN 64
#define MAVLINK_MSG_ID_600_LEN 64
#define MAVLINK_MSG_ID_600_MIN_LEN 64

#define MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_CRC 170
#define MAVLINK_MSG_ID_600_CRC 170



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_INPUT_WRAPPER_DEFINITION { \
    600, \
    "INPUT_WRAPPER_DEFINITION", \
    8, \
    {  { "setpoint", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_input_wrapper_definition_t, setpoint) }, \
         { "ultrasound_measurement", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_input_wrapper_definition_t, ultrasound_measurement) }, \
         { "usoundMeasDataRef1", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_input_wrapper_definition_t, usoundMeasDataRef1) }, \
         { "usoundMeasDataThresh", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_input_wrapper_definition_t, usoundMeasDataThresh) }, \
         { "kp", NULL, MAVLINK_TYPE_DOUBLE, 0, 32, offsetof(mavlink_input_wrapper_definition_t, kp) }, \
         { "ki", NULL, MAVLINK_TYPE_DOUBLE, 0, 40, offsetof(mavlink_input_wrapper_definition_t, ki) }, \
         { "kd", NULL, MAVLINK_TYPE_DOUBLE, 0, 48, offsetof(mavlink_input_wrapper_definition_t, kd) }, \
         { "rudderlength", NULL, MAVLINK_TYPE_DOUBLE, 0, 56, offsetof(mavlink_input_wrapper_definition_t, rudderlength) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_INPUT_WRAPPER_DEFINITION { \
    "INPUT_WRAPPER_DEFINITION", \
    8, \
    {  { "setpoint", NULL, MAVLINK_TYPE_DOUBLE, 0, 0, offsetof(mavlink_input_wrapper_definition_t, setpoint) }, \
         { "ultrasound_measurement", NULL, MAVLINK_TYPE_DOUBLE, 0, 8, offsetof(mavlink_input_wrapper_definition_t, ultrasound_measurement) }, \
         { "usoundMeasDataRef1", NULL, MAVLINK_TYPE_DOUBLE, 0, 16, offsetof(mavlink_input_wrapper_definition_t, usoundMeasDataRef1) }, \
         { "usoundMeasDataThresh", NULL, MAVLINK_TYPE_DOUBLE, 0, 24, offsetof(mavlink_input_wrapper_definition_t, usoundMeasDataThresh) }, \
         { "kp", NULL, MAVLINK_TYPE_DOUBLE, 0, 32, offsetof(mavlink_input_wrapper_definition_t, kp) }, \
         { "ki", NULL, MAVLINK_TYPE_DOUBLE, 0, 40, offsetof(mavlink_input_wrapper_definition_t, ki) }, \
         { "kd", NULL, MAVLINK_TYPE_DOUBLE, 0, 48, offsetof(mavlink_input_wrapper_definition_t, kd) }, \
         { "rudderlength", NULL, MAVLINK_TYPE_DOUBLE, 0, 56, offsetof(mavlink_input_wrapper_definition_t, rudderlength) }, \
         } \
}
#endif

/**
 * @brief Pack a input_wrapper_definition message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param setpoint [m]  Desired depth (ho) 
 * @param ultrasound_measurement [m]  Measurement obtained by ultrasound sensor 
 * @param usoundMeasDataRef1 [m]  Reference value when there are no waves 
 * @param usoundMeasDataThresh [m]  Filtering ripple 
 * @param kp   Proportional constant 
 * @param ki   Integral constant 
 * @param kd   Derivative constant 
 * @param rudderlength [m]  Length of overall rudder 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_input_wrapper_definition_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               double setpoint, double ultrasound_measurement, double usoundMeasDataRef1, double usoundMeasDataThresh, double kp, double ki, double kd, double rudderlength)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN];
    _mav_put_double(buf, 0, setpoint);
    _mav_put_double(buf, 8, ultrasound_measurement);
    _mav_put_double(buf, 16, usoundMeasDataRef1);
    _mav_put_double(buf, 24, usoundMeasDataThresh);
    _mav_put_double(buf, 32, kp);
    _mav_put_double(buf, 40, ki);
    _mav_put_double(buf, 48, kd);
    _mav_put_double(buf, 56, rudderlength);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN);
#else
    mavlink_input_wrapper_definition_t packet;
    packet.setpoint = setpoint;
    packet.ultrasound_measurement = ultrasound_measurement;
    packet.usoundMeasDataRef1 = usoundMeasDataRef1;
    packet.usoundMeasDataThresh = usoundMeasDataThresh;
    packet.kp = kp;
    packet.ki = ki;
    packet.kd = kd;
    packet.rudderlength = rudderlength;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_CRC);
}

/**
 * @brief Pack a input_wrapper_definition message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param setpoint [m]  Desired depth (ho) 
 * @param ultrasound_measurement [m]  Measurement obtained by ultrasound sensor 
 * @param usoundMeasDataRef1 [m]  Reference value when there are no waves 
 * @param usoundMeasDataThresh [m]  Filtering ripple 
 * @param kp   Proportional constant 
 * @param ki   Integral constant 
 * @param kd   Derivative constant 
 * @param rudderlength [m]  Length of overall rudder 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_input_wrapper_definition_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   double setpoint,double ultrasound_measurement,double usoundMeasDataRef1,double usoundMeasDataThresh,double kp,double ki,double kd,double rudderlength)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN];
    _mav_put_double(buf, 0, setpoint);
    _mav_put_double(buf, 8, ultrasound_measurement);
    _mav_put_double(buf, 16, usoundMeasDataRef1);
    _mav_put_double(buf, 24, usoundMeasDataThresh);
    _mav_put_double(buf, 32, kp);
    _mav_put_double(buf, 40, ki);
    _mav_put_double(buf, 48, kd);
    _mav_put_double(buf, 56, rudderlength);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN);
#else
    mavlink_input_wrapper_definition_t packet;
    packet.setpoint = setpoint;
    packet.ultrasound_measurement = ultrasound_measurement;
    packet.usoundMeasDataRef1 = usoundMeasDataRef1;
    packet.usoundMeasDataThresh = usoundMeasDataThresh;
    packet.kp = kp;
    packet.ki = ki;
    packet.kd = kd;
    packet.rudderlength = rudderlength;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_CRC);
}

/**
 * @brief Encode a input_wrapper_definition struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param input_wrapper_definition C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_input_wrapper_definition_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_input_wrapper_definition_t* input_wrapper_definition)
{
    return mavlink_msg_input_wrapper_definition_pack(system_id, component_id, msg, input_wrapper_definition->setpoint, input_wrapper_definition->ultrasound_measurement, input_wrapper_definition->usoundMeasDataRef1, input_wrapper_definition->usoundMeasDataThresh, input_wrapper_definition->kp, input_wrapper_definition->ki, input_wrapper_definition->kd, input_wrapper_definition->rudderlength);
}

/**
 * @brief Encode a input_wrapper_definition struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param input_wrapper_definition C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_input_wrapper_definition_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_input_wrapper_definition_t* input_wrapper_definition)
{
    return mavlink_msg_input_wrapper_definition_pack_chan(system_id, component_id, chan, msg, input_wrapper_definition->setpoint, input_wrapper_definition->ultrasound_measurement, input_wrapper_definition->usoundMeasDataRef1, input_wrapper_definition->usoundMeasDataThresh, input_wrapper_definition->kp, input_wrapper_definition->ki, input_wrapper_definition->kd, input_wrapper_definition->rudderlength);
}

/**
 * @brief Send a input_wrapper_definition message
 * @param chan MAVLink channel to send the message
 *
 * @param setpoint [m]  Desired depth (ho) 
 * @param ultrasound_measurement [m]  Measurement obtained by ultrasound sensor 
 * @param usoundMeasDataRef1 [m]  Reference value when there are no waves 
 * @param usoundMeasDataThresh [m]  Filtering ripple 
 * @param kp   Proportional constant 
 * @param ki   Integral constant 
 * @param kd   Derivative constant 
 * @param rudderlength [m]  Length of overall rudder 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_input_wrapper_definition_send(mavlink_channel_t chan, double setpoint, double ultrasound_measurement, double usoundMeasDataRef1, double usoundMeasDataThresh, double kp, double ki, double kd, double rudderlength)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN];
    _mav_put_double(buf, 0, setpoint);
    _mav_put_double(buf, 8, ultrasound_measurement);
    _mav_put_double(buf, 16, usoundMeasDataRef1);
    _mav_put_double(buf, 24, usoundMeasDataThresh);
    _mav_put_double(buf, 32, kp);
    _mav_put_double(buf, 40, ki);
    _mav_put_double(buf, 48, kd);
    _mav_put_double(buf, 56, rudderlength);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION, buf, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_CRC);
#else
    mavlink_input_wrapper_definition_t packet;
    packet.setpoint = setpoint;
    packet.ultrasound_measurement = ultrasound_measurement;
    packet.usoundMeasDataRef1 = usoundMeasDataRef1;
    packet.usoundMeasDataThresh = usoundMeasDataThresh;
    packet.kp = kp;
    packet.ki = ki;
    packet.kd = kd;
    packet.rudderlength = rudderlength;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION, (const char *)&packet, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_CRC);
#endif
}

/**
 * @brief Send a input_wrapper_definition message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_input_wrapper_definition_send_struct(mavlink_channel_t chan, const mavlink_input_wrapper_definition_t* input_wrapper_definition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_input_wrapper_definition_send(chan, input_wrapper_definition->setpoint, input_wrapper_definition->ultrasound_measurement, input_wrapper_definition->usoundMeasDataRef1, input_wrapper_definition->usoundMeasDataThresh, input_wrapper_definition->kp, input_wrapper_definition->ki, input_wrapper_definition->kd, input_wrapper_definition->rudderlength);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION, (const char *)input_wrapper_definition, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_input_wrapper_definition_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  double setpoint, double ultrasound_measurement, double usoundMeasDataRef1, double usoundMeasDataThresh, double kp, double ki, double kd, double rudderlength)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_double(buf, 0, setpoint);
    _mav_put_double(buf, 8, ultrasound_measurement);
    _mav_put_double(buf, 16, usoundMeasDataRef1);
    _mav_put_double(buf, 24, usoundMeasDataThresh);
    _mav_put_double(buf, 32, kp);
    _mav_put_double(buf, 40, ki);
    _mav_put_double(buf, 48, kd);
    _mav_put_double(buf, 56, rudderlength);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION, buf, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_CRC);
#else
    mavlink_input_wrapper_definition_t *packet = (mavlink_input_wrapper_definition_t *)msgbuf;
    packet->setpoint = setpoint;
    packet->ultrasound_measurement = ultrasound_measurement;
    packet->usoundMeasDataRef1 = usoundMeasDataRef1;
    packet->usoundMeasDataThresh = usoundMeasDataThresh;
    packet->kp = kp;
    packet->ki = ki;
    packet->kd = kd;
    packet->rudderlength = rudderlength;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION, (const char *)packet, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_MIN_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_CRC);
#endif
}
#endif

#endif

// MESSAGE INPUT_WRAPPER_DEFINITION UNPACKING


/**
 * @brief Get field setpoint from input_wrapper_definition message
 *
 * @return [m]  Desired depth (ho) 
 */
static inline double mavlink_msg_input_wrapper_definition_get_setpoint(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  0);
}

/**
 * @brief Get field ultrasound_measurement from input_wrapper_definition message
 *
 * @return [m]  Measurement obtained by ultrasound sensor 
 */
static inline double mavlink_msg_input_wrapper_definition_get_ultrasound_measurement(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  8);
}

/**
 * @brief Get field usoundMeasDataRef1 from input_wrapper_definition message
 *
 * @return [m]  Reference value when there are no waves 
 */
static inline double mavlink_msg_input_wrapper_definition_get_usoundMeasDataRef1(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  16);
}

/**
 * @brief Get field usoundMeasDataThresh from input_wrapper_definition message
 *
 * @return [m]  Filtering ripple 
 */
static inline double mavlink_msg_input_wrapper_definition_get_usoundMeasDataThresh(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  24);
}

/**
 * @brief Get field kp from input_wrapper_definition message
 *
 * @return   Proportional constant 
 */
static inline double mavlink_msg_input_wrapper_definition_get_kp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  32);
}

/**
 * @brief Get field ki from input_wrapper_definition message
 *
 * @return   Integral constant 
 */
static inline double mavlink_msg_input_wrapper_definition_get_ki(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  40);
}

/**
 * @brief Get field kd from input_wrapper_definition message
 *
 * @return   Derivative constant 
 */
static inline double mavlink_msg_input_wrapper_definition_get_kd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  48);
}

/**
 * @brief Get field rudderlength from input_wrapper_definition message
 *
 * @return [m]  Length of overall rudder 
 */
static inline double mavlink_msg_input_wrapper_definition_get_rudderlength(const mavlink_message_t* msg)
{
    return _MAV_RETURN_double(msg,  56);
}

/**
 * @brief Decode a input_wrapper_definition message into a struct
 *
 * @param msg The message to decode
 * @param input_wrapper_definition C-struct to decode the message contents into
 */
static inline void mavlink_msg_input_wrapper_definition_decode(const mavlink_message_t* msg, mavlink_input_wrapper_definition_t* input_wrapper_definition)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    input_wrapper_definition->setpoint = mavlink_msg_input_wrapper_definition_get_setpoint(msg);
    input_wrapper_definition->ultrasound_measurement = mavlink_msg_input_wrapper_definition_get_ultrasound_measurement(msg);
    input_wrapper_definition->usoundMeasDataRef1 = mavlink_msg_input_wrapper_definition_get_usoundMeasDataRef1(msg);
    input_wrapper_definition->usoundMeasDataThresh = mavlink_msg_input_wrapper_definition_get_usoundMeasDataThresh(msg);
    input_wrapper_definition->kp = mavlink_msg_input_wrapper_definition_get_kp(msg);
    input_wrapper_definition->ki = mavlink_msg_input_wrapper_definition_get_ki(msg);
    input_wrapper_definition->kd = mavlink_msg_input_wrapper_definition_get_kd(msg);
    input_wrapper_definition->rudderlength = mavlink_msg_input_wrapper_definition_get_rudderlength(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN? msg->len : MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN;
        memset(input_wrapper_definition, 0, MAVLINK_MSG_ID_INPUT_WRAPPER_DEFINITION_LEN);
    memcpy(input_wrapper_definition, _MAV_PAYLOAD(msg), len);
#endif
}
