#pragma once
// MESSAGE SYSTEM_ERRORS_PX4 PACKING

#define MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4 614

MAVPACKED(
typedef struct __mavlink_system_errors_px4_t {
 uint8_t usound_com_err; /*<   Serial port not connected or unable to establish connection */
 uint8_t usound_com_timeout; /*<   Communication with serial port is lost during execution */
 uint8_t usound_wrong_val[3]; /*<   Received or calculated value is out of bounds */
 uint8_t press_com_err; /*<   Serial port not connected or unable to establish connection */
 uint8_t press_com_timeout; /*<   Communication with serial port is lost during execution */
 uint8_t press_wrong_val[3]; /*<   Received or calculated value is out of bounds */
 uint8_t nke_com_err; /*<   Serial port not connected or unable to establish connection */
 uint8_t nke_com_timeout; /*<   Communication with serial port is lost during execution */
 uint8_t nke_wrong_val[3]; /*<   Received or calculated value is out of bounds */
 uint8_t can_open_timeout; /*<   CAN on STM32 cannot be opened */
 uint8_t can_close_timeout; /*<   CAN On STM32 cannot be closed */
 uint8_t can_bus_timeout; /*<   Received >= 100 CAN Error Frames*/
 uint8_t can_error_frame; /*<   Enable torque on Maxon actuators */
 uint8_t maxon_homing_err[5]; /*<   Homming cannot be attained */
 uint8_t maxon_out_of_range[5]; /*<   Read a position out of specific range > 5 times */
 uint8_t general_err[5]; /*<   Other EPOS4 error not covered in the other messages */
}) mavlink_system_errors_px4_t;

#define MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN 34
#define MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_MIN_LEN 34
#define MAVLINK_MSG_ID_614_LEN 34
#define MAVLINK_MSG_ID_614_MIN_LEN 34

#define MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_CRC 230
#define MAVLINK_MSG_ID_614_CRC 230

#define MAVLINK_MSG_SYSTEM_ERRORS_PX4_FIELD_USOUND_WRONG_VAL_LEN 3
#define MAVLINK_MSG_SYSTEM_ERRORS_PX4_FIELD_PRESS_WRONG_VAL_LEN 3
#define MAVLINK_MSG_SYSTEM_ERRORS_PX4_FIELD_NKE_WRONG_VAL_LEN 3
#define MAVLINK_MSG_SYSTEM_ERRORS_PX4_FIELD_MAXON_HOMING_ERR_LEN 5
#define MAVLINK_MSG_SYSTEM_ERRORS_PX4_FIELD_MAXON_OUT_OF_RANGE_LEN 5
#define MAVLINK_MSG_SYSTEM_ERRORS_PX4_FIELD_GENERAL_ERR_LEN 5

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SYSTEM_ERRORS_PX4 { \
    614, \
    "SYSTEM_ERRORS_PX4", \
    16, \
    {  { "usound_com_err", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_system_errors_px4_t, usound_com_err) }, \
         { "usound_com_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_system_errors_px4_t, usound_com_timeout) }, \
         { "usound_wrong_val", NULL, MAVLINK_TYPE_UINT8_T, 3, 2, offsetof(mavlink_system_errors_px4_t, usound_wrong_val) }, \
         { "press_com_err", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_system_errors_px4_t, press_com_err) }, \
         { "press_com_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_system_errors_px4_t, press_com_timeout) }, \
         { "press_wrong_val", NULL, MAVLINK_TYPE_UINT8_T, 3, 7, offsetof(mavlink_system_errors_px4_t, press_wrong_val) }, \
         { "nke_com_err", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_system_errors_px4_t, nke_com_err) }, \
         { "nke_com_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_system_errors_px4_t, nke_com_timeout) }, \
         { "nke_wrong_val", NULL, MAVLINK_TYPE_UINT8_T, 3, 12, offsetof(mavlink_system_errors_px4_t, nke_wrong_val) }, \
         { "can_open_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_system_errors_px4_t, can_open_timeout) }, \
         { "can_close_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_system_errors_px4_t, can_close_timeout) }, \
         { "can_bus_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_system_errors_px4_t, can_bus_timeout) }, \
         { "can_error_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_system_errors_px4_t, can_error_frame) }, \
         { "maxon_homing_err", NULL, MAVLINK_TYPE_UINT8_T, 5, 19, offsetof(mavlink_system_errors_px4_t, maxon_homing_err) }, \
         { "maxon_out_of_range", NULL, MAVLINK_TYPE_UINT8_T, 5, 24, offsetof(mavlink_system_errors_px4_t, maxon_out_of_range) }, \
         { "general_err", NULL, MAVLINK_TYPE_UINT8_T, 5, 29, offsetof(mavlink_system_errors_px4_t, general_err) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SYSTEM_ERRORS_PX4 { \
    "SYSTEM_ERRORS_PX4", \
    16, \
    {  { "usound_com_err", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_system_errors_px4_t, usound_com_err) }, \
         { "usound_com_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 1, offsetof(mavlink_system_errors_px4_t, usound_com_timeout) }, \
         { "usound_wrong_val", NULL, MAVLINK_TYPE_UINT8_T, 3, 2, offsetof(mavlink_system_errors_px4_t, usound_wrong_val) }, \
         { "press_com_err", NULL, MAVLINK_TYPE_UINT8_T, 0, 5, offsetof(mavlink_system_errors_px4_t, press_com_err) }, \
         { "press_com_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 6, offsetof(mavlink_system_errors_px4_t, press_com_timeout) }, \
         { "press_wrong_val", NULL, MAVLINK_TYPE_UINT8_T, 3, 7, offsetof(mavlink_system_errors_px4_t, press_wrong_val) }, \
         { "nke_com_err", NULL, MAVLINK_TYPE_UINT8_T, 0, 10, offsetof(mavlink_system_errors_px4_t, nke_com_err) }, \
         { "nke_com_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 11, offsetof(mavlink_system_errors_px4_t, nke_com_timeout) }, \
         { "nke_wrong_val", NULL, MAVLINK_TYPE_UINT8_T, 3, 12, offsetof(mavlink_system_errors_px4_t, nke_wrong_val) }, \
         { "can_open_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_system_errors_px4_t, can_open_timeout) }, \
         { "can_close_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_system_errors_px4_t, can_close_timeout) }, \
         { "can_bus_timeout", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_system_errors_px4_t, can_bus_timeout) }, \
         { "can_error_frame", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_system_errors_px4_t, can_error_frame) }, \
         { "maxon_homing_err", NULL, MAVLINK_TYPE_UINT8_T, 5, 19, offsetof(mavlink_system_errors_px4_t, maxon_homing_err) }, \
         { "maxon_out_of_range", NULL, MAVLINK_TYPE_UINT8_T, 5, 24, offsetof(mavlink_system_errors_px4_t, maxon_out_of_range) }, \
         { "general_err", NULL, MAVLINK_TYPE_UINT8_T, 5, 29, offsetof(mavlink_system_errors_px4_t, general_err) }, \
         } \
}
#endif

/**
 * @brief Pack a system_errors_px4 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param usound_com_err   Serial port not connected or unable to establish connection 
 * @param usound_com_timeout   Communication with serial port is lost during execution 
 * @param usound_wrong_val   Received or calculated value is out of bounds 
 * @param press_com_err   Serial port not connected or unable to establish connection 
 * @param press_com_timeout   Communication with serial port is lost during execution 
 * @param press_wrong_val   Received or calculated value is out of bounds 
 * @param nke_com_err   Serial port not connected or unable to establish connection 
 * @param nke_com_timeout   Communication with serial port is lost during execution 
 * @param nke_wrong_val   Received or calculated value is out of bounds 
 * @param can_open_timeout   CAN on STM32 cannot be opened 
 * @param can_close_timeout   CAN On STM32 cannot be closed 
 * @param can_bus_timeout   Received >= 100 CAN Error Frames
 * @param can_error_frame   Enable torque on Maxon actuators 
 * @param maxon_homing_err   Homming cannot be attained 
 * @param maxon_out_of_range   Read a position out of specific range > 5 times 
 * @param general_err   Other EPOS4 error not covered in the other messages 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_errors_px4_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t usound_com_err, uint8_t usound_com_timeout, const uint8_t *usound_wrong_val, uint8_t press_com_err, uint8_t press_com_timeout, const uint8_t *press_wrong_val, uint8_t nke_com_err, uint8_t nke_com_timeout, const uint8_t *nke_wrong_val, uint8_t can_open_timeout, uint8_t can_close_timeout, uint8_t can_bus_timeout, uint8_t can_error_frame, const uint8_t *maxon_homing_err, const uint8_t *maxon_out_of_range, const uint8_t *general_err)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN];
    _mav_put_uint8_t(buf, 0, usound_com_err);
    _mav_put_uint8_t(buf, 1, usound_com_timeout);
    _mav_put_uint8_t(buf, 5, press_com_err);
    _mav_put_uint8_t(buf, 6, press_com_timeout);
    _mav_put_uint8_t(buf, 10, nke_com_err);
    _mav_put_uint8_t(buf, 11, nke_com_timeout);
    _mav_put_uint8_t(buf, 15, can_open_timeout);
    _mav_put_uint8_t(buf, 16, can_close_timeout);
    _mav_put_uint8_t(buf, 17, can_bus_timeout);
    _mav_put_uint8_t(buf, 18, can_error_frame);
    _mav_put_uint8_t_array(buf, 2, usound_wrong_val, 3);
    _mav_put_uint8_t_array(buf, 7, press_wrong_val, 3);
    _mav_put_uint8_t_array(buf, 12, nke_wrong_val, 3);
    _mav_put_uint8_t_array(buf, 19, maxon_homing_err, 5);
    _mav_put_uint8_t_array(buf, 24, maxon_out_of_range, 5);
    _mav_put_uint8_t_array(buf, 29, general_err, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN);
#else
    mavlink_system_errors_px4_t packet;
    packet.usound_com_err = usound_com_err;
    packet.usound_com_timeout = usound_com_timeout;
    packet.press_com_err = press_com_err;
    packet.press_com_timeout = press_com_timeout;
    packet.nke_com_err = nke_com_err;
    packet.nke_com_timeout = nke_com_timeout;
    packet.can_open_timeout = can_open_timeout;
    packet.can_close_timeout = can_close_timeout;
    packet.can_bus_timeout = can_bus_timeout;
    packet.can_error_frame = can_error_frame;
    mav_array_memcpy(packet.usound_wrong_val, usound_wrong_val, sizeof(uint8_t)*3);
    mav_array_memcpy(packet.press_wrong_val, press_wrong_val, sizeof(uint8_t)*3);
    mav_array_memcpy(packet.nke_wrong_val, nke_wrong_val, sizeof(uint8_t)*3);
    mav_array_memcpy(packet.maxon_homing_err, maxon_homing_err, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.maxon_out_of_range, maxon_out_of_range, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.general_err, general_err, sizeof(uint8_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_CRC);
}

/**
 * @brief Pack a system_errors_px4 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param usound_com_err   Serial port not connected or unable to establish connection 
 * @param usound_com_timeout   Communication with serial port is lost during execution 
 * @param usound_wrong_val   Received or calculated value is out of bounds 
 * @param press_com_err   Serial port not connected or unable to establish connection 
 * @param press_com_timeout   Communication with serial port is lost during execution 
 * @param press_wrong_val   Received or calculated value is out of bounds 
 * @param nke_com_err   Serial port not connected or unable to establish connection 
 * @param nke_com_timeout   Communication with serial port is lost during execution 
 * @param nke_wrong_val   Received or calculated value is out of bounds 
 * @param can_open_timeout   CAN on STM32 cannot be opened 
 * @param can_close_timeout   CAN On STM32 cannot be closed 
 * @param can_bus_timeout   Received >= 100 CAN Error Frames
 * @param can_error_frame   Enable torque on Maxon actuators 
 * @param maxon_homing_err   Homming cannot be attained 
 * @param maxon_out_of_range   Read a position out of specific range > 5 times 
 * @param general_err   Other EPOS4 error not covered in the other messages 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_system_errors_px4_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t usound_com_err,uint8_t usound_com_timeout,const uint8_t *usound_wrong_val,uint8_t press_com_err,uint8_t press_com_timeout,const uint8_t *press_wrong_val,uint8_t nke_com_err,uint8_t nke_com_timeout,const uint8_t *nke_wrong_val,uint8_t can_open_timeout,uint8_t can_close_timeout,uint8_t can_bus_timeout,uint8_t can_error_frame,const uint8_t *maxon_homing_err,const uint8_t *maxon_out_of_range,const uint8_t *general_err)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN];
    _mav_put_uint8_t(buf, 0, usound_com_err);
    _mav_put_uint8_t(buf, 1, usound_com_timeout);
    _mav_put_uint8_t(buf, 5, press_com_err);
    _mav_put_uint8_t(buf, 6, press_com_timeout);
    _mav_put_uint8_t(buf, 10, nke_com_err);
    _mav_put_uint8_t(buf, 11, nke_com_timeout);
    _mav_put_uint8_t(buf, 15, can_open_timeout);
    _mav_put_uint8_t(buf, 16, can_close_timeout);
    _mav_put_uint8_t(buf, 17, can_bus_timeout);
    _mav_put_uint8_t(buf, 18, can_error_frame);
    _mav_put_uint8_t_array(buf, 2, usound_wrong_val, 3);
    _mav_put_uint8_t_array(buf, 7, press_wrong_val, 3);
    _mav_put_uint8_t_array(buf, 12, nke_wrong_val, 3);
    _mav_put_uint8_t_array(buf, 19, maxon_homing_err, 5);
    _mav_put_uint8_t_array(buf, 24, maxon_out_of_range, 5);
    _mav_put_uint8_t_array(buf, 29, general_err, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN);
#else
    mavlink_system_errors_px4_t packet;
    packet.usound_com_err = usound_com_err;
    packet.usound_com_timeout = usound_com_timeout;
    packet.press_com_err = press_com_err;
    packet.press_com_timeout = press_com_timeout;
    packet.nke_com_err = nke_com_err;
    packet.nke_com_timeout = nke_com_timeout;
    packet.can_open_timeout = can_open_timeout;
    packet.can_close_timeout = can_close_timeout;
    packet.can_bus_timeout = can_bus_timeout;
    packet.can_error_frame = can_error_frame;
    mav_array_memcpy(packet.usound_wrong_val, usound_wrong_val, sizeof(uint8_t)*3);
    mav_array_memcpy(packet.press_wrong_val, press_wrong_val, sizeof(uint8_t)*3);
    mav_array_memcpy(packet.nke_wrong_val, nke_wrong_val, sizeof(uint8_t)*3);
    mav_array_memcpy(packet.maxon_homing_err, maxon_homing_err, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.maxon_out_of_range, maxon_out_of_range, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.general_err, general_err, sizeof(uint8_t)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_CRC);
}

/**
 * @brief Encode a system_errors_px4 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param system_errors_px4 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_system_errors_px4_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_system_errors_px4_t* system_errors_px4)
{
    return mavlink_msg_system_errors_px4_pack(system_id, component_id, msg, system_errors_px4->usound_com_err, system_errors_px4->usound_com_timeout, system_errors_px4->usound_wrong_val, system_errors_px4->press_com_err, system_errors_px4->press_com_timeout, system_errors_px4->press_wrong_val, system_errors_px4->nke_com_err, system_errors_px4->nke_com_timeout, system_errors_px4->nke_wrong_val, system_errors_px4->can_open_timeout, system_errors_px4->can_close_timeout, system_errors_px4->can_bus_timeout, system_errors_px4->can_error_frame, system_errors_px4->maxon_homing_err, system_errors_px4->maxon_out_of_range, system_errors_px4->general_err);
}

/**
 * @brief Encode a system_errors_px4 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param system_errors_px4 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_system_errors_px4_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_system_errors_px4_t* system_errors_px4)
{
    return mavlink_msg_system_errors_px4_pack_chan(system_id, component_id, chan, msg, system_errors_px4->usound_com_err, system_errors_px4->usound_com_timeout, system_errors_px4->usound_wrong_val, system_errors_px4->press_com_err, system_errors_px4->press_com_timeout, system_errors_px4->press_wrong_val, system_errors_px4->nke_com_err, system_errors_px4->nke_com_timeout, system_errors_px4->nke_wrong_val, system_errors_px4->can_open_timeout, system_errors_px4->can_close_timeout, system_errors_px4->can_bus_timeout, system_errors_px4->can_error_frame, system_errors_px4->maxon_homing_err, system_errors_px4->maxon_out_of_range, system_errors_px4->general_err);
}

/**
 * @brief Send a system_errors_px4 message
 * @param chan MAVLink channel to send the message
 *
 * @param usound_com_err   Serial port not connected or unable to establish connection 
 * @param usound_com_timeout   Communication with serial port is lost during execution 
 * @param usound_wrong_val   Received or calculated value is out of bounds 
 * @param press_com_err   Serial port not connected or unable to establish connection 
 * @param press_com_timeout   Communication with serial port is lost during execution 
 * @param press_wrong_val   Received or calculated value is out of bounds 
 * @param nke_com_err   Serial port not connected or unable to establish connection 
 * @param nke_com_timeout   Communication with serial port is lost during execution 
 * @param nke_wrong_val   Received or calculated value is out of bounds 
 * @param can_open_timeout   CAN on STM32 cannot be opened 
 * @param can_close_timeout   CAN On STM32 cannot be closed 
 * @param can_bus_timeout   Received >= 100 CAN Error Frames
 * @param can_error_frame   Enable torque on Maxon actuators 
 * @param maxon_homing_err   Homming cannot be attained 
 * @param maxon_out_of_range   Read a position out of specific range > 5 times 
 * @param general_err   Other EPOS4 error not covered in the other messages 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_system_errors_px4_send(mavlink_channel_t chan, uint8_t usound_com_err, uint8_t usound_com_timeout, const uint8_t *usound_wrong_val, uint8_t press_com_err, uint8_t press_com_timeout, const uint8_t *press_wrong_val, uint8_t nke_com_err, uint8_t nke_com_timeout, const uint8_t *nke_wrong_val, uint8_t can_open_timeout, uint8_t can_close_timeout, uint8_t can_bus_timeout, uint8_t can_error_frame, const uint8_t *maxon_homing_err, const uint8_t *maxon_out_of_range, const uint8_t *general_err)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN];
    _mav_put_uint8_t(buf, 0, usound_com_err);
    _mav_put_uint8_t(buf, 1, usound_com_timeout);
    _mav_put_uint8_t(buf, 5, press_com_err);
    _mav_put_uint8_t(buf, 6, press_com_timeout);
    _mav_put_uint8_t(buf, 10, nke_com_err);
    _mav_put_uint8_t(buf, 11, nke_com_timeout);
    _mav_put_uint8_t(buf, 15, can_open_timeout);
    _mav_put_uint8_t(buf, 16, can_close_timeout);
    _mav_put_uint8_t(buf, 17, can_bus_timeout);
    _mav_put_uint8_t(buf, 18, can_error_frame);
    _mav_put_uint8_t_array(buf, 2, usound_wrong_val, 3);
    _mav_put_uint8_t_array(buf, 7, press_wrong_val, 3);
    _mav_put_uint8_t_array(buf, 12, nke_wrong_val, 3);
    _mav_put_uint8_t_array(buf, 19, maxon_homing_err, 5);
    _mav_put_uint8_t_array(buf, 24, maxon_out_of_range, 5);
    _mav_put_uint8_t_array(buf, 29, general_err, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4, buf, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_CRC);
#else
    mavlink_system_errors_px4_t packet;
    packet.usound_com_err = usound_com_err;
    packet.usound_com_timeout = usound_com_timeout;
    packet.press_com_err = press_com_err;
    packet.press_com_timeout = press_com_timeout;
    packet.nke_com_err = nke_com_err;
    packet.nke_com_timeout = nke_com_timeout;
    packet.can_open_timeout = can_open_timeout;
    packet.can_close_timeout = can_close_timeout;
    packet.can_bus_timeout = can_bus_timeout;
    packet.can_error_frame = can_error_frame;
    mav_array_memcpy(packet.usound_wrong_val, usound_wrong_val, sizeof(uint8_t)*3);
    mav_array_memcpy(packet.press_wrong_val, press_wrong_val, sizeof(uint8_t)*3);
    mav_array_memcpy(packet.nke_wrong_val, nke_wrong_val, sizeof(uint8_t)*3);
    mav_array_memcpy(packet.maxon_homing_err, maxon_homing_err, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.maxon_out_of_range, maxon_out_of_range, sizeof(uint8_t)*5);
    mav_array_memcpy(packet.general_err, general_err, sizeof(uint8_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4, (const char *)&packet, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_CRC);
#endif
}

/**
 * @brief Send a system_errors_px4 message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_system_errors_px4_send_struct(mavlink_channel_t chan, const mavlink_system_errors_px4_t* system_errors_px4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_system_errors_px4_send(chan, system_errors_px4->usound_com_err, system_errors_px4->usound_com_timeout, system_errors_px4->usound_wrong_val, system_errors_px4->press_com_err, system_errors_px4->press_com_timeout, system_errors_px4->press_wrong_val, system_errors_px4->nke_com_err, system_errors_px4->nke_com_timeout, system_errors_px4->nke_wrong_val, system_errors_px4->can_open_timeout, system_errors_px4->can_close_timeout, system_errors_px4->can_bus_timeout, system_errors_px4->can_error_frame, system_errors_px4->maxon_homing_err, system_errors_px4->maxon_out_of_range, system_errors_px4->general_err);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4, (const char *)system_errors_px4, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_CRC);
#endif
}

#if MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_system_errors_px4_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t usound_com_err, uint8_t usound_com_timeout, const uint8_t *usound_wrong_val, uint8_t press_com_err, uint8_t press_com_timeout, const uint8_t *press_wrong_val, uint8_t nke_com_err, uint8_t nke_com_timeout, const uint8_t *nke_wrong_val, uint8_t can_open_timeout, uint8_t can_close_timeout, uint8_t can_bus_timeout, uint8_t can_error_frame, const uint8_t *maxon_homing_err, const uint8_t *maxon_out_of_range, const uint8_t *general_err)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, usound_com_err);
    _mav_put_uint8_t(buf, 1, usound_com_timeout);
    _mav_put_uint8_t(buf, 5, press_com_err);
    _mav_put_uint8_t(buf, 6, press_com_timeout);
    _mav_put_uint8_t(buf, 10, nke_com_err);
    _mav_put_uint8_t(buf, 11, nke_com_timeout);
    _mav_put_uint8_t(buf, 15, can_open_timeout);
    _mav_put_uint8_t(buf, 16, can_close_timeout);
    _mav_put_uint8_t(buf, 17, can_bus_timeout);
    _mav_put_uint8_t(buf, 18, can_error_frame);
    _mav_put_uint8_t_array(buf, 2, usound_wrong_val, 3);
    _mav_put_uint8_t_array(buf, 7, press_wrong_val, 3);
    _mav_put_uint8_t_array(buf, 12, nke_wrong_val, 3);
    _mav_put_uint8_t_array(buf, 19, maxon_homing_err, 5);
    _mav_put_uint8_t_array(buf, 24, maxon_out_of_range, 5);
    _mav_put_uint8_t_array(buf, 29, general_err, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4, buf, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_CRC);
#else
    mavlink_system_errors_px4_t *packet = (mavlink_system_errors_px4_t *)msgbuf;
    packet->usound_com_err = usound_com_err;
    packet->usound_com_timeout = usound_com_timeout;
    packet->press_com_err = press_com_err;
    packet->press_com_timeout = press_com_timeout;
    packet->nke_com_err = nke_com_err;
    packet->nke_com_timeout = nke_com_timeout;
    packet->can_open_timeout = can_open_timeout;
    packet->can_close_timeout = can_close_timeout;
    packet->can_bus_timeout = can_bus_timeout;
    packet->can_error_frame = can_error_frame;
    mav_array_memcpy(packet->usound_wrong_val, usound_wrong_val, sizeof(uint8_t)*3);
    mav_array_memcpy(packet->press_wrong_val, press_wrong_val, sizeof(uint8_t)*3);
    mav_array_memcpy(packet->nke_wrong_val, nke_wrong_val, sizeof(uint8_t)*3);
    mav_array_memcpy(packet->maxon_homing_err, maxon_homing_err, sizeof(uint8_t)*5);
    mav_array_memcpy(packet->maxon_out_of_range, maxon_out_of_range, sizeof(uint8_t)*5);
    mav_array_memcpy(packet->general_err, general_err, sizeof(uint8_t)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4, (const char *)packet, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_MIN_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_CRC);
#endif
}
#endif

#endif

// MESSAGE SYSTEM_ERRORS_PX4 UNPACKING


/**
 * @brief Get field usound_com_err from system_errors_px4 message
 *
 * @return   Serial port not connected or unable to establish connection 
 */
static inline uint8_t mavlink_msg_system_errors_px4_get_usound_com_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Get field usound_com_timeout from system_errors_px4 message
 *
 * @return   Communication with serial port is lost during execution 
 */
static inline uint8_t mavlink_msg_system_errors_px4_get_usound_com_timeout(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  1);
}

/**
 * @brief Get field usound_wrong_val from system_errors_px4 message
 *
 * @return   Received or calculated value is out of bounds 
 */
static inline uint16_t mavlink_msg_system_errors_px4_get_usound_wrong_val(const mavlink_message_t* msg, uint8_t *usound_wrong_val)
{
    return _MAV_RETURN_uint8_t_array(msg, usound_wrong_val, 3,  2);
}

/**
 * @brief Get field press_com_err from system_errors_px4 message
 *
 * @return   Serial port not connected or unable to establish connection 
 */
static inline uint8_t mavlink_msg_system_errors_px4_get_press_com_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  5);
}

/**
 * @brief Get field press_com_timeout from system_errors_px4 message
 *
 * @return   Communication with serial port is lost during execution 
 */
static inline uint8_t mavlink_msg_system_errors_px4_get_press_com_timeout(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  6);
}

/**
 * @brief Get field press_wrong_val from system_errors_px4 message
 *
 * @return   Received or calculated value is out of bounds 
 */
static inline uint16_t mavlink_msg_system_errors_px4_get_press_wrong_val(const mavlink_message_t* msg, uint8_t *press_wrong_val)
{
    return _MAV_RETURN_uint8_t_array(msg, press_wrong_val, 3,  7);
}

/**
 * @brief Get field nke_com_err from system_errors_px4 message
 *
 * @return   Serial port not connected or unable to establish connection 
 */
static inline uint8_t mavlink_msg_system_errors_px4_get_nke_com_err(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  10);
}

/**
 * @brief Get field nke_com_timeout from system_errors_px4 message
 *
 * @return   Communication with serial port is lost during execution 
 */
static inline uint8_t mavlink_msg_system_errors_px4_get_nke_com_timeout(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  11);
}

/**
 * @brief Get field nke_wrong_val from system_errors_px4 message
 *
 * @return   Received or calculated value is out of bounds 
 */
static inline uint16_t mavlink_msg_system_errors_px4_get_nke_wrong_val(const mavlink_message_t* msg, uint8_t *nke_wrong_val)
{
    return _MAV_RETURN_uint8_t_array(msg, nke_wrong_val, 3,  12);
}

/**
 * @brief Get field can_open_timeout from system_errors_px4 message
 *
 * @return   CAN on STM32 cannot be opened 
 */
static inline uint8_t mavlink_msg_system_errors_px4_get_can_open_timeout(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field can_close_timeout from system_errors_px4 message
 *
 * @return   CAN On STM32 cannot be closed 
 */
static inline uint8_t mavlink_msg_system_errors_px4_get_can_close_timeout(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field can_bus_timeout from system_errors_px4 message
 *
 * @return   Received >= 100 CAN Error Frames
 */
static inline uint8_t mavlink_msg_system_errors_px4_get_can_bus_timeout(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field can_error_frame from system_errors_px4 message
 *
 * @return   Enable torque on Maxon actuators 
 */
static inline uint8_t mavlink_msg_system_errors_px4_get_can_error_frame(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field maxon_homing_err from system_errors_px4 message
 *
 * @return   Homming cannot be attained 
 */
static inline uint16_t mavlink_msg_system_errors_px4_get_maxon_homing_err(const mavlink_message_t* msg, uint8_t *maxon_homing_err)
{
    return _MAV_RETURN_uint8_t_array(msg, maxon_homing_err, 5,  19);
}

/**
 * @brief Get field maxon_out_of_range from system_errors_px4 message
 *
 * @return   Read a position out of specific range > 5 times 
 */
static inline uint16_t mavlink_msg_system_errors_px4_get_maxon_out_of_range(const mavlink_message_t* msg, uint8_t *maxon_out_of_range)
{
    return _MAV_RETURN_uint8_t_array(msg, maxon_out_of_range, 5,  24);
}

/**
 * @brief Get field general_err from system_errors_px4 message
 *
 * @return   Other EPOS4 error not covered in the other messages 
 */
static inline uint16_t mavlink_msg_system_errors_px4_get_general_err(const mavlink_message_t* msg, uint8_t *general_err)
{
    return _MAV_RETURN_uint8_t_array(msg, general_err, 5,  29);
}

/**
 * @brief Decode a system_errors_px4 message into a struct
 *
 * @param msg The message to decode
 * @param system_errors_px4 C-struct to decode the message contents into
 */
static inline void mavlink_msg_system_errors_px4_decode(const mavlink_message_t* msg, mavlink_system_errors_px4_t* system_errors_px4)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    system_errors_px4->usound_com_err = mavlink_msg_system_errors_px4_get_usound_com_err(msg);
    system_errors_px4->usound_com_timeout = mavlink_msg_system_errors_px4_get_usound_com_timeout(msg);
    mavlink_msg_system_errors_px4_get_usound_wrong_val(msg, system_errors_px4->usound_wrong_val);
    system_errors_px4->press_com_err = mavlink_msg_system_errors_px4_get_press_com_err(msg);
    system_errors_px4->press_com_timeout = mavlink_msg_system_errors_px4_get_press_com_timeout(msg);
    mavlink_msg_system_errors_px4_get_press_wrong_val(msg, system_errors_px4->press_wrong_val);
    system_errors_px4->nke_com_err = mavlink_msg_system_errors_px4_get_nke_com_err(msg);
    system_errors_px4->nke_com_timeout = mavlink_msg_system_errors_px4_get_nke_com_timeout(msg);
    mavlink_msg_system_errors_px4_get_nke_wrong_val(msg, system_errors_px4->nke_wrong_val);
    system_errors_px4->can_open_timeout = mavlink_msg_system_errors_px4_get_can_open_timeout(msg);
    system_errors_px4->can_close_timeout = mavlink_msg_system_errors_px4_get_can_close_timeout(msg);
    system_errors_px4->can_bus_timeout = mavlink_msg_system_errors_px4_get_can_bus_timeout(msg);
    system_errors_px4->can_error_frame = mavlink_msg_system_errors_px4_get_can_error_frame(msg);
    mavlink_msg_system_errors_px4_get_maxon_homing_err(msg, system_errors_px4->maxon_homing_err);
    mavlink_msg_system_errors_px4_get_maxon_out_of_range(msg, system_errors_px4->maxon_out_of_range);
    mavlink_msg_system_errors_px4_get_general_err(msg, system_errors_px4->general_err);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN? msg->len : MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN;
        memset(system_errors_px4, 0, MAVLINK_MSG_ID_SYSTEM_ERRORS_PX4_LEN);
    memcpy(system_errors_px4, _MAV_PAYLOAD(msg), len);
#endif
}
