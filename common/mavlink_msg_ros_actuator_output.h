#pragma once
// MESSAGE ROS_ACTUATOR_OUTPUT PACKING

#define MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT 609


typedef struct __mavlink_ros_actuator_output_t {
 float angle_set_maxon[5]; /*< [deg]  aoa output to be set to Maxon */
 uint8_t enable_torque; /*<   enable torque on Maxon actuators */
 uint8_t cyclic_sync_mode; /*<   enable Cyclic Sync Position mode */
} mavlink_ros_actuator_output_t;

#define MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN 22
#define MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_MIN_LEN 22
#define MAVLINK_MSG_ID_609_LEN 22
#define MAVLINK_MSG_ID_609_MIN_LEN 22

#define MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_CRC 115
#define MAVLINK_MSG_ID_609_CRC 115

#define MAVLINK_MSG_ROS_ACTUATOR_OUTPUT_FIELD_ANGLE_SET_MAXON_LEN 5

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_ROS_ACTUATOR_OUTPUT { \
    609, \
    "ROS_ACTUATOR_OUTPUT", \
    3, \
    {  { "angle_set_maxon", NULL, MAVLINK_TYPE_FLOAT, 5, 0, offsetof(mavlink_ros_actuator_output_t, angle_set_maxon) }, \
         { "enable_torque", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_ros_actuator_output_t, enable_torque) }, \
         { "cyclic_sync_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_ros_actuator_output_t, cyclic_sync_mode) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_ROS_ACTUATOR_OUTPUT { \
    "ROS_ACTUATOR_OUTPUT", \
    3, \
    {  { "angle_set_maxon", NULL, MAVLINK_TYPE_FLOAT, 5, 0, offsetof(mavlink_ros_actuator_output_t, angle_set_maxon) }, \
         { "enable_torque", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_ros_actuator_output_t, enable_torque) }, \
         { "cyclic_sync_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_ros_actuator_output_t, cyclic_sync_mode) }, \
         } \
}
#endif

/**
 * @brief Pack a ros_actuator_output message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param angle_set_maxon [deg]  aoa output to be set to Maxon 
 * @param enable_torque   enable torque on Maxon actuators 
 * @param cyclic_sync_mode   enable Cyclic Sync Position mode 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ros_actuator_output_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               const float *angle_set_maxon, uint8_t enable_torque, uint8_t cyclic_sync_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN];
    _mav_put_uint8_t(buf, 20, enable_torque);
    _mav_put_uint8_t(buf, 21, cyclic_sync_mode);
    _mav_put_float_array(buf, 0, angle_set_maxon, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN);
#else
    mavlink_ros_actuator_output_t packet;
    packet.enable_torque = enable_torque;
    packet.cyclic_sync_mode = cyclic_sync_mode;
    mav_array_memcpy(packet.angle_set_maxon, angle_set_maxon, sizeof(float)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_CRC);
}

/**
 * @brief Pack a ros_actuator_output message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param angle_set_maxon [deg]  aoa output to be set to Maxon 
 * @param enable_torque   enable torque on Maxon actuators 
 * @param cyclic_sync_mode   enable Cyclic Sync Position mode 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ros_actuator_output_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   const float *angle_set_maxon,uint8_t enable_torque,uint8_t cyclic_sync_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN];
    _mav_put_uint8_t(buf, 20, enable_torque);
    _mav_put_uint8_t(buf, 21, cyclic_sync_mode);
    _mav_put_float_array(buf, 0, angle_set_maxon, 5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN);
#else
    mavlink_ros_actuator_output_t packet;
    packet.enable_torque = enable_torque;
    packet.cyclic_sync_mode = cyclic_sync_mode;
    mav_array_memcpy(packet.angle_set_maxon, angle_set_maxon, sizeof(float)*5);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_CRC);
}

/**
 * @brief Encode a ros_actuator_output struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ros_actuator_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ros_actuator_output_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ros_actuator_output_t* ros_actuator_output)
{
    return mavlink_msg_ros_actuator_output_pack(system_id, component_id, msg, ros_actuator_output->angle_set_maxon, ros_actuator_output->enable_torque, ros_actuator_output->cyclic_sync_mode);
}

/**
 * @brief Encode a ros_actuator_output struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ros_actuator_output C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ros_actuator_output_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ros_actuator_output_t* ros_actuator_output)
{
    return mavlink_msg_ros_actuator_output_pack_chan(system_id, component_id, chan, msg, ros_actuator_output->angle_set_maxon, ros_actuator_output->enable_torque, ros_actuator_output->cyclic_sync_mode);
}

/**
 * @brief Send a ros_actuator_output message
 * @param chan MAVLink channel to send the message
 *
 * @param angle_set_maxon [deg]  aoa output to be set to Maxon 
 * @param enable_torque   enable torque on Maxon actuators 
 * @param cyclic_sync_mode   enable Cyclic Sync Position mode 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ros_actuator_output_send(mavlink_channel_t chan, const float *angle_set_maxon, uint8_t enable_torque, uint8_t cyclic_sync_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN];
    _mav_put_uint8_t(buf, 20, enable_torque);
    _mav_put_uint8_t(buf, 21, cyclic_sync_mode);
    _mav_put_float_array(buf, 0, angle_set_maxon, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT, buf, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_CRC);
#else
    mavlink_ros_actuator_output_t packet;
    packet.enable_torque = enable_torque;
    packet.cyclic_sync_mode = cyclic_sync_mode;
    mav_array_memcpy(packet.angle_set_maxon, angle_set_maxon, sizeof(float)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT, (const char *)&packet, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_CRC);
#endif
}

/**
 * @brief Send a ros_actuator_output message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ros_actuator_output_send_struct(mavlink_channel_t chan, const mavlink_ros_actuator_output_t* ros_actuator_output)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ros_actuator_output_send(chan, ros_actuator_output->angle_set_maxon, ros_actuator_output->enable_torque, ros_actuator_output->cyclic_sync_mode);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT, (const char *)ros_actuator_output, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_CRC);
#endif
}

#if MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ros_actuator_output_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  const float *angle_set_maxon, uint8_t enable_torque, uint8_t cyclic_sync_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 20, enable_torque);
    _mav_put_uint8_t(buf, 21, cyclic_sync_mode);
    _mav_put_float_array(buf, 0, angle_set_maxon, 5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT, buf, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_CRC);
#else
    mavlink_ros_actuator_output_t *packet = (mavlink_ros_actuator_output_t *)msgbuf;
    packet->enable_torque = enable_torque;
    packet->cyclic_sync_mode = cyclic_sync_mode;
    mav_array_memcpy(packet->angle_set_maxon, angle_set_maxon, sizeof(float)*5);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT, (const char *)packet, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_MIN_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_CRC);
#endif
}
#endif

#endif

// MESSAGE ROS_ACTUATOR_OUTPUT UNPACKING


/**
 * @brief Get field angle_set_maxon from ros_actuator_output message
 *
 * @return [deg]  aoa output to be set to Maxon 
 */
static inline uint16_t mavlink_msg_ros_actuator_output_get_angle_set_maxon(const mavlink_message_t* msg, float *angle_set_maxon)
{
    return _MAV_RETURN_float_array(msg, angle_set_maxon, 5,  0);
}

/**
 * @brief Get field enable_torque from ros_actuator_output message
 *
 * @return   enable torque on Maxon actuators 
 */
static inline uint8_t mavlink_msg_ros_actuator_output_get_enable_torque(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field cyclic_sync_mode from ros_actuator_output message
 *
 * @return   enable Cyclic Sync Position mode 
 */
static inline uint8_t mavlink_msg_ros_actuator_output_get_cyclic_sync_mode(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Decode a ros_actuator_output message into a struct
 *
 * @param msg The message to decode
 * @param ros_actuator_output C-struct to decode the message contents into
 */
static inline void mavlink_msg_ros_actuator_output_decode(const mavlink_message_t* msg, mavlink_ros_actuator_output_t* ros_actuator_output)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ros_actuator_output_get_angle_set_maxon(msg, ros_actuator_output->angle_set_maxon);
    ros_actuator_output->enable_torque = mavlink_msg_ros_actuator_output_get_enable_torque(msg);
    ros_actuator_output->cyclic_sync_mode = mavlink_msg_ros_actuator_output_get_cyclic_sync_mode(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN? msg->len : MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN;
        memset(ros_actuator_output, 0, MAVLINK_MSG_ID_ROS_ACTUATOR_OUTPUT_LEN);
    memcpy(ros_actuator_output, _MAV_PAYLOAD(msg), len);
#endif
}
