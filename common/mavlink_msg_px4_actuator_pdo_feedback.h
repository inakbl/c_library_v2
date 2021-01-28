#pragma once
// MESSAGE PX4_ACTUATOR_PDO_FEEDBACK PACKING

#define MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK 611

MAVPACKED(
typedef struct __mavlink_px4_actuator_pdo_feedback_t {
 float rudder_pdo_feedback; /*< [deg]  rudder position reported trough PDO */
 float starboard_pdo_feedback; /*< [deg]  starboard position reported trough PDO */
 float port_pdo_feedback; /*< [deg]  port position reported trough PDO */
 float central_1_pdo_feedback; /*< [deg]  central 1 position reported trough PDO */
 float central_2_pdo_feedback; /*< [deg]  central 2 position reported trough PDO */
 uint32_t elapsed; /*<   time elapsed between calls */
 uint32_t no_cycles; /*<   no_cycles */
 uint32_t average_time; /*<   average_time */
 uint32_t min_cycle_time; /*<   min_cycle_time */
 uint32_t max_cycle_time; /*<   max_cycle_time */
 float standard_deviation; /*<   standard_deviation */
}) mavlink_px4_actuator_pdo_feedback_t;

#define MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN 44
#define MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_MIN_LEN 44
#define MAVLINK_MSG_ID_611_LEN 44
#define MAVLINK_MSG_ID_611_MIN_LEN 44

#define MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_CRC 213
#define MAVLINK_MSG_ID_611_CRC 213



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PX4_ACTUATOR_PDO_FEEDBACK { \
    611, \
    "PX4_ACTUATOR_PDO_FEEDBACK", \
    11, \
    {  { "rudder_pdo_feedback", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_px4_actuator_pdo_feedback_t, rudder_pdo_feedback) }, \
         { "starboard_pdo_feedback", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_px4_actuator_pdo_feedback_t, starboard_pdo_feedback) }, \
         { "port_pdo_feedback", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_px4_actuator_pdo_feedback_t, port_pdo_feedback) }, \
         { "central_1_pdo_feedback", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_px4_actuator_pdo_feedback_t, central_1_pdo_feedback) }, \
         { "central_2_pdo_feedback", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_px4_actuator_pdo_feedback_t, central_2_pdo_feedback) }, \
         { "elapsed", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_px4_actuator_pdo_feedback_t, elapsed) }, \
         { "no_cycles", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_px4_actuator_pdo_feedback_t, no_cycles) }, \
         { "average_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_px4_actuator_pdo_feedback_t, average_time) }, \
         { "min_cycle_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_px4_actuator_pdo_feedback_t, min_cycle_time) }, \
         { "max_cycle_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 36, offsetof(mavlink_px4_actuator_pdo_feedback_t, max_cycle_time) }, \
         { "standard_deviation", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_px4_actuator_pdo_feedback_t, standard_deviation) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PX4_ACTUATOR_PDO_FEEDBACK { \
    "PX4_ACTUATOR_PDO_FEEDBACK", \
    11, \
    {  { "rudder_pdo_feedback", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_px4_actuator_pdo_feedback_t, rudder_pdo_feedback) }, \
         { "starboard_pdo_feedback", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_px4_actuator_pdo_feedback_t, starboard_pdo_feedback) }, \
         { "port_pdo_feedback", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_px4_actuator_pdo_feedback_t, port_pdo_feedback) }, \
         { "central_1_pdo_feedback", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_px4_actuator_pdo_feedback_t, central_1_pdo_feedback) }, \
         { "central_2_pdo_feedback", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_px4_actuator_pdo_feedback_t, central_2_pdo_feedback) }, \
         { "elapsed", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_px4_actuator_pdo_feedback_t, elapsed) }, \
         { "no_cycles", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_px4_actuator_pdo_feedback_t, no_cycles) }, \
         { "average_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_px4_actuator_pdo_feedback_t, average_time) }, \
         { "min_cycle_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_px4_actuator_pdo_feedback_t, min_cycle_time) }, \
         { "max_cycle_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 36, offsetof(mavlink_px4_actuator_pdo_feedback_t, max_cycle_time) }, \
         { "standard_deviation", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_px4_actuator_pdo_feedback_t, standard_deviation) }, \
         } \
}
#endif

/**
 * @brief Pack a px4_actuator_pdo_feedback message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rudder_pdo_feedback [deg]  rudder position reported trough PDO 
 * @param starboard_pdo_feedback [deg]  starboard position reported trough PDO 
 * @param port_pdo_feedback [deg]  port position reported trough PDO 
 * @param central_1_pdo_feedback [deg]  central 1 position reported trough PDO 
 * @param central_2_pdo_feedback [deg]  central 2 position reported trough PDO 
 * @param elapsed   time elapsed between calls 
 * @param no_cycles   no_cycles 
 * @param average_time   average_time 
 * @param min_cycle_time   min_cycle_time 
 * @param max_cycle_time   max_cycle_time 
 * @param standard_deviation   standard_deviation 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4_actuator_pdo_feedback_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float rudder_pdo_feedback, float starboard_pdo_feedback, float port_pdo_feedback, float central_1_pdo_feedback, float central_2_pdo_feedback, uint32_t elapsed, uint32_t no_cycles, uint32_t average_time, uint32_t min_cycle_time, uint32_t max_cycle_time, float standard_deviation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN];
    _mav_put_float(buf, 0, rudder_pdo_feedback);
    _mav_put_float(buf, 4, starboard_pdo_feedback);
    _mav_put_float(buf, 8, port_pdo_feedback);
    _mav_put_float(buf, 12, central_1_pdo_feedback);
    _mav_put_float(buf, 16, central_2_pdo_feedback);
    _mav_put_uint32_t(buf, 20, elapsed);
    _mav_put_uint32_t(buf, 24, no_cycles);
    _mav_put_uint32_t(buf, 28, average_time);
    _mav_put_uint32_t(buf, 32, min_cycle_time);
    _mav_put_uint32_t(buf, 36, max_cycle_time);
    _mav_put_float(buf, 40, standard_deviation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN);
#else
    mavlink_px4_actuator_pdo_feedback_t packet;
    packet.rudder_pdo_feedback = rudder_pdo_feedback;
    packet.starboard_pdo_feedback = starboard_pdo_feedback;
    packet.port_pdo_feedback = port_pdo_feedback;
    packet.central_1_pdo_feedback = central_1_pdo_feedback;
    packet.central_2_pdo_feedback = central_2_pdo_feedback;
    packet.elapsed = elapsed;
    packet.no_cycles = no_cycles;
    packet.average_time = average_time;
    packet.min_cycle_time = min_cycle_time;
    packet.max_cycle_time = max_cycle_time;
    packet.standard_deviation = standard_deviation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_CRC);
}

/**
 * @brief Pack a px4_actuator_pdo_feedback message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rudder_pdo_feedback [deg]  rudder position reported trough PDO 
 * @param starboard_pdo_feedback [deg]  starboard position reported trough PDO 
 * @param port_pdo_feedback [deg]  port position reported trough PDO 
 * @param central_1_pdo_feedback [deg]  central 1 position reported trough PDO 
 * @param central_2_pdo_feedback [deg]  central 2 position reported trough PDO 
 * @param elapsed   time elapsed between calls 
 * @param no_cycles   no_cycles 
 * @param average_time   average_time 
 * @param min_cycle_time   min_cycle_time 
 * @param max_cycle_time   max_cycle_time 
 * @param standard_deviation   standard_deviation 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4_actuator_pdo_feedback_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float rudder_pdo_feedback,float starboard_pdo_feedback,float port_pdo_feedback,float central_1_pdo_feedback,float central_2_pdo_feedback,uint32_t elapsed,uint32_t no_cycles,uint32_t average_time,uint32_t min_cycle_time,uint32_t max_cycle_time,float standard_deviation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN];
    _mav_put_float(buf, 0, rudder_pdo_feedback);
    _mav_put_float(buf, 4, starboard_pdo_feedback);
    _mav_put_float(buf, 8, port_pdo_feedback);
    _mav_put_float(buf, 12, central_1_pdo_feedback);
    _mav_put_float(buf, 16, central_2_pdo_feedback);
    _mav_put_uint32_t(buf, 20, elapsed);
    _mav_put_uint32_t(buf, 24, no_cycles);
    _mav_put_uint32_t(buf, 28, average_time);
    _mav_put_uint32_t(buf, 32, min_cycle_time);
    _mav_put_uint32_t(buf, 36, max_cycle_time);
    _mav_put_float(buf, 40, standard_deviation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN);
#else
    mavlink_px4_actuator_pdo_feedback_t packet;
    packet.rudder_pdo_feedback = rudder_pdo_feedback;
    packet.starboard_pdo_feedback = starboard_pdo_feedback;
    packet.port_pdo_feedback = port_pdo_feedback;
    packet.central_1_pdo_feedback = central_1_pdo_feedback;
    packet.central_2_pdo_feedback = central_2_pdo_feedback;
    packet.elapsed = elapsed;
    packet.no_cycles = no_cycles;
    packet.average_time = average_time;
    packet.min_cycle_time = min_cycle_time;
    packet.max_cycle_time = max_cycle_time;
    packet.standard_deviation = standard_deviation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_CRC);
}

/**
 * @brief Encode a px4_actuator_pdo_feedback struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param px4_actuator_pdo_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4_actuator_pdo_feedback_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_px4_actuator_pdo_feedback_t* px4_actuator_pdo_feedback)
{
    return mavlink_msg_px4_actuator_pdo_feedback_pack(system_id, component_id, msg, px4_actuator_pdo_feedback->rudder_pdo_feedback, px4_actuator_pdo_feedback->starboard_pdo_feedback, px4_actuator_pdo_feedback->port_pdo_feedback, px4_actuator_pdo_feedback->central_1_pdo_feedback, px4_actuator_pdo_feedback->central_2_pdo_feedback, px4_actuator_pdo_feedback->elapsed, px4_actuator_pdo_feedback->no_cycles, px4_actuator_pdo_feedback->average_time, px4_actuator_pdo_feedback->min_cycle_time, px4_actuator_pdo_feedback->max_cycle_time, px4_actuator_pdo_feedback->standard_deviation);
}

/**
 * @brief Encode a px4_actuator_pdo_feedback struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param px4_actuator_pdo_feedback C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4_actuator_pdo_feedback_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_px4_actuator_pdo_feedback_t* px4_actuator_pdo_feedback)
{
    return mavlink_msg_px4_actuator_pdo_feedback_pack_chan(system_id, component_id, chan, msg, px4_actuator_pdo_feedback->rudder_pdo_feedback, px4_actuator_pdo_feedback->starboard_pdo_feedback, px4_actuator_pdo_feedback->port_pdo_feedback, px4_actuator_pdo_feedback->central_1_pdo_feedback, px4_actuator_pdo_feedback->central_2_pdo_feedback, px4_actuator_pdo_feedback->elapsed, px4_actuator_pdo_feedback->no_cycles, px4_actuator_pdo_feedback->average_time, px4_actuator_pdo_feedback->min_cycle_time, px4_actuator_pdo_feedback->max_cycle_time, px4_actuator_pdo_feedback->standard_deviation);
}

/**
 * @brief Send a px4_actuator_pdo_feedback message
 * @param chan MAVLink channel to send the message
 *
 * @param rudder_pdo_feedback [deg]  rudder position reported trough PDO 
 * @param starboard_pdo_feedback [deg]  starboard position reported trough PDO 
 * @param port_pdo_feedback [deg]  port position reported trough PDO 
 * @param central_1_pdo_feedback [deg]  central 1 position reported trough PDO 
 * @param central_2_pdo_feedback [deg]  central 2 position reported trough PDO 
 * @param elapsed   time elapsed between calls 
 * @param no_cycles   no_cycles 
 * @param average_time   average_time 
 * @param min_cycle_time   min_cycle_time 
 * @param max_cycle_time   max_cycle_time 
 * @param standard_deviation   standard_deviation 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_px4_actuator_pdo_feedback_send(mavlink_channel_t chan, float rudder_pdo_feedback, float starboard_pdo_feedback, float port_pdo_feedback, float central_1_pdo_feedback, float central_2_pdo_feedback, uint32_t elapsed, uint32_t no_cycles, uint32_t average_time, uint32_t min_cycle_time, uint32_t max_cycle_time, float standard_deviation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN];
    _mav_put_float(buf, 0, rudder_pdo_feedback);
    _mav_put_float(buf, 4, starboard_pdo_feedback);
    _mav_put_float(buf, 8, port_pdo_feedback);
    _mav_put_float(buf, 12, central_1_pdo_feedback);
    _mav_put_float(buf, 16, central_2_pdo_feedback);
    _mav_put_uint32_t(buf, 20, elapsed);
    _mav_put_uint32_t(buf, 24, no_cycles);
    _mav_put_uint32_t(buf, 28, average_time);
    _mav_put_uint32_t(buf, 32, min_cycle_time);
    _mav_put_uint32_t(buf, 36, max_cycle_time);
    _mav_put_float(buf, 40, standard_deviation);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK, buf, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_CRC);
#else
    mavlink_px4_actuator_pdo_feedback_t packet;
    packet.rudder_pdo_feedback = rudder_pdo_feedback;
    packet.starboard_pdo_feedback = starboard_pdo_feedback;
    packet.port_pdo_feedback = port_pdo_feedback;
    packet.central_1_pdo_feedback = central_1_pdo_feedback;
    packet.central_2_pdo_feedback = central_2_pdo_feedback;
    packet.elapsed = elapsed;
    packet.no_cycles = no_cycles;
    packet.average_time = average_time;
    packet.min_cycle_time = min_cycle_time;
    packet.max_cycle_time = max_cycle_time;
    packet.standard_deviation = standard_deviation;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK, (const char *)&packet, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_CRC);
#endif
}

/**
 * @brief Send a px4_actuator_pdo_feedback message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_px4_actuator_pdo_feedback_send_struct(mavlink_channel_t chan, const mavlink_px4_actuator_pdo_feedback_t* px4_actuator_pdo_feedback)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_px4_actuator_pdo_feedback_send(chan, px4_actuator_pdo_feedback->rudder_pdo_feedback, px4_actuator_pdo_feedback->starboard_pdo_feedback, px4_actuator_pdo_feedback->port_pdo_feedback, px4_actuator_pdo_feedback->central_1_pdo_feedback, px4_actuator_pdo_feedback->central_2_pdo_feedback, px4_actuator_pdo_feedback->elapsed, px4_actuator_pdo_feedback->no_cycles, px4_actuator_pdo_feedback->average_time, px4_actuator_pdo_feedback->min_cycle_time, px4_actuator_pdo_feedback->max_cycle_time, px4_actuator_pdo_feedback->standard_deviation);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK, (const char *)px4_actuator_pdo_feedback, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_px4_actuator_pdo_feedback_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float rudder_pdo_feedback, float starboard_pdo_feedback, float port_pdo_feedback, float central_1_pdo_feedback, float central_2_pdo_feedback, uint32_t elapsed, uint32_t no_cycles, uint32_t average_time, uint32_t min_cycle_time, uint32_t max_cycle_time, float standard_deviation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, rudder_pdo_feedback);
    _mav_put_float(buf, 4, starboard_pdo_feedback);
    _mav_put_float(buf, 8, port_pdo_feedback);
    _mav_put_float(buf, 12, central_1_pdo_feedback);
    _mav_put_float(buf, 16, central_2_pdo_feedback);
    _mav_put_uint32_t(buf, 20, elapsed);
    _mav_put_uint32_t(buf, 24, no_cycles);
    _mav_put_uint32_t(buf, 28, average_time);
    _mav_put_uint32_t(buf, 32, min_cycle_time);
    _mav_put_uint32_t(buf, 36, max_cycle_time);
    _mav_put_float(buf, 40, standard_deviation);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK, buf, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_CRC);
#else
    mavlink_px4_actuator_pdo_feedback_t *packet = (mavlink_px4_actuator_pdo_feedback_t *)msgbuf;
    packet->rudder_pdo_feedback = rudder_pdo_feedback;
    packet->starboard_pdo_feedback = starboard_pdo_feedback;
    packet->port_pdo_feedback = port_pdo_feedback;
    packet->central_1_pdo_feedback = central_1_pdo_feedback;
    packet->central_2_pdo_feedback = central_2_pdo_feedback;
    packet->elapsed = elapsed;
    packet->no_cycles = no_cycles;
    packet->average_time = average_time;
    packet->min_cycle_time = min_cycle_time;
    packet->max_cycle_time = max_cycle_time;
    packet->standard_deviation = standard_deviation;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK, (const char *)packet, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_CRC);
#endif
}
#endif

#endif

// MESSAGE PX4_ACTUATOR_PDO_FEEDBACK UNPACKING


/**
 * @brief Get field rudder_pdo_feedback from px4_actuator_pdo_feedback message
 *
 * @return [deg]  rudder position reported trough PDO 
 */
static inline float mavlink_msg_px4_actuator_pdo_feedback_get_rudder_pdo_feedback(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field starboard_pdo_feedback from px4_actuator_pdo_feedback message
 *
 * @return [deg]  starboard position reported trough PDO 
 */
static inline float mavlink_msg_px4_actuator_pdo_feedback_get_starboard_pdo_feedback(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field port_pdo_feedback from px4_actuator_pdo_feedback message
 *
 * @return [deg]  port position reported trough PDO 
 */
static inline float mavlink_msg_px4_actuator_pdo_feedback_get_port_pdo_feedback(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field central_1_pdo_feedback from px4_actuator_pdo_feedback message
 *
 * @return [deg]  central 1 position reported trough PDO 
 */
static inline float mavlink_msg_px4_actuator_pdo_feedback_get_central_1_pdo_feedback(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field central_2_pdo_feedback from px4_actuator_pdo_feedback message
 *
 * @return [deg]  central 2 position reported trough PDO 
 */
static inline float mavlink_msg_px4_actuator_pdo_feedback_get_central_2_pdo_feedback(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field elapsed from px4_actuator_pdo_feedback message
 *
 * @return   time elapsed between calls 
 */
static inline uint32_t mavlink_msg_px4_actuator_pdo_feedback_get_elapsed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field no_cycles from px4_actuator_pdo_feedback message
 *
 * @return   no_cycles 
 */
static inline uint32_t mavlink_msg_px4_actuator_pdo_feedback_get_no_cycles(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field average_time from px4_actuator_pdo_feedback message
 *
 * @return   average_time 
 */
static inline uint32_t mavlink_msg_px4_actuator_pdo_feedback_get_average_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Get field min_cycle_time from px4_actuator_pdo_feedback message
 *
 * @return   min_cycle_time 
 */
static inline uint32_t mavlink_msg_px4_actuator_pdo_feedback_get_min_cycle_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  32);
}

/**
 * @brief Get field max_cycle_time from px4_actuator_pdo_feedback message
 *
 * @return   max_cycle_time 
 */
static inline uint32_t mavlink_msg_px4_actuator_pdo_feedback_get_max_cycle_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  36);
}

/**
 * @brief Get field standard_deviation from px4_actuator_pdo_feedback message
 *
 * @return   standard_deviation 
 */
static inline float mavlink_msg_px4_actuator_pdo_feedback_get_standard_deviation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a px4_actuator_pdo_feedback message into a struct
 *
 * @param msg The message to decode
 * @param px4_actuator_pdo_feedback C-struct to decode the message contents into
 */
static inline void mavlink_msg_px4_actuator_pdo_feedback_decode(const mavlink_message_t* msg, mavlink_px4_actuator_pdo_feedback_t* px4_actuator_pdo_feedback)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    px4_actuator_pdo_feedback->rudder_pdo_feedback = mavlink_msg_px4_actuator_pdo_feedback_get_rudder_pdo_feedback(msg);
    px4_actuator_pdo_feedback->starboard_pdo_feedback = mavlink_msg_px4_actuator_pdo_feedback_get_starboard_pdo_feedback(msg);
    px4_actuator_pdo_feedback->port_pdo_feedback = mavlink_msg_px4_actuator_pdo_feedback_get_port_pdo_feedback(msg);
    px4_actuator_pdo_feedback->central_1_pdo_feedback = mavlink_msg_px4_actuator_pdo_feedback_get_central_1_pdo_feedback(msg);
    px4_actuator_pdo_feedback->central_2_pdo_feedback = mavlink_msg_px4_actuator_pdo_feedback_get_central_2_pdo_feedback(msg);
    px4_actuator_pdo_feedback->elapsed = mavlink_msg_px4_actuator_pdo_feedback_get_elapsed(msg);
    px4_actuator_pdo_feedback->no_cycles = mavlink_msg_px4_actuator_pdo_feedback_get_no_cycles(msg);
    px4_actuator_pdo_feedback->average_time = mavlink_msg_px4_actuator_pdo_feedback_get_average_time(msg);
    px4_actuator_pdo_feedback->min_cycle_time = mavlink_msg_px4_actuator_pdo_feedback_get_min_cycle_time(msg);
    px4_actuator_pdo_feedback->max_cycle_time = mavlink_msg_px4_actuator_pdo_feedback_get_max_cycle_time(msg);
    px4_actuator_pdo_feedback->standard_deviation = mavlink_msg_px4_actuator_pdo_feedback_get_standard_deviation(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN? msg->len : MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN;
        memset(px4_actuator_pdo_feedback, 0, MAVLINK_MSG_ID_PX4_ACTUATOR_PDO_FEEDBACK_LEN);
    memcpy(px4_actuator_pdo_feedback, _MAV_PAYLOAD(msg), len);
#endif
}
