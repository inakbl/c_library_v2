#pragma once
// MESSAGE PX4_ACTUATOR_CONTROL_DEMAND PACKING

#define MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND 610


typedef struct __mavlink_px4_actuator_control_demand_t {
 float rudder_demanded_pos; /*< [deg]  aoa output from wrapper to be set to rudder in uavcan driver */
 float starboard_demanded_pos; /*< [deg]  aoa output from wrapper to be set to starboard in uavcan driver */
 float port_demanded_pos; /*< [deg]  aoa output from wrapper to be set to port in uavcan driver */
 float central_1_demanded_pos; /*< [deg]  aoa output from wrapper to be set to central 1 in uavcan driver */
 float central_2_demanded_pos; /*< [deg]  aoa output from wrapper to be set to central 2 in uavcan driver */
 uint32_t elapsed; /*<   time elapsed between calls */
 uint32_t no_cycles; /*<   no_cycles */
 uint32_t average_time; /*<   average_time */
 uint32_t min_cycle_time; /*<   min_cycle_time */
 uint32_t max_cycle_time; /*<   max_cycle_time */
 float standard_deviation; /*<   standard_deviation */
} mavlink_px4_actuator_control_demand_t;

#define MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN 44
#define MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_MIN_LEN 44
#define MAVLINK_MSG_ID_610_LEN 44
#define MAVLINK_MSG_ID_610_MIN_LEN 44

#define MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_CRC 79
#define MAVLINK_MSG_ID_610_CRC 79



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_PX4_ACTUATOR_CONTROL_DEMAND { \
    610, \
    "PX4_ACTUATOR_CONTROL_DEMAND", \
    11, \
    {  { "rudder_demanded_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_px4_actuator_control_demand_t, rudder_demanded_pos) }, \
         { "starboard_demanded_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_px4_actuator_control_demand_t, starboard_demanded_pos) }, \
         { "port_demanded_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_px4_actuator_control_demand_t, port_demanded_pos) }, \
         { "central_1_demanded_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_px4_actuator_control_demand_t, central_1_demanded_pos) }, \
         { "central_2_demanded_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_px4_actuator_control_demand_t, central_2_demanded_pos) }, \
         { "elapsed", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_px4_actuator_control_demand_t, elapsed) }, \
         { "no_cycles", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_px4_actuator_control_demand_t, no_cycles) }, \
         { "average_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_px4_actuator_control_demand_t, average_time) }, \
         { "min_cycle_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_px4_actuator_control_demand_t, min_cycle_time) }, \
         { "max_cycle_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 36, offsetof(mavlink_px4_actuator_control_demand_t, max_cycle_time) }, \
         { "standard_deviation", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_px4_actuator_control_demand_t, standard_deviation) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_PX4_ACTUATOR_CONTROL_DEMAND { \
    "PX4_ACTUATOR_CONTROL_DEMAND", \
    11, \
    {  { "rudder_demanded_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_px4_actuator_control_demand_t, rudder_demanded_pos) }, \
         { "starboard_demanded_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_px4_actuator_control_demand_t, starboard_demanded_pos) }, \
         { "port_demanded_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_px4_actuator_control_demand_t, port_demanded_pos) }, \
         { "central_1_demanded_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_px4_actuator_control_demand_t, central_1_demanded_pos) }, \
         { "central_2_demanded_pos", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_px4_actuator_control_demand_t, central_2_demanded_pos) }, \
         { "elapsed", NULL, MAVLINK_TYPE_UINT32_T, 0, 20, offsetof(mavlink_px4_actuator_control_demand_t, elapsed) }, \
         { "no_cycles", NULL, MAVLINK_TYPE_UINT32_T, 0, 24, offsetof(mavlink_px4_actuator_control_demand_t, no_cycles) }, \
         { "average_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 28, offsetof(mavlink_px4_actuator_control_demand_t, average_time) }, \
         { "min_cycle_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 32, offsetof(mavlink_px4_actuator_control_demand_t, min_cycle_time) }, \
         { "max_cycle_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 36, offsetof(mavlink_px4_actuator_control_demand_t, max_cycle_time) }, \
         { "standard_deviation", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_px4_actuator_control_demand_t, standard_deviation) }, \
         } \
}
#endif

/**
 * @brief Pack a px4_actuator_control_demand message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rudder_demanded_pos [deg]  aoa output from wrapper to be set to rudder in uavcan driver 
 * @param starboard_demanded_pos [deg]  aoa output from wrapper to be set to starboard in uavcan driver 
 * @param port_demanded_pos [deg]  aoa output from wrapper to be set to port in uavcan driver 
 * @param central_1_demanded_pos [deg]  aoa output from wrapper to be set to central 1 in uavcan driver 
 * @param central_2_demanded_pos [deg]  aoa output from wrapper to be set to central 2 in uavcan driver 
 * @param elapsed   time elapsed between calls 
 * @param no_cycles   no_cycles 
 * @param average_time   average_time 
 * @param min_cycle_time   min_cycle_time 
 * @param max_cycle_time   max_cycle_time 
 * @param standard_deviation   standard_deviation 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4_actuator_control_demand_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               float rudder_demanded_pos, float starboard_demanded_pos, float port_demanded_pos, float central_1_demanded_pos, float central_2_demanded_pos, uint32_t elapsed, uint32_t no_cycles, uint32_t average_time, uint32_t min_cycle_time, uint32_t max_cycle_time, float standard_deviation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN];
    _mav_put_float(buf, 0, rudder_demanded_pos);
    _mav_put_float(buf, 4, starboard_demanded_pos);
    _mav_put_float(buf, 8, port_demanded_pos);
    _mav_put_float(buf, 12, central_1_demanded_pos);
    _mav_put_float(buf, 16, central_2_demanded_pos);
    _mav_put_uint32_t(buf, 20, elapsed);
    _mav_put_uint32_t(buf, 24, no_cycles);
    _mav_put_uint32_t(buf, 28, average_time);
    _mav_put_uint32_t(buf, 32, min_cycle_time);
    _mav_put_uint32_t(buf, 36, max_cycle_time);
    _mav_put_float(buf, 40, standard_deviation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN);
#else
    mavlink_px4_actuator_control_demand_t packet;
    packet.rudder_demanded_pos = rudder_demanded_pos;
    packet.starboard_demanded_pos = starboard_demanded_pos;
    packet.port_demanded_pos = port_demanded_pos;
    packet.central_1_demanded_pos = central_1_demanded_pos;
    packet.central_2_demanded_pos = central_2_demanded_pos;
    packet.elapsed = elapsed;
    packet.no_cycles = no_cycles;
    packet.average_time = average_time;
    packet.min_cycle_time = min_cycle_time;
    packet.max_cycle_time = max_cycle_time;
    packet.standard_deviation = standard_deviation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_CRC);
}

/**
 * @brief Pack a px4_actuator_control_demand message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param rudder_demanded_pos [deg]  aoa output from wrapper to be set to rudder in uavcan driver 
 * @param starboard_demanded_pos [deg]  aoa output from wrapper to be set to starboard in uavcan driver 
 * @param port_demanded_pos [deg]  aoa output from wrapper to be set to port in uavcan driver 
 * @param central_1_demanded_pos [deg]  aoa output from wrapper to be set to central 1 in uavcan driver 
 * @param central_2_demanded_pos [deg]  aoa output from wrapper to be set to central 2 in uavcan driver 
 * @param elapsed   time elapsed between calls 
 * @param no_cycles   no_cycles 
 * @param average_time   average_time 
 * @param min_cycle_time   min_cycle_time 
 * @param max_cycle_time   max_cycle_time 
 * @param standard_deviation   standard_deviation 
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_px4_actuator_control_demand_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   float rudder_demanded_pos,float starboard_demanded_pos,float port_demanded_pos,float central_1_demanded_pos,float central_2_demanded_pos,uint32_t elapsed,uint32_t no_cycles,uint32_t average_time,uint32_t min_cycle_time,uint32_t max_cycle_time,float standard_deviation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN];
    _mav_put_float(buf, 0, rudder_demanded_pos);
    _mav_put_float(buf, 4, starboard_demanded_pos);
    _mav_put_float(buf, 8, port_demanded_pos);
    _mav_put_float(buf, 12, central_1_demanded_pos);
    _mav_put_float(buf, 16, central_2_demanded_pos);
    _mav_put_uint32_t(buf, 20, elapsed);
    _mav_put_uint32_t(buf, 24, no_cycles);
    _mav_put_uint32_t(buf, 28, average_time);
    _mav_put_uint32_t(buf, 32, min_cycle_time);
    _mav_put_uint32_t(buf, 36, max_cycle_time);
    _mav_put_float(buf, 40, standard_deviation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN);
#else
    mavlink_px4_actuator_control_demand_t packet;
    packet.rudder_demanded_pos = rudder_demanded_pos;
    packet.starboard_demanded_pos = starboard_demanded_pos;
    packet.port_demanded_pos = port_demanded_pos;
    packet.central_1_demanded_pos = central_1_demanded_pos;
    packet.central_2_demanded_pos = central_2_demanded_pos;
    packet.elapsed = elapsed;
    packet.no_cycles = no_cycles;
    packet.average_time = average_time;
    packet.min_cycle_time = min_cycle_time;
    packet.max_cycle_time = max_cycle_time;
    packet.standard_deviation = standard_deviation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_CRC);
}

/**
 * @brief Encode a px4_actuator_control_demand struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param px4_actuator_control_demand C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4_actuator_control_demand_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_px4_actuator_control_demand_t* px4_actuator_control_demand)
{
    return mavlink_msg_px4_actuator_control_demand_pack(system_id, component_id, msg, px4_actuator_control_demand->rudder_demanded_pos, px4_actuator_control_demand->starboard_demanded_pos, px4_actuator_control_demand->port_demanded_pos, px4_actuator_control_demand->central_1_demanded_pos, px4_actuator_control_demand->central_2_demanded_pos, px4_actuator_control_demand->elapsed, px4_actuator_control_demand->no_cycles, px4_actuator_control_demand->average_time, px4_actuator_control_demand->min_cycle_time, px4_actuator_control_demand->max_cycle_time, px4_actuator_control_demand->standard_deviation);
}

/**
 * @brief Encode a px4_actuator_control_demand struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param px4_actuator_control_demand C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_px4_actuator_control_demand_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_px4_actuator_control_demand_t* px4_actuator_control_demand)
{
    return mavlink_msg_px4_actuator_control_demand_pack_chan(system_id, component_id, chan, msg, px4_actuator_control_demand->rudder_demanded_pos, px4_actuator_control_demand->starboard_demanded_pos, px4_actuator_control_demand->port_demanded_pos, px4_actuator_control_demand->central_1_demanded_pos, px4_actuator_control_demand->central_2_demanded_pos, px4_actuator_control_demand->elapsed, px4_actuator_control_demand->no_cycles, px4_actuator_control_demand->average_time, px4_actuator_control_demand->min_cycle_time, px4_actuator_control_demand->max_cycle_time, px4_actuator_control_demand->standard_deviation);
}

/**
 * @brief Send a px4_actuator_control_demand message
 * @param chan MAVLink channel to send the message
 *
 * @param rudder_demanded_pos [deg]  aoa output from wrapper to be set to rudder in uavcan driver 
 * @param starboard_demanded_pos [deg]  aoa output from wrapper to be set to starboard in uavcan driver 
 * @param port_demanded_pos [deg]  aoa output from wrapper to be set to port in uavcan driver 
 * @param central_1_demanded_pos [deg]  aoa output from wrapper to be set to central 1 in uavcan driver 
 * @param central_2_demanded_pos [deg]  aoa output from wrapper to be set to central 2 in uavcan driver 
 * @param elapsed   time elapsed between calls 
 * @param no_cycles   no_cycles 
 * @param average_time   average_time 
 * @param min_cycle_time   min_cycle_time 
 * @param max_cycle_time   max_cycle_time 
 * @param standard_deviation   standard_deviation 
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_px4_actuator_control_demand_send(mavlink_channel_t chan, float rudder_demanded_pos, float starboard_demanded_pos, float port_demanded_pos, float central_1_demanded_pos, float central_2_demanded_pos, uint32_t elapsed, uint32_t no_cycles, uint32_t average_time, uint32_t min_cycle_time, uint32_t max_cycle_time, float standard_deviation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN];
    _mav_put_float(buf, 0, rudder_demanded_pos);
    _mav_put_float(buf, 4, starboard_demanded_pos);
    _mav_put_float(buf, 8, port_demanded_pos);
    _mav_put_float(buf, 12, central_1_demanded_pos);
    _mav_put_float(buf, 16, central_2_demanded_pos);
    _mav_put_uint32_t(buf, 20, elapsed);
    _mav_put_uint32_t(buf, 24, no_cycles);
    _mav_put_uint32_t(buf, 28, average_time);
    _mav_put_uint32_t(buf, 32, min_cycle_time);
    _mav_put_uint32_t(buf, 36, max_cycle_time);
    _mav_put_float(buf, 40, standard_deviation);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND, buf, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_CRC);
#else
    mavlink_px4_actuator_control_demand_t packet;
    packet.rudder_demanded_pos = rudder_demanded_pos;
    packet.starboard_demanded_pos = starboard_demanded_pos;
    packet.port_demanded_pos = port_demanded_pos;
    packet.central_1_demanded_pos = central_1_demanded_pos;
    packet.central_2_demanded_pos = central_2_demanded_pos;
    packet.elapsed = elapsed;
    packet.no_cycles = no_cycles;
    packet.average_time = average_time;
    packet.min_cycle_time = min_cycle_time;
    packet.max_cycle_time = max_cycle_time;
    packet.standard_deviation = standard_deviation;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND, (const char *)&packet, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_CRC);
#endif
}

/**
 * @brief Send a px4_actuator_control_demand message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_px4_actuator_control_demand_send_struct(mavlink_channel_t chan, const mavlink_px4_actuator_control_demand_t* px4_actuator_control_demand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_px4_actuator_control_demand_send(chan, px4_actuator_control_demand->rudder_demanded_pos, px4_actuator_control_demand->starboard_demanded_pos, px4_actuator_control_demand->port_demanded_pos, px4_actuator_control_demand->central_1_demanded_pos, px4_actuator_control_demand->central_2_demanded_pos, px4_actuator_control_demand->elapsed, px4_actuator_control_demand->no_cycles, px4_actuator_control_demand->average_time, px4_actuator_control_demand->min_cycle_time, px4_actuator_control_demand->max_cycle_time, px4_actuator_control_demand->standard_deviation);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND, (const char *)px4_actuator_control_demand, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_CRC);
#endif
}

#if MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_px4_actuator_control_demand_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float rudder_demanded_pos, float starboard_demanded_pos, float port_demanded_pos, float central_1_demanded_pos, float central_2_demanded_pos, uint32_t elapsed, uint32_t no_cycles, uint32_t average_time, uint32_t min_cycle_time, uint32_t max_cycle_time, float standard_deviation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_float(buf, 0, rudder_demanded_pos);
    _mav_put_float(buf, 4, starboard_demanded_pos);
    _mav_put_float(buf, 8, port_demanded_pos);
    _mav_put_float(buf, 12, central_1_demanded_pos);
    _mav_put_float(buf, 16, central_2_demanded_pos);
    _mav_put_uint32_t(buf, 20, elapsed);
    _mav_put_uint32_t(buf, 24, no_cycles);
    _mav_put_uint32_t(buf, 28, average_time);
    _mav_put_uint32_t(buf, 32, min_cycle_time);
    _mav_put_uint32_t(buf, 36, max_cycle_time);
    _mav_put_float(buf, 40, standard_deviation);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND, buf, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_CRC);
#else
    mavlink_px4_actuator_control_demand_t *packet = (mavlink_px4_actuator_control_demand_t *)msgbuf;
    packet->rudder_demanded_pos = rudder_demanded_pos;
    packet->starboard_demanded_pos = starboard_demanded_pos;
    packet->port_demanded_pos = port_demanded_pos;
    packet->central_1_demanded_pos = central_1_demanded_pos;
    packet->central_2_demanded_pos = central_2_demanded_pos;
    packet->elapsed = elapsed;
    packet->no_cycles = no_cycles;
    packet->average_time = average_time;
    packet->min_cycle_time = min_cycle_time;
    packet->max_cycle_time = max_cycle_time;
    packet->standard_deviation = standard_deviation;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND, (const char *)packet, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_MIN_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_CRC);
#endif
}
#endif

#endif

// MESSAGE PX4_ACTUATOR_CONTROL_DEMAND UNPACKING


/**
 * @brief Get field rudder_demanded_pos from px4_actuator_control_demand message
 *
 * @return [deg]  aoa output from wrapper to be set to rudder in uavcan driver 
 */
static inline float mavlink_msg_px4_actuator_control_demand_get_rudder_demanded_pos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field starboard_demanded_pos from px4_actuator_control_demand message
 *
 * @return [deg]  aoa output from wrapper to be set to starboard in uavcan driver 
 */
static inline float mavlink_msg_px4_actuator_control_demand_get_starboard_demanded_pos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field port_demanded_pos from px4_actuator_control_demand message
 *
 * @return [deg]  aoa output from wrapper to be set to port in uavcan driver 
 */
static inline float mavlink_msg_px4_actuator_control_demand_get_port_demanded_pos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field central_1_demanded_pos from px4_actuator_control_demand message
 *
 * @return [deg]  aoa output from wrapper to be set to central 1 in uavcan driver 
 */
static inline float mavlink_msg_px4_actuator_control_demand_get_central_1_demanded_pos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field central_2_demanded_pos from px4_actuator_control_demand message
 *
 * @return [deg]  aoa output from wrapper to be set to central 2 in uavcan driver 
 */
static inline float mavlink_msg_px4_actuator_control_demand_get_central_2_demanded_pos(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field elapsed from px4_actuator_control_demand message
 *
 * @return   time elapsed between calls 
 */
static inline uint32_t mavlink_msg_px4_actuator_control_demand_get_elapsed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  20);
}

/**
 * @brief Get field no_cycles from px4_actuator_control_demand message
 *
 * @return   no_cycles 
 */
static inline uint32_t mavlink_msg_px4_actuator_control_demand_get_no_cycles(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  24);
}

/**
 * @brief Get field average_time from px4_actuator_control_demand message
 *
 * @return   average_time 
 */
static inline uint32_t mavlink_msg_px4_actuator_control_demand_get_average_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  28);
}

/**
 * @brief Get field min_cycle_time from px4_actuator_control_demand message
 *
 * @return   min_cycle_time 
 */
static inline uint32_t mavlink_msg_px4_actuator_control_demand_get_min_cycle_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  32);
}

/**
 * @brief Get field max_cycle_time from px4_actuator_control_demand message
 *
 * @return   max_cycle_time 
 */
static inline uint32_t mavlink_msg_px4_actuator_control_demand_get_max_cycle_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  36);
}

/**
 * @brief Get field standard_deviation from px4_actuator_control_demand message
 *
 * @return   standard_deviation 
 */
static inline float mavlink_msg_px4_actuator_control_demand_get_standard_deviation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Decode a px4_actuator_control_demand message into a struct
 *
 * @param msg The message to decode
 * @param px4_actuator_control_demand C-struct to decode the message contents into
 */
static inline void mavlink_msg_px4_actuator_control_demand_decode(const mavlink_message_t* msg, mavlink_px4_actuator_control_demand_t* px4_actuator_control_demand)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    px4_actuator_control_demand->rudder_demanded_pos = mavlink_msg_px4_actuator_control_demand_get_rudder_demanded_pos(msg);
    px4_actuator_control_demand->starboard_demanded_pos = mavlink_msg_px4_actuator_control_demand_get_starboard_demanded_pos(msg);
    px4_actuator_control_demand->port_demanded_pos = mavlink_msg_px4_actuator_control_demand_get_port_demanded_pos(msg);
    px4_actuator_control_demand->central_1_demanded_pos = mavlink_msg_px4_actuator_control_demand_get_central_1_demanded_pos(msg);
    px4_actuator_control_demand->central_2_demanded_pos = mavlink_msg_px4_actuator_control_demand_get_central_2_demanded_pos(msg);
    px4_actuator_control_demand->elapsed = mavlink_msg_px4_actuator_control_demand_get_elapsed(msg);
    px4_actuator_control_demand->no_cycles = mavlink_msg_px4_actuator_control_demand_get_no_cycles(msg);
    px4_actuator_control_demand->average_time = mavlink_msg_px4_actuator_control_demand_get_average_time(msg);
    px4_actuator_control_demand->min_cycle_time = mavlink_msg_px4_actuator_control_demand_get_min_cycle_time(msg);
    px4_actuator_control_demand->max_cycle_time = mavlink_msg_px4_actuator_control_demand_get_max_cycle_time(msg);
    px4_actuator_control_demand->standard_deviation = mavlink_msg_px4_actuator_control_demand_get_standard_deviation(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN? msg->len : MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN;
        memset(px4_actuator_control_demand, 0, MAVLINK_MSG_ID_PX4_ACTUATOR_CONTROL_DEMAND_LEN);
    memcpy(px4_actuator_control_demand, _MAV_PAYLOAD(msg), len);
#endif
}
