#pragma once
// MESSAGE SBG_GPS_HDT PACKING

#define MAVLINK_MSG_ID_SBG_GPS_HDT 637

MAVPACKED(
typedef struct __mavlink_sbg_gps_hdt_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 uint32_t time_stamp; /*< [us] Time since sensor is powered up us.*/
 uint32_t tow; /*< [ms] GPS Time of Week.*/
 float true_heading; /*< [deg] True heading angle (0 to 360 deg).*/
 float true_heading_acc; /*< [deg] 1 sigma True heading estimated accuracy (0 to 360 deg).*/
 float pitch; /*<  Pitch angle from the master to the rover.*/
 float pitch_acc; /*<  1 sigma pitch estimated accuracy.*/
 uint16_t status; /*< [us] GPS True Heading status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 HEIGHT_LIMIT.*/
}) mavlink_sbg_gps_hdt_t;

#define MAVLINK_MSG_ID_SBG_GPS_HDT_LEN 34
#define MAVLINK_MSG_ID_SBG_GPS_HDT_MIN_LEN 34
#define MAVLINK_MSG_ID_637_LEN 34
#define MAVLINK_MSG_ID_637_MIN_LEN 34

#define MAVLINK_MSG_ID_SBG_GPS_HDT_CRC 153
#define MAVLINK_MSG_ID_637_CRC 153



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SBG_GPS_HDT { \
    637, \
    "SBG_GPS_HDT", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_gps_hdt_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_gps_hdt_t, time_stamp) }, \
         { "status", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_sbg_gps_hdt_t, status) }, \
         { "tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_sbg_gps_hdt_t, tow) }, \
         { "true_heading", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sbg_gps_hdt_t, true_heading) }, \
         { "true_heading_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sbg_gps_hdt_t, true_heading_acc) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sbg_gps_hdt_t, pitch) }, \
         { "pitch_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sbg_gps_hdt_t, pitch_acc) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SBG_GPS_HDT { \
    "SBG_GPS_HDT", \
    8, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_gps_hdt_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_gps_hdt_t, time_stamp) }, \
         { "status", NULL, MAVLINK_TYPE_UINT16_T, 0, 32, offsetof(mavlink_sbg_gps_hdt_t, status) }, \
         { "tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_sbg_gps_hdt_t, tow) }, \
         { "true_heading", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sbg_gps_hdt_t, true_heading) }, \
         { "true_heading_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sbg_gps_hdt_t, true_heading_acc) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sbg_gps_hdt_t, pitch) }, \
         { "pitch_acc", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sbg_gps_hdt_t, pitch_acc) }, \
         } \
}
#endif

/**
 * @brief Pack a sbg_gps_hdt message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param status [us] GPS True Heading status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 HEIGHT_LIMIT.
 * @param tow [ms] GPS Time of Week.
 * @param true_heading [deg] True heading angle (0 to 360 deg).
 * @param true_heading_acc [deg] 1 sigma True heading estimated accuracy (0 to 360 deg).
 * @param pitch  Pitch angle from the master to the rover.
 * @param pitch_acc  1 sigma pitch estimated accuracy.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_gps_hdt_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t time_stamp, uint16_t status, uint32_t tow, float true_heading, float true_heading_acc, float pitch, float pitch_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_GPS_HDT_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, tow);
    _mav_put_float(buf, 16, true_heading);
    _mav_put_float(buf, 20, true_heading_acc);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, pitch_acc);
    _mav_put_uint16_t(buf, 32, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_GPS_HDT_LEN);
#else
    mavlink_sbg_gps_hdt_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.tow = tow;
    packet.true_heading = true_heading;
    packet.true_heading_acc = true_heading_acc;
    packet.pitch = pitch;
    packet.pitch_acc = pitch_acc;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_GPS_HDT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_GPS_HDT;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SBG_GPS_HDT_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_CRC);
}

/**
 * @brief Pack a sbg_gps_hdt message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param status [us] GPS True Heading status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 HEIGHT_LIMIT.
 * @param tow [ms] GPS Time of Week.
 * @param true_heading [deg] True heading angle (0 to 360 deg).
 * @param true_heading_acc [deg] 1 sigma True heading estimated accuracy (0 to 360 deg).
 * @param pitch  Pitch angle from the master to the rover.
 * @param pitch_acc  1 sigma pitch estimated accuracy.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_gps_hdt_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t time_stamp,uint16_t status,uint32_t tow,float true_heading,float true_heading_acc,float pitch,float pitch_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_GPS_HDT_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, tow);
    _mav_put_float(buf, 16, true_heading);
    _mav_put_float(buf, 20, true_heading_acc);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, pitch_acc);
    _mav_put_uint16_t(buf, 32, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_GPS_HDT_LEN);
#else
    mavlink_sbg_gps_hdt_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.tow = tow;
    packet.true_heading = true_heading;
    packet.true_heading_acc = true_heading_acc;
    packet.pitch = pitch;
    packet.pitch_acc = pitch_acc;
    packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_GPS_HDT_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_GPS_HDT;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SBG_GPS_HDT_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_CRC);
}

/**
 * @brief Encode a sbg_gps_hdt struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbg_gps_hdt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_gps_hdt_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbg_gps_hdt_t* sbg_gps_hdt)
{
    return mavlink_msg_sbg_gps_hdt_pack(system_id, component_id, msg, sbg_gps_hdt->timestamp, sbg_gps_hdt->time_stamp, sbg_gps_hdt->status, sbg_gps_hdt->tow, sbg_gps_hdt->true_heading, sbg_gps_hdt->true_heading_acc, sbg_gps_hdt->pitch, sbg_gps_hdt->pitch_acc);
}

/**
 * @brief Encode a sbg_gps_hdt struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sbg_gps_hdt C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_gps_hdt_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sbg_gps_hdt_t* sbg_gps_hdt)
{
    return mavlink_msg_sbg_gps_hdt_pack_chan(system_id, component_id, chan, msg, sbg_gps_hdt->timestamp, sbg_gps_hdt->time_stamp, sbg_gps_hdt->status, sbg_gps_hdt->tow, sbg_gps_hdt->true_heading, sbg_gps_hdt->true_heading_acc, sbg_gps_hdt->pitch, sbg_gps_hdt->pitch_acc);
}

/**
 * @brief Send a sbg_gps_hdt message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param status [us] GPS True Heading status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 HEIGHT_LIMIT.
 * @param tow [ms] GPS Time of Week.
 * @param true_heading [deg] True heading angle (0 to 360 deg).
 * @param true_heading_acc [deg] 1 sigma True heading estimated accuracy (0 to 360 deg).
 * @param pitch  Pitch angle from the master to the rover.
 * @param pitch_acc  1 sigma pitch estimated accuracy.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sbg_gps_hdt_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t time_stamp, uint16_t status, uint32_t tow, float true_heading, float true_heading_acc, float pitch, float pitch_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_GPS_HDT_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, tow);
    _mav_put_float(buf, 16, true_heading);
    _mav_put_float(buf, 20, true_heading_acc);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, pitch_acc);
    _mav_put_uint16_t(buf, 32, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_HDT, buf, MAVLINK_MSG_ID_SBG_GPS_HDT_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_CRC);
#else
    mavlink_sbg_gps_hdt_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.tow = tow;
    packet.true_heading = true_heading;
    packet.true_heading_acc = true_heading_acc;
    packet.pitch = pitch;
    packet.pitch_acc = pitch_acc;
    packet.status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_HDT, (const char *)&packet, MAVLINK_MSG_ID_SBG_GPS_HDT_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_CRC);
#endif
}

/**
 * @brief Send a sbg_gps_hdt message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sbg_gps_hdt_send_struct(mavlink_channel_t chan, const mavlink_sbg_gps_hdt_t* sbg_gps_hdt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sbg_gps_hdt_send(chan, sbg_gps_hdt->timestamp, sbg_gps_hdt->time_stamp, sbg_gps_hdt->status, sbg_gps_hdt->tow, sbg_gps_hdt->true_heading, sbg_gps_hdt->true_heading_acc, sbg_gps_hdt->pitch, sbg_gps_hdt->pitch_acc);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_HDT, (const char *)sbg_gps_hdt, MAVLINK_MSG_ID_SBG_GPS_HDT_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_CRC);
#endif
}

#if MAVLINK_MSG_ID_SBG_GPS_HDT_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sbg_gps_hdt_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t time_stamp, uint16_t status, uint32_t tow, float true_heading, float true_heading_acc, float pitch, float pitch_acc)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, tow);
    _mav_put_float(buf, 16, true_heading);
    _mav_put_float(buf, 20, true_heading_acc);
    _mav_put_float(buf, 24, pitch);
    _mav_put_float(buf, 28, pitch_acc);
    _mav_put_uint16_t(buf, 32, status);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_HDT, buf, MAVLINK_MSG_ID_SBG_GPS_HDT_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_CRC);
#else
    mavlink_sbg_gps_hdt_t *packet = (mavlink_sbg_gps_hdt_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->time_stamp = time_stamp;
    packet->tow = tow;
    packet->true_heading = true_heading;
    packet->true_heading_acc = true_heading_acc;
    packet->pitch = pitch;
    packet->pitch_acc = pitch_acc;
    packet->status = status;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_HDT, (const char *)packet, MAVLINK_MSG_ID_SBG_GPS_HDT_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_LEN, MAVLINK_MSG_ID_SBG_GPS_HDT_CRC);
#endif
}
#endif

#endif

// MESSAGE SBG_GPS_HDT UNPACKING


/**
 * @brief Get field timestamp from sbg_gps_hdt message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_sbg_gps_hdt_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_stamp from sbg_gps_hdt message
 *
 * @return [us] Time since sensor is powered up us.
 */
static inline uint32_t mavlink_msg_sbg_gps_hdt_get_time_stamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field status from sbg_gps_hdt message
 *
 * @return [us] GPS True Heading status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 HEIGHT_LIMIT.
 */
static inline uint16_t mavlink_msg_sbg_gps_hdt_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  32);
}

/**
 * @brief Get field tow from sbg_gps_hdt message
 *
 * @return [ms] GPS Time of Week.
 */
static inline uint32_t mavlink_msg_sbg_gps_hdt_get_tow(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field true_heading from sbg_gps_hdt message
 *
 * @return [deg] True heading angle (0 to 360 deg).
 */
static inline float mavlink_msg_sbg_gps_hdt_get_true_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field true_heading_acc from sbg_gps_hdt message
 *
 * @return [deg] 1 sigma True heading estimated accuracy (0 to 360 deg).
 */
static inline float mavlink_msg_sbg_gps_hdt_get_true_heading_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field pitch from sbg_gps_hdt message
 *
 * @return  Pitch angle from the master to the rover.
 */
static inline float mavlink_msg_sbg_gps_hdt_get_pitch(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field pitch_acc from sbg_gps_hdt message
 *
 * @return  1 sigma pitch estimated accuracy.
 */
static inline float mavlink_msg_sbg_gps_hdt_get_pitch_acc(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a sbg_gps_hdt message into a struct
 *
 * @param msg The message to decode
 * @param sbg_gps_hdt C-struct to decode the message contents into
 */
static inline void mavlink_msg_sbg_gps_hdt_decode(const mavlink_message_t* msg, mavlink_sbg_gps_hdt_t* sbg_gps_hdt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sbg_gps_hdt->timestamp = mavlink_msg_sbg_gps_hdt_get_timestamp(msg);
    sbg_gps_hdt->time_stamp = mavlink_msg_sbg_gps_hdt_get_time_stamp(msg);
    sbg_gps_hdt->tow = mavlink_msg_sbg_gps_hdt_get_tow(msg);
    sbg_gps_hdt->true_heading = mavlink_msg_sbg_gps_hdt_get_true_heading(msg);
    sbg_gps_hdt->true_heading_acc = mavlink_msg_sbg_gps_hdt_get_true_heading_acc(msg);
    sbg_gps_hdt->pitch = mavlink_msg_sbg_gps_hdt_get_pitch(msg);
    sbg_gps_hdt->pitch_acc = mavlink_msg_sbg_gps_hdt_get_pitch_acc(msg);
    sbg_gps_hdt->status = mavlink_msg_sbg_gps_hdt_get_status(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SBG_GPS_HDT_LEN? msg->len : MAVLINK_MSG_ID_SBG_GPS_HDT_LEN;
        memset(sbg_gps_hdt, 0, MAVLINK_MSG_ID_SBG_GPS_HDT_LEN);
    memcpy(sbg_gps_hdt, _MAV_PAYLOAD(msg), len);
#endif
}
