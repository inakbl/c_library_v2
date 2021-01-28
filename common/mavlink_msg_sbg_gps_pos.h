#pragma once
// MESSAGE SBG_GPS_POS PACKING

#define MAVLINK_MSG_ID_SBG_GPS_POS 635

MAVPACKED(
typedef struct __mavlink_sbg_gps_pos_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 uint32_t time_stamp; /*< [us] Time since sensor is powered up us.*/
 uint32_t gps_tow; /*< [ms] GPS Time of Week ms.*/
 float position[3]; /*< [deg] [Latitude (positive North deg), Longitude (positive East deg), Altitude (Above Mean Sea Level m)].*/
 float undulation; /*<  Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude) (Height above Ellipsoid = altitude + undulation).*/
 float position_accuracy[3]; /*< [m] Latitude, Longitude, Vertical accuracy m (1 sigma).*/
 uint16_t base_station_id; /*<  ID of the DGPS/RTK base station in use.*/
 uint16_t diff_age; /*<  Differential data age 0.01 s.*/
 uint8_t status; /*<  The raw GPS position status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 HEIGHT_LIMIT.*/
 uint8_t type; /*<  The raw GPS position type. 0 NO_SOLUTION, 1 UNKNOWN_TYPE, 2 SINGLE, 3 PSRDIFF, 4 SBAS, 5 OMNISTAR, 6 RTK_FLOAT, 7 RTK_INT, 8 PPP_FLOAT, 9 PPP_INT, 10 FIXED.*/
 uint8_t gps_l1_used; /*<  True if GPS L1 is used in the solution.*/
 uint8_t gps_l2_used; /*<  True if GPS L2 is used in the solution.*/
 uint8_t gps_l5_used; /*<  True if GPS L5 is used in the solution.*/
 uint8_t glo_l1_used; /*<  True if GLONASS L1 is used in the solution.*/
 uint8_t glo_l2_used; /*<  True if GLONASS L2 is used in the solution.*/
 uint8_t num_sv_used; /*<  Number of space vehicles used in GNSS solution.*/
}) mavlink_sbg_gps_pos_t;

#define MAVLINK_MSG_ID_SBG_GPS_POS_LEN 56
#define MAVLINK_MSG_ID_SBG_GPS_POS_MIN_LEN 56
#define MAVLINK_MSG_ID_635_LEN 56
#define MAVLINK_MSG_ID_635_MIN_LEN 56

#define MAVLINK_MSG_ID_SBG_GPS_POS_CRC 251
#define MAVLINK_MSG_ID_635_CRC 251

#define MAVLINK_MSG_SBG_GPS_POS_FIELD_POSITION_LEN 3
#define MAVLINK_MSG_SBG_GPS_POS_FIELD_POSITION_ACCURACY_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SBG_GPS_POS { \
    635, \
    "SBG_GPS_POS", \
    16, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_gps_pos_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_gps_pos_t, time_stamp) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_sbg_gps_pos_t, status) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_sbg_gps_pos_t, type) }, \
         { "gps_l1_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_sbg_gps_pos_t, gps_l1_used) }, \
         { "gps_l2_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_sbg_gps_pos_t, gps_l2_used) }, \
         { "gps_l5_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_sbg_gps_pos_t, gps_l5_used) }, \
         { "glo_l1_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_sbg_gps_pos_t, glo_l1_used) }, \
         { "glo_l2_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_sbg_gps_pos_t, glo_l2_used) }, \
         { "gps_tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_sbg_gps_pos_t, gps_tow) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 16, offsetof(mavlink_sbg_gps_pos_t, position) }, \
         { "undulation", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sbg_gps_pos_t, undulation) }, \
         { "position_accuracy", NULL, MAVLINK_TYPE_FLOAT, 3, 32, offsetof(mavlink_sbg_gps_pos_t, position_accuracy) }, \
         { "num_sv_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_sbg_gps_pos_t, num_sv_used) }, \
         { "base_station_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_sbg_gps_pos_t, base_station_id) }, \
         { "diff_age", NULL, MAVLINK_TYPE_UINT16_T, 0, 46, offsetof(mavlink_sbg_gps_pos_t, diff_age) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SBG_GPS_POS { \
    "SBG_GPS_POS", \
    16, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_gps_pos_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_gps_pos_t, time_stamp) }, \
         { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 48, offsetof(mavlink_sbg_gps_pos_t, status) }, \
         { "type", NULL, MAVLINK_TYPE_UINT8_T, 0, 49, offsetof(mavlink_sbg_gps_pos_t, type) }, \
         { "gps_l1_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 50, offsetof(mavlink_sbg_gps_pos_t, gps_l1_used) }, \
         { "gps_l2_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 51, offsetof(mavlink_sbg_gps_pos_t, gps_l2_used) }, \
         { "gps_l5_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 52, offsetof(mavlink_sbg_gps_pos_t, gps_l5_used) }, \
         { "glo_l1_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 53, offsetof(mavlink_sbg_gps_pos_t, glo_l1_used) }, \
         { "glo_l2_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 54, offsetof(mavlink_sbg_gps_pos_t, glo_l2_used) }, \
         { "gps_tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_sbg_gps_pos_t, gps_tow) }, \
         { "position", NULL, MAVLINK_TYPE_FLOAT, 3, 16, offsetof(mavlink_sbg_gps_pos_t, position) }, \
         { "undulation", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sbg_gps_pos_t, undulation) }, \
         { "position_accuracy", NULL, MAVLINK_TYPE_FLOAT, 3, 32, offsetof(mavlink_sbg_gps_pos_t, position_accuracy) }, \
         { "num_sv_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 55, offsetof(mavlink_sbg_gps_pos_t, num_sv_used) }, \
         { "base_station_id", NULL, MAVLINK_TYPE_UINT16_T, 0, 44, offsetof(mavlink_sbg_gps_pos_t, base_station_id) }, \
         { "diff_age", NULL, MAVLINK_TYPE_UINT16_T, 0, 46, offsetof(mavlink_sbg_gps_pos_t, diff_age) }, \
         } \
}
#endif

/**
 * @brief Pack a sbg_gps_pos message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param status  The raw GPS position status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 HEIGHT_LIMIT.
 * @param type  The raw GPS position type. 0 NO_SOLUTION, 1 UNKNOWN_TYPE, 2 SINGLE, 3 PSRDIFF, 4 SBAS, 5 OMNISTAR, 6 RTK_FLOAT, 7 RTK_INT, 8 PPP_FLOAT, 9 PPP_INT, 10 FIXED.
 * @param gps_l1_used  True if GPS L1 is used in the solution.
 * @param gps_l2_used  True if GPS L2 is used in the solution.
 * @param gps_l5_used  True if GPS L5 is used in the solution.
 * @param glo_l1_used  True if GLONASS L1 is used in the solution.
 * @param glo_l2_used  True if GLONASS L2 is used in the solution.
 * @param gps_tow [ms] GPS Time of Week ms.
 * @param position [deg] [Latitude (positive North deg), Longitude (positive East deg), Altitude (Above Mean Sea Level m)].
 * @param undulation  Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude) (Height above Ellipsoid = altitude + undulation).
 * @param position_accuracy [m] Latitude, Longitude, Vertical accuracy m (1 sigma).
 * @param num_sv_used  Number of space vehicles used in GNSS solution.
 * @param base_station_id  ID of the DGPS/RTK base station in use.
 * @param diff_age  Differential data age 0.01 s.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_gps_pos_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t time_stamp, uint8_t status, uint8_t type, uint8_t gps_l1_used, uint8_t gps_l2_used, uint8_t gps_l5_used, uint8_t glo_l1_used, uint8_t glo_l2_used, uint32_t gps_tow, const float *position, float undulation, const float *position_accuracy, uint8_t num_sv_used, uint16_t base_station_id, uint16_t diff_age)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_GPS_POS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, gps_tow);
    _mav_put_float(buf, 28, undulation);
    _mav_put_uint16_t(buf, 44, base_station_id);
    _mav_put_uint16_t(buf, 46, diff_age);
    _mav_put_uint8_t(buf, 48, status);
    _mav_put_uint8_t(buf, 49, type);
    _mav_put_uint8_t(buf, 50, gps_l1_used);
    _mav_put_uint8_t(buf, 51, gps_l2_used);
    _mav_put_uint8_t(buf, 52, gps_l5_used);
    _mav_put_uint8_t(buf, 53, glo_l1_used);
    _mav_put_uint8_t(buf, 54, glo_l2_used);
    _mav_put_uint8_t(buf, 55, num_sv_used);
    _mav_put_float_array(buf, 16, position, 3);
    _mav_put_float_array(buf, 32, position_accuracy, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_GPS_POS_LEN);
#else
    mavlink_sbg_gps_pos_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.gps_tow = gps_tow;
    packet.undulation = undulation;
    packet.base_station_id = base_station_id;
    packet.diff_age = diff_age;
    packet.status = status;
    packet.type = type;
    packet.gps_l1_used = gps_l1_used;
    packet.gps_l2_used = gps_l2_used;
    packet.gps_l5_used = gps_l5_used;
    packet.glo_l1_used = glo_l1_used;
    packet.glo_l2_used = glo_l2_used;
    packet.num_sv_used = num_sv_used;
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.position_accuracy, position_accuracy, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_GPS_POS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_GPS_POS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SBG_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_CRC);
}

/**
 * @brief Pack a sbg_gps_pos message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param status  The raw GPS position status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 HEIGHT_LIMIT.
 * @param type  The raw GPS position type. 0 NO_SOLUTION, 1 UNKNOWN_TYPE, 2 SINGLE, 3 PSRDIFF, 4 SBAS, 5 OMNISTAR, 6 RTK_FLOAT, 7 RTK_INT, 8 PPP_FLOAT, 9 PPP_INT, 10 FIXED.
 * @param gps_l1_used  True if GPS L1 is used in the solution.
 * @param gps_l2_used  True if GPS L2 is used in the solution.
 * @param gps_l5_used  True if GPS L5 is used in the solution.
 * @param glo_l1_used  True if GLONASS L1 is used in the solution.
 * @param glo_l2_used  True if GLONASS L2 is used in the solution.
 * @param gps_tow [ms] GPS Time of Week ms.
 * @param position [deg] [Latitude (positive North deg), Longitude (positive East deg), Altitude (Above Mean Sea Level m)].
 * @param undulation  Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude) (Height above Ellipsoid = altitude + undulation).
 * @param position_accuracy [m] Latitude, Longitude, Vertical accuracy m (1 sigma).
 * @param num_sv_used  Number of space vehicles used in GNSS solution.
 * @param base_station_id  ID of the DGPS/RTK base station in use.
 * @param diff_age  Differential data age 0.01 s.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_gps_pos_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t time_stamp,uint8_t status,uint8_t type,uint8_t gps_l1_used,uint8_t gps_l2_used,uint8_t gps_l5_used,uint8_t glo_l1_used,uint8_t glo_l2_used,uint32_t gps_tow,const float *position,float undulation,const float *position_accuracy,uint8_t num_sv_used,uint16_t base_station_id,uint16_t diff_age)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_GPS_POS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, gps_tow);
    _mav_put_float(buf, 28, undulation);
    _mav_put_uint16_t(buf, 44, base_station_id);
    _mav_put_uint16_t(buf, 46, diff_age);
    _mav_put_uint8_t(buf, 48, status);
    _mav_put_uint8_t(buf, 49, type);
    _mav_put_uint8_t(buf, 50, gps_l1_used);
    _mav_put_uint8_t(buf, 51, gps_l2_used);
    _mav_put_uint8_t(buf, 52, gps_l5_used);
    _mav_put_uint8_t(buf, 53, glo_l1_used);
    _mav_put_uint8_t(buf, 54, glo_l2_used);
    _mav_put_uint8_t(buf, 55, num_sv_used);
    _mav_put_float_array(buf, 16, position, 3);
    _mav_put_float_array(buf, 32, position_accuracy, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_GPS_POS_LEN);
#else
    mavlink_sbg_gps_pos_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.gps_tow = gps_tow;
    packet.undulation = undulation;
    packet.base_station_id = base_station_id;
    packet.diff_age = diff_age;
    packet.status = status;
    packet.type = type;
    packet.gps_l1_used = gps_l1_used;
    packet.gps_l2_used = gps_l2_used;
    packet.gps_l5_used = gps_l5_used;
    packet.glo_l1_used = glo_l1_used;
    packet.glo_l2_used = glo_l2_used;
    packet.num_sv_used = num_sv_used;
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.position_accuracy, position_accuracy, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_GPS_POS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_GPS_POS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SBG_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_CRC);
}

/**
 * @brief Encode a sbg_gps_pos struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbg_gps_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_gps_pos_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbg_gps_pos_t* sbg_gps_pos)
{
    return mavlink_msg_sbg_gps_pos_pack(system_id, component_id, msg, sbg_gps_pos->timestamp, sbg_gps_pos->time_stamp, sbg_gps_pos->status, sbg_gps_pos->type, sbg_gps_pos->gps_l1_used, sbg_gps_pos->gps_l2_used, sbg_gps_pos->gps_l5_used, sbg_gps_pos->glo_l1_used, sbg_gps_pos->glo_l2_used, sbg_gps_pos->gps_tow, sbg_gps_pos->position, sbg_gps_pos->undulation, sbg_gps_pos->position_accuracy, sbg_gps_pos->num_sv_used, sbg_gps_pos->base_station_id, sbg_gps_pos->diff_age);
}

/**
 * @brief Encode a sbg_gps_pos struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sbg_gps_pos C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_gps_pos_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sbg_gps_pos_t* sbg_gps_pos)
{
    return mavlink_msg_sbg_gps_pos_pack_chan(system_id, component_id, chan, msg, sbg_gps_pos->timestamp, sbg_gps_pos->time_stamp, sbg_gps_pos->status, sbg_gps_pos->type, sbg_gps_pos->gps_l1_used, sbg_gps_pos->gps_l2_used, sbg_gps_pos->gps_l5_used, sbg_gps_pos->glo_l1_used, sbg_gps_pos->glo_l2_used, sbg_gps_pos->gps_tow, sbg_gps_pos->position, sbg_gps_pos->undulation, sbg_gps_pos->position_accuracy, sbg_gps_pos->num_sv_used, sbg_gps_pos->base_station_id, sbg_gps_pos->diff_age);
}

/**
 * @brief Send a sbg_gps_pos message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param status  The raw GPS position status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 HEIGHT_LIMIT.
 * @param type  The raw GPS position type. 0 NO_SOLUTION, 1 UNKNOWN_TYPE, 2 SINGLE, 3 PSRDIFF, 4 SBAS, 5 OMNISTAR, 6 RTK_FLOAT, 7 RTK_INT, 8 PPP_FLOAT, 9 PPP_INT, 10 FIXED.
 * @param gps_l1_used  True if GPS L1 is used in the solution.
 * @param gps_l2_used  True if GPS L2 is used in the solution.
 * @param gps_l5_used  True if GPS L5 is used in the solution.
 * @param glo_l1_used  True if GLONASS L1 is used in the solution.
 * @param glo_l2_used  True if GLONASS L2 is used in the solution.
 * @param gps_tow [ms] GPS Time of Week ms.
 * @param position [deg] [Latitude (positive North deg), Longitude (positive East deg), Altitude (Above Mean Sea Level m)].
 * @param undulation  Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude) (Height above Ellipsoid = altitude + undulation).
 * @param position_accuracy [m] Latitude, Longitude, Vertical accuracy m (1 sigma).
 * @param num_sv_used  Number of space vehicles used in GNSS solution.
 * @param base_station_id  ID of the DGPS/RTK base station in use.
 * @param diff_age  Differential data age 0.01 s.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sbg_gps_pos_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t time_stamp, uint8_t status, uint8_t type, uint8_t gps_l1_used, uint8_t gps_l2_used, uint8_t gps_l5_used, uint8_t glo_l1_used, uint8_t glo_l2_used, uint32_t gps_tow, const float *position, float undulation, const float *position_accuracy, uint8_t num_sv_used, uint16_t base_station_id, uint16_t diff_age)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_GPS_POS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, gps_tow);
    _mav_put_float(buf, 28, undulation);
    _mav_put_uint16_t(buf, 44, base_station_id);
    _mav_put_uint16_t(buf, 46, diff_age);
    _mav_put_uint8_t(buf, 48, status);
    _mav_put_uint8_t(buf, 49, type);
    _mav_put_uint8_t(buf, 50, gps_l1_used);
    _mav_put_uint8_t(buf, 51, gps_l2_used);
    _mav_put_uint8_t(buf, 52, gps_l5_used);
    _mav_put_uint8_t(buf, 53, glo_l1_used);
    _mav_put_uint8_t(buf, 54, glo_l2_used);
    _mav_put_uint8_t(buf, 55, num_sv_used);
    _mav_put_float_array(buf, 16, position, 3);
    _mav_put_float_array(buf, 32, position_accuracy, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_POS, buf, MAVLINK_MSG_ID_SBG_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_CRC);
#else
    mavlink_sbg_gps_pos_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.gps_tow = gps_tow;
    packet.undulation = undulation;
    packet.base_station_id = base_station_id;
    packet.diff_age = diff_age;
    packet.status = status;
    packet.type = type;
    packet.gps_l1_used = gps_l1_used;
    packet.gps_l2_used = gps_l2_used;
    packet.gps_l5_used = gps_l5_used;
    packet.glo_l1_used = glo_l1_used;
    packet.glo_l2_used = glo_l2_used;
    packet.num_sv_used = num_sv_used;
    mav_array_memcpy(packet.position, position, sizeof(float)*3);
    mav_array_memcpy(packet.position_accuracy, position_accuracy, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_POS, (const char *)&packet, MAVLINK_MSG_ID_SBG_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_CRC);
#endif
}

/**
 * @brief Send a sbg_gps_pos message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sbg_gps_pos_send_struct(mavlink_channel_t chan, const mavlink_sbg_gps_pos_t* sbg_gps_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sbg_gps_pos_send(chan, sbg_gps_pos->timestamp, sbg_gps_pos->time_stamp, sbg_gps_pos->status, sbg_gps_pos->type, sbg_gps_pos->gps_l1_used, sbg_gps_pos->gps_l2_used, sbg_gps_pos->gps_l5_used, sbg_gps_pos->glo_l1_used, sbg_gps_pos->glo_l2_used, sbg_gps_pos->gps_tow, sbg_gps_pos->position, sbg_gps_pos->undulation, sbg_gps_pos->position_accuracy, sbg_gps_pos->num_sv_used, sbg_gps_pos->base_station_id, sbg_gps_pos->diff_age);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_POS, (const char *)sbg_gps_pos, MAVLINK_MSG_ID_SBG_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SBG_GPS_POS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sbg_gps_pos_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t time_stamp, uint8_t status, uint8_t type, uint8_t gps_l1_used, uint8_t gps_l2_used, uint8_t gps_l5_used, uint8_t glo_l1_used, uint8_t glo_l2_used, uint32_t gps_tow, const float *position, float undulation, const float *position_accuracy, uint8_t num_sv_used, uint16_t base_station_id, uint16_t diff_age)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, gps_tow);
    _mav_put_float(buf, 28, undulation);
    _mav_put_uint16_t(buf, 44, base_station_id);
    _mav_put_uint16_t(buf, 46, diff_age);
    _mav_put_uint8_t(buf, 48, status);
    _mav_put_uint8_t(buf, 49, type);
    _mav_put_uint8_t(buf, 50, gps_l1_used);
    _mav_put_uint8_t(buf, 51, gps_l2_used);
    _mav_put_uint8_t(buf, 52, gps_l5_used);
    _mav_put_uint8_t(buf, 53, glo_l1_used);
    _mav_put_uint8_t(buf, 54, glo_l2_used);
    _mav_put_uint8_t(buf, 55, num_sv_used);
    _mav_put_float_array(buf, 16, position, 3);
    _mav_put_float_array(buf, 32, position_accuracy, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_POS, buf, MAVLINK_MSG_ID_SBG_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_CRC);
#else
    mavlink_sbg_gps_pos_t *packet = (mavlink_sbg_gps_pos_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->time_stamp = time_stamp;
    packet->gps_tow = gps_tow;
    packet->undulation = undulation;
    packet->base_station_id = base_station_id;
    packet->diff_age = diff_age;
    packet->status = status;
    packet->type = type;
    packet->gps_l1_used = gps_l1_used;
    packet->gps_l2_used = gps_l2_used;
    packet->gps_l5_used = gps_l5_used;
    packet->glo_l1_used = glo_l1_used;
    packet->glo_l2_used = glo_l2_used;
    packet->num_sv_used = num_sv_used;
    mav_array_memcpy(packet->position, position, sizeof(float)*3);
    mav_array_memcpy(packet->position_accuracy, position_accuracy, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_GPS_POS, (const char *)packet, MAVLINK_MSG_ID_SBG_GPS_POS_MIN_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_LEN, MAVLINK_MSG_ID_SBG_GPS_POS_CRC);
#endif
}
#endif

#endif

// MESSAGE SBG_GPS_POS UNPACKING


/**
 * @brief Get field timestamp from sbg_gps_pos message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_sbg_gps_pos_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_stamp from sbg_gps_pos message
 *
 * @return [us] Time since sensor is powered up us.
 */
static inline uint32_t mavlink_msg_sbg_gps_pos_get_time_stamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field status from sbg_gps_pos message
 *
 * @return  The raw GPS position status. 0 SOL_COMPUTED, 1 INSUFFICIENT_OBS, 2 INTERNAL_ERROR, 3 HEIGHT_LIMIT.
 */
static inline uint8_t mavlink_msg_sbg_gps_pos_get_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  48);
}

/**
 * @brief Get field type from sbg_gps_pos message
 *
 * @return  The raw GPS position type. 0 NO_SOLUTION, 1 UNKNOWN_TYPE, 2 SINGLE, 3 PSRDIFF, 4 SBAS, 5 OMNISTAR, 6 RTK_FLOAT, 7 RTK_INT, 8 PPP_FLOAT, 9 PPP_INT, 10 FIXED.
 */
static inline uint8_t mavlink_msg_sbg_gps_pos_get_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  49);
}

/**
 * @brief Get field gps_l1_used from sbg_gps_pos message
 *
 * @return  True if GPS L1 is used in the solution.
 */
static inline uint8_t mavlink_msg_sbg_gps_pos_get_gps_l1_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  50);
}

/**
 * @brief Get field gps_l2_used from sbg_gps_pos message
 *
 * @return  True if GPS L2 is used in the solution.
 */
static inline uint8_t mavlink_msg_sbg_gps_pos_get_gps_l2_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  51);
}

/**
 * @brief Get field gps_l5_used from sbg_gps_pos message
 *
 * @return  True if GPS L5 is used in the solution.
 */
static inline uint8_t mavlink_msg_sbg_gps_pos_get_gps_l5_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  52);
}

/**
 * @brief Get field glo_l1_used from sbg_gps_pos message
 *
 * @return  True if GLONASS L1 is used in the solution.
 */
static inline uint8_t mavlink_msg_sbg_gps_pos_get_glo_l1_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  53);
}

/**
 * @brief Get field glo_l2_used from sbg_gps_pos message
 *
 * @return  True if GLONASS L2 is used in the solution.
 */
static inline uint8_t mavlink_msg_sbg_gps_pos_get_glo_l2_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  54);
}

/**
 * @brief Get field gps_tow from sbg_gps_pos message
 *
 * @return [ms] GPS Time of Week ms.
 */
static inline uint32_t mavlink_msg_sbg_gps_pos_get_gps_tow(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field position from sbg_gps_pos message
 *
 * @return [deg] [Latitude (positive North deg), Longitude (positive East deg), Altitude (Above Mean Sea Level m)].
 */
static inline uint16_t mavlink_msg_sbg_gps_pos_get_position(const mavlink_message_t* msg, float *position)
{
    return _MAV_RETURN_float_array(msg, position, 3,  16);
}

/**
 * @brief Get field undulation from sbg_gps_pos message
 *
 * @return  Altitude difference between the geoid and the Ellipsoid (WGS-84 Altitude - MSL Altitude) (Height above Ellipsoid = altitude + undulation).
 */
static inline float mavlink_msg_sbg_gps_pos_get_undulation(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field position_accuracy from sbg_gps_pos message
 *
 * @return [m] Latitude, Longitude, Vertical accuracy m (1 sigma).
 */
static inline uint16_t mavlink_msg_sbg_gps_pos_get_position_accuracy(const mavlink_message_t* msg, float *position_accuracy)
{
    return _MAV_RETURN_float_array(msg, position_accuracy, 3,  32);
}

/**
 * @brief Get field num_sv_used from sbg_gps_pos message
 *
 * @return  Number of space vehicles used in GNSS solution.
 */
static inline uint8_t mavlink_msg_sbg_gps_pos_get_num_sv_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  55);
}

/**
 * @brief Get field base_station_id from sbg_gps_pos message
 *
 * @return  ID of the DGPS/RTK base station in use.
 */
static inline uint16_t mavlink_msg_sbg_gps_pos_get_base_station_id(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  44);
}

/**
 * @brief Get field diff_age from sbg_gps_pos message
 *
 * @return  Differential data age 0.01 s.
 */
static inline uint16_t mavlink_msg_sbg_gps_pos_get_diff_age(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  46);
}

/**
 * @brief Decode a sbg_gps_pos message into a struct
 *
 * @param msg The message to decode
 * @param sbg_gps_pos C-struct to decode the message contents into
 */
static inline void mavlink_msg_sbg_gps_pos_decode(const mavlink_message_t* msg, mavlink_sbg_gps_pos_t* sbg_gps_pos)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sbg_gps_pos->timestamp = mavlink_msg_sbg_gps_pos_get_timestamp(msg);
    sbg_gps_pos->time_stamp = mavlink_msg_sbg_gps_pos_get_time_stamp(msg);
    sbg_gps_pos->gps_tow = mavlink_msg_sbg_gps_pos_get_gps_tow(msg);
    mavlink_msg_sbg_gps_pos_get_position(msg, sbg_gps_pos->position);
    sbg_gps_pos->undulation = mavlink_msg_sbg_gps_pos_get_undulation(msg);
    mavlink_msg_sbg_gps_pos_get_position_accuracy(msg, sbg_gps_pos->position_accuracy);
    sbg_gps_pos->base_station_id = mavlink_msg_sbg_gps_pos_get_base_station_id(msg);
    sbg_gps_pos->diff_age = mavlink_msg_sbg_gps_pos_get_diff_age(msg);
    sbg_gps_pos->status = mavlink_msg_sbg_gps_pos_get_status(msg);
    sbg_gps_pos->type = mavlink_msg_sbg_gps_pos_get_type(msg);
    sbg_gps_pos->gps_l1_used = mavlink_msg_sbg_gps_pos_get_gps_l1_used(msg);
    sbg_gps_pos->gps_l2_used = mavlink_msg_sbg_gps_pos_get_gps_l2_used(msg);
    sbg_gps_pos->gps_l5_used = mavlink_msg_sbg_gps_pos_get_gps_l5_used(msg);
    sbg_gps_pos->glo_l1_used = mavlink_msg_sbg_gps_pos_get_glo_l1_used(msg);
    sbg_gps_pos->glo_l2_used = mavlink_msg_sbg_gps_pos_get_glo_l2_used(msg);
    sbg_gps_pos->num_sv_used = mavlink_msg_sbg_gps_pos_get_num_sv_used(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SBG_GPS_POS_LEN? msg->len : MAVLINK_MSG_ID_SBG_GPS_POS_LEN;
        memset(sbg_gps_pos, 0, MAVLINK_MSG_ID_SBG_GPS_POS_LEN);
    memcpy(sbg_gps_pos, _MAV_PAYLOAD(msg), len);
#endif
}
