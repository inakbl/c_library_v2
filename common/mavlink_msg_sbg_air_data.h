#pragma once
// MESSAGE SBG_AIR_DATA PACKING

#define MAVLINK_MSG_ID_SBG_AIR_DATA 638

MAVPACKED(
typedef struct __mavlink_sbg_air_data_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 uint32_t time_stamp; /*< [us] Time since sensor is powered up us.*/
 float pressure_abs; /*<  Raw absolute pressure measured by the barometer sensor in Pascals.*/
 float altitude; /*<  Altitude computed from barometric altimeter in meters and positive upward.*/
 float pressure_diff; /*<  Raw differential pressure measured by the pitot tube in Pascal.*/
 float true_air_speed; /*<  True airspeed measured by a pitot tube in m.s-1 and positive forward.*/
 float air_temperature; /*<  Outside air temperature in degrees C that could be used to compute true airspeed from differential pressure.*/
 uint8_t is_delay_time; /*<  True if the time stamp field represents a delay instead of an absolute time stamp.*/
 uint8_t pressure_valid; /*<  True if the pressure field is filled and valid.*/
 uint8_t altitude_valid; /*<  True if the barometric altitude field is filled and valid.*/
 uint8_t pressure_diff_valid; /*<  True if the differential pressure field is filled and valid.*/
 uint8_t air_speed_valid; /*<  True if the true airspeed field is filled and valid.*/
 uint8_t air_temperature_valid; /*<  True if the output air temperature field is filled and valid.*/
}) mavlink_sbg_air_data_t;

#define MAVLINK_MSG_ID_SBG_AIR_DATA_LEN 38
#define MAVLINK_MSG_ID_SBG_AIR_DATA_MIN_LEN 38
#define MAVLINK_MSG_ID_638_LEN 38
#define MAVLINK_MSG_ID_638_MIN_LEN 38

#define MAVLINK_MSG_ID_SBG_AIR_DATA_CRC 208
#define MAVLINK_MSG_ID_638_CRC 208



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SBG_AIR_DATA { \
    638, \
    "SBG_AIR_DATA", \
    13, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_air_data_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_air_data_t, time_stamp) }, \
         { "is_delay_time", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_sbg_air_data_t, is_delay_time) }, \
         { "pressure_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_sbg_air_data_t, pressure_valid) }, \
         { "altitude_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_sbg_air_data_t, altitude_valid) }, \
         { "pressure_diff_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_sbg_air_data_t, pressure_diff_valid) }, \
         { "air_speed_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_sbg_air_data_t, air_speed_valid) }, \
         { "air_temperature_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_sbg_air_data_t, air_temperature_valid) }, \
         { "pressure_abs", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sbg_air_data_t, pressure_abs) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sbg_air_data_t, altitude) }, \
         { "pressure_diff", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sbg_air_data_t, pressure_diff) }, \
         { "true_air_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sbg_air_data_t, true_air_speed) }, \
         { "air_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sbg_air_data_t, air_temperature) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SBG_AIR_DATA { \
    "SBG_AIR_DATA", \
    13, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_air_data_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_air_data_t, time_stamp) }, \
         { "is_delay_time", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_sbg_air_data_t, is_delay_time) }, \
         { "pressure_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_sbg_air_data_t, pressure_valid) }, \
         { "altitude_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_sbg_air_data_t, altitude_valid) }, \
         { "pressure_diff_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_sbg_air_data_t, pressure_diff_valid) }, \
         { "air_speed_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_sbg_air_data_t, air_speed_valid) }, \
         { "air_temperature_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_sbg_air_data_t, air_temperature_valid) }, \
         { "pressure_abs", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_sbg_air_data_t, pressure_abs) }, \
         { "altitude", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_sbg_air_data_t, altitude) }, \
         { "pressure_diff", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_sbg_air_data_t, pressure_diff) }, \
         { "true_air_speed", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_sbg_air_data_t, true_air_speed) }, \
         { "air_temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_sbg_air_data_t, air_temperature) }, \
         } \
}
#endif

/**
 * @brief Pack a sbg_air_data message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param is_delay_time  True if the time stamp field represents a delay instead of an absolute time stamp.
 * @param pressure_valid  True if the pressure field is filled and valid.
 * @param altitude_valid  True if the barometric altitude field is filled and valid.
 * @param pressure_diff_valid  True if the differential pressure field is filled and valid.
 * @param air_speed_valid  True if the true airspeed field is filled and valid.
 * @param air_temperature_valid  True if the output air temperature field is filled and valid.
 * @param pressure_abs  Raw absolute pressure measured by the barometer sensor in Pascals.
 * @param altitude  Altitude computed from barometric altimeter in meters and positive upward.
 * @param pressure_diff  Raw differential pressure measured by the pitot tube in Pascal.
 * @param true_air_speed  True airspeed measured by a pitot tube in m.s-1 and positive forward.
 * @param air_temperature  Outside air temperature in degrees C that could be used to compute true airspeed from differential pressure.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_air_data_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t time_stamp, uint8_t is_delay_time, uint8_t pressure_valid, uint8_t altitude_valid, uint8_t pressure_diff_valid, uint8_t air_speed_valid, uint8_t air_temperature_valid, float pressure_abs, float altitude, float pressure_diff, float true_air_speed, float air_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_AIR_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_float(buf, 12, pressure_abs);
    _mav_put_float(buf, 16, altitude);
    _mav_put_float(buf, 20, pressure_diff);
    _mav_put_float(buf, 24, true_air_speed);
    _mav_put_float(buf, 28, air_temperature);
    _mav_put_uint8_t(buf, 32, is_delay_time);
    _mav_put_uint8_t(buf, 33, pressure_valid);
    _mav_put_uint8_t(buf, 34, altitude_valid);
    _mav_put_uint8_t(buf, 35, pressure_diff_valid);
    _mav_put_uint8_t(buf, 36, air_speed_valid);
    _mav_put_uint8_t(buf, 37, air_temperature_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_AIR_DATA_LEN);
#else
    mavlink_sbg_air_data_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.pressure_abs = pressure_abs;
    packet.altitude = altitude;
    packet.pressure_diff = pressure_diff;
    packet.true_air_speed = true_air_speed;
    packet.air_temperature = air_temperature;
    packet.is_delay_time = is_delay_time;
    packet.pressure_valid = pressure_valid;
    packet.altitude_valid = altitude_valid;
    packet.pressure_diff_valid = pressure_diff_valid;
    packet.air_speed_valid = air_speed_valid;
    packet.air_temperature_valid = air_temperature_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_AIR_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_AIR_DATA;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SBG_AIR_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_CRC);
}

/**
 * @brief Pack a sbg_air_data message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param is_delay_time  True if the time stamp field represents a delay instead of an absolute time stamp.
 * @param pressure_valid  True if the pressure field is filled and valid.
 * @param altitude_valid  True if the barometric altitude field is filled and valid.
 * @param pressure_diff_valid  True if the differential pressure field is filled and valid.
 * @param air_speed_valid  True if the true airspeed field is filled and valid.
 * @param air_temperature_valid  True if the output air temperature field is filled and valid.
 * @param pressure_abs  Raw absolute pressure measured by the barometer sensor in Pascals.
 * @param altitude  Altitude computed from barometric altimeter in meters and positive upward.
 * @param pressure_diff  Raw differential pressure measured by the pitot tube in Pascal.
 * @param true_air_speed  True airspeed measured by a pitot tube in m.s-1 and positive forward.
 * @param air_temperature  Outside air temperature in degrees C that could be used to compute true airspeed from differential pressure.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_air_data_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t time_stamp,uint8_t is_delay_time,uint8_t pressure_valid,uint8_t altitude_valid,uint8_t pressure_diff_valid,uint8_t air_speed_valid,uint8_t air_temperature_valid,float pressure_abs,float altitude,float pressure_diff,float true_air_speed,float air_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_AIR_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_float(buf, 12, pressure_abs);
    _mav_put_float(buf, 16, altitude);
    _mav_put_float(buf, 20, pressure_diff);
    _mav_put_float(buf, 24, true_air_speed);
    _mav_put_float(buf, 28, air_temperature);
    _mav_put_uint8_t(buf, 32, is_delay_time);
    _mav_put_uint8_t(buf, 33, pressure_valid);
    _mav_put_uint8_t(buf, 34, altitude_valid);
    _mav_put_uint8_t(buf, 35, pressure_diff_valid);
    _mav_put_uint8_t(buf, 36, air_speed_valid);
    _mav_put_uint8_t(buf, 37, air_temperature_valid);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_AIR_DATA_LEN);
#else
    mavlink_sbg_air_data_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.pressure_abs = pressure_abs;
    packet.altitude = altitude;
    packet.pressure_diff = pressure_diff;
    packet.true_air_speed = true_air_speed;
    packet.air_temperature = air_temperature;
    packet.is_delay_time = is_delay_time;
    packet.pressure_valid = pressure_valid;
    packet.altitude_valid = altitude_valid;
    packet.pressure_diff_valid = pressure_diff_valid;
    packet.air_speed_valid = air_speed_valid;
    packet.air_temperature_valid = air_temperature_valid;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_AIR_DATA_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_AIR_DATA;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SBG_AIR_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_CRC);
}

/**
 * @brief Encode a sbg_air_data struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbg_air_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_air_data_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbg_air_data_t* sbg_air_data)
{
    return mavlink_msg_sbg_air_data_pack(system_id, component_id, msg, sbg_air_data->timestamp, sbg_air_data->time_stamp, sbg_air_data->is_delay_time, sbg_air_data->pressure_valid, sbg_air_data->altitude_valid, sbg_air_data->pressure_diff_valid, sbg_air_data->air_speed_valid, sbg_air_data->air_temperature_valid, sbg_air_data->pressure_abs, sbg_air_data->altitude, sbg_air_data->pressure_diff, sbg_air_data->true_air_speed, sbg_air_data->air_temperature);
}

/**
 * @brief Encode a sbg_air_data struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sbg_air_data C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_air_data_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sbg_air_data_t* sbg_air_data)
{
    return mavlink_msg_sbg_air_data_pack_chan(system_id, component_id, chan, msg, sbg_air_data->timestamp, sbg_air_data->time_stamp, sbg_air_data->is_delay_time, sbg_air_data->pressure_valid, sbg_air_data->altitude_valid, sbg_air_data->pressure_diff_valid, sbg_air_data->air_speed_valid, sbg_air_data->air_temperature_valid, sbg_air_data->pressure_abs, sbg_air_data->altitude, sbg_air_data->pressure_diff, sbg_air_data->true_air_speed, sbg_air_data->air_temperature);
}

/**
 * @brief Send a sbg_air_data message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param is_delay_time  True if the time stamp field represents a delay instead of an absolute time stamp.
 * @param pressure_valid  True if the pressure field is filled and valid.
 * @param altitude_valid  True if the barometric altitude field is filled and valid.
 * @param pressure_diff_valid  True if the differential pressure field is filled and valid.
 * @param air_speed_valid  True if the true airspeed field is filled and valid.
 * @param air_temperature_valid  True if the output air temperature field is filled and valid.
 * @param pressure_abs  Raw absolute pressure measured by the barometer sensor in Pascals.
 * @param altitude  Altitude computed from barometric altimeter in meters and positive upward.
 * @param pressure_diff  Raw differential pressure measured by the pitot tube in Pascal.
 * @param true_air_speed  True airspeed measured by a pitot tube in m.s-1 and positive forward.
 * @param air_temperature  Outside air temperature in degrees C that could be used to compute true airspeed from differential pressure.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sbg_air_data_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t time_stamp, uint8_t is_delay_time, uint8_t pressure_valid, uint8_t altitude_valid, uint8_t pressure_diff_valid, uint8_t air_speed_valid, uint8_t air_temperature_valid, float pressure_abs, float altitude, float pressure_diff, float true_air_speed, float air_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_AIR_DATA_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_float(buf, 12, pressure_abs);
    _mav_put_float(buf, 16, altitude);
    _mav_put_float(buf, 20, pressure_diff);
    _mav_put_float(buf, 24, true_air_speed);
    _mav_put_float(buf, 28, air_temperature);
    _mav_put_uint8_t(buf, 32, is_delay_time);
    _mav_put_uint8_t(buf, 33, pressure_valid);
    _mav_put_uint8_t(buf, 34, altitude_valid);
    _mav_put_uint8_t(buf, 35, pressure_diff_valid);
    _mav_put_uint8_t(buf, 36, air_speed_valid);
    _mav_put_uint8_t(buf, 37, air_temperature_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_AIR_DATA, buf, MAVLINK_MSG_ID_SBG_AIR_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_CRC);
#else
    mavlink_sbg_air_data_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.pressure_abs = pressure_abs;
    packet.altitude = altitude;
    packet.pressure_diff = pressure_diff;
    packet.true_air_speed = true_air_speed;
    packet.air_temperature = air_temperature;
    packet.is_delay_time = is_delay_time;
    packet.pressure_valid = pressure_valid;
    packet.altitude_valid = altitude_valid;
    packet.pressure_diff_valid = pressure_diff_valid;
    packet.air_speed_valid = air_speed_valid;
    packet.air_temperature_valid = air_temperature_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_AIR_DATA, (const char *)&packet, MAVLINK_MSG_ID_SBG_AIR_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_CRC);
#endif
}

/**
 * @brief Send a sbg_air_data message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sbg_air_data_send_struct(mavlink_channel_t chan, const mavlink_sbg_air_data_t* sbg_air_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sbg_air_data_send(chan, sbg_air_data->timestamp, sbg_air_data->time_stamp, sbg_air_data->is_delay_time, sbg_air_data->pressure_valid, sbg_air_data->altitude_valid, sbg_air_data->pressure_diff_valid, sbg_air_data->air_speed_valid, sbg_air_data->air_temperature_valid, sbg_air_data->pressure_abs, sbg_air_data->altitude, sbg_air_data->pressure_diff, sbg_air_data->true_air_speed, sbg_air_data->air_temperature);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_AIR_DATA, (const char *)sbg_air_data, MAVLINK_MSG_ID_SBG_AIR_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_CRC);
#endif
}

#if MAVLINK_MSG_ID_SBG_AIR_DATA_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sbg_air_data_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t time_stamp, uint8_t is_delay_time, uint8_t pressure_valid, uint8_t altitude_valid, uint8_t pressure_diff_valid, uint8_t air_speed_valid, uint8_t air_temperature_valid, float pressure_abs, float altitude, float pressure_diff, float true_air_speed, float air_temperature)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_float(buf, 12, pressure_abs);
    _mav_put_float(buf, 16, altitude);
    _mav_put_float(buf, 20, pressure_diff);
    _mav_put_float(buf, 24, true_air_speed);
    _mav_put_float(buf, 28, air_temperature);
    _mav_put_uint8_t(buf, 32, is_delay_time);
    _mav_put_uint8_t(buf, 33, pressure_valid);
    _mav_put_uint8_t(buf, 34, altitude_valid);
    _mav_put_uint8_t(buf, 35, pressure_diff_valid);
    _mav_put_uint8_t(buf, 36, air_speed_valid);
    _mav_put_uint8_t(buf, 37, air_temperature_valid);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_AIR_DATA, buf, MAVLINK_MSG_ID_SBG_AIR_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_CRC);
#else
    mavlink_sbg_air_data_t *packet = (mavlink_sbg_air_data_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->time_stamp = time_stamp;
    packet->pressure_abs = pressure_abs;
    packet->altitude = altitude;
    packet->pressure_diff = pressure_diff;
    packet->true_air_speed = true_air_speed;
    packet->air_temperature = air_temperature;
    packet->is_delay_time = is_delay_time;
    packet->pressure_valid = pressure_valid;
    packet->altitude_valid = altitude_valid;
    packet->pressure_diff_valid = pressure_diff_valid;
    packet->air_speed_valid = air_speed_valid;
    packet->air_temperature_valid = air_temperature_valid;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_AIR_DATA, (const char *)packet, MAVLINK_MSG_ID_SBG_AIR_DATA_MIN_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_LEN, MAVLINK_MSG_ID_SBG_AIR_DATA_CRC);
#endif
}
#endif

#endif

// MESSAGE SBG_AIR_DATA UNPACKING


/**
 * @brief Get field timestamp from sbg_air_data message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_sbg_air_data_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_stamp from sbg_air_data message
 *
 * @return [us] Time since sensor is powered up us.
 */
static inline uint32_t mavlink_msg_sbg_air_data_get_time_stamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field is_delay_time from sbg_air_data message
 *
 * @return  True if the time stamp field represents a delay instead of an absolute time stamp.
 */
static inline uint8_t mavlink_msg_sbg_air_data_get_is_delay_time(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field pressure_valid from sbg_air_data message
 *
 * @return  True if the pressure field is filled and valid.
 */
static inline uint8_t mavlink_msg_sbg_air_data_get_pressure_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field altitude_valid from sbg_air_data message
 *
 * @return  True if the barometric altitude field is filled and valid.
 */
static inline uint8_t mavlink_msg_sbg_air_data_get_altitude_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field pressure_diff_valid from sbg_air_data message
 *
 * @return  True if the differential pressure field is filled and valid.
 */
static inline uint8_t mavlink_msg_sbg_air_data_get_pressure_diff_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field air_speed_valid from sbg_air_data message
 *
 * @return  True if the true airspeed field is filled and valid.
 */
static inline uint8_t mavlink_msg_sbg_air_data_get_air_speed_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field air_temperature_valid from sbg_air_data message
 *
 * @return  True if the output air temperature field is filled and valid.
 */
static inline uint8_t mavlink_msg_sbg_air_data_get_air_temperature_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field pressure_abs from sbg_air_data message
 *
 * @return  Raw absolute pressure measured by the barometer sensor in Pascals.
 */
static inline float mavlink_msg_sbg_air_data_get_pressure_abs(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field altitude from sbg_air_data message
 *
 * @return  Altitude computed from barometric altimeter in meters and positive upward.
 */
static inline float mavlink_msg_sbg_air_data_get_altitude(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field pressure_diff from sbg_air_data message
 *
 * @return  Raw differential pressure measured by the pitot tube in Pascal.
 */
static inline float mavlink_msg_sbg_air_data_get_pressure_diff(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field true_air_speed from sbg_air_data message
 *
 * @return  True airspeed measured by a pitot tube in m.s-1 and positive forward.
 */
static inline float mavlink_msg_sbg_air_data_get_true_air_speed(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field air_temperature from sbg_air_data message
 *
 * @return  Outside air temperature in degrees C that could be used to compute true airspeed from differential pressure.
 */
static inline float mavlink_msg_sbg_air_data_get_air_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a sbg_air_data message into a struct
 *
 * @param msg The message to decode
 * @param sbg_air_data C-struct to decode the message contents into
 */
static inline void mavlink_msg_sbg_air_data_decode(const mavlink_message_t* msg, mavlink_sbg_air_data_t* sbg_air_data)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sbg_air_data->timestamp = mavlink_msg_sbg_air_data_get_timestamp(msg);
    sbg_air_data->time_stamp = mavlink_msg_sbg_air_data_get_time_stamp(msg);
    sbg_air_data->pressure_abs = mavlink_msg_sbg_air_data_get_pressure_abs(msg);
    sbg_air_data->altitude = mavlink_msg_sbg_air_data_get_altitude(msg);
    sbg_air_data->pressure_diff = mavlink_msg_sbg_air_data_get_pressure_diff(msg);
    sbg_air_data->true_air_speed = mavlink_msg_sbg_air_data_get_true_air_speed(msg);
    sbg_air_data->air_temperature = mavlink_msg_sbg_air_data_get_air_temperature(msg);
    sbg_air_data->is_delay_time = mavlink_msg_sbg_air_data_get_is_delay_time(msg);
    sbg_air_data->pressure_valid = mavlink_msg_sbg_air_data_get_pressure_valid(msg);
    sbg_air_data->altitude_valid = mavlink_msg_sbg_air_data_get_altitude_valid(msg);
    sbg_air_data->pressure_diff_valid = mavlink_msg_sbg_air_data_get_pressure_diff_valid(msg);
    sbg_air_data->air_speed_valid = mavlink_msg_sbg_air_data_get_air_speed_valid(msg);
    sbg_air_data->air_temperature_valid = mavlink_msg_sbg_air_data_get_air_temperature_valid(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SBG_AIR_DATA_LEN? msg->len : MAVLINK_MSG_ID_SBG_AIR_DATA_LEN;
        memset(sbg_air_data, 0, MAVLINK_MSG_ID_SBG_AIR_DATA_LEN);
    memcpy(sbg_air_data, _MAV_PAYLOAD(msg), len);
#endif
}
