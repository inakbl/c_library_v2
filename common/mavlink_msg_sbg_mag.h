#pragma once
// MESSAGE SBG_MAG PACKING

#define MAVLINK_MSG_ID_SBG_MAG 634

MAVPACKED(
typedef struct __mavlink_sbg_mag_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 uint32_t time_stamp; /*< [us] Time since sensor is powered up us.*/
 float mag[3]; /*<  Magnetometer output (X, Y, Z).*/
 float accel[3]; /*< [m/s2] Accelerometer output (X, Y, Z) in m/s2.*/
 uint8_t mag_x; /*<  True if the magnetometer X has passed the self test.*/
 uint8_t mag_y; /*<  True if the magnetometer Y has passed the self test.*/
 uint8_t mag_z; /*<  True if the magnetometer Z has passed the self test.*/
 uint8_t accel_x; /*<  True if the accelerometer X has passed the self test.*/
 uint8_t accel_y; /*<  True if the accelerometer Y has passed the self test.*/
 uint8_t accel_z; /*<  True if the accelerometer Z has passed the self test.*/
 uint8_t mags_in_range; /*<  True if magnetometer is not saturated*/
 uint8_t accels_in_range; /*<  True if accelerometer is not saturated*/
 uint8_t calibration; /*<  True if magnetometer seems to be calibrated*/
}) mavlink_sbg_mag_t;

#define MAVLINK_MSG_ID_SBG_MAG_LEN 45
#define MAVLINK_MSG_ID_SBG_MAG_MIN_LEN 45
#define MAVLINK_MSG_ID_634_LEN 45
#define MAVLINK_MSG_ID_634_MIN_LEN 45

#define MAVLINK_MSG_ID_SBG_MAG_CRC 232
#define MAVLINK_MSG_ID_634_CRC 232

#define MAVLINK_MSG_SBG_MAG_FIELD_MAG_LEN 3
#define MAVLINK_MSG_SBG_MAG_FIELD_ACCEL_LEN 3

#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SBG_MAG { \
    634, \
    "SBG_MAG", \
    13, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_mag_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_mag_t, time_stamp) }, \
         { "mag", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_sbg_mag_t, mag) }, \
         { "accel", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_sbg_mag_t, accel) }, \
         { "mag_x", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_sbg_mag_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_sbg_mag_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_sbg_mag_t, mag_z) }, \
         { "accel_x", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_sbg_mag_t, accel_x) }, \
         { "accel_y", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_sbg_mag_t, accel_y) }, \
         { "accel_z", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_sbg_mag_t, accel_z) }, \
         { "mags_in_range", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_sbg_mag_t, mags_in_range) }, \
         { "accels_in_range", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_sbg_mag_t, accels_in_range) }, \
         { "calibration", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_sbg_mag_t, calibration) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SBG_MAG { \
    "SBG_MAG", \
    13, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_mag_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_mag_t, time_stamp) }, \
         { "mag", NULL, MAVLINK_TYPE_FLOAT, 3, 12, offsetof(mavlink_sbg_mag_t, mag) }, \
         { "accel", NULL, MAVLINK_TYPE_FLOAT, 3, 24, offsetof(mavlink_sbg_mag_t, accel) }, \
         { "mag_x", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_sbg_mag_t, mag_x) }, \
         { "mag_y", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_sbg_mag_t, mag_y) }, \
         { "mag_z", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_sbg_mag_t, mag_z) }, \
         { "accel_x", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_sbg_mag_t, accel_x) }, \
         { "accel_y", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_sbg_mag_t, accel_y) }, \
         { "accel_z", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_sbg_mag_t, accel_z) }, \
         { "mags_in_range", NULL, MAVLINK_TYPE_UINT8_T, 0, 42, offsetof(mavlink_sbg_mag_t, mags_in_range) }, \
         { "accels_in_range", NULL, MAVLINK_TYPE_UINT8_T, 0, 43, offsetof(mavlink_sbg_mag_t, accels_in_range) }, \
         { "calibration", NULL, MAVLINK_TYPE_UINT8_T, 0, 44, offsetof(mavlink_sbg_mag_t, calibration) }, \
         } \
}
#endif

/**
 * @brief Pack a sbg_mag message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param mag  Magnetometer output (X, Y, Z).
 * @param accel [m/s2] Accelerometer output (X, Y, Z) in m/s2.
 * @param mag_x  True if the magnetometer X has passed the self test.
 * @param mag_y  True if the magnetometer Y has passed the self test.
 * @param mag_z  True if the magnetometer Z has passed the self test.
 * @param accel_x  True if the accelerometer X has passed the self test.
 * @param accel_y  True if the accelerometer Y has passed the self test.
 * @param accel_z  True if the accelerometer Z has passed the self test.
 * @param mags_in_range  True if magnetometer is not saturated
 * @param accels_in_range  True if accelerometer is not saturated
 * @param calibration  True if magnetometer seems to be calibrated
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_mag_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t time_stamp, const float *mag, const float *accel, uint8_t mag_x, uint8_t mag_y, uint8_t mag_z, uint8_t accel_x, uint8_t accel_y, uint8_t accel_z, uint8_t mags_in_range, uint8_t accels_in_range, uint8_t calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_MAG_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint8_t(buf, 36, mag_x);
    _mav_put_uint8_t(buf, 37, mag_y);
    _mav_put_uint8_t(buf, 38, mag_z);
    _mav_put_uint8_t(buf, 39, accel_x);
    _mav_put_uint8_t(buf, 40, accel_y);
    _mav_put_uint8_t(buf, 41, accel_z);
    _mav_put_uint8_t(buf, 42, mags_in_range);
    _mav_put_uint8_t(buf, 43, accels_in_range);
    _mav_put_uint8_t(buf, 44, calibration);
    _mav_put_float_array(buf, 12, mag, 3);
    _mav_put_float_array(buf, 24, accel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_MAG_LEN);
#else
    mavlink_sbg_mag_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.accel_x = accel_x;
    packet.accel_y = accel_y;
    packet.accel_z = accel_z;
    packet.mags_in_range = mags_in_range;
    packet.accels_in_range = accels_in_range;
    packet.calibration = calibration;
    mav_array_memcpy(packet.mag, mag, sizeof(float)*3);
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_MAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_MAG;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SBG_MAG_MIN_LEN, MAVLINK_MSG_ID_SBG_MAG_LEN, MAVLINK_MSG_ID_SBG_MAG_CRC);
}

/**
 * @brief Pack a sbg_mag message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param mag  Magnetometer output (X, Y, Z).
 * @param accel [m/s2] Accelerometer output (X, Y, Z) in m/s2.
 * @param mag_x  True if the magnetometer X has passed the self test.
 * @param mag_y  True if the magnetometer Y has passed the self test.
 * @param mag_z  True if the magnetometer Z has passed the self test.
 * @param accel_x  True if the accelerometer X has passed the self test.
 * @param accel_y  True if the accelerometer Y has passed the self test.
 * @param accel_z  True if the accelerometer Z has passed the self test.
 * @param mags_in_range  True if magnetometer is not saturated
 * @param accels_in_range  True if accelerometer is not saturated
 * @param calibration  True if magnetometer seems to be calibrated
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_mag_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t time_stamp,const float *mag,const float *accel,uint8_t mag_x,uint8_t mag_y,uint8_t mag_z,uint8_t accel_x,uint8_t accel_y,uint8_t accel_z,uint8_t mags_in_range,uint8_t accels_in_range,uint8_t calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_MAG_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint8_t(buf, 36, mag_x);
    _mav_put_uint8_t(buf, 37, mag_y);
    _mav_put_uint8_t(buf, 38, mag_z);
    _mav_put_uint8_t(buf, 39, accel_x);
    _mav_put_uint8_t(buf, 40, accel_y);
    _mav_put_uint8_t(buf, 41, accel_z);
    _mav_put_uint8_t(buf, 42, mags_in_range);
    _mav_put_uint8_t(buf, 43, accels_in_range);
    _mav_put_uint8_t(buf, 44, calibration);
    _mav_put_float_array(buf, 12, mag, 3);
    _mav_put_float_array(buf, 24, accel, 3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_MAG_LEN);
#else
    mavlink_sbg_mag_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.accel_x = accel_x;
    packet.accel_y = accel_y;
    packet.accel_z = accel_z;
    packet.mags_in_range = mags_in_range;
    packet.accels_in_range = accels_in_range;
    packet.calibration = calibration;
    mav_array_memcpy(packet.mag, mag, sizeof(float)*3);
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_MAG_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_MAG;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SBG_MAG_MIN_LEN, MAVLINK_MSG_ID_SBG_MAG_LEN, MAVLINK_MSG_ID_SBG_MAG_CRC);
}

/**
 * @brief Encode a sbg_mag struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbg_mag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_mag_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbg_mag_t* sbg_mag)
{
    return mavlink_msg_sbg_mag_pack(system_id, component_id, msg, sbg_mag->timestamp, sbg_mag->time_stamp, sbg_mag->mag, sbg_mag->accel, sbg_mag->mag_x, sbg_mag->mag_y, sbg_mag->mag_z, sbg_mag->accel_x, sbg_mag->accel_y, sbg_mag->accel_z, sbg_mag->mags_in_range, sbg_mag->accels_in_range, sbg_mag->calibration);
}

/**
 * @brief Encode a sbg_mag struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sbg_mag C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_mag_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sbg_mag_t* sbg_mag)
{
    return mavlink_msg_sbg_mag_pack_chan(system_id, component_id, chan, msg, sbg_mag->timestamp, sbg_mag->time_stamp, sbg_mag->mag, sbg_mag->accel, sbg_mag->mag_x, sbg_mag->mag_y, sbg_mag->mag_z, sbg_mag->accel_x, sbg_mag->accel_y, sbg_mag->accel_z, sbg_mag->mags_in_range, sbg_mag->accels_in_range, sbg_mag->calibration);
}

/**
 * @brief Send a sbg_mag message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param mag  Magnetometer output (X, Y, Z).
 * @param accel [m/s2] Accelerometer output (X, Y, Z) in m/s2.
 * @param mag_x  True if the magnetometer X has passed the self test.
 * @param mag_y  True if the magnetometer Y has passed the self test.
 * @param mag_z  True if the magnetometer Z has passed the self test.
 * @param accel_x  True if the accelerometer X has passed the self test.
 * @param accel_y  True if the accelerometer Y has passed the self test.
 * @param accel_z  True if the accelerometer Z has passed the self test.
 * @param mags_in_range  True if magnetometer is not saturated
 * @param accels_in_range  True if accelerometer is not saturated
 * @param calibration  True if magnetometer seems to be calibrated
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sbg_mag_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t time_stamp, const float *mag, const float *accel, uint8_t mag_x, uint8_t mag_y, uint8_t mag_z, uint8_t accel_x, uint8_t accel_y, uint8_t accel_z, uint8_t mags_in_range, uint8_t accels_in_range, uint8_t calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_MAG_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint8_t(buf, 36, mag_x);
    _mav_put_uint8_t(buf, 37, mag_y);
    _mav_put_uint8_t(buf, 38, mag_z);
    _mav_put_uint8_t(buf, 39, accel_x);
    _mav_put_uint8_t(buf, 40, accel_y);
    _mav_put_uint8_t(buf, 41, accel_z);
    _mav_put_uint8_t(buf, 42, mags_in_range);
    _mav_put_uint8_t(buf, 43, accels_in_range);
    _mav_put_uint8_t(buf, 44, calibration);
    _mav_put_float_array(buf, 12, mag, 3);
    _mav_put_float_array(buf, 24, accel, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_MAG, buf, MAVLINK_MSG_ID_SBG_MAG_MIN_LEN, MAVLINK_MSG_ID_SBG_MAG_LEN, MAVLINK_MSG_ID_SBG_MAG_CRC);
#else
    mavlink_sbg_mag_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.mag_x = mag_x;
    packet.mag_y = mag_y;
    packet.mag_z = mag_z;
    packet.accel_x = accel_x;
    packet.accel_y = accel_y;
    packet.accel_z = accel_z;
    packet.mags_in_range = mags_in_range;
    packet.accels_in_range = accels_in_range;
    packet.calibration = calibration;
    mav_array_memcpy(packet.mag, mag, sizeof(float)*3);
    mav_array_memcpy(packet.accel, accel, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_MAG, (const char *)&packet, MAVLINK_MSG_ID_SBG_MAG_MIN_LEN, MAVLINK_MSG_ID_SBG_MAG_LEN, MAVLINK_MSG_ID_SBG_MAG_CRC);
#endif
}

/**
 * @brief Send a sbg_mag message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sbg_mag_send_struct(mavlink_channel_t chan, const mavlink_sbg_mag_t* sbg_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sbg_mag_send(chan, sbg_mag->timestamp, sbg_mag->time_stamp, sbg_mag->mag, sbg_mag->accel, sbg_mag->mag_x, sbg_mag->mag_y, sbg_mag->mag_z, sbg_mag->accel_x, sbg_mag->accel_y, sbg_mag->accel_z, sbg_mag->mags_in_range, sbg_mag->accels_in_range, sbg_mag->calibration);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_MAG, (const char *)sbg_mag, MAVLINK_MSG_ID_SBG_MAG_MIN_LEN, MAVLINK_MSG_ID_SBG_MAG_LEN, MAVLINK_MSG_ID_SBG_MAG_CRC);
#endif
}

#if MAVLINK_MSG_ID_SBG_MAG_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sbg_mag_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t time_stamp, const float *mag, const float *accel, uint8_t mag_x, uint8_t mag_y, uint8_t mag_z, uint8_t accel_x, uint8_t accel_y, uint8_t accel_z, uint8_t mags_in_range, uint8_t accels_in_range, uint8_t calibration)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint8_t(buf, 36, mag_x);
    _mav_put_uint8_t(buf, 37, mag_y);
    _mav_put_uint8_t(buf, 38, mag_z);
    _mav_put_uint8_t(buf, 39, accel_x);
    _mav_put_uint8_t(buf, 40, accel_y);
    _mav_put_uint8_t(buf, 41, accel_z);
    _mav_put_uint8_t(buf, 42, mags_in_range);
    _mav_put_uint8_t(buf, 43, accels_in_range);
    _mav_put_uint8_t(buf, 44, calibration);
    _mav_put_float_array(buf, 12, mag, 3);
    _mav_put_float_array(buf, 24, accel, 3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_MAG, buf, MAVLINK_MSG_ID_SBG_MAG_MIN_LEN, MAVLINK_MSG_ID_SBG_MAG_LEN, MAVLINK_MSG_ID_SBG_MAG_CRC);
#else
    mavlink_sbg_mag_t *packet = (mavlink_sbg_mag_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->time_stamp = time_stamp;
    packet->mag_x = mag_x;
    packet->mag_y = mag_y;
    packet->mag_z = mag_z;
    packet->accel_x = accel_x;
    packet->accel_y = accel_y;
    packet->accel_z = accel_z;
    packet->mags_in_range = mags_in_range;
    packet->accels_in_range = accels_in_range;
    packet->calibration = calibration;
    mav_array_memcpy(packet->mag, mag, sizeof(float)*3);
    mav_array_memcpy(packet->accel, accel, sizeof(float)*3);
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_MAG, (const char *)packet, MAVLINK_MSG_ID_SBG_MAG_MIN_LEN, MAVLINK_MSG_ID_SBG_MAG_LEN, MAVLINK_MSG_ID_SBG_MAG_CRC);
#endif
}
#endif

#endif

// MESSAGE SBG_MAG UNPACKING


/**
 * @brief Get field timestamp from sbg_mag message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_sbg_mag_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_stamp from sbg_mag message
 *
 * @return [us] Time since sensor is powered up us.
 */
static inline uint32_t mavlink_msg_sbg_mag_get_time_stamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field mag from sbg_mag message
 *
 * @return  Magnetometer output (X, Y, Z).
 */
static inline uint16_t mavlink_msg_sbg_mag_get_mag(const mavlink_message_t* msg, float *mag)
{
    return _MAV_RETURN_float_array(msg, mag, 3,  12);
}

/**
 * @brief Get field accel from sbg_mag message
 *
 * @return [m/s2] Accelerometer output (X, Y, Z) in m/s2.
 */
static inline uint16_t mavlink_msg_sbg_mag_get_accel(const mavlink_message_t* msg, float *accel)
{
    return _MAV_RETURN_float_array(msg, accel, 3,  24);
}

/**
 * @brief Get field mag_x from sbg_mag message
 *
 * @return  True if the magnetometer X has passed the self test.
 */
static inline uint8_t mavlink_msg_sbg_mag_get_mag_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field mag_y from sbg_mag message
 *
 * @return  True if the magnetometer Y has passed the self test.
 */
static inline uint8_t mavlink_msg_sbg_mag_get_mag_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field mag_z from sbg_mag message
 *
 * @return  True if the magnetometer Z has passed the self test.
 */
static inline uint8_t mavlink_msg_sbg_mag_get_mag_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field accel_x from sbg_mag message
 *
 * @return  True if the accelerometer X has passed the self test.
 */
static inline uint8_t mavlink_msg_sbg_mag_get_accel_x(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field accel_y from sbg_mag message
 *
 * @return  True if the accelerometer Y has passed the self test.
 */
static inline uint8_t mavlink_msg_sbg_mag_get_accel_y(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field accel_z from sbg_mag message
 *
 * @return  True if the accelerometer Z has passed the self test.
 */
static inline uint8_t mavlink_msg_sbg_mag_get_accel_z(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Get field mags_in_range from sbg_mag message
 *
 * @return  True if magnetometer is not saturated
 */
static inline uint8_t mavlink_msg_sbg_mag_get_mags_in_range(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  42);
}

/**
 * @brief Get field accels_in_range from sbg_mag message
 *
 * @return  True if accelerometer is not saturated
 */
static inline uint8_t mavlink_msg_sbg_mag_get_accels_in_range(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  43);
}

/**
 * @brief Get field calibration from sbg_mag message
 *
 * @return  True if magnetometer seems to be calibrated
 */
static inline uint8_t mavlink_msg_sbg_mag_get_calibration(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  44);
}

/**
 * @brief Decode a sbg_mag message into a struct
 *
 * @param msg The message to decode
 * @param sbg_mag C-struct to decode the message contents into
 */
static inline void mavlink_msg_sbg_mag_decode(const mavlink_message_t* msg, mavlink_sbg_mag_t* sbg_mag)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sbg_mag->timestamp = mavlink_msg_sbg_mag_get_timestamp(msg);
    sbg_mag->time_stamp = mavlink_msg_sbg_mag_get_time_stamp(msg);
    mavlink_msg_sbg_mag_get_mag(msg, sbg_mag->mag);
    mavlink_msg_sbg_mag_get_accel(msg, sbg_mag->accel);
    sbg_mag->mag_x = mavlink_msg_sbg_mag_get_mag_x(msg);
    sbg_mag->mag_y = mavlink_msg_sbg_mag_get_mag_y(msg);
    sbg_mag->mag_z = mavlink_msg_sbg_mag_get_mag_z(msg);
    sbg_mag->accel_x = mavlink_msg_sbg_mag_get_accel_x(msg);
    sbg_mag->accel_y = mavlink_msg_sbg_mag_get_accel_y(msg);
    sbg_mag->accel_z = mavlink_msg_sbg_mag_get_accel_z(msg);
    sbg_mag->mags_in_range = mavlink_msg_sbg_mag_get_mags_in_range(msg);
    sbg_mag->accels_in_range = mavlink_msg_sbg_mag_get_accels_in_range(msg);
    sbg_mag->calibration = mavlink_msg_sbg_mag_get_calibration(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SBG_MAG_LEN? msg->len : MAVLINK_MSG_ID_SBG_MAG_LEN;
        memset(sbg_mag, 0, MAVLINK_MSG_ID_SBG_MAG_LEN);
    memcpy(sbg_mag, _MAV_PAYLOAD(msg), len);
#endif
}
