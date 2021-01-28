#pragma once
// MESSAGE SBG_UTC_TIME PACKING

#define MAVLINK_MSG_ID_SBG_UTC_TIME 633


typedef struct __mavlink_sbg_utc_time_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 uint32_t time_stamp; /*< [us] Time since sensor is powered up us.*/
 uint32_t nanosec; /*<  Nanosecond of second.*/
 uint32_t gps_tow; /*< [ms] GPS Time of Week ms.*/
 uint16_t year; /*<  Year.*/
 uint8_t clock_stable; /*<  True when a clock input can be used to synchronize the internal clock.*/
 uint8_t clock_status; /*<  Define the internal clock estimation status. 0 error, 1 internal crystal, 2 PPS detected and the clock is converging to it, 3 Clock has converged to the PPS and is within 500ns.*/
 uint8_t clock_utc_sync; /*<  True if UTC time is synchronized with a PPS.*/
 uint8_t clock_utc_status; /*<  UTC validity status. 0 The UTC time is not known, 1 received valid UTC but we don't have the leap seconds information, 2 received valid UTC time data with valid leap seconds*/
 uint8_t month; /*<  Month in Year [1 ... 12].*/
 uint8_t day; /*<  Day in Month [1 ... 31].*/
 uint8_t hour; /*<  Hour in day [0 ... 23].*/
 uint8_t min; /*<  Minute in hour [0 ... 59].*/
 uint8_t sec; /*<  Second in minute [0 ... 60], Note 60 is when a leap second is added.*/
} mavlink_sbg_utc_time_t;

#define MAVLINK_MSG_ID_SBG_UTC_TIME_LEN 31
#define MAVLINK_MSG_ID_SBG_UTC_TIME_MIN_LEN 31
#define MAVLINK_MSG_ID_633_LEN 31
#define MAVLINK_MSG_ID_633_MIN_LEN 31

#define MAVLINK_MSG_ID_SBG_UTC_TIME_CRC 66
#define MAVLINK_MSG_ID_633_CRC 66



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SBG_UTC_TIME { \
    633, \
    "SBG_UTC_TIME", \
    14, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_utc_time_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_utc_time_t, time_stamp) }, \
         { "clock_stable", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_sbg_utc_time_t, clock_stable) }, \
         { "clock_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_sbg_utc_time_t, clock_status) }, \
         { "clock_utc_sync", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_sbg_utc_time_t, clock_utc_sync) }, \
         { "clock_utc_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_sbg_utc_time_t, clock_utc_status) }, \
         { "year", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_sbg_utc_time_t, year) }, \
         { "month", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_sbg_utc_time_t, month) }, \
         { "day", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_sbg_utc_time_t, day) }, \
         { "hour", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_sbg_utc_time_t, hour) }, \
         { "min", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_sbg_utc_time_t, min) }, \
         { "sec", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_sbg_utc_time_t, sec) }, \
         { "nanosec", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_sbg_utc_time_t, nanosec) }, \
         { "gps_tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_sbg_utc_time_t, gps_tow) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SBG_UTC_TIME { \
    "SBG_UTC_TIME", \
    14, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_utc_time_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_utc_time_t, time_stamp) }, \
         { "clock_stable", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_sbg_utc_time_t, clock_stable) }, \
         { "clock_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_sbg_utc_time_t, clock_status) }, \
         { "clock_utc_sync", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_sbg_utc_time_t, clock_utc_sync) }, \
         { "clock_utc_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_sbg_utc_time_t, clock_utc_status) }, \
         { "year", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_sbg_utc_time_t, year) }, \
         { "month", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_sbg_utc_time_t, month) }, \
         { "day", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_sbg_utc_time_t, day) }, \
         { "hour", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_sbg_utc_time_t, hour) }, \
         { "min", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_sbg_utc_time_t, min) }, \
         { "sec", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_sbg_utc_time_t, sec) }, \
         { "nanosec", NULL, MAVLINK_TYPE_UINT32_T, 0, 12, offsetof(mavlink_sbg_utc_time_t, nanosec) }, \
         { "gps_tow", NULL, MAVLINK_TYPE_UINT32_T, 0, 16, offsetof(mavlink_sbg_utc_time_t, gps_tow) }, \
         } \
}
#endif

/**
 * @brief Pack a sbg_utc_time message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param clock_stable  True when a clock input can be used to synchronize the internal clock.
 * @param clock_status  Define the internal clock estimation status. 0 error, 1 internal crystal, 2 PPS detected and the clock is converging to it, 3 Clock has converged to the PPS and is within 500ns.
 * @param clock_utc_sync  True if UTC time is synchronized with a PPS.
 * @param clock_utc_status  UTC validity status. 0 The UTC time is not known, 1 received valid UTC but we don't have the leap seconds information, 2 received valid UTC time data with valid leap seconds
 * @param year  Year.
 * @param month  Month in Year [1 ... 12].
 * @param day  Day in Month [1 ... 31].
 * @param hour  Hour in day [0 ... 23].
 * @param min  Minute in hour [0 ... 59].
 * @param sec  Second in minute [0 ... 60], Note 60 is when a leap second is added.
 * @param nanosec  Nanosecond of second.
 * @param gps_tow [ms] GPS Time of Week ms.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_utc_time_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t time_stamp, uint8_t clock_stable, uint8_t clock_status, uint8_t clock_utc_sync, uint8_t clock_utc_status, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint32_t nanosec, uint32_t gps_tow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_UTC_TIME_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, nanosec);
    _mav_put_uint32_t(buf, 16, gps_tow);
    _mav_put_uint16_t(buf, 20, year);
    _mav_put_uint8_t(buf, 22, clock_stable);
    _mav_put_uint8_t(buf, 23, clock_status);
    _mav_put_uint8_t(buf, 24, clock_utc_sync);
    _mav_put_uint8_t(buf, 25, clock_utc_status);
    _mav_put_uint8_t(buf, 26, month);
    _mav_put_uint8_t(buf, 27, day);
    _mav_put_uint8_t(buf, 28, hour);
    _mav_put_uint8_t(buf, 29, min);
    _mav_put_uint8_t(buf, 30, sec);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_UTC_TIME_LEN);
#else
    mavlink_sbg_utc_time_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.nanosec = nanosec;
    packet.gps_tow = gps_tow;
    packet.year = year;
    packet.clock_stable = clock_stable;
    packet.clock_status = clock_status;
    packet.clock_utc_sync = clock_utc_sync;
    packet.clock_utc_status = clock_utc_status;
    packet.month = month;
    packet.day = day;
    packet.hour = hour;
    packet.min = min;
    packet.sec = sec;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_UTC_TIME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_UTC_TIME;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SBG_UTC_TIME_MIN_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_CRC);
}

/**
 * @brief Pack a sbg_utc_time message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param clock_stable  True when a clock input can be used to synchronize the internal clock.
 * @param clock_status  Define the internal clock estimation status. 0 error, 1 internal crystal, 2 PPS detected and the clock is converging to it, 3 Clock has converged to the PPS and is within 500ns.
 * @param clock_utc_sync  True if UTC time is synchronized with a PPS.
 * @param clock_utc_status  UTC validity status. 0 The UTC time is not known, 1 received valid UTC but we don't have the leap seconds information, 2 received valid UTC time data with valid leap seconds
 * @param year  Year.
 * @param month  Month in Year [1 ... 12].
 * @param day  Day in Month [1 ... 31].
 * @param hour  Hour in day [0 ... 23].
 * @param min  Minute in hour [0 ... 59].
 * @param sec  Second in minute [0 ... 60], Note 60 is when a leap second is added.
 * @param nanosec  Nanosecond of second.
 * @param gps_tow [ms] GPS Time of Week ms.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_utc_time_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t time_stamp,uint8_t clock_stable,uint8_t clock_status,uint8_t clock_utc_sync,uint8_t clock_utc_status,uint16_t year,uint8_t month,uint8_t day,uint8_t hour,uint8_t min,uint8_t sec,uint32_t nanosec,uint32_t gps_tow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_UTC_TIME_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, nanosec);
    _mav_put_uint32_t(buf, 16, gps_tow);
    _mav_put_uint16_t(buf, 20, year);
    _mav_put_uint8_t(buf, 22, clock_stable);
    _mav_put_uint8_t(buf, 23, clock_status);
    _mav_put_uint8_t(buf, 24, clock_utc_sync);
    _mav_put_uint8_t(buf, 25, clock_utc_status);
    _mav_put_uint8_t(buf, 26, month);
    _mav_put_uint8_t(buf, 27, day);
    _mav_put_uint8_t(buf, 28, hour);
    _mav_put_uint8_t(buf, 29, min);
    _mav_put_uint8_t(buf, 30, sec);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_UTC_TIME_LEN);
#else
    mavlink_sbg_utc_time_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.nanosec = nanosec;
    packet.gps_tow = gps_tow;
    packet.year = year;
    packet.clock_stable = clock_stable;
    packet.clock_status = clock_status;
    packet.clock_utc_sync = clock_utc_sync;
    packet.clock_utc_status = clock_utc_status;
    packet.month = month;
    packet.day = day;
    packet.hour = hour;
    packet.min = min;
    packet.sec = sec;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_UTC_TIME_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_UTC_TIME;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SBG_UTC_TIME_MIN_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_CRC);
}

/**
 * @brief Encode a sbg_utc_time struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbg_utc_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_utc_time_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbg_utc_time_t* sbg_utc_time)
{
    return mavlink_msg_sbg_utc_time_pack(system_id, component_id, msg, sbg_utc_time->timestamp, sbg_utc_time->time_stamp, sbg_utc_time->clock_stable, sbg_utc_time->clock_status, sbg_utc_time->clock_utc_sync, sbg_utc_time->clock_utc_status, sbg_utc_time->year, sbg_utc_time->month, sbg_utc_time->day, sbg_utc_time->hour, sbg_utc_time->min, sbg_utc_time->sec, sbg_utc_time->nanosec, sbg_utc_time->gps_tow);
}

/**
 * @brief Encode a sbg_utc_time struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sbg_utc_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_utc_time_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sbg_utc_time_t* sbg_utc_time)
{
    return mavlink_msg_sbg_utc_time_pack_chan(system_id, component_id, chan, msg, sbg_utc_time->timestamp, sbg_utc_time->time_stamp, sbg_utc_time->clock_stable, sbg_utc_time->clock_status, sbg_utc_time->clock_utc_sync, sbg_utc_time->clock_utc_status, sbg_utc_time->year, sbg_utc_time->month, sbg_utc_time->day, sbg_utc_time->hour, sbg_utc_time->min, sbg_utc_time->sec, sbg_utc_time->nanosec, sbg_utc_time->gps_tow);
}

/**
 * @brief Send a sbg_utc_time message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param clock_stable  True when a clock input can be used to synchronize the internal clock.
 * @param clock_status  Define the internal clock estimation status. 0 error, 1 internal crystal, 2 PPS detected and the clock is converging to it, 3 Clock has converged to the PPS and is within 500ns.
 * @param clock_utc_sync  True if UTC time is synchronized with a PPS.
 * @param clock_utc_status  UTC validity status. 0 The UTC time is not known, 1 received valid UTC but we don't have the leap seconds information, 2 received valid UTC time data with valid leap seconds
 * @param year  Year.
 * @param month  Month in Year [1 ... 12].
 * @param day  Day in Month [1 ... 31].
 * @param hour  Hour in day [0 ... 23].
 * @param min  Minute in hour [0 ... 59].
 * @param sec  Second in minute [0 ... 60], Note 60 is when a leap second is added.
 * @param nanosec  Nanosecond of second.
 * @param gps_tow [ms] GPS Time of Week ms.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sbg_utc_time_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t time_stamp, uint8_t clock_stable, uint8_t clock_status, uint8_t clock_utc_sync, uint8_t clock_utc_status, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint32_t nanosec, uint32_t gps_tow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_UTC_TIME_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, nanosec);
    _mav_put_uint32_t(buf, 16, gps_tow);
    _mav_put_uint16_t(buf, 20, year);
    _mav_put_uint8_t(buf, 22, clock_stable);
    _mav_put_uint8_t(buf, 23, clock_status);
    _mav_put_uint8_t(buf, 24, clock_utc_sync);
    _mav_put_uint8_t(buf, 25, clock_utc_status);
    _mav_put_uint8_t(buf, 26, month);
    _mav_put_uint8_t(buf, 27, day);
    _mav_put_uint8_t(buf, 28, hour);
    _mav_put_uint8_t(buf, 29, min);
    _mav_put_uint8_t(buf, 30, sec);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_UTC_TIME, buf, MAVLINK_MSG_ID_SBG_UTC_TIME_MIN_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_CRC);
#else
    mavlink_sbg_utc_time_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.nanosec = nanosec;
    packet.gps_tow = gps_tow;
    packet.year = year;
    packet.clock_stable = clock_stable;
    packet.clock_status = clock_status;
    packet.clock_utc_sync = clock_utc_sync;
    packet.clock_utc_status = clock_utc_status;
    packet.month = month;
    packet.day = day;
    packet.hour = hour;
    packet.min = min;
    packet.sec = sec;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_UTC_TIME, (const char *)&packet, MAVLINK_MSG_ID_SBG_UTC_TIME_MIN_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_CRC);
#endif
}

/**
 * @brief Send a sbg_utc_time message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sbg_utc_time_send_struct(mavlink_channel_t chan, const mavlink_sbg_utc_time_t* sbg_utc_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sbg_utc_time_send(chan, sbg_utc_time->timestamp, sbg_utc_time->time_stamp, sbg_utc_time->clock_stable, sbg_utc_time->clock_status, sbg_utc_time->clock_utc_sync, sbg_utc_time->clock_utc_status, sbg_utc_time->year, sbg_utc_time->month, sbg_utc_time->day, sbg_utc_time->hour, sbg_utc_time->min, sbg_utc_time->sec, sbg_utc_time->nanosec, sbg_utc_time->gps_tow);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_UTC_TIME, (const char *)sbg_utc_time, MAVLINK_MSG_ID_SBG_UTC_TIME_MIN_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_CRC);
#endif
}

#if MAVLINK_MSG_ID_SBG_UTC_TIME_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sbg_utc_time_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t time_stamp, uint8_t clock_stable, uint8_t clock_status, uint8_t clock_utc_sync, uint8_t clock_utc_status, uint16_t year, uint8_t month, uint8_t day, uint8_t hour, uint8_t min, uint8_t sec, uint32_t nanosec, uint32_t gps_tow)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint32_t(buf, 12, nanosec);
    _mav_put_uint32_t(buf, 16, gps_tow);
    _mav_put_uint16_t(buf, 20, year);
    _mav_put_uint8_t(buf, 22, clock_stable);
    _mav_put_uint8_t(buf, 23, clock_status);
    _mav_put_uint8_t(buf, 24, clock_utc_sync);
    _mav_put_uint8_t(buf, 25, clock_utc_status);
    _mav_put_uint8_t(buf, 26, month);
    _mav_put_uint8_t(buf, 27, day);
    _mav_put_uint8_t(buf, 28, hour);
    _mav_put_uint8_t(buf, 29, min);
    _mav_put_uint8_t(buf, 30, sec);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_UTC_TIME, buf, MAVLINK_MSG_ID_SBG_UTC_TIME_MIN_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_CRC);
#else
    mavlink_sbg_utc_time_t *packet = (mavlink_sbg_utc_time_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->time_stamp = time_stamp;
    packet->nanosec = nanosec;
    packet->gps_tow = gps_tow;
    packet->year = year;
    packet->clock_stable = clock_stable;
    packet->clock_status = clock_status;
    packet->clock_utc_sync = clock_utc_sync;
    packet->clock_utc_status = clock_utc_status;
    packet->month = month;
    packet->day = day;
    packet->hour = hour;
    packet->min = min;
    packet->sec = sec;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_UTC_TIME, (const char *)packet, MAVLINK_MSG_ID_SBG_UTC_TIME_MIN_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_LEN, MAVLINK_MSG_ID_SBG_UTC_TIME_CRC);
#endif
}
#endif

#endif

// MESSAGE SBG_UTC_TIME UNPACKING


/**
 * @brief Get field timestamp from sbg_utc_time message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_sbg_utc_time_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_stamp from sbg_utc_time message
 *
 * @return [us] Time since sensor is powered up us.
 */
static inline uint32_t mavlink_msg_sbg_utc_time_get_time_stamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field clock_stable from sbg_utc_time message
 *
 * @return  True when a clock input can be used to synchronize the internal clock.
 */
static inline uint8_t mavlink_msg_sbg_utc_time_get_clock_stable(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field clock_status from sbg_utc_time message
 *
 * @return  Define the internal clock estimation status. 0 error, 1 internal crystal, 2 PPS detected and the clock is converging to it, 3 Clock has converged to the PPS and is within 500ns.
 */
static inline uint8_t mavlink_msg_sbg_utc_time_get_clock_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field clock_utc_sync from sbg_utc_time message
 *
 * @return  True if UTC time is synchronized with a PPS.
 */
static inline uint8_t mavlink_msg_sbg_utc_time_get_clock_utc_sync(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field clock_utc_status from sbg_utc_time message
 *
 * @return  UTC validity status. 0 The UTC time is not known, 1 received valid UTC but we don't have the leap seconds information, 2 received valid UTC time data with valid leap seconds
 */
static inline uint8_t mavlink_msg_sbg_utc_time_get_clock_utc_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field year from sbg_utc_time message
 *
 * @return  Year.
 */
static inline uint16_t mavlink_msg_sbg_utc_time_get_year(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field month from sbg_utc_time message
 *
 * @return  Month in Year [1 ... 12].
 */
static inline uint8_t mavlink_msg_sbg_utc_time_get_month(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field day from sbg_utc_time message
 *
 * @return  Day in Month [1 ... 31].
 */
static inline uint8_t mavlink_msg_sbg_utc_time_get_day(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field hour from sbg_utc_time message
 *
 * @return  Hour in day [0 ... 23].
 */
static inline uint8_t mavlink_msg_sbg_utc_time_get_hour(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field min from sbg_utc_time message
 *
 * @return  Minute in hour [0 ... 59].
 */
static inline uint8_t mavlink_msg_sbg_utc_time_get_min(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field sec from sbg_utc_time message
 *
 * @return  Second in minute [0 ... 60], Note 60 is when a leap second is added.
 */
static inline uint8_t mavlink_msg_sbg_utc_time_get_sec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field nanosec from sbg_utc_time message
 *
 * @return  Nanosecond of second.
 */
static inline uint32_t mavlink_msg_sbg_utc_time_get_nanosec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  12);
}

/**
 * @brief Get field gps_tow from sbg_utc_time message
 *
 * @return [ms] GPS Time of Week ms.
 */
static inline uint32_t mavlink_msg_sbg_utc_time_get_gps_tow(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  16);
}

/**
 * @brief Decode a sbg_utc_time message into a struct
 *
 * @param msg The message to decode
 * @param sbg_utc_time C-struct to decode the message contents into
 */
static inline void mavlink_msg_sbg_utc_time_decode(const mavlink_message_t* msg, mavlink_sbg_utc_time_t* sbg_utc_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sbg_utc_time->timestamp = mavlink_msg_sbg_utc_time_get_timestamp(msg);
    sbg_utc_time->time_stamp = mavlink_msg_sbg_utc_time_get_time_stamp(msg);
    sbg_utc_time->nanosec = mavlink_msg_sbg_utc_time_get_nanosec(msg);
    sbg_utc_time->gps_tow = mavlink_msg_sbg_utc_time_get_gps_tow(msg);
    sbg_utc_time->year = mavlink_msg_sbg_utc_time_get_year(msg);
    sbg_utc_time->clock_stable = mavlink_msg_sbg_utc_time_get_clock_stable(msg);
    sbg_utc_time->clock_status = mavlink_msg_sbg_utc_time_get_clock_status(msg);
    sbg_utc_time->clock_utc_sync = mavlink_msg_sbg_utc_time_get_clock_utc_sync(msg);
    sbg_utc_time->clock_utc_status = mavlink_msg_sbg_utc_time_get_clock_utc_status(msg);
    sbg_utc_time->month = mavlink_msg_sbg_utc_time_get_month(msg);
    sbg_utc_time->day = mavlink_msg_sbg_utc_time_get_day(msg);
    sbg_utc_time->hour = mavlink_msg_sbg_utc_time_get_hour(msg);
    sbg_utc_time->min = mavlink_msg_sbg_utc_time_get_min(msg);
    sbg_utc_time->sec = mavlink_msg_sbg_utc_time_get_sec(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SBG_UTC_TIME_LEN? msg->len : MAVLINK_MSG_ID_SBG_UTC_TIME_LEN;
        memset(sbg_utc_time, 0, MAVLINK_MSG_ID_SBG_UTC_TIME_LEN);
    memcpy(sbg_utc_time, _MAV_PAYLOAD(msg), len);
#endif
}
