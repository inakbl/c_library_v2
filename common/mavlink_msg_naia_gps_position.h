#pragma once
// MESSAGE NAIA_GPS_POSITION PACKING

#define MAVLINK_MSG_ID_NAIA_GPS_POSITION 620


typedef struct __mavlink_naia_gps_position_t {
 uint64_t time_utc_usec; /*< [us] Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0*/
 int32_t lat; /*<  Latitude in 1E-7 degrees*/
 int32_t lon; /*<  Longitude in 1E-7 degrees*/
 int32_t alt; /*< [mm] Altitude in 1E-3 meters above MSL, (millimetres)*/
 int32_t alt_ellipsoid; /*< [mm] Altitude in 1E-3 meters bove Ellipsoid, (millimetres)*/
 float s_variance_m_s; /*< [m/s] GPS speed accuracy estimate, (metres/sec)*/
 float c_variance_rad; /*< [rad] GPS course accuracy estimate, (radians)*/
 float eph; /*< [m] GPS horizontal position accuracy (metres)*/
 float epv; /*< [m] GPS vertical position accuracy (metres)*/
 float hdop; /*<  Horizontal dilution of precision*/
 float vdop; /*<  Vertical dilution of precision*/
 int32_t noise_per_ms; /*<  GPS noise per millisecond*/
 int32_t jamming_indicator; /*<  indicates jamming is occurring*/
 float vel_m_s; /*< [m/s] GPS ground speed, (metres/sec)*/
 float vel_n_m_s; /*< [m/s] GPS North velocity, (metres/sec)*/
 float vel_e_m_s; /*< [m/s] GPS East velocity, (metres/sec)*/
 float vel_d_m_s; /*< [m/s] GPS Down velocity, (metres/sec)*/
 float cog_rad; /*< [rad] Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)*/
 int32_t timestamp_time_relative; /*< [us] timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)*/
 float heading; /*< [rad] heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])*/
 float heading_offset; /*< [rad] heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])*/
 uint8_t fix_type; /*<  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
 uint8_t vel_ned_valid; /*<  True if NED velocity is valid*/
 uint8_t satellites_used; /*<  Number of satellites used*/
} mavlink_naia_gps_position_t;

#define MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN 91
#define MAVLINK_MSG_ID_NAIA_GPS_POSITION_MIN_LEN 91
#define MAVLINK_MSG_ID_620_LEN 91
#define MAVLINK_MSG_ID_620_MIN_LEN 91

#define MAVLINK_MSG_ID_NAIA_GPS_POSITION_CRC 199
#define MAVLINK_MSG_ID_620_CRC 199



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_NAIA_GPS_POSITION { \
    620, \
    "NAIA_GPS_POSITION", \
    24, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_naia_gps_position_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_naia_gps_position_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_naia_gps_position_t, alt) }, \
         { "alt_ellipsoid", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_naia_gps_position_t, alt_ellipsoid) }, \
         { "s_variance_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_naia_gps_position_t, s_variance_m_s) }, \
         { "c_variance_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_naia_gps_position_t, c_variance_rad) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 88, offsetof(mavlink_naia_gps_position_t, fix_type) }, \
         { "eph", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_naia_gps_position_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_naia_gps_position_t, epv) }, \
         { "hdop", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_naia_gps_position_t, hdop) }, \
         { "vdop", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_naia_gps_position_t, vdop) }, \
         { "noise_per_ms", NULL, MAVLINK_TYPE_INT32_T, 0, 48, offsetof(mavlink_naia_gps_position_t, noise_per_ms) }, \
         { "jamming_indicator", NULL, MAVLINK_TYPE_INT32_T, 0, 52, offsetof(mavlink_naia_gps_position_t, jamming_indicator) }, \
         { "vel_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_naia_gps_position_t, vel_m_s) }, \
         { "vel_n_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_naia_gps_position_t, vel_n_m_s) }, \
         { "vel_e_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_naia_gps_position_t, vel_e_m_s) }, \
         { "vel_d_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_naia_gps_position_t, vel_d_m_s) }, \
         { "cog_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_naia_gps_position_t, cog_rad) }, \
         { "vel_ned_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 89, offsetof(mavlink_naia_gps_position_t, vel_ned_valid) }, \
         { "timestamp_time_relative", NULL, MAVLINK_TYPE_INT32_T, 0, 76, offsetof(mavlink_naia_gps_position_t, timestamp_time_relative) }, \
         { "time_utc_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_naia_gps_position_t, time_utc_usec) }, \
         { "satellites_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 90, offsetof(mavlink_naia_gps_position_t, satellites_used) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_naia_gps_position_t, heading) }, \
         { "heading_offset", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_naia_gps_position_t, heading_offset) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_NAIA_GPS_POSITION { \
    "NAIA_GPS_POSITION", \
    24, \
    {  { "lat", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_naia_gps_position_t, lat) }, \
         { "lon", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_naia_gps_position_t, lon) }, \
         { "alt", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_naia_gps_position_t, alt) }, \
         { "alt_ellipsoid", NULL, MAVLINK_TYPE_INT32_T, 0, 20, offsetof(mavlink_naia_gps_position_t, alt_ellipsoid) }, \
         { "s_variance_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_naia_gps_position_t, s_variance_m_s) }, \
         { "c_variance_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_naia_gps_position_t, c_variance_rad) }, \
         { "fix_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 88, offsetof(mavlink_naia_gps_position_t, fix_type) }, \
         { "eph", NULL, MAVLINK_TYPE_FLOAT, 0, 32, offsetof(mavlink_naia_gps_position_t, eph) }, \
         { "epv", NULL, MAVLINK_TYPE_FLOAT, 0, 36, offsetof(mavlink_naia_gps_position_t, epv) }, \
         { "hdop", NULL, MAVLINK_TYPE_FLOAT, 0, 40, offsetof(mavlink_naia_gps_position_t, hdop) }, \
         { "vdop", NULL, MAVLINK_TYPE_FLOAT, 0, 44, offsetof(mavlink_naia_gps_position_t, vdop) }, \
         { "noise_per_ms", NULL, MAVLINK_TYPE_INT32_T, 0, 48, offsetof(mavlink_naia_gps_position_t, noise_per_ms) }, \
         { "jamming_indicator", NULL, MAVLINK_TYPE_INT32_T, 0, 52, offsetof(mavlink_naia_gps_position_t, jamming_indicator) }, \
         { "vel_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 56, offsetof(mavlink_naia_gps_position_t, vel_m_s) }, \
         { "vel_n_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 60, offsetof(mavlink_naia_gps_position_t, vel_n_m_s) }, \
         { "vel_e_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 64, offsetof(mavlink_naia_gps_position_t, vel_e_m_s) }, \
         { "vel_d_m_s", NULL, MAVLINK_TYPE_FLOAT, 0, 68, offsetof(mavlink_naia_gps_position_t, vel_d_m_s) }, \
         { "cog_rad", NULL, MAVLINK_TYPE_FLOAT, 0, 72, offsetof(mavlink_naia_gps_position_t, cog_rad) }, \
         { "vel_ned_valid", NULL, MAVLINK_TYPE_UINT8_T, 0, 89, offsetof(mavlink_naia_gps_position_t, vel_ned_valid) }, \
         { "timestamp_time_relative", NULL, MAVLINK_TYPE_INT32_T, 0, 76, offsetof(mavlink_naia_gps_position_t, timestamp_time_relative) }, \
         { "time_utc_usec", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_naia_gps_position_t, time_utc_usec) }, \
         { "satellites_used", NULL, MAVLINK_TYPE_UINT8_T, 0, 90, offsetof(mavlink_naia_gps_position_t, satellites_used) }, \
         { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 80, offsetof(mavlink_naia_gps_position_t, heading) }, \
         { "heading_offset", NULL, MAVLINK_TYPE_FLOAT, 0, 84, offsetof(mavlink_naia_gps_position_t, heading_offset) }, \
         } \
}
#endif

/**
 * @brief Pack a naia_gps_position message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param lat  Latitude in 1E-7 degrees
 * @param lon  Longitude in 1E-7 degrees
 * @param alt [mm] Altitude in 1E-3 meters above MSL, (millimetres)
 * @param alt_ellipsoid [mm] Altitude in 1E-3 meters bove Ellipsoid, (millimetres)
 * @param s_variance_m_s [m/s] GPS speed accuracy estimate, (metres/sec)
 * @param c_variance_rad [rad] GPS course accuracy estimate, (radians)
 * @param fix_type  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param eph [m] GPS horizontal position accuracy (metres)
 * @param epv [m] GPS vertical position accuracy (metres)
 * @param hdop  Horizontal dilution of precision
 * @param vdop  Vertical dilution of precision
 * @param noise_per_ms  GPS noise per millisecond
 * @param jamming_indicator  indicates jamming is occurring
 * @param vel_m_s [m/s] GPS ground speed, (metres/sec)
 * @param vel_n_m_s [m/s] GPS North velocity, (metres/sec)
 * @param vel_e_m_s [m/s] GPS East velocity, (metres/sec)
 * @param vel_d_m_s [m/s] GPS Down velocity, (metres/sec)
 * @param cog_rad [rad] Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
 * @param vel_ned_valid  True if NED velocity is valid
 * @param timestamp_time_relative [us] timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
 * @param time_utc_usec [us] Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0
 * @param satellites_used  Number of satellites used
 * @param heading [rad] heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
 * @param heading_offset [rad] heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_gps_position_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               int32_t lat, int32_t lon, int32_t alt, int32_t alt_ellipsoid, float s_variance_m_s, float c_variance_rad, uint8_t fix_type, float eph, float epv, float hdop, float vdop, int32_t noise_per_ms, int32_t jamming_indicator, float vel_m_s, float vel_n_m_s, float vel_e_m_s, float vel_d_m_s, float cog_rad, uint8_t vel_ned_valid, int32_t timestamp_time_relative, uint64_t time_utc_usec, uint8_t satellites_used, float heading, float heading_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN];
    _mav_put_uint64_t(buf, 0, time_utc_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_int32_t(buf, 20, alt_ellipsoid);
    _mav_put_float(buf, 24, s_variance_m_s);
    _mav_put_float(buf, 28, c_variance_rad);
    _mav_put_float(buf, 32, eph);
    _mav_put_float(buf, 36, epv);
    _mav_put_float(buf, 40, hdop);
    _mav_put_float(buf, 44, vdop);
    _mav_put_int32_t(buf, 48, noise_per_ms);
    _mav_put_int32_t(buf, 52, jamming_indicator);
    _mav_put_float(buf, 56, vel_m_s);
    _mav_put_float(buf, 60, vel_n_m_s);
    _mav_put_float(buf, 64, vel_e_m_s);
    _mav_put_float(buf, 68, vel_d_m_s);
    _mav_put_float(buf, 72, cog_rad);
    _mav_put_int32_t(buf, 76, timestamp_time_relative);
    _mav_put_float(buf, 80, heading);
    _mav_put_float(buf, 84, heading_offset);
    _mav_put_uint8_t(buf, 88, fix_type);
    _mav_put_uint8_t(buf, 89, vel_ned_valid);
    _mav_put_uint8_t(buf, 90, satellites_used);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN);
#else
    mavlink_naia_gps_position_t packet;
    packet.time_utc_usec = time_utc_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.s_variance_m_s = s_variance_m_s;
    packet.c_variance_rad = c_variance_rad;
    packet.eph = eph;
    packet.epv = epv;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.noise_per_ms = noise_per_ms;
    packet.jamming_indicator = jamming_indicator;
    packet.vel_m_s = vel_m_s;
    packet.vel_n_m_s = vel_n_m_s;
    packet.vel_e_m_s = vel_e_m_s;
    packet.vel_d_m_s = vel_d_m_s;
    packet.cog_rad = cog_rad;
    packet.timestamp_time_relative = timestamp_time_relative;
    packet.heading = heading;
    packet.heading_offset = heading_offset;
    packet.fix_type = fix_type;
    packet.vel_ned_valid = vel_ned_valid;
    packet.satellites_used = satellites_used;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_GPS_POSITION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NAIA_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_CRC);
}

/**
 * @brief Pack a naia_gps_position message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lat  Latitude in 1E-7 degrees
 * @param lon  Longitude in 1E-7 degrees
 * @param alt [mm] Altitude in 1E-3 meters above MSL, (millimetres)
 * @param alt_ellipsoid [mm] Altitude in 1E-3 meters bove Ellipsoid, (millimetres)
 * @param s_variance_m_s [m/s] GPS speed accuracy estimate, (metres/sec)
 * @param c_variance_rad [rad] GPS course accuracy estimate, (radians)
 * @param fix_type  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param eph [m] GPS horizontal position accuracy (metres)
 * @param epv [m] GPS vertical position accuracy (metres)
 * @param hdop  Horizontal dilution of precision
 * @param vdop  Vertical dilution of precision
 * @param noise_per_ms  GPS noise per millisecond
 * @param jamming_indicator  indicates jamming is occurring
 * @param vel_m_s [m/s] GPS ground speed, (metres/sec)
 * @param vel_n_m_s [m/s] GPS North velocity, (metres/sec)
 * @param vel_e_m_s [m/s] GPS East velocity, (metres/sec)
 * @param vel_d_m_s [m/s] GPS Down velocity, (metres/sec)
 * @param cog_rad [rad] Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
 * @param vel_ned_valid  True if NED velocity is valid
 * @param timestamp_time_relative [us] timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
 * @param time_utc_usec [us] Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0
 * @param satellites_used  Number of satellites used
 * @param heading [rad] heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
 * @param heading_offset [rad] heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_naia_gps_position_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   int32_t lat,int32_t lon,int32_t alt,int32_t alt_ellipsoid,float s_variance_m_s,float c_variance_rad,uint8_t fix_type,float eph,float epv,float hdop,float vdop,int32_t noise_per_ms,int32_t jamming_indicator,float vel_m_s,float vel_n_m_s,float vel_e_m_s,float vel_d_m_s,float cog_rad,uint8_t vel_ned_valid,int32_t timestamp_time_relative,uint64_t time_utc_usec,uint8_t satellites_used,float heading,float heading_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN];
    _mav_put_uint64_t(buf, 0, time_utc_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_int32_t(buf, 20, alt_ellipsoid);
    _mav_put_float(buf, 24, s_variance_m_s);
    _mav_put_float(buf, 28, c_variance_rad);
    _mav_put_float(buf, 32, eph);
    _mav_put_float(buf, 36, epv);
    _mav_put_float(buf, 40, hdop);
    _mav_put_float(buf, 44, vdop);
    _mav_put_int32_t(buf, 48, noise_per_ms);
    _mav_put_int32_t(buf, 52, jamming_indicator);
    _mav_put_float(buf, 56, vel_m_s);
    _mav_put_float(buf, 60, vel_n_m_s);
    _mav_put_float(buf, 64, vel_e_m_s);
    _mav_put_float(buf, 68, vel_d_m_s);
    _mav_put_float(buf, 72, cog_rad);
    _mav_put_int32_t(buf, 76, timestamp_time_relative);
    _mav_put_float(buf, 80, heading);
    _mav_put_float(buf, 84, heading_offset);
    _mav_put_uint8_t(buf, 88, fix_type);
    _mav_put_uint8_t(buf, 89, vel_ned_valid);
    _mav_put_uint8_t(buf, 90, satellites_used);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN);
#else
    mavlink_naia_gps_position_t packet;
    packet.time_utc_usec = time_utc_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.s_variance_m_s = s_variance_m_s;
    packet.c_variance_rad = c_variance_rad;
    packet.eph = eph;
    packet.epv = epv;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.noise_per_ms = noise_per_ms;
    packet.jamming_indicator = jamming_indicator;
    packet.vel_m_s = vel_m_s;
    packet.vel_n_m_s = vel_n_m_s;
    packet.vel_e_m_s = vel_e_m_s;
    packet.vel_d_m_s = vel_d_m_s;
    packet.cog_rad = cog_rad;
    packet.timestamp_time_relative = timestamp_time_relative;
    packet.heading = heading;
    packet.heading_offset = heading_offset;
    packet.fix_type = fix_type;
    packet.vel_ned_valid = vel_ned_valid;
    packet.satellites_used = satellites_used;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_NAIA_GPS_POSITION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NAIA_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_CRC);
}

/**
 * @brief Encode a naia_gps_position struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param naia_gps_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_gps_position_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_naia_gps_position_t* naia_gps_position)
{
    return mavlink_msg_naia_gps_position_pack(system_id, component_id, msg, naia_gps_position->lat, naia_gps_position->lon, naia_gps_position->alt, naia_gps_position->alt_ellipsoid, naia_gps_position->s_variance_m_s, naia_gps_position->c_variance_rad, naia_gps_position->fix_type, naia_gps_position->eph, naia_gps_position->epv, naia_gps_position->hdop, naia_gps_position->vdop, naia_gps_position->noise_per_ms, naia_gps_position->jamming_indicator, naia_gps_position->vel_m_s, naia_gps_position->vel_n_m_s, naia_gps_position->vel_e_m_s, naia_gps_position->vel_d_m_s, naia_gps_position->cog_rad, naia_gps_position->vel_ned_valid, naia_gps_position->timestamp_time_relative, naia_gps_position->time_utc_usec, naia_gps_position->satellites_used, naia_gps_position->heading, naia_gps_position->heading_offset);
}

/**
 * @brief Encode a naia_gps_position struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param naia_gps_position C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_naia_gps_position_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_naia_gps_position_t* naia_gps_position)
{
    return mavlink_msg_naia_gps_position_pack_chan(system_id, component_id, chan, msg, naia_gps_position->lat, naia_gps_position->lon, naia_gps_position->alt, naia_gps_position->alt_ellipsoid, naia_gps_position->s_variance_m_s, naia_gps_position->c_variance_rad, naia_gps_position->fix_type, naia_gps_position->eph, naia_gps_position->epv, naia_gps_position->hdop, naia_gps_position->vdop, naia_gps_position->noise_per_ms, naia_gps_position->jamming_indicator, naia_gps_position->vel_m_s, naia_gps_position->vel_n_m_s, naia_gps_position->vel_e_m_s, naia_gps_position->vel_d_m_s, naia_gps_position->cog_rad, naia_gps_position->vel_ned_valid, naia_gps_position->timestamp_time_relative, naia_gps_position->time_utc_usec, naia_gps_position->satellites_used, naia_gps_position->heading, naia_gps_position->heading_offset);
}

/**
 * @brief Send a naia_gps_position message
 * @param chan MAVLink channel to send the message
 *
 * @param lat  Latitude in 1E-7 degrees
 * @param lon  Longitude in 1E-7 degrees
 * @param alt [mm] Altitude in 1E-3 meters above MSL, (millimetres)
 * @param alt_ellipsoid [mm] Altitude in 1E-3 meters bove Ellipsoid, (millimetres)
 * @param s_variance_m_s [m/s] GPS speed accuracy estimate, (metres/sec)
 * @param c_variance_rad [rad] GPS course accuracy estimate, (radians)
 * @param fix_type  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 * @param eph [m] GPS horizontal position accuracy (metres)
 * @param epv [m] GPS vertical position accuracy (metres)
 * @param hdop  Horizontal dilution of precision
 * @param vdop  Vertical dilution of precision
 * @param noise_per_ms  GPS noise per millisecond
 * @param jamming_indicator  indicates jamming is occurring
 * @param vel_m_s [m/s] GPS ground speed, (metres/sec)
 * @param vel_n_m_s [m/s] GPS North velocity, (metres/sec)
 * @param vel_e_m_s [m/s] GPS East velocity, (metres/sec)
 * @param vel_d_m_s [m/s] GPS Down velocity, (metres/sec)
 * @param cog_rad [rad] Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
 * @param vel_ned_valid  True if NED velocity is valid
 * @param timestamp_time_relative [us] timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
 * @param time_utc_usec [us] Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0
 * @param satellites_used  Number of satellites used
 * @param heading [rad] heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
 * @param heading_offset [rad] heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_naia_gps_position_send(mavlink_channel_t chan, int32_t lat, int32_t lon, int32_t alt, int32_t alt_ellipsoid, float s_variance_m_s, float c_variance_rad, uint8_t fix_type, float eph, float epv, float hdop, float vdop, int32_t noise_per_ms, int32_t jamming_indicator, float vel_m_s, float vel_n_m_s, float vel_e_m_s, float vel_d_m_s, float cog_rad, uint8_t vel_ned_valid, int32_t timestamp_time_relative, uint64_t time_utc_usec, uint8_t satellites_used, float heading, float heading_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN];
    _mav_put_uint64_t(buf, 0, time_utc_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_int32_t(buf, 20, alt_ellipsoid);
    _mav_put_float(buf, 24, s_variance_m_s);
    _mav_put_float(buf, 28, c_variance_rad);
    _mav_put_float(buf, 32, eph);
    _mav_put_float(buf, 36, epv);
    _mav_put_float(buf, 40, hdop);
    _mav_put_float(buf, 44, vdop);
    _mav_put_int32_t(buf, 48, noise_per_ms);
    _mav_put_int32_t(buf, 52, jamming_indicator);
    _mav_put_float(buf, 56, vel_m_s);
    _mav_put_float(buf, 60, vel_n_m_s);
    _mav_put_float(buf, 64, vel_e_m_s);
    _mav_put_float(buf, 68, vel_d_m_s);
    _mav_put_float(buf, 72, cog_rad);
    _mav_put_int32_t(buf, 76, timestamp_time_relative);
    _mav_put_float(buf, 80, heading);
    _mav_put_float(buf, 84, heading_offset);
    _mav_put_uint8_t(buf, 88, fix_type);
    _mav_put_uint8_t(buf, 89, vel_ned_valid);
    _mav_put_uint8_t(buf, 90, satellites_used);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_GPS_POSITION, buf, MAVLINK_MSG_ID_NAIA_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_CRC);
#else
    mavlink_naia_gps_position_t packet;
    packet.time_utc_usec = time_utc_usec;
    packet.lat = lat;
    packet.lon = lon;
    packet.alt = alt;
    packet.alt_ellipsoid = alt_ellipsoid;
    packet.s_variance_m_s = s_variance_m_s;
    packet.c_variance_rad = c_variance_rad;
    packet.eph = eph;
    packet.epv = epv;
    packet.hdop = hdop;
    packet.vdop = vdop;
    packet.noise_per_ms = noise_per_ms;
    packet.jamming_indicator = jamming_indicator;
    packet.vel_m_s = vel_m_s;
    packet.vel_n_m_s = vel_n_m_s;
    packet.vel_e_m_s = vel_e_m_s;
    packet.vel_d_m_s = vel_d_m_s;
    packet.cog_rad = cog_rad;
    packet.timestamp_time_relative = timestamp_time_relative;
    packet.heading = heading;
    packet.heading_offset = heading_offset;
    packet.fix_type = fix_type;
    packet.vel_ned_valid = vel_ned_valid;
    packet.satellites_used = satellites_used;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_GPS_POSITION, (const char *)&packet, MAVLINK_MSG_ID_NAIA_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_CRC);
#endif
}

/**
 * @brief Send a naia_gps_position message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_naia_gps_position_send_struct(mavlink_channel_t chan, const mavlink_naia_gps_position_t* naia_gps_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_naia_gps_position_send(chan, naia_gps_position->lat, naia_gps_position->lon, naia_gps_position->alt, naia_gps_position->alt_ellipsoid, naia_gps_position->s_variance_m_s, naia_gps_position->c_variance_rad, naia_gps_position->fix_type, naia_gps_position->eph, naia_gps_position->epv, naia_gps_position->hdop, naia_gps_position->vdop, naia_gps_position->noise_per_ms, naia_gps_position->jamming_indicator, naia_gps_position->vel_m_s, naia_gps_position->vel_n_m_s, naia_gps_position->vel_e_m_s, naia_gps_position->vel_d_m_s, naia_gps_position->cog_rad, naia_gps_position->vel_ned_valid, naia_gps_position->timestamp_time_relative, naia_gps_position->time_utc_usec, naia_gps_position->satellites_used, naia_gps_position->heading, naia_gps_position->heading_offset);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_GPS_POSITION, (const char *)naia_gps_position, MAVLINK_MSG_ID_NAIA_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_CRC);
#endif
}

#if MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_naia_gps_position_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int32_t lat, int32_t lon, int32_t alt, int32_t alt_ellipsoid, float s_variance_m_s, float c_variance_rad, uint8_t fix_type, float eph, float epv, float hdop, float vdop, int32_t noise_per_ms, int32_t jamming_indicator, float vel_m_s, float vel_n_m_s, float vel_e_m_s, float vel_d_m_s, float cog_rad, uint8_t vel_ned_valid, int32_t timestamp_time_relative, uint64_t time_utc_usec, uint8_t satellites_used, float heading, float heading_offset)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, time_utc_usec);
    _mav_put_int32_t(buf, 8, lat);
    _mav_put_int32_t(buf, 12, lon);
    _mav_put_int32_t(buf, 16, alt);
    _mav_put_int32_t(buf, 20, alt_ellipsoid);
    _mav_put_float(buf, 24, s_variance_m_s);
    _mav_put_float(buf, 28, c_variance_rad);
    _mav_put_float(buf, 32, eph);
    _mav_put_float(buf, 36, epv);
    _mav_put_float(buf, 40, hdop);
    _mav_put_float(buf, 44, vdop);
    _mav_put_int32_t(buf, 48, noise_per_ms);
    _mav_put_int32_t(buf, 52, jamming_indicator);
    _mav_put_float(buf, 56, vel_m_s);
    _mav_put_float(buf, 60, vel_n_m_s);
    _mav_put_float(buf, 64, vel_e_m_s);
    _mav_put_float(buf, 68, vel_d_m_s);
    _mav_put_float(buf, 72, cog_rad);
    _mav_put_int32_t(buf, 76, timestamp_time_relative);
    _mav_put_float(buf, 80, heading);
    _mav_put_float(buf, 84, heading_offset);
    _mav_put_uint8_t(buf, 88, fix_type);
    _mav_put_uint8_t(buf, 89, vel_ned_valid);
    _mav_put_uint8_t(buf, 90, satellites_used);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_GPS_POSITION, buf, MAVLINK_MSG_ID_NAIA_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_CRC);
#else
    mavlink_naia_gps_position_t *packet = (mavlink_naia_gps_position_t *)msgbuf;
    packet->time_utc_usec = time_utc_usec;
    packet->lat = lat;
    packet->lon = lon;
    packet->alt = alt;
    packet->alt_ellipsoid = alt_ellipsoid;
    packet->s_variance_m_s = s_variance_m_s;
    packet->c_variance_rad = c_variance_rad;
    packet->eph = eph;
    packet->epv = epv;
    packet->hdop = hdop;
    packet->vdop = vdop;
    packet->noise_per_ms = noise_per_ms;
    packet->jamming_indicator = jamming_indicator;
    packet->vel_m_s = vel_m_s;
    packet->vel_n_m_s = vel_n_m_s;
    packet->vel_e_m_s = vel_e_m_s;
    packet->vel_d_m_s = vel_d_m_s;
    packet->cog_rad = cog_rad;
    packet->timestamp_time_relative = timestamp_time_relative;
    packet->heading = heading;
    packet->heading_offset = heading_offset;
    packet->fix_type = fix_type;
    packet->vel_ned_valid = vel_ned_valid;
    packet->satellites_used = satellites_used;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NAIA_GPS_POSITION, (const char *)packet, MAVLINK_MSG_ID_NAIA_GPS_POSITION_MIN_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN, MAVLINK_MSG_ID_NAIA_GPS_POSITION_CRC);
#endif
}
#endif

#endif

// MESSAGE NAIA_GPS_POSITION UNPACKING


/**
 * @brief Get field lat from naia_gps_position message
 *
 * @return  Latitude in 1E-7 degrees
 */
static inline int32_t mavlink_msg_naia_gps_position_get_lat(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field lon from naia_gps_position message
 *
 * @return  Longitude in 1E-7 degrees
 */
static inline int32_t mavlink_msg_naia_gps_position_get_lon(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field alt from naia_gps_position message
 *
 * @return [mm] Altitude in 1E-3 meters above MSL, (millimetres)
 */
static inline int32_t mavlink_msg_naia_gps_position_get_alt(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field alt_ellipsoid from naia_gps_position message
 *
 * @return [mm] Altitude in 1E-3 meters bove Ellipsoid, (millimetres)
 */
static inline int32_t mavlink_msg_naia_gps_position_get_alt_ellipsoid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  20);
}

/**
 * @brief Get field s_variance_m_s from naia_gps_position message
 *
 * @return [m/s] GPS speed accuracy estimate, (metres/sec)
 */
static inline float mavlink_msg_naia_gps_position_get_s_variance_m_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field c_variance_rad from naia_gps_position message
 *
 * @return [rad] GPS course accuracy estimate, (radians)
 */
static inline float mavlink_msg_naia_gps_position_get_c_variance_rad(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Get field fix_type from naia_gps_position message
 *
 * @return  0-1: no fix, 2: 2D fix, 3: 3D fix, 4: RTCM code differential, 5: Real-Time Kinematic, float, 6: Real-Time Kinematic, fixed, 8: Extrapolated. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.
 */
static inline uint8_t mavlink_msg_naia_gps_position_get_fix_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  88);
}

/**
 * @brief Get field eph from naia_gps_position message
 *
 * @return [m] GPS horizontal position accuracy (metres)
 */
static inline float mavlink_msg_naia_gps_position_get_eph(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  32);
}

/**
 * @brief Get field epv from naia_gps_position message
 *
 * @return [m] GPS vertical position accuracy (metres)
 */
static inline float mavlink_msg_naia_gps_position_get_epv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  36);
}

/**
 * @brief Get field hdop from naia_gps_position message
 *
 * @return  Horizontal dilution of precision
 */
static inline float mavlink_msg_naia_gps_position_get_hdop(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  40);
}

/**
 * @brief Get field vdop from naia_gps_position message
 *
 * @return  Vertical dilution of precision
 */
static inline float mavlink_msg_naia_gps_position_get_vdop(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  44);
}

/**
 * @brief Get field noise_per_ms from naia_gps_position message
 *
 * @return  GPS noise per millisecond
 */
static inline int32_t mavlink_msg_naia_gps_position_get_noise_per_ms(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  48);
}

/**
 * @brief Get field jamming_indicator from naia_gps_position message
 *
 * @return  indicates jamming is occurring
 */
static inline int32_t mavlink_msg_naia_gps_position_get_jamming_indicator(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  52);
}

/**
 * @brief Get field vel_m_s from naia_gps_position message
 *
 * @return [m/s] GPS ground speed, (metres/sec)
 */
static inline float mavlink_msg_naia_gps_position_get_vel_m_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  56);
}

/**
 * @brief Get field vel_n_m_s from naia_gps_position message
 *
 * @return [m/s] GPS North velocity, (metres/sec)
 */
static inline float mavlink_msg_naia_gps_position_get_vel_n_m_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  60);
}

/**
 * @brief Get field vel_e_m_s from naia_gps_position message
 *
 * @return [m/s] GPS East velocity, (metres/sec)
 */
static inline float mavlink_msg_naia_gps_position_get_vel_e_m_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  64);
}

/**
 * @brief Get field vel_d_m_s from naia_gps_position message
 *
 * @return [m/s] GPS Down velocity, (metres/sec)
 */
static inline float mavlink_msg_naia_gps_position_get_vel_d_m_s(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  68);
}

/**
 * @brief Get field cog_rad from naia_gps_position message
 *
 * @return [rad] Course over ground (NOT heading, but direction of movement), -PI..PI, (radians)
 */
static inline float mavlink_msg_naia_gps_position_get_cog_rad(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  72);
}

/**
 * @brief Get field vel_ned_valid from naia_gps_position message
 *
 * @return  True if NED velocity is valid
 */
static inline uint8_t mavlink_msg_naia_gps_position_get_vel_ned_valid(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  89);
}

/**
 * @brief Get field timestamp_time_relative from naia_gps_position message
 *
 * @return [us] timestamp + timestamp_time_relative = Time of the UTC timestamp since system start, (microseconds)
 */
static inline int32_t mavlink_msg_naia_gps_position_get_timestamp_time_relative(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int32_t(msg,  76);
}

/**
 * @brief Get field time_utc_usec from naia_gps_position message
 *
 * @return [us] Timestamp (microseconds, UTC), this is the timestamp which comes from the gps module. It might be unavailable right after cold start, indicated by a value of 0
 */
static inline uint64_t mavlink_msg_naia_gps_position_get_time_utc_usec(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field satellites_used from naia_gps_position message
 *
 * @return  Number of satellites used
 */
static inline uint8_t mavlink_msg_naia_gps_position_get_satellites_used(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  90);
}

/**
 * @brief Get field heading from naia_gps_position message
 *
 * @return [rad] heading angle of XYZ body frame rel to NED. Set to NaN if not available and updated (used for dual antenna GPS), (rad, [-PI, PI])
 */
static inline float mavlink_msg_naia_gps_position_get_heading(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  80);
}

/**
 * @brief Get field heading_offset from naia_gps_position message
 *
 * @return [rad] heading offset of dual antenna array in body frame. Set to NaN if not applicable. (rad, [-PI, PI])
 */
static inline float mavlink_msg_naia_gps_position_get_heading_offset(const mavlink_message_t* msg)
{
    return _MAV_RETURN_float(msg,  84);
}

/**
 * @brief Decode a naia_gps_position message into a struct
 *
 * @param msg The message to decode
 * @param naia_gps_position C-struct to decode the message contents into
 */
static inline void mavlink_msg_naia_gps_position_decode(const mavlink_message_t* msg, mavlink_naia_gps_position_t* naia_gps_position)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    naia_gps_position->time_utc_usec = mavlink_msg_naia_gps_position_get_time_utc_usec(msg);
    naia_gps_position->lat = mavlink_msg_naia_gps_position_get_lat(msg);
    naia_gps_position->lon = mavlink_msg_naia_gps_position_get_lon(msg);
    naia_gps_position->alt = mavlink_msg_naia_gps_position_get_alt(msg);
    naia_gps_position->alt_ellipsoid = mavlink_msg_naia_gps_position_get_alt_ellipsoid(msg);
    naia_gps_position->s_variance_m_s = mavlink_msg_naia_gps_position_get_s_variance_m_s(msg);
    naia_gps_position->c_variance_rad = mavlink_msg_naia_gps_position_get_c_variance_rad(msg);
    naia_gps_position->eph = mavlink_msg_naia_gps_position_get_eph(msg);
    naia_gps_position->epv = mavlink_msg_naia_gps_position_get_epv(msg);
    naia_gps_position->hdop = mavlink_msg_naia_gps_position_get_hdop(msg);
    naia_gps_position->vdop = mavlink_msg_naia_gps_position_get_vdop(msg);
    naia_gps_position->noise_per_ms = mavlink_msg_naia_gps_position_get_noise_per_ms(msg);
    naia_gps_position->jamming_indicator = mavlink_msg_naia_gps_position_get_jamming_indicator(msg);
    naia_gps_position->vel_m_s = mavlink_msg_naia_gps_position_get_vel_m_s(msg);
    naia_gps_position->vel_n_m_s = mavlink_msg_naia_gps_position_get_vel_n_m_s(msg);
    naia_gps_position->vel_e_m_s = mavlink_msg_naia_gps_position_get_vel_e_m_s(msg);
    naia_gps_position->vel_d_m_s = mavlink_msg_naia_gps_position_get_vel_d_m_s(msg);
    naia_gps_position->cog_rad = mavlink_msg_naia_gps_position_get_cog_rad(msg);
    naia_gps_position->timestamp_time_relative = mavlink_msg_naia_gps_position_get_timestamp_time_relative(msg);
    naia_gps_position->heading = mavlink_msg_naia_gps_position_get_heading(msg);
    naia_gps_position->heading_offset = mavlink_msg_naia_gps_position_get_heading_offset(msg);
    naia_gps_position->fix_type = mavlink_msg_naia_gps_position_get_fix_type(msg);
    naia_gps_position->vel_ned_valid = mavlink_msg_naia_gps_position_get_vel_ned_valid(msg);
    naia_gps_position->satellites_used = mavlink_msg_naia_gps_position_get_satellites_used(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN? msg->len : MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN;
        memset(naia_gps_position, 0, MAVLINK_MSG_ID_NAIA_GPS_POSITION_LEN);
    memcpy(naia_gps_position, _MAV_PAYLOAD(msg), len);
#endif
}
