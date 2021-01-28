#pragma once
// MESSAGE SBG_STATUS PACKING

#define MAVLINK_MSG_ID_SBG_STATUS 628

MAVPACKED(
typedef struct __mavlink_sbg_status_t {
 uint64_t timestamp; /*< [us] Timestamp.*/
 uint32_t time_stamp; /*< [us] Time since sensor is powered up us.*/
 uint8_t main_power; /*<  General main power. True when main power supply is OK.*/
 uint8_t imu_power; /*<  General imu power. True when IMU power supply is OK.*/
 uint8_t gps_power; /*<  General gps power. Set to True when GPS power supply is OK.*/
 uint8_t settings; /*<  General Settings. True if settings were correctly loaded*/
 uint8_t temperature; /*<  General Temperature. True when temperature is within specified limits.*/
 uint8_t port_a; /*<  PORT A: False in case of low level communication error.*/
 uint8_t port_b; /*<  PORT B: False in case of low level communication error.*/
 uint8_t port_c; /*<  PORT C: False in case of low level communication error.*/
 uint8_t port_d; /*<  PORT D: False in case of low level communication error.*/
 uint8_t port_e; /*<  PORT E: False in case of low level communication error.*/
 uint8_t port_a_rx; /*<  PORT A RX: False in case of saturation on PORT A input.*/
 uint8_t port_a_tx; /*<  PORT A TX: False in case of saturation on PORT A output.*/
 uint8_t port_b_rx; /*<  PORT B RX: False in case of saturation on PORT B input.*/
 uint8_t port_b_tx; /*<  PORT B TX: False in case of saturation on PORT B output.*/
 uint8_t port_c_rx; /*<  PORT C RX: False in case of saturation on PORT C input.*/
 uint8_t port_c_tx; /*<  PORT C TX: False in case of saturation on PORT C output.*/
 uint8_t port_d_rx; /*<  PORT D RX: False in case of saturation on PORT D input.*/
 uint8_t port_d_tx; /*<  PORT D TX: False in case of saturation on PORT D output.*/
 uint8_t port_e_rx; /*<  PORT E RX: False in case of saturation on PORT E input.*/
 uint8_t port_e_tx; /*<  PORT E TX: False in case of saturation on PORT E output.*/
 uint8_t can_rx; /*<  CAN RX: False in case of saturation on CAN Bus output buffer.*/
 uint8_t can_tx; /*<  CAN TX: False in case of saturation on CAN Bus input buffer.*/
 uint8_t can_status; /*<  CAN BUS. 0 CAN BUS OFF Bus OFF operation due to too much errors. 1 CAN BUS TX_RX_ERR Transmit or received error. 2 CAN BUS OK The CAN bus is working correctly. 3 CAN BUS ERROR A general error has occurred on the CAN bus.*/
 uint8_t gps1_pos_recv; /*<  AIDING_GPS1_POS_RECV true when valid GPS 1 position data is received.*/
 uint8_t gps1_vel_recv; /*<  AIDING_GPS1_VEL_RECV true when valid GPS 1 velocity data is received.*/
 uint8_t gps1_hdt_recv; /*<  AIDING_GPS1_HDT_RECV true when valid GPS 1 true heading data is received.*/
 uint8_t gps1_utc_recv; /*<  AIDING_GPS1_UTC_RECV true when valid GPS 1 UTC time data is received.*/
 uint8_t mag_recv; /*<  AIDING_MAG_RECV true when valid Magnetometer data is received.*/
 uint8_t odo_recv; /*<  AIDING_ODO_RECV true when Odometer pulse is received.*/
 uint8_t dvl_recv; /*<  AIDING_DVL_RECV true when valid DVL data is received*/
}) mavlink_sbg_status_t;

#define MAVLINK_MSG_ID_SBG_STATUS_LEN 42
#define MAVLINK_MSG_ID_SBG_STATUS_MIN_LEN 42
#define MAVLINK_MSG_ID_628_LEN 42
#define MAVLINK_MSG_ID_628_MIN_LEN 42

#define MAVLINK_MSG_ID_SBG_STATUS_CRC 67
#define MAVLINK_MSG_ID_628_CRC 67



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_SBG_STATUS { \
    628, \
    "SBG_STATUS", \
    32, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_status_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_status_t, time_stamp) }, \
         { "main_power", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_sbg_status_t, main_power) }, \
         { "imu_power", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_sbg_status_t, imu_power) }, \
         { "gps_power", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_sbg_status_t, gps_power) }, \
         { "settings", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_sbg_status_t, settings) }, \
         { "temperature", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_sbg_status_t, temperature) }, \
         { "port_a", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_sbg_status_t, port_a) }, \
         { "port_b", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_sbg_status_t, port_b) }, \
         { "port_c", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_sbg_status_t, port_c) }, \
         { "port_d", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_sbg_status_t, port_d) }, \
         { "port_e", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_sbg_status_t, port_e) }, \
         { "port_a_rx", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_sbg_status_t, port_a_rx) }, \
         { "port_a_tx", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_sbg_status_t, port_a_tx) }, \
         { "port_b_rx", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_sbg_status_t, port_b_rx) }, \
         { "port_b_tx", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_sbg_status_t, port_b_tx) }, \
         { "port_c_rx", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_sbg_status_t, port_c_rx) }, \
         { "port_c_tx", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_sbg_status_t, port_c_tx) }, \
         { "port_d_rx", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_sbg_status_t, port_d_rx) }, \
         { "port_d_tx", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_sbg_status_t, port_d_tx) }, \
         { "port_e_rx", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_sbg_status_t, port_e_rx) }, \
         { "port_e_tx", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_sbg_status_t, port_e_tx) }, \
         { "can_rx", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_sbg_status_t, can_rx) }, \
         { "can_tx", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_sbg_status_t, can_tx) }, \
         { "can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_sbg_status_t, can_status) }, \
         { "gps1_pos_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_sbg_status_t, gps1_pos_recv) }, \
         { "gps1_vel_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_sbg_status_t, gps1_vel_recv) }, \
         { "gps1_hdt_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_sbg_status_t, gps1_hdt_recv) }, \
         { "gps1_utc_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_sbg_status_t, gps1_utc_recv) }, \
         { "mag_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_sbg_status_t, mag_recv) }, \
         { "odo_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_sbg_status_t, odo_recv) }, \
         { "dvl_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_sbg_status_t, dvl_recv) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_SBG_STATUS { \
    "SBG_STATUS", \
    32, \
    {  { "timestamp", NULL, MAVLINK_TYPE_UINT64_T, 0, 0, offsetof(mavlink_sbg_status_t, timestamp) }, \
         { "time_stamp", NULL, MAVLINK_TYPE_UINT32_T, 0, 8, offsetof(mavlink_sbg_status_t, time_stamp) }, \
         { "main_power", NULL, MAVLINK_TYPE_UINT8_T, 0, 12, offsetof(mavlink_sbg_status_t, main_power) }, \
         { "imu_power", NULL, MAVLINK_TYPE_UINT8_T, 0, 13, offsetof(mavlink_sbg_status_t, imu_power) }, \
         { "gps_power", NULL, MAVLINK_TYPE_UINT8_T, 0, 14, offsetof(mavlink_sbg_status_t, gps_power) }, \
         { "settings", NULL, MAVLINK_TYPE_UINT8_T, 0, 15, offsetof(mavlink_sbg_status_t, settings) }, \
         { "temperature", NULL, MAVLINK_TYPE_UINT8_T, 0, 16, offsetof(mavlink_sbg_status_t, temperature) }, \
         { "port_a", NULL, MAVLINK_TYPE_UINT8_T, 0, 17, offsetof(mavlink_sbg_status_t, port_a) }, \
         { "port_b", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_sbg_status_t, port_b) }, \
         { "port_c", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_sbg_status_t, port_c) }, \
         { "port_d", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_sbg_status_t, port_d) }, \
         { "port_e", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_sbg_status_t, port_e) }, \
         { "port_a_rx", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_sbg_status_t, port_a_rx) }, \
         { "port_a_tx", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_sbg_status_t, port_a_tx) }, \
         { "port_b_rx", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_sbg_status_t, port_b_rx) }, \
         { "port_b_tx", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_sbg_status_t, port_b_tx) }, \
         { "port_c_rx", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_sbg_status_t, port_c_rx) }, \
         { "port_c_tx", NULL, MAVLINK_TYPE_UINT8_T, 0, 27, offsetof(mavlink_sbg_status_t, port_c_tx) }, \
         { "port_d_rx", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_sbg_status_t, port_d_rx) }, \
         { "port_d_tx", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_sbg_status_t, port_d_tx) }, \
         { "port_e_rx", NULL, MAVLINK_TYPE_UINT8_T, 0, 30, offsetof(mavlink_sbg_status_t, port_e_rx) }, \
         { "port_e_tx", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_sbg_status_t, port_e_tx) }, \
         { "can_rx", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_sbg_status_t, can_rx) }, \
         { "can_tx", NULL, MAVLINK_TYPE_UINT8_T, 0, 33, offsetof(mavlink_sbg_status_t, can_tx) }, \
         { "can_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_sbg_status_t, can_status) }, \
         { "gps1_pos_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_sbg_status_t, gps1_pos_recv) }, \
         { "gps1_vel_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 36, offsetof(mavlink_sbg_status_t, gps1_vel_recv) }, \
         { "gps1_hdt_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_sbg_status_t, gps1_hdt_recv) }, \
         { "gps1_utc_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_sbg_status_t, gps1_utc_recv) }, \
         { "mag_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 39, offsetof(mavlink_sbg_status_t, mag_recv) }, \
         { "odo_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_sbg_status_t, odo_recv) }, \
         { "dvl_recv", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_sbg_status_t, dvl_recv) }, \
         } \
}
#endif

/**
 * @brief Pack a sbg_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param main_power  General main power. True when main power supply is OK.
 * @param imu_power  General imu power. True when IMU power supply is OK.
 * @param gps_power  General gps power. Set to True when GPS power supply is OK.
 * @param settings  General Settings. True if settings were correctly loaded
 * @param temperature  General Temperature. True when temperature is within specified limits.
 * @param port_a  PORT A: False in case of low level communication error.
 * @param port_b  PORT B: False in case of low level communication error.
 * @param port_c  PORT C: False in case of low level communication error.
 * @param port_d  PORT D: False in case of low level communication error.
 * @param port_e  PORT E: False in case of low level communication error.
 * @param port_a_rx  PORT A RX: False in case of saturation on PORT A input.
 * @param port_a_tx  PORT A TX: False in case of saturation on PORT A output.
 * @param port_b_rx  PORT B RX: False in case of saturation on PORT B input.
 * @param port_b_tx  PORT B TX: False in case of saturation on PORT B output.
 * @param port_c_rx  PORT C RX: False in case of saturation on PORT C input.
 * @param port_c_tx  PORT C TX: False in case of saturation on PORT C output.
 * @param port_d_rx  PORT D RX: False in case of saturation on PORT D input.
 * @param port_d_tx  PORT D TX: False in case of saturation on PORT D output.
 * @param port_e_rx  PORT E RX: False in case of saturation on PORT E input.
 * @param port_e_tx  PORT E TX: False in case of saturation on PORT E output.
 * @param can_rx  CAN RX: False in case of saturation on CAN Bus output buffer.
 * @param can_tx  CAN TX: False in case of saturation on CAN Bus input buffer.
 * @param can_status  CAN BUS. 0 CAN BUS OFF Bus OFF operation due to too much errors. 1 CAN BUS TX_RX_ERR Transmit or received error. 2 CAN BUS OK The CAN bus is working correctly. 3 CAN BUS ERROR A general error has occurred on the CAN bus.
 * @param gps1_pos_recv  AIDING_GPS1_POS_RECV true when valid GPS 1 position data is received.
 * @param gps1_vel_recv  AIDING_GPS1_VEL_RECV true when valid GPS 1 velocity data is received.
 * @param gps1_hdt_recv  AIDING_GPS1_HDT_RECV true when valid GPS 1 true heading data is received.
 * @param gps1_utc_recv  AIDING_GPS1_UTC_RECV true when valid GPS 1 UTC time data is received.
 * @param mag_recv  AIDING_MAG_RECV true when valid Magnetometer data is received.
 * @param odo_recv  AIDING_ODO_RECV true when Odometer pulse is received.
 * @param dvl_recv  AIDING_DVL_RECV true when valid DVL data is received
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint64_t timestamp, uint32_t time_stamp, uint8_t main_power, uint8_t imu_power, uint8_t gps_power, uint8_t settings, uint8_t temperature, uint8_t port_a, uint8_t port_b, uint8_t port_c, uint8_t port_d, uint8_t port_e, uint8_t port_a_rx, uint8_t port_a_tx, uint8_t port_b_rx, uint8_t port_b_tx, uint8_t port_c_rx, uint8_t port_c_tx, uint8_t port_d_rx, uint8_t port_d_tx, uint8_t port_e_rx, uint8_t port_e_tx, uint8_t can_rx, uint8_t can_tx, uint8_t can_status, uint8_t gps1_pos_recv, uint8_t gps1_vel_recv, uint8_t gps1_hdt_recv, uint8_t gps1_utc_recv, uint8_t mag_recv, uint8_t odo_recv, uint8_t dvl_recv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint8_t(buf, 12, main_power);
    _mav_put_uint8_t(buf, 13, imu_power);
    _mav_put_uint8_t(buf, 14, gps_power);
    _mav_put_uint8_t(buf, 15, settings);
    _mav_put_uint8_t(buf, 16, temperature);
    _mav_put_uint8_t(buf, 17, port_a);
    _mav_put_uint8_t(buf, 18, port_b);
    _mav_put_uint8_t(buf, 19, port_c);
    _mav_put_uint8_t(buf, 20, port_d);
    _mav_put_uint8_t(buf, 21, port_e);
    _mav_put_uint8_t(buf, 22, port_a_rx);
    _mav_put_uint8_t(buf, 23, port_a_tx);
    _mav_put_uint8_t(buf, 24, port_b_rx);
    _mav_put_uint8_t(buf, 25, port_b_tx);
    _mav_put_uint8_t(buf, 26, port_c_rx);
    _mav_put_uint8_t(buf, 27, port_c_tx);
    _mav_put_uint8_t(buf, 28, port_d_rx);
    _mav_put_uint8_t(buf, 29, port_d_tx);
    _mav_put_uint8_t(buf, 30, port_e_rx);
    _mav_put_uint8_t(buf, 31, port_e_tx);
    _mav_put_uint8_t(buf, 32, can_rx);
    _mav_put_uint8_t(buf, 33, can_tx);
    _mav_put_uint8_t(buf, 34, can_status);
    _mav_put_uint8_t(buf, 35, gps1_pos_recv);
    _mav_put_uint8_t(buf, 36, gps1_vel_recv);
    _mav_put_uint8_t(buf, 37, gps1_hdt_recv);
    _mav_put_uint8_t(buf, 38, gps1_utc_recv);
    _mav_put_uint8_t(buf, 39, mag_recv);
    _mav_put_uint8_t(buf, 40, odo_recv);
    _mav_put_uint8_t(buf, 41, dvl_recv);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_STATUS_LEN);
#else
    mavlink_sbg_status_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.main_power = main_power;
    packet.imu_power = imu_power;
    packet.gps_power = gps_power;
    packet.settings = settings;
    packet.temperature = temperature;
    packet.port_a = port_a;
    packet.port_b = port_b;
    packet.port_c = port_c;
    packet.port_d = port_d;
    packet.port_e = port_e;
    packet.port_a_rx = port_a_rx;
    packet.port_a_tx = port_a_tx;
    packet.port_b_rx = port_b_rx;
    packet.port_b_tx = port_b_tx;
    packet.port_c_rx = port_c_rx;
    packet.port_c_tx = port_c_tx;
    packet.port_d_rx = port_d_rx;
    packet.port_d_tx = port_d_tx;
    packet.port_e_rx = port_e_rx;
    packet.port_e_tx = port_e_tx;
    packet.can_rx = can_rx;
    packet.can_tx = can_tx;
    packet.can_status = can_status;
    packet.gps1_pos_recv = gps1_pos_recv;
    packet.gps1_vel_recv = gps1_vel_recv;
    packet.gps1_hdt_recv = gps1_hdt_recv;
    packet.gps1_utc_recv = gps1_utc_recv;
    packet.mag_recv = mag_recv;
    packet.odo_recv = odo_recv;
    packet.dvl_recv = dvl_recv;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SBG_STATUS_MIN_LEN, MAVLINK_MSG_ID_SBG_STATUS_LEN, MAVLINK_MSG_ID_SBG_STATUS_CRC);
}

/**
 * @brief Pack a sbg_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param main_power  General main power. True when main power supply is OK.
 * @param imu_power  General imu power. True when IMU power supply is OK.
 * @param gps_power  General gps power. Set to True when GPS power supply is OK.
 * @param settings  General Settings. True if settings were correctly loaded
 * @param temperature  General Temperature. True when temperature is within specified limits.
 * @param port_a  PORT A: False in case of low level communication error.
 * @param port_b  PORT B: False in case of low level communication error.
 * @param port_c  PORT C: False in case of low level communication error.
 * @param port_d  PORT D: False in case of low level communication error.
 * @param port_e  PORT E: False in case of low level communication error.
 * @param port_a_rx  PORT A RX: False in case of saturation on PORT A input.
 * @param port_a_tx  PORT A TX: False in case of saturation on PORT A output.
 * @param port_b_rx  PORT B RX: False in case of saturation on PORT B input.
 * @param port_b_tx  PORT B TX: False in case of saturation on PORT B output.
 * @param port_c_rx  PORT C RX: False in case of saturation on PORT C input.
 * @param port_c_tx  PORT C TX: False in case of saturation on PORT C output.
 * @param port_d_rx  PORT D RX: False in case of saturation on PORT D input.
 * @param port_d_tx  PORT D TX: False in case of saturation on PORT D output.
 * @param port_e_rx  PORT E RX: False in case of saturation on PORT E input.
 * @param port_e_tx  PORT E TX: False in case of saturation on PORT E output.
 * @param can_rx  CAN RX: False in case of saturation on CAN Bus output buffer.
 * @param can_tx  CAN TX: False in case of saturation on CAN Bus input buffer.
 * @param can_status  CAN BUS. 0 CAN BUS OFF Bus OFF operation due to too much errors. 1 CAN BUS TX_RX_ERR Transmit or received error. 2 CAN BUS OK The CAN bus is working correctly. 3 CAN BUS ERROR A general error has occurred on the CAN bus.
 * @param gps1_pos_recv  AIDING_GPS1_POS_RECV true when valid GPS 1 position data is received.
 * @param gps1_vel_recv  AIDING_GPS1_VEL_RECV true when valid GPS 1 velocity data is received.
 * @param gps1_hdt_recv  AIDING_GPS1_HDT_RECV true when valid GPS 1 true heading data is received.
 * @param gps1_utc_recv  AIDING_GPS1_UTC_RECV true when valid GPS 1 UTC time data is received.
 * @param mag_recv  AIDING_MAG_RECV true when valid Magnetometer data is received.
 * @param odo_recv  AIDING_ODO_RECV true when Odometer pulse is received.
 * @param dvl_recv  AIDING_DVL_RECV true when valid DVL data is received
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_sbg_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint64_t timestamp,uint32_t time_stamp,uint8_t main_power,uint8_t imu_power,uint8_t gps_power,uint8_t settings,uint8_t temperature,uint8_t port_a,uint8_t port_b,uint8_t port_c,uint8_t port_d,uint8_t port_e,uint8_t port_a_rx,uint8_t port_a_tx,uint8_t port_b_rx,uint8_t port_b_tx,uint8_t port_c_rx,uint8_t port_c_tx,uint8_t port_d_rx,uint8_t port_d_tx,uint8_t port_e_rx,uint8_t port_e_tx,uint8_t can_rx,uint8_t can_tx,uint8_t can_status,uint8_t gps1_pos_recv,uint8_t gps1_vel_recv,uint8_t gps1_hdt_recv,uint8_t gps1_utc_recv,uint8_t mag_recv,uint8_t odo_recv,uint8_t dvl_recv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint8_t(buf, 12, main_power);
    _mav_put_uint8_t(buf, 13, imu_power);
    _mav_put_uint8_t(buf, 14, gps_power);
    _mav_put_uint8_t(buf, 15, settings);
    _mav_put_uint8_t(buf, 16, temperature);
    _mav_put_uint8_t(buf, 17, port_a);
    _mav_put_uint8_t(buf, 18, port_b);
    _mav_put_uint8_t(buf, 19, port_c);
    _mav_put_uint8_t(buf, 20, port_d);
    _mav_put_uint8_t(buf, 21, port_e);
    _mav_put_uint8_t(buf, 22, port_a_rx);
    _mav_put_uint8_t(buf, 23, port_a_tx);
    _mav_put_uint8_t(buf, 24, port_b_rx);
    _mav_put_uint8_t(buf, 25, port_b_tx);
    _mav_put_uint8_t(buf, 26, port_c_rx);
    _mav_put_uint8_t(buf, 27, port_c_tx);
    _mav_put_uint8_t(buf, 28, port_d_rx);
    _mav_put_uint8_t(buf, 29, port_d_tx);
    _mav_put_uint8_t(buf, 30, port_e_rx);
    _mav_put_uint8_t(buf, 31, port_e_tx);
    _mav_put_uint8_t(buf, 32, can_rx);
    _mav_put_uint8_t(buf, 33, can_tx);
    _mav_put_uint8_t(buf, 34, can_status);
    _mav_put_uint8_t(buf, 35, gps1_pos_recv);
    _mav_put_uint8_t(buf, 36, gps1_vel_recv);
    _mav_put_uint8_t(buf, 37, gps1_hdt_recv);
    _mav_put_uint8_t(buf, 38, gps1_utc_recv);
    _mav_put_uint8_t(buf, 39, mag_recv);
    _mav_put_uint8_t(buf, 40, odo_recv);
    _mav_put_uint8_t(buf, 41, dvl_recv);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SBG_STATUS_LEN);
#else
    mavlink_sbg_status_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.main_power = main_power;
    packet.imu_power = imu_power;
    packet.gps_power = gps_power;
    packet.settings = settings;
    packet.temperature = temperature;
    packet.port_a = port_a;
    packet.port_b = port_b;
    packet.port_c = port_c;
    packet.port_d = port_d;
    packet.port_e = port_e;
    packet.port_a_rx = port_a_rx;
    packet.port_a_tx = port_a_tx;
    packet.port_b_rx = port_b_rx;
    packet.port_b_tx = port_b_tx;
    packet.port_c_rx = port_c_rx;
    packet.port_c_tx = port_c_tx;
    packet.port_d_rx = port_d_rx;
    packet.port_d_tx = port_d_tx;
    packet.port_e_rx = port_e_rx;
    packet.port_e_tx = port_e_tx;
    packet.can_rx = can_rx;
    packet.can_tx = can_tx;
    packet.can_status = can_status;
    packet.gps1_pos_recv = gps1_pos_recv;
    packet.gps1_vel_recv = gps1_vel_recv;
    packet.gps1_hdt_recv = gps1_hdt_recv;
    packet.gps1_utc_recv = gps1_utc_recv;
    packet.mag_recv = mag_recv;
    packet.odo_recv = odo_recv;
    packet.dvl_recv = dvl_recv;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SBG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_SBG_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SBG_STATUS_MIN_LEN, MAVLINK_MSG_ID_SBG_STATUS_LEN, MAVLINK_MSG_ID_SBG_STATUS_CRC);
}

/**
 * @brief Encode a sbg_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param sbg_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_sbg_status_t* sbg_status)
{
    return mavlink_msg_sbg_status_pack(system_id, component_id, msg, sbg_status->timestamp, sbg_status->time_stamp, sbg_status->main_power, sbg_status->imu_power, sbg_status->gps_power, sbg_status->settings, sbg_status->temperature, sbg_status->port_a, sbg_status->port_b, sbg_status->port_c, sbg_status->port_d, sbg_status->port_e, sbg_status->port_a_rx, sbg_status->port_a_tx, sbg_status->port_b_rx, sbg_status->port_b_tx, sbg_status->port_c_rx, sbg_status->port_c_tx, sbg_status->port_d_rx, sbg_status->port_d_tx, sbg_status->port_e_rx, sbg_status->port_e_tx, sbg_status->can_rx, sbg_status->can_tx, sbg_status->can_status, sbg_status->gps1_pos_recv, sbg_status->gps1_vel_recv, sbg_status->gps1_hdt_recv, sbg_status->gps1_utc_recv, sbg_status->mag_recv, sbg_status->odo_recv, sbg_status->dvl_recv);
}

/**
 * @brief Encode a sbg_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sbg_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_sbg_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_sbg_status_t* sbg_status)
{
    return mavlink_msg_sbg_status_pack_chan(system_id, component_id, chan, msg, sbg_status->timestamp, sbg_status->time_stamp, sbg_status->main_power, sbg_status->imu_power, sbg_status->gps_power, sbg_status->settings, sbg_status->temperature, sbg_status->port_a, sbg_status->port_b, sbg_status->port_c, sbg_status->port_d, sbg_status->port_e, sbg_status->port_a_rx, sbg_status->port_a_tx, sbg_status->port_b_rx, sbg_status->port_b_tx, sbg_status->port_c_rx, sbg_status->port_c_tx, sbg_status->port_d_rx, sbg_status->port_d_tx, sbg_status->port_e_rx, sbg_status->port_e_tx, sbg_status->can_rx, sbg_status->can_tx, sbg_status->can_status, sbg_status->gps1_pos_recv, sbg_status->gps1_vel_recv, sbg_status->gps1_hdt_recv, sbg_status->gps1_utc_recv, sbg_status->mag_recv, sbg_status->odo_recv, sbg_status->dvl_recv);
}

/**
 * @brief Send a sbg_status message
 * @param chan MAVLink channel to send the message
 *
 * @param timestamp [us] Timestamp.
 * @param time_stamp [us] Time since sensor is powered up us.
 * @param main_power  General main power. True when main power supply is OK.
 * @param imu_power  General imu power. True when IMU power supply is OK.
 * @param gps_power  General gps power. Set to True when GPS power supply is OK.
 * @param settings  General Settings. True if settings were correctly loaded
 * @param temperature  General Temperature. True when temperature is within specified limits.
 * @param port_a  PORT A: False in case of low level communication error.
 * @param port_b  PORT B: False in case of low level communication error.
 * @param port_c  PORT C: False in case of low level communication error.
 * @param port_d  PORT D: False in case of low level communication error.
 * @param port_e  PORT E: False in case of low level communication error.
 * @param port_a_rx  PORT A RX: False in case of saturation on PORT A input.
 * @param port_a_tx  PORT A TX: False in case of saturation on PORT A output.
 * @param port_b_rx  PORT B RX: False in case of saturation on PORT B input.
 * @param port_b_tx  PORT B TX: False in case of saturation on PORT B output.
 * @param port_c_rx  PORT C RX: False in case of saturation on PORT C input.
 * @param port_c_tx  PORT C TX: False in case of saturation on PORT C output.
 * @param port_d_rx  PORT D RX: False in case of saturation on PORT D input.
 * @param port_d_tx  PORT D TX: False in case of saturation on PORT D output.
 * @param port_e_rx  PORT E RX: False in case of saturation on PORT E input.
 * @param port_e_tx  PORT E TX: False in case of saturation on PORT E output.
 * @param can_rx  CAN RX: False in case of saturation on CAN Bus output buffer.
 * @param can_tx  CAN TX: False in case of saturation on CAN Bus input buffer.
 * @param can_status  CAN BUS. 0 CAN BUS OFF Bus OFF operation due to too much errors. 1 CAN BUS TX_RX_ERR Transmit or received error. 2 CAN BUS OK The CAN bus is working correctly. 3 CAN BUS ERROR A general error has occurred on the CAN bus.
 * @param gps1_pos_recv  AIDING_GPS1_POS_RECV true when valid GPS 1 position data is received.
 * @param gps1_vel_recv  AIDING_GPS1_VEL_RECV true when valid GPS 1 velocity data is received.
 * @param gps1_hdt_recv  AIDING_GPS1_HDT_RECV true when valid GPS 1 true heading data is received.
 * @param gps1_utc_recv  AIDING_GPS1_UTC_RECV true when valid GPS 1 UTC time data is received.
 * @param mag_recv  AIDING_MAG_RECV true when valid Magnetometer data is received.
 * @param odo_recv  AIDING_ODO_RECV true when Odometer pulse is received.
 * @param dvl_recv  AIDING_DVL_RECV true when valid DVL data is received
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_sbg_status_send(mavlink_channel_t chan, uint64_t timestamp, uint32_t time_stamp, uint8_t main_power, uint8_t imu_power, uint8_t gps_power, uint8_t settings, uint8_t temperature, uint8_t port_a, uint8_t port_b, uint8_t port_c, uint8_t port_d, uint8_t port_e, uint8_t port_a_rx, uint8_t port_a_tx, uint8_t port_b_rx, uint8_t port_b_tx, uint8_t port_c_rx, uint8_t port_c_tx, uint8_t port_d_rx, uint8_t port_d_tx, uint8_t port_e_rx, uint8_t port_e_tx, uint8_t can_rx, uint8_t can_tx, uint8_t can_status, uint8_t gps1_pos_recv, uint8_t gps1_vel_recv, uint8_t gps1_hdt_recv, uint8_t gps1_utc_recv, uint8_t mag_recv, uint8_t odo_recv, uint8_t dvl_recv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_SBG_STATUS_LEN];
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint8_t(buf, 12, main_power);
    _mav_put_uint8_t(buf, 13, imu_power);
    _mav_put_uint8_t(buf, 14, gps_power);
    _mav_put_uint8_t(buf, 15, settings);
    _mav_put_uint8_t(buf, 16, temperature);
    _mav_put_uint8_t(buf, 17, port_a);
    _mav_put_uint8_t(buf, 18, port_b);
    _mav_put_uint8_t(buf, 19, port_c);
    _mav_put_uint8_t(buf, 20, port_d);
    _mav_put_uint8_t(buf, 21, port_e);
    _mav_put_uint8_t(buf, 22, port_a_rx);
    _mav_put_uint8_t(buf, 23, port_a_tx);
    _mav_put_uint8_t(buf, 24, port_b_rx);
    _mav_put_uint8_t(buf, 25, port_b_tx);
    _mav_put_uint8_t(buf, 26, port_c_rx);
    _mav_put_uint8_t(buf, 27, port_c_tx);
    _mav_put_uint8_t(buf, 28, port_d_rx);
    _mav_put_uint8_t(buf, 29, port_d_tx);
    _mav_put_uint8_t(buf, 30, port_e_rx);
    _mav_put_uint8_t(buf, 31, port_e_tx);
    _mav_put_uint8_t(buf, 32, can_rx);
    _mav_put_uint8_t(buf, 33, can_tx);
    _mav_put_uint8_t(buf, 34, can_status);
    _mav_put_uint8_t(buf, 35, gps1_pos_recv);
    _mav_put_uint8_t(buf, 36, gps1_vel_recv);
    _mav_put_uint8_t(buf, 37, gps1_hdt_recv);
    _mav_put_uint8_t(buf, 38, gps1_utc_recv);
    _mav_put_uint8_t(buf, 39, mag_recv);
    _mav_put_uint8_t(buf, 40, odo_recv);
    _mav_put_uint8_t(buf, 41, dvl_recv);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_STATUS, buf, MAVLINK_MSG_ID_SBG_STATUS_MIN_LEN, MAVLINK_MSG_ID_SBG_STATUS_LEN, MAVLINK_MSG_ID_SBG_STATUS_CRC);
#else
    mavlink_sbg_status_t packet;
    packet.timestamp = timestamp;
    packet.time_stamp = time_stamp;
    packet.main_power = main_power;
    packet.imu_power = imu_power;
    packet.gps_power = gps_power;
    packet.settings = settings;
    packet.temperature = temperature;
    packet.port_a = port_a;
    packet.port_b = port_b;
    packet.port_c = port_c;
    packet.port_d = port_d;
    packet.port_e = port_e;
    packet.port_a_rx = port_a_rx;
    packet.port_a_tx = port_a_tx;
    packet.port_b_rx = port_b_rx;
    packet.port_b_tx = port_b_tx;
    packet.port_c_rx = port_c_rx;
    packet.port_c_tx = port_c_tx;
    packet.port_d_rx = port_d_rx;
    packet.port_d_tx = port_d_tx;
    packet.port_e_rx = port_e_rx;
    packet.port_e_tx = port_e_tx;
    packet.can_rx = can_rx;
    packet.can_tx = can_tx;
    packet.can_status = can_status;
    packet.gps1_pos_recv = gps1_pos_recv;
    packet.gps1_vel_recv = gps1_vel_recv;
    packet.gps1_hdt_recv = gps1_hdt_recv;
    packet.gps1_utc_recv = gps1_utc_recv;
    packet.mag_recv = mag_recv;
    packet.odo_recv = odo_recv;
    packet.dvl_recv = dvl_recv;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_STATUS, (const char *)&packet, MAVLINK_MSG_ID_SBG_STATUS_MIN_LEN, MAVLINK_MSG_ID_SBG_STATUS_LEN, MAVLINK_MSG_ID_SBG_STATUS_CRC);
#endif
}

/**
 * @brief Send a sbg_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_sbg_status_send_struct(mavlink_channel_t chan, const mavlink_sbg_status_t* sbg_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_sbg_status_send(chan, sbg_status->timestamp, sbg_status->time_stamp, sbg_status->main_power, sbg_status->imu_power, sbg_status->gps_power, sbg_status->settings, sbg_status->temperature, sbg_status->port_a, sbg_status->port_b, sbg_status->port_c, sbg_status->port_d, sbg_status->port_e, sbg_status->port_a_rx, sbg_status->port_a_tx, sbg_status->port_b_rx, sbg_status->port_b_tx, sbg_status->port_c_rx, sbg_status->port_c_tx, sbg_status->port_d_rx, sbg_status->port_d_tx, sbg_status->port_e_rx, sbg_status->port_e_tx, sbg_status->can_rx, sbg_status->can_tx, sbg_status->can_status, sbg_status->gps1_pos_recv, sbg_status->gps1_vel_recv, sbg_status->gps1_hdt_recv, sbg_status->gps1_utc_recv, sbg_status->mag_recv, sbg_status->odo_recv, sbg_status->dvl_recv);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_STATUS, (const char *)sbg_status, MAVLINK_MSG_ID_SBG_STATUS_MIN_LEN, MAVLINK_MSG_ID_SBG_STATUS_LEN, MAVLINK_MSG_ID_SBG_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_SBG_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_sbg_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint64_t timestamp, uint32_t time_stamp, uint8_t main_power, uint8_t imu_power, uint8_t gps_power, uint8_t settings, uint8_t temperature, uint8_t port_a, uint8_t port_b, uint8_t port_c, uint8_t port_d, uint8_t port_e, uint8_t port_a_rx, uint8_t port_a_tx, uint8_t port_b_rx, uint8_t port_b_tx, uint8_t port_c_rx, uint8_t port_c_tx, uint8_t port_d_rx, uint8_t port_d_tx, uint8_t port_e_rx, uint8_t port_e_tx, uint8_t can_rx, uint8_t can_tx, uint8_t can_status, uint8_t gps1_pos_recv, uint8_t gps1_vel_recv, uint8_t gps1_hdt_recv, uint8_t gps1_utc_recv, uint8_t mag_recv, uint8_t odo_recv, uint8_t dvl_recv)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint64_t(buf, 0, timestamp);
    _mav_put_uint32_t(buf, 8, time_stamp);
    _mav_put_uint8_t(buf, 12, main_power);
    _mav_put_uint8_t(buf, 13, imu_power);
    _mav_put_uint8_t(buf, 14, gps_power);
    _mav_put_uint8_t(buf, 15, settings);
    _mav_put_uint8_t(buf, 16, temperature);
    _mav_put_uint8_t(buf, 17, port_a);
    _mav_put_uint8_t(buf, 18, port_b);
    _mav_put_uint8_t(buf, 19, port_c);
    _mav_put_uint8_t(buf, 20, port_d);
    _mav_put_uint8_t(buf, 21, port_e);
    _mav_put_uint8_t(buf, 22, port_a_rx);
    _mav_put_uint8_t(buf, 23, port_a_tx);
    _mav_put_uint8_t(buf, 24, port_b_rx);
    _mav_put_uint8_t(buf, 25, port_b_tx);
    _mav_put_uint8_t(buf, 26, port_c_rx);
    _mav_put_uint8_t(buf, 27, port_c_tx);
    _mav_put_uint8_t(buf, 28, port_d_rx);
    _mav_put_uint8_t(buf, 29, port_d_tx);
    _mav_put_uint8_t(buf, 30, port_e_rx);
    _mav_put_uint8_t(buf, 31, port_e_tx);
    _mav_put_uint8_t(buf, 32, can_rx);
    _mav_put_uint8_t(buf, 33, can_tx);
    _mav_put_uint8_t(buf, 34, can_status);
    _mav_put_uint8_t(buf, 35, gps1_pos_recv);
    _mav_put_uint8_t(buf, 36, gps1_vel_recv);
    _mav_put_uint8_t(buf, 37, gps1_hdt_recv);
    _mav_put_uint8_t(buf, 38, gps1_utc_recv);
    _mav_put_uint8_t(buf, 39, mag_recv);
    _mav_put_uint8_t(buf, 40, odo_recv);
    _mav_put_uint8_t(buf, 41, dvl_recv);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_STATUS, buf, MAVLINK_MSG_ID_SBG_STATUS_MIN_LEN, MAVLINK_MSG_ID_SBG_STATUS_LEN, MAVLINK_MSG_ID_SBG_STATUS_CRC);
#else
    mavlink_sbg_status_t *packet = (mavlink_sbg_status_t *)msgbuf;
    packet->timestamp = timestamp;
    packet->time_stamp = time_stamp;
    packet->main_power = main_power;
    packet->imu_power = imu_power;
    packet->gps_power = gps_power;
    packet->settings = settings;
    packet->temperature = temperature;
    packet->port_a = port_a;
    packet->port_b = port_b;
    packet->port_c = port_c;
    packet->port_d = port_d;
    packet->port_e = port_e;
    packet->port_a_rx = port_a_rx;
    packet->port_a_tx = port_a_tx;
    packet->port_b_rx = port_b_rx;
    packet->port_b_tx = port_b_tx;
    packet->port_c_rx = port_c_rx;
    packet->port_c_tx = port_c_tx;
    packet->port_d_rx = port_d_rx;
    packet->port_d_tx = port_d_tx;
    packet->port_e_rx = port_e_rx;
    packet->port_e_tx = port_e_tx;
    packet->can_rx = can_rx;
    packet->can_tx = can_tx;
    packet->can_status = can_status;
    packet->gps1_pos_recv = gps1_pos_recv;
    packet->gps1_vel_recv = gps1_vel_recv;
    packet->gps1_hdt_recv = gps1_hdt_recv;
    packet->gps1_utc_recv = gps1_utc_recv;
    packet->mag_recv = mag_recv;
    packet->odo_recv = odo_recv;
    packet->dvl_recv = dvl_recv;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SBG_STATUS, (const char *)packet, MAVLINK_MSG_ID_SBG_STATUS_MIN_LEN, MAVLINK_MSG_ID_SBG_STATUS_LEN, MAVLINK_MSG_ID_SBG_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE SBG_STATUS UNPACKING


/**
 * @brief Get field timestamp from sbg_status message
 *
 * @return [us] Timestamp.
 */
static inline uint64_t mavlink_msg_sbg_status_get_timestamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint64_t(msg,  0);
}

/**
 * @brief Get field time_stamp from sbg_status message
 *
 * @return [us] Time since sensor is powered up us.
 */
static inline uint32_t mavlink_msg_sbg_status_get_time_stamp(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint32_t(msg,  8);
}

/**
 * @brief Get field main_power from sbg_status message
 *
 * @return  General main power. True when main power supply is OK.
 */
static inline uint8_t mavlink_msg_sbg_status_get_main_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  12);
}

/**
 * @brief Get field imu_power from sbg_status message
 *
 * @return  General imu power. True when IMU power supply is OK.
 */
static inline uint8_t mavlink_msg_sbg_status_get_imu_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  13);
}

/**
 * @brief Get field gps_power from sbg_status message
 *
 * @return  General gps power. Set to True when GPS power supply is OK.
 */
static inline uint8_t mavlink_msg_sbg_status_get_gps_power(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  14);
}

/**
 * @brief Get field settings from sbg_status message
 *
 * @return  General Settings. True if settings were correctly loaded
 */
static inline uint8_t mavlink_msg_sbg_status_get_settings(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  15);
}

/**
 * @brief Get field temperature from sbg_status message
 *
 * @return  General Temperature. True when temperature is within specified limits.
 */
static inline uint8_t mavlink_msg_sbg_status_get_temperature(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  16);
}

/**
 * @brief Get field port_a from sbg_status message
 *
 * @return  PORT A: False in case of low level communication error.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_a(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  17);
}

/**
 * @brief Get field port_b from sbg_status message
 *
 * @return  PORT B: False in case of low level communication error.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_b(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field port_c from sbg_status message
 *
 * @return  PORT C: False in case of low level communication error.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_c(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field port_d from sbg_status message
 *
 * @return  PORT D: False in case of low level communication error.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_d(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field port_e from sbg_status message
 *
 * @return  PORT E: False in case of low level communication error.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_e(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field port_a_rx from sbg_status message
 *
 * @return  PORT A RX: False in case of saturation on PORT A input.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_a_rx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field port_a_tx from sbg_status message
 *
 * @return  PORT A TX: False in case of saturation on PORT A output.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_a_tx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field port_b_rx from sbg_status message
 *
 * @return  PORT B RX: False in case of saturation on PORT B input.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_b_rx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Get field port_b_tx from sbg_status message
 *
 * @return  PORT B TX: False in case of saturation on PORT B output.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_b_tx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field port_c_rx from sbg_status message
 *
 * @return  PORT C RX: False in case of saturation on PORT C input.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_c_rx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field port_c_tx from sbg_status message
 *
 * @return  PORT C TX: False in case of saturation on PORT C output.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_c_tx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  27);
}

/**
 * @brief Get field port_d_rx from sbg_status message
 *
 * @return  PORT D RX: False in case of saturation on PORT D input.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_d_rx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field port_d_tx from sbg_status message
 *
 * @return  PORT D TX: False in case of saturation on PORT D output.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_d_tx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field port_e_rx from sbg_status message
 *
 * @return  PORT E RX: False in case of saturation on PORT E input.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_e_rx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  30);
}

/**
 * @brief Get field port_e_tx from sbg_status message
 *
 * @return  PORT E TX: False in case of saturation on PORT E output.
 */
static inline uint8_t mavlink_msg_sbg_status_get_port_e_tx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field can_rx from sbg_status message
 *
 * @return  CAN RX: False in case of saturation on CAN Bus output buffer.
 */
static inline uint8_t mavlink_msg_sbg_status_get_can_rx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field can_tx from sbg_status message
 *
 * @return  CAN TX: False in case of saturation on CAN Bus input buffer.
 */
static inline uint8_t mavlink_msg_sbg_status_get_can_tx(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  33);
}

/**
 * @brief Get field can_status from sbg_status message
 *
 * @return  CAN BUS. 0 CAN BUS OFF Bus OFF operation due to too much errors. 1 CAN BUS TX_RX_ERR Transmit or received error. 2 CAN BUS OK The CAN bus is working correctly. 3 CAN BUS ERROR A general error has occurred on the CAN bus.
 */
static inline uint8_t mavlink_msg_sbg_status_get_can_status(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field gps1_pos_recv from sbg_status message
 *
 * @return  AIDING_GPS1_POS_RECV true when valid GPS 1 position data is received.
 */
static inline uint8_t mavlink_msg_sbg_status_get_gps1_pos_recv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field gps1_vel_recv from sbg_status message
 *
 * @return  AIDING_GPS1_VEL_RECV true when valid GPS 1 velocity data is received.
 */
static inline uint8_t mavlink_msg_sbg_status_get_gps1_vel_recv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  36);
}

/**
 * @brief Get field gps1_hdt_recv from sbg_status message
 *
 * @return  AIDING_GPS1_HDT_RECV true when valid GPS 1 true heading data is received.
 */
static inline uint8_t mavlink_msg_sbg_status_get_gps1_hdt_recv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field gps1_utc_recv from sbg_status message
 *
 * @return  AIDING_GPS1_UTC_RECV true when valid GPS 1 UTC time data is received.
 */
static inline uint8_t mavlink_msg_sbg_status_get_gps1_utc_recv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field mag_recv from sbg_status message
 *
 * @return  AIDING_MAG_RECV true when valid Magnetometer data is received.
 */
static inline uint8_t mavlink_msg_sbg_status_get_mag_recv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  39);
}

/**
 * @brief Get field odo_recv from sbg_status message
 *
 * @return  AIDING_ODO_RECV true when Odometer pulse is received.
 */
static inline uint8_t mavlink_msg_sbg_status_get_odo_recv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field dvl_recv from sbg_status message
 *
 * @return  AIDING_DVL_RECV true when valid DVL data is received
 */
static inline uint8_t mavlink_msg_sbg_status_get_dvl_recv(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Decode a sbg_status message into a struct
 *
 * @param msg The message to decode
 * @param sbg_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_sbg_status_decode(const mavlink_message_t* msg, mavlink_sbg_status_t* sbg_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    sbg_status->timestamp = mavlink_msg_sbg_status_get_timestamp(msg);
    sbg_status->time_stamp = mavlink_msg_sbg_status_get_time_stamp(msg);
    sbg_status->main_power = mavlink_msg_sbg_status_get_main_power(msg);
    sbg_status->imu_power = mavlink_msg_sbg_status_get_imu_power(msg);
    sbg_status->gps_power = mavlink_msg_sbg_status_get_gps_power(msg);
    sbg_status->settings = mavlink_msg_sbg_status_get_settings(msg);
    sbg_status->temperature = mavlink_msg_sbg_status_get_temperature(msg);
    sbg_status->port_a = mavlink_msg_sbg_status_get_port_a(msg);
    sbg_status->port_b = mavlink_msg_sbg_status_get_port_b(msg);
    sbg_status->port_c = mavlink_msg_sbg_status_get_port_c(msg);
    sbg_status->port_d = mavlink_msg_sbg_status_get_port_d(msg);
    sbg_status->port_e = mavlink_msg_sbg_status_get_port_e(msg);
    sbg_status->port_a_rx = mavlink_msg_sbg_status_get_port_a_rx(msg);
    sbg_status->port_a_tx = mavlink_msg_sbg_status_get_port_a_tx(msg);
    sbg_status->port_b_rx = mavlink_msg_sbg_status_get_port_b_rx(msg);
    sbg_status->port_b_tx = mavlink_msg_sbg_status_get_port_b_tx(msg);
    sbg_status->port_c_rx = mavlink_msg_sbg_status_get_port_c_rx(msg);
    sbg_status->port_c_tx = mavlink_msg_sbg_status_get_port_c_tx(msg);
    sbg_status->port_d_rx = mavlink_msg_sbg_status_get_port_d_rx(msg);
    sbg_status->port_d_tx = mavlink_msg_sbg_status_get_port_d_tx(msg);
    sbg_status->port_e_rx = mavlink_msg_sbg_status_get_port_e_rx(msg);
    sbg_status->port_e_tx = mavlink_msg_sbg_status_get_port_e_tx(msg);
    sbg_status->can_rx = mavlink_msg_sbg_status_get_can_rx(msg);
    sbg_status->can_tx = mavlink_msg_sbg_status_get_can_tx(msg);
    sbg_status->can_status = mavlink_msg_sbg_status_get_can_status(msg);
    sbg_status->gps1_pos_recv = mavlink_msg_sbg_status_get_gps1_pos_recv(msg);
    sbg_status->gps1_vel_recv = mavlink_msg_sbg_status_get_gps1_vel_recv(msg);
    sbg_status->gps1_hdt_recv = mavlink_msg_sbg_status_get_gps1_hdt_recv(msg);
    sbg_status->gps1_utc_recv = mavlink_msg_sbg_status_get_gps1_utc_recv(msg);
    sbg_status->mag_recv = mavlink_msg_sbg_status_get_mag_recv(msg);
    sbg_status->odo_recv = mavlink_msg_sbg_status_get_odo_recv(msg);
    sbg_status->dvl_recv = mavlink_msg_sbg_status_get_dvl_recv(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_SBG_STATUS_LEN? msg->len : MAVLINK_MSG_ID_SBG_STATUS_LEN;
        memset(sbg_status, 0, MAVLINK_MSG_ID_SBG_STATUS_LEN);
    memcpy(sbg_status, _MAV_PAYLOAD(msg), len);
#endif
}
