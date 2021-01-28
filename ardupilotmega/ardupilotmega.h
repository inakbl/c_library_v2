/** @file
 *  @brief MAVLink comm protocol generated from ardupilotmega.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_ARDUPILOTMEGA_H
#define MAVLINK_ARDUPILOTMEGA_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_ARDUPILOTMEGA.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 1

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{150, 134, 42, 0, 0, 0}, {151, 219, 8, 3, 6, 7}, {152, 208, 4, 0, 0, 0}, {153, 188, 12, 0, 0, 0}, {154, 84, 15, 3, 6, 7}, {155, 22, 13, 3, 4, 5}, {156, 19, 6, 3, 0, 1}, {157, 21, 15, 3, 12, 13}, {158, 134, 14, 3, 12, 13}, {160, 78, 12, 3, 8, 9}, {161, 68, 3, 3, 0, 1}, {163, 127, 28, 0, 0, 0}, {164, 154, 44, 0, 0, 0}, {165, 21, 3, 0, 0, 0}, {166, 21, 9, 0, 0, 0}, {167, 144, 22, 0, 0, 0}, {168, 1, 12, 0, 0, 0}, {169, 234, 18, 0, 0, 0}, {170, 73, 34, 0, 0, 0}, {171, 181, 66, 0, 0, 0}, {172, 22, 98, 0, 0, 0}, {173, 83, 8, 0, 0, 0}, {174, 167, 48, 0, 0, 0}, {175, 138, 19, 3, 14, 15}, {176, 234, 3, 3, 0, 1}, {177, 240, 20, 0, 0, 0}, {178, 47, 24, 0, 0, 0}, {179, 189, 29, 1, 26, 0}, {180, 52, 45, 1, 42, 0}, {181, 174, 4, 0, 0, 0}, {182, 229, 40, 0, 0, 0}, {183, 85, 2, 3, 0, 1}, {184, 159, 206, 3, 4, 5}, {185, 186, 7, 3, 4, 5}, {186, 72, 29, 3, 0, 1}, {191, 92, 27, 0, 0, 0}, {193, 71, 22, 0, 0, 0}, {194, 98, 25, 0, 0, 0}, {195, 120, 37, 0, 0, 0}, {200, 134, 42, 3, 40, 41}, {201, 205, 14, 3, 12, 13}, {214, 69, 8, 3, 6, 7}, {215, 101, 3, 0, 0, 0}, {216, 50, 3, 3, 0, 1}, {217, 202, 6, 0, 0, 0}, {218, 17, 7, 3, 0, 1}, {219, 162, 2, 0, 0, 0}, {226, 207, 8, 0, 0, 0}, {11000, 134, 51, 3, 4, 5}, {11001, 15, 135, 0, 0, 0}, {11002, 234, 179, 3, 4, 5}, {11003, 64, 5, 0, 0, 0}, {11010, 46, 49, 0, 0, 0}, {11011, 106, 44, 0, 0, 0}, {11020, 205, 16, 0, 0, 0}, {11030, 144, 44, 0, 0, 0}, {11031, 133, 44, 0, 0, 0}, {11032, 85, 44, 0, 0, 0}, {11033, 195, 37, 3, 16, 17}, {11034, 79, 5, 0, 0, 0}, {11035, 128, 8, 3, 4, 5}, {11036, 177, 34, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_ARDUPILOTMEGA

// ENUM DEFINITIONS


/** @brief  */
#ifndef HAVE_ENUM_ACCELCAL_VEHICLE_POS
#define HAVE_ENUM_ACCELCAL_VEHICLE_POS
typedef enum ACCELCAL_VEHICLE_POS
{
   ACCELCAL_VEHICLE_POS_LEVEL=1, /*  | */
   ACCELCAL_VEHICLE_POS_LEFT=2, /*  | */
   ACCELCAL_VEHICLE_POS_RIGHT=3, /*  | */
   ACCELCAL_VEHICLE_POS_NOSEDOWN=4, /*  | */
   ACCELCAL_VEHICLE_POS_NOSEUP=5, /*  | */
   ACCELCAL_VEHICLE_POS_BACK=6, /*  | */
   ACCELCAL_VEHICLE_POS_SUCCESS=16777215, /*  | */
   ACCELCAL_VEHICLE_POS_FAILED=16777216, /*  | */
   ACCELCAL_VEHICLE_POS_ENUM_END=16777217, /*  | */
} ACCELCAL_VEHICLE_POS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_HEADING_TYPE
#define HAVE_ENUM_HEADING_TYPE
typedef enum HEADING_TYPE
{
   HEADING_TYPE_COURSE_OVER_GROUND=0, /*  | */
   HEADING_TYPE_HEADING=1, /*  | */
   HEADING_TYPE_ENUM_END=2, /*  | */
} HEADING_TYPE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_SPEED_TYPE
#define HAVE_ENUM_SPEED_TYPE
typedef enum SPEED_TYPE
{
   SPEED_TYPE_AIRSPEED=0, /*  | */
   SPEED_TYPE_GROUNDSPEED=1, /*  | */
   SPEED_TYPE_ENUM_END=2, /*  | */
} SPEED_TYPE;
#endif

/** @brief Commands to be executed by the MAV. They can be executed on user request, or as part of a mission script. If the action is used in a mission, the parameter mapping to the waypoint/mission message is as follows: Param 1, Param 2, Param 3, Param 4, X: Param 5, Y:Param 6, Z:Param 7. This command list is similar what ARINC 424 is for commercial aircraft: A data format how to interpret waypoint/mission data. NaN and INT32_MAX may be used in float/integer params (respectively) to indicate optional/default values (e.g. to use the component's current yaw or latitude rather than a specific value). See https://mavlink.io/en/guide/xml_schema.html#MAV_CMD for information about the structure of the MAV_CMD entries */
#ifndef HAVE_ENUM_MAV_CMD
#define HAVE_ENUM_MAV_CMD
typedef enum MAV_CMD
{
   MAV_CMD_NAV_WAYPOINT=16, /* Navigate to waypoint. |1: Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)| 2: Acceptance radius (if the sphere with this radius is hit, the waypoint counts as reached)| 3: 0 radius to pass by WP. Positive value for clockwise orbit, negative value for counter-clockwise orbit. Allows trajectory control.| 4: Desired yaw angle at waypoint (rotary wing). NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_NAV_LOITER_UNLIM=17, /* Loiter around this waypoint an unlimited amount of time |1: Empty| 2: Empty| 3: Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise| 4: Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_NAV_LOITER_TURNS=18, /* Loiter around this waypoint for X turns |1: Number of turns.| 2: Leave loiter circle only once heading towards the next waypoint (0 = False)| 3: Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise| 4: nter xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_NAV_LOITER_TIME=19, /* Loiter at the specified latitude, longitude and altitude for a certain amount of time. Multicopter vehicles stop at the point (within a vehicle-specific acceptance radius). Forward-only moving vehicles (e.g. fixed-wing) circle the point with the specified radius/direction. If the Heading Required parameter (2) is non-zero forward moving aircraft will only leave the loiter circle once heading towards the next waypoint. |1: Loiter time (only starts once Lat, Lon and Alt is reached).| 2: Leave loiter circle only once heading towards the next waypoint (0 = False)| 3: Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, else counter-clockwise.| 4: tem default xtrack behaviour.| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_NAV_RETURN_TO_LAUNCH=20, /* Return to launch location |1: Empty| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_NAV_LAND=21, /* Land at location. |1: Minimum target altitude if landing is aborted (0 = undefined/use system default).| 2: Precision land mode.| 3: Empty| 4: Desired yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| 5: Latitude.| 6: Longitude.| 7: Landing altitude (ground level in current frame).|  */
   MAV_CMD_NAV_TAKEOFF=22, /* Takeoff from ground / hand. Vehicles that support multiple takeoff modes (e.g. VTOL quadplane) should take off using the currently configured mode. |1: Minimum pitch (if airspeed sensor present), desired pitch without sensor| 2: Empty| 3: Empty| 4: Yaw angle (if magnetometer present), ignored without magnetometer. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_NAV_LAND_LOCAL=23, /* Land at local position (local frame only) |1: Landing target number (if available)| 2: Maximum accepted offset from desired landing position - computed magnitude from spherical coordinates: d = sqrt(x^2 + y^2 + z^2), which gives the maximum accepted distance between the desired landing position and the position where the vehicle is about to land| 3: Landing descend rate| 4: Desired yaw angle| 5: Y-axis position| 6: X-axis position| 7: Z-axis / ground level position|  */
   MAV_CMD_NAV_TAKEOFF_LOCAL=24, /* Takeoff from local position (local frame only) |1: Minimum pitch (if airspeed sensor present), desired pitch without sensor| 2: Empty| 3: Takeoff ascend rate| 4: Yaw angle (if magnetometer or another yaw estimation source present), ignored without one of these| 5: Y-axis position| 6: X-axis position| 7: Z-axis position|  */
   MAV_CMD_NAV_FOLLOW=25, /* Vehicle following, i.e. this waypoint represents the position of a moving vehicle |1: Following logic to use (e.g. loitering or sinusoidal following) - depends on specific autopilot implementation| 2: Ground speed of vehicle to be followed| 3: Radius around waypoint. If positive loiter clockwise, else counter-clockwise| 4: Desired yaw angle.| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT=30, /* Continue on the current course and climb/descend to specified altitude.  When the altitude is reached continue to the next command (i.e., don't proceed to the next command until the desired altitude is reached. |1: Climb or Descend (0 = Neutral, command completes when within 5m of this command's altitude, 1 = Climbing, command completes when at or above this command's altitude, 2 = Descending, command completes when at or below this command's altitude.| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Desired altitude|  */
   MAV_CMD_NAV_LOITER_TO_ALT=31, /* Begin loiter at the specified Latitude and Longitude.  If Lat=Lon=0, then loiter at the current position.  Don't consider the navigation command complete (don't leave loiter) until the altitude has been reached. Additionally, if the Heading Required parameter is non-zero the aircraft will not leave the loiter until heading toward the next waypoint. |1: Leave loiter circle only once heading towards the next waypoint (0 = False)| 2: Loiter radius around waypoint for forward-only moving vehicles (not multicopters). If positive loiter clockwise, negative counter-clockwise, 0 means no change to standard loiter.| 3: Empty| 4: Loiter circle exit location and/or path to next waypoint ("xtrack") for forward-only moving vehicles (not multicopters). 0 for the vehicle to converge towards the center xtrack when it leaves the loiter (the line between the centers of the current and next waypoint), 1 to converge to the direct line between the location that the vehicle exits the loiter radius and the next waypoint. Otherwise the angle (in degrees) between the tangent of the loiter circle and the center xtrack at which the vehicle must leave the loiter (and converge to the center xtrack). NaN to use the current system default xtrack behaviour.| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_DO_FOLLOW=32, /* Begin following a target |1: System ID (of the FOLLOW_TARGET beacon). Send 0 to disable follow-me and return to the default position hold mode.| 2: Reserved| 3: Reserved| 4: Altitude mode: 0: Keep current altitude, 1: keep altitude difference to target, 2: go to a fixed altitude above home.| 5: Altitude above home. (used if mode=2)| 6: Reserved| 7: Time to land in which the MAV should go to the default position hold mode after a message RX timeout.|  */
   MAV_CMD_DO_FOLLOW_REPOSITION=33, /* Reposition the MAV after a follow target command has been sent |1: Camera q1 (where 0 is on the ray from the camera to the tracking device)| 2: Camera q2| 3: Camera q3| 4: Camera q4| 5: altitude offset from target| 6: X offset from target| 7: Y offset from target|  */
   MAV_CMD_DO_ORBIT=34, /* Start orbiting on the circumference of a circle defined by the parameters. Setting any value NaN results in using defaults. |1: Radius of the circle. positive: Orbit clockwise. negative: Orbit counter-clockwise.| 2: Tangential Velocity. NaN: Vehicle configuration default.| 3: Yaw behavior of the vehicle.| 4: Reserved (e.g. for dynamic center beacon options)| 5: Center point latitude (if no MAV_FRAME specified) / X coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.| 6: Center point longitude (if no MAV_FRAME specified) / Y coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.| 7: Center point altitude (MSL) (if no MAV_FRAME specified) / Z coordinate according to MAV_FRAME. NaN: Use current vehicle position or current center if already orbiting.|  */
   MAV_CMD_NAV_ROI=80, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. |1: Region of interest mode.| 2: Waypoint index/ target ID. (see MAV_ROI enum)| 3: ROI index (allows a vehicle to manage multiple ROI's)| 4: Empty| 5: x the location of the fixed ROI (see MAV_FRAME)| 6: y| 7: z|  */
   MAV_CMD_NAV_PATHPLANNING=81, /* Control autonomous path planning on the MAV. |1: 0: Disable local obstacle avoidance / local path planning (without resetting map), 1: Enable local path planning, 2: Enable and reset local path planning| 2: 0: Disable full path planning (without resetting map), 1: Enable, 2: Enable and reset map/occupancy grid, 3: Enable and reset planned route, but not occupancy grid| 3: Empty| 4: Yaw angle at goal| 5: Latitude/X of goal| 6: Longitude/Y of goal| 7: Altitude/Z of goal|  */
   MAV_CMD_NAV_SPLINE_WAYPOINT=82, /* Navigate to waypoint using a spline path. |1: Hold time. (ignored by fixed wing, time to stay at waypoint for rotary wing)| 2: Empty| 3: Empty| 4: Empty| 5: Latitude/X of goal| 6: Longitude/Y of goal| 7: Altitude/Z of goal|  */
   MAV_CMD_NAV_ALTITUDE_WAIT=83, /* Mission command to wait for an altitude or downwards vertical speed. This is meant for high altitude balloon launches, allowing the aircraft to be idle until either an altitude is reached or a negative vertical speed is reached (indicating early balloon burst). The wiggle time is how often to wiggle the control surfaces to prevent them seizing up. |1: Altitude.| 2: Descent speed.| 3: How long to wiggle the control surfaces to prevent them seizing up.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_NAV_VTOL_TAKEOFF=84, /* Takeoff from ground using VTOL mode, and transition to forward flight with specified heading. The command should be ignored by vehicles that dont support both VTOL and fixed-wing flight (multicopters, boats,etc.). |1: Empty| 2: Front transition heading.| 3: Empty| 4: Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_NAV_VTOL_LAND=85, /* Land using VTOL mode |1: Empty| 2: Empty| 3: Approach altitude (with the same reference as the Altitude field). NaN if unspecified.| 4: Yaw angle. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.).| 5: Latitude| 6: Longitude| 7: Altitude (ground level)|  */
   MAV_CMD_NAV_GUIDED_ENABLE=92, /* hand control over to an external controller |1: 0.5f on)| 2: Empty| 3: Empty| 4: y| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_NAV_DELAY=93, /* Delay the next navigation command a number of seconds or until a specified time |1: Delay (-1 to enable time-of-day fields)| 2: hour (24h format, UTC, -1 to ignore)| 3: minute (24h format, UTC, -1 to ignore)| 4: second (24h format, UTC, -1 to ignore)| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_NAV_PAYLOAD_PLACE=94, /* Descend and place payload. Vehicle moves to specified location, descends until it detects a hanging payload has reached the ground, and then releases the payload. If ground is not detected before the reaching the maximum descent value (param1), the command will complete without releasing the payload. |1: Maximum distance to descend.| 2: Empty| 3: Empty| 4: Empty| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_NAV_LAST=95, /* NOP - This command is only used to mark the upper limit of the NAV/ACTION commands in the enumeration |1: Empty| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_CONDITION_DELAY=112, /* Delay mission state machine. |1: Delay| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_CONDITION_CHANGE_ALT=113, /* Ascend/descend to target altitude at specified rate. Delay mission state machine until desired altitude reached. |1: Descent / Ascend rate.| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Target Altitude|  */
   MAV_CMD_CONDITION_DISTANCE=114, /* Delay mission state machine until within desired distance of next NAV point. |1: Distance.| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_CONDITION_YAW=115, /* Reach a certain target angle. |1: target angle, 0 is north| 2: angular speed| 3: direction: -1: counter clockwise, 1: clockwise| 4: 0: absolute angle, 1: relative offset| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_CONDITION_LAST=159, /* NOP - This command is only used to mark the upper limit of the CONDITION commands in the enumeration |1: Empty| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_SET_MODE=176, /* Set system mode. |1: Mode| 2: Custom mode - this is system specific, please refer to the individual autopilot specifications for details.| 3: Custom sub mode - this is system specific, please refer to the individual autopilot specifications for details.| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_JUMP=177, /* Jump to the desired command in the mission list.  Repeat this action only the specified number of times |1: Sequence number| 2: Repeat count| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_CHANGE_SPEED=178, /* Change speed and/or throttle set points. |1: Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)| 2: Speed (-1 indicates no change)| 3: Throttle (-1 indicates no change)| 4: 0: absolute, 1: relative| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_SET_HOME=179, /* Changes the home location either to the current location or a specified location. |1: Use current (1=use current location, 0=use specified location)| 2: Empty| 3: Empty| 4: Yaw angle. NaN to use default heading| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_DO_SET_PARAMETER=180, /* Set a system parameter.  Caution!  Use of this command requires knowledge of the numeric enumeration value of the parameter. |1: Parameter number| 2: Parameter value| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_SET_RELAY=181, /* Set a relay to a condition. |1: Relay instance number.| 2: Setting. (1=on, 0=off, others possible depending on system hardware)| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_REPEAT_RELAY=182, /* Cycle a relay on and off for a desired number of cycles with a desired period. |1: Relay instance number.| 2: Cycle count.| 3: Cycle time.| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_SET_SERVO=183, /* Set a servo to a desired PWM value. |1: Servo instance number.| 2: Pulse Width Modulation.| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_REPEAT_SERVO=184, /* Cycle a between its nominal setting and a desired PWM for a desired number of cycles with a desired period. |1: Servo instance number.| 2: Pulse Width Modulation.| 3: Cycle count.| 4: Cycle time.| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_FLIGHTTERMINATION=185, /* Terminate flight immediately |1: 0.5| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_CHANGE_ALTITUDE=186, /* Change altitude set point. |1: Altitude| 2: Frame of new altitude.| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_SET_ACTUATOR=187, /* Sets actuators (e.g. servos) to a desired value. The actuator numbers are mapped to specific outputs (e.g. on any MAIN or AUX PWM or UAVCAN) using a flight-stack specific mechanism (i.e. a parameter). |1: Actuator 1 value, scaled from [-1 to 1]. NaN to ignore.| 2: Actuator 2 value, scaled from [-1 to 1]. NaN to ignore.| 3: Actuator 3 value, scaled from [-1 to 1]. NaN to ignore.| 4: Actuator 4 value, scaled from [-1 to 1]. NaN to ignore.| 5: Actuator 5 value, scaled from [-1 to 1]. NaN to ignore.| 6: Actuator 6 value, scaled from [-1 to 1]. NaN to ignore.| 7: Index of actuator set (i.e if set to 1, Actuator 1 becomes Actuator 7)|  */
   MAV_CMD_DO_LAND_START=189, /* Mission command to perform a landing. This is used as a marker in a mission to tell the autopilot where a sequence of mission items that represents a landing starts. It may also be sent via a COMMAND_LONG to trigger a landing, in which case the nearest (geographically) landing sequence in the mission will be used. The Latitude/Longitude is optional, and may be set to 0 if not needed. If specified then it will be used to help find the closest landing sequence. |1: Empty| 2: Empty| 3: Empty| 4: Empty| 5: Latitude| 6: Longitude| 7: Empty|  */
   MAV_CMD_DO_RALLY_LAND=190, /* Mission command to perform a landing from a rally point. |1: Break altitude| 2: Landing speed| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_GO_AROUND=191, /* Mission command to safely abort an autonomous landing. |1: Altitude| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_REPOSITION=192, /* Reposition the vehicle to a specific WGS84 global position. |1: Ground speed, less than 0 (-1) for default| 2: Bitmask of option flags.| 3: Reserved| 4: heading. NaN to use the current system yaw heading mode (e.g. yaw towards next waypoint, yaw to home, etc.). For planes indicates loiter direction (0: clockwise, 1: counter clockwise)| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_DO_PAUSE_CONTINUE=193, /* If in a GPS controlled position mode, hold the current position or continue. |1: 0: Pause current mission or reposition command, hold current position. 1: Continue mission. A VTOL capable vehicle should enter hover mode (multicopter and VTOL planes). A plane should loiter with the default loiter radius.| 2: Reserved| 3: Reserved| 4: Reserved| 5: Reserved| 6: Reserved| 7: Reserved|  */
   MAV_CMD_DO_SET_REVERSE=194, /* Set moving direction to forward or reverse. |1: Direction (0=Forward, 1=Reverse)| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_SET_ROI_LOCATION=195, /* Sets the region of interest (ROI) to a location. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal is not to react to this message. |1: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).| 2: Empty| 3: Empty| 4: Empty| 5: Latitude of ROI location| 6: Longitude of ROI location| 7: Altitude of ROI location|  */
   MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET=196, /* Sets the region of interest (ROI) to be toward next waypoint, with optional pitch/roll/yaw offset. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. |1: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).| 2: Empty| 3: Empty| 4: Empty| 5: Pitch offset from next waypoint, positive pitching up| 6: roll offset from next waypoint, positive rolling to the right| 7: yaw offset from next waypoint, positive yawing to the right|  */
   MAV_CMD_DO_SET_ROI_NONE=197, /* Cancels any previous ROI command returning the vehicle/sensors to default flight characteristics. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. After this command the gimbal manager should go back to manual input if available, and otherwise assume a neutral position. |1: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_SET_ROI_SYSID=198, /* Mount tracks system with specified system ID. Determination of target vehicle position may be done with GLOBAL_POSITION_INT or any other means. This command can be sent to a gimbal manager but not to a gimbal device. A gimbal device is not to react to this message. |1: System ID| 2: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_DO_CONTROL_VIDEO=200, /* Control onboard camera system. |1: Camera ID (-1 for all)| 2: Transmission: 0: disabled, 1: enabled compressed, 2: enabled raw| 3: 0: single images every n seconds| 4: Recording: 0: disabled, 1: enabled compressed, 2: enabled raw| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_SET_ROI=201, /* Sets the region of interest (ROI) for a sensor set or the vehicle itself. This can then be used by the vehicle's control system to control the vehicle attitude and the attitude of various sensors such as cameras. |1: Region of interest mode.| 2: Waypoint index/ target ID (depends on param 1).| 3: Region of interest index. (allows a vehicle to manage multiple ROI's)| 4: Empty| 5: MAV_ROI_WPNEXT: pitch offset from next waypoint, MAV_ROI_LOCATION: latitude| 6: MAV_ROI_WPNEXT: roll offset from next waypoint, MAV_ROI_LOCATION: longitude| 7: MAV_ROI_WPNEXT: yaw offset from next waypoint, MAV_ROI_LOCATION: altitude|  */
   MAV_CMD_DO_DIGICAM_CONFIGURE=202, /* Configure digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). |1: Modes: P, TV, AV, M, Etc.| 2: Shutter speed: Divisor number for one second.| 3: Aperture: F stop number.| 4: ISO number e.g. 80, 100, 200, Etc.| 5: Exposure type enumerator.| 6: Command Identity.| 7: Main engine cut-off time before camera trigger. (0 means no cut-off)|  */
   MAV_CMD_DO_DIGICAM_CONTROL=203, /* Control digital camera. This is a fallback message for systems that have not yet implemented PARAM_EXT_XXX messages and camera definition files (see https://mavlink.io/en/services/camera_def.html ). |1: Session control e.g. show/hide lens| 2: Zoom's absolute position| 3: Zooming step value to offset zoom from the current position| 4: Focus Locking, Unlocking or Re-locking| 5: Shooting Command| 6: Command Identity| 7: Test shot identifier. If set to 1, image will only be captured, but not counted towards internal frame count.|  */
   MAV_CMD_DO_MOUNT_CONFIGURE=204, /* Mission command to configure a camera or antenna mount |1: Mount operation mode| 2: stabilize roll? (1 = yes, 0 = no)| 3: stabilize pitch? (1 = yes, 0 = no)| 4: stabilize yaw? (1 = yes, 0 = no)| 5: roll input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)| 6: pitch input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)| 7: yaw input (0 = angle body frame, 1 = angular rate, 2 = angle absolute frame)|  */
   MAV_CMD_DO_MOUNT_CONTROL=205, /* Mission command to control a camera or antenna mount |1: pitch depending on mount mode (degrees or degrees/second depending on pitch input).| 2: roll depending on mount mode (degrees or degrees/second depending on roll input).| 3: yaw depending on mount mode (degrees or degrees/second depending on yaw input).| 4: altitude depending on mount mode.| 5: latitude, set if appropriate mount mode.| 6: longitude, set if appropriate mount mode.| 7: Mount mode.|  */
   MAV_CMD_DO_SET_CAM_TRIGG_DIST=206, /* Mission command to set camera trigger distance for this flight. The camera is triggered each time this distance is exceeded. This command can also be used to set the shutter integration time for the camera. |1: Camera trigger distance. 0 to stop triggering.| 2: Camera shutter integration time. -1 or 0 to ignore| 3: Trigger camera once immediately. (0 = no trigger, 1 = trigger)| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_FENCE_ENABLE=207, /* Mission command to enable the geofence |1: enable? (0=disable, 1=enable, 2=disable_floor_only)| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_PARACHUTE=208, /* Mission item/command to release a parachute or enable/disable auto release. |1: Action| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_MOTOR_TEST=209, /* Mission command to perform motor test. |1: Motor instance number. (from 1 to max number of motors on the vehicle)| 2: Throttle type.| 3: Throttle.| 4: Timeout.| 5: Motor count. (number of motors to test to test in sequence, waiting for the timeout above between them; 0=1 motor, 1=1 motor, 2=2 motors...)| 6: Motor test order.| 7: Empty|  */
   MAV_CMD_DO_INVERTED_FLIGHT=210, /* Change to/from inverted flight. |1: Inverted flight. (0=normal, 1=inverted)| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_GRIPPER=211, /* Mission command to operate a gripper. |1: Gripper instance number.| 2: Gripper action to perform.| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_AUTOTUNE_ENABLE=212, /* Enable/disable autotune. |1: Enable (1: enable, 0:disable).| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_NAV_SET_YAW_SPEED=213, /* Sets a desired vehicle turn angle and speed change. |1: Yaw angle to adjust steering by.| 2: Speed.| 3: Final angle. (0=absolute, 1=relative)| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL=214, /* Mission command to set camera trigger interval for this flight. If triggering is enabled, the camera is triggered each time this interval expires. This command can also be used to set the shutter integration time for the camera. |1: Camera trigger cycle time. -1 or 0 to ignore.| 2: Camera shutter integration time. Should be less than trigger cycle time. -1 or 0 to ignore.| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_SET_RESUME_REPEAT_DIST=215, /* Set the distance to be repeated on mission resume |1: Distance.| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_DO_MOUNT_CONTROL_QUAT=220, /* Mission command to control a camera or antenna mount, using a quaternion as reference. |1: quaternion param q1, w (1 in null-rotation)| 2: quaternion param q2, x (0 in null-rotation)| 3: quaternion param q3, y (0 in null-rotation)| 4: quaternion param q4, z (0 in null-rotation)| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_GUIDED_MASTER=221, /* set id of master controller |1: System ID| 2: Component ID| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_GUIDED_LIMITS=222, /* Set limits for external control |1: Timeout - maximum time that external controller will be allowed to control vehicle. 0 means no timeout.| 2: Altitude (MSL) min - if vehicle moves below this alt, the command will be aborted and the mission will continue. 0 means no lower altitude limit.| 3: Altitude (MSL) max - if vehicle moves above this alt, the command will be aborted and the mission will continue. 0 means no upper altitude limit.| 4: Horizontal move limit - if vehicle moves more than this distance from its location at the moment the command was executed, the command will be aborted and the mission will continue. 0 means no horizontal move limit.| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_ENGINE_CONTROL=223, /* Control vehicle engine. This is interpreted by the vehicles engine controller to change the target engine state. It is intended for vehicles with internal combustion engines |1: 0: Stop engine, 1:Start Engine| 2: 0: Warm start, 1:Cold start. Controls use of choke where applicable| 3: Height delay. This is for commanding engine start only after the vehicle has gained the specified height. Used in VTOL vehicles during takeoff to start engine after the aircraft is off the ground. Zero for no delay.| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_SET_MISSION_CURRENT=224, /* Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between). |1: Mission sequence value to set| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_DO_LAST=240, /* NOP - This command is only used to mark the upper limit of the DO commands in the enumeration |1: Empty| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_PREFLIGHT_CALIBRATION=241, /* Trigger calibration. This command will be only accepted if in pre-flight mode. Except for Temperature Calibration, only one sensor should be set in a single message and all others should be zero. |1: 1: gyro calibration, 3: gyro temperature calibration| 2: 1: magnetometer calibration| 3: pressure calibration| 4: 1: radio RC calibration, 2: RC trim calibration| 5: 1: accelerometer calibration, 2: board level calibration, 3: accelerometer temperature calibration, 4: simple accelerometer calibration| 6: 1: APM: compass/motor interference calibration (PX4: airspeed calibration, deprecated), 2: airspeed calibration| 7: 1: ESC calibration, 3: barometer temperature calibration|  */
   MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS=242, /* Set sensor offsets. This command will be only accepted if in pre-flight mode. |1: Sensor to adjust the offsets for: 0: gyros, 1: accelerometer, 2: magnetometer, 3: barometer, 4: optical flow, 5: second magnetometer, 6: third magnetometer| 2: X axis offset (or generic dimension 1), in the sensor's raw units| 3: Y axis offset (or generic dimension 2), in the sensor's raw units| 4: Z axis offset (or generic dimension 3), in the sensor's raw units| 5: Generic dimension 4, in the sensor's raw units| 6: Generic dimension 5, in the sensor's raw units| 7: Generic dimension 6, in the sensor's raw units|  */
   MAV_CMD_PREFLIGHT_UAVCAN=243, /* Trigger UAVCAN configuration (actuator ID assignment and direction mapping). Note that this maps to the legacy UAVCAN v0 function UAVCAN_ENUMERATE, which is intended to be executed just once during initial vehicle configuration (it is not a normal pre-flight command and has been poorly named). |1: 1: Trigger actuator ID assignment and direction mapping. 0: Cancel command.| 2: Reserved| 3: Reserved| 4: Reserved| 5: Reserved| 6: Reserved| 7: Reserved|  */
   MAV_CMD_PREFLIGHT_STORAGE=245, /* Request storage of different parameter values and logs. This command will be only accepted if in pre-flight mode. |1: Parameter storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| 2: Mission storage: 0: READ FROM FLASH/EEPROM, 1: WRITE CURRENT TO FLASH/EEPROM, 2: Reset to defaults| 3: 1: logging rate (e.g. set to 1000 for 1000 Hz logging)| 4: Reserved| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN=246, /* Request the reboot or shutdown of system components. |1: for autopilot, 1: Reboot autopilot, 2: Shutdown autopilot, 3: Reboot autopilot and keep it in the bootloader until upgraded.| 2: 0: Do nothing for onboard computer, 1: Reboot onboard computer, 2: Shutdown onboard computer, 3: Reboot onboard computer and keep it in the bootloader until upgraded.| 3: WIP: 0: Do nothing for camera, 1: Reboot onboard camera, 2: Shutdown onboard camera, 3: Reboot onboard camera and keep it in the bootloader until upgraded| 4: WIP: 0: Do nothing for mount (e.g. gimbal), 1: Reboot mount, 2: Shutdown mount, 3: Reboot mount and keep it in the bootloader until upgraded| 5: Reserved (set to 0)| 6: Reserved (set to 0)| 7: WIP: ID (e.g. camera ID -1 for all IDs)|  */
   MAV_CMD_DO_UPGRADE=247, /* Request a target system to start an upgrade of one (or all) of its components. For example, the command might be sent to a companion computer to cause it to upgrade a connected flight controller. The system doing the upgrade will report progress using the normal command protocol sequence for a long running operation. Command protocol information: https://mavlink.io/en/services/command.html. |1: Component id of the component to be upgraded. If set to 0, all components should be upgraded.| 2: 0: Do not reboot component after the action is executed, 1: Reboot component after the action is executed.| 3: Reserved| 4: Reserved| 5: Reserved| 6: Reserved| 7: WIP: upgrade progress report rate (can be used for more granular control).|  */
   MAV_CMD_OVERRIDE_GOTO=252, /* Override current mission with command to pause mission, pause mission and move to position, continue/resume mission. When param 1 indicates that the mission is paused (MAV_GOTO_DO_HOLD), param 2 defines whether it holds in place or moves to another position. |1: MAV_GOTO_DO_HOLD: pause mission and either hold or move to specified position (depending on param2), MAV_GOTO_DO_CONTINUE: resume mission.| 2: MAV_GOTO_HOLD_AT_CURRENT_POSITION: hold at current position, MAV_GOTO_HOLD_AT_SPECIFIED_POSITION: hold at specified position.| 3: Coordinate frame of hold point.| 4: Desired yaw angle.| 5: Latitude/X position.| 6: Longitude/Y position.| 7: Altitude/Z position.|  */
   MAV_CMD_OBLIQUE_SURVEY=260, /* Mission command to set a Camera Auto Mount Pivoting Oblique Survey (Replaces CAM_TRIGG_DIST for this purpose). The camera is triggered each time this distance is exceeded, then the mount moves to the next position. Params 4~6 set-up the angle limits and number of positions for oblique survey, where mount-enabled vehicles automatically roll the camera between shots to emulate an oblique camera setup (providing an increased HFOV). This command can also be used to set the shutter integration time for the camera. |1: Camera trigger distance. 0 to stop triggering.| 2: Camera shutter integration time. 0 to ignore| 3: The minimum interval in which the camera is capable of taking subsequent pictures repeatedly. 0 to ignore.| 4: Total number of roll positions at which the camera will capture photos (images captures spread evenly across the limits defined by param5).| 5: Angle limits that the camera can be rolled to left and right of center.| 6: Fixed pitch angle that the camera will hold in oblique mode if the mount is actuated in the pitch axis.| 7: Empty|  */
   MAV_CMD_MISSION_START=300, /* start running a mission |1: first_item: the first mission item to run| 2: last_item:  the last mission item to run (after this item is run, the mission ends)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_COMPONENT_ARM_DISARM=400, /* Arms / Disarms a component |1: 0: disarm, 1: arm| 2: 0: arm-disarm unless prevented by safety checks (i.e. when landed), 21196: force arming/disarming (e.g. allow arming to override preflight checks and disarming in flight)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_ILLUMINATOR_ON_OFF=405, /* Turns illuminators ON/OFF. An illuminator is a light source that is used for lighting up dark areas external to the sytstem: e.g. a torch or searchlight (as opposed to a light source for illuminating the system itself, e.g. an indicator light). |1: 0: Illuminators OFF, 1: Illuminators ON| 2: Reserved (default:0)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_GET_HOME_POSITION=410, /* Request the home position from the vehicle. |1: Reserved| 2: Reserved| 3: Reserved| 4: Reserved| 5: Reserved| 6: Reserved| 7: Reserved|  */
   MAV_CMD_INJECT_FAILURE=420, /* Inject artificial failure for testing purposes. Note that autopilots should implement an additional protection before accepting this command such as a specific param setting. |1: The unit which is affected by the failure.| 2: failure manifests itself.| 3: Instance affected by failure (0 to signal all).| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_START_RX_PAIR=500, /* Starts receiver pairing. |1: 0:Spektrum.| 2: RC type.| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_GET_MESSAGE_INTERVAL=510, /* Request the interval between messages for a particular MAVLink message ID. The receiver should ACK the command and then emit its response in a MESSAGE_INTERVAL message. |1: The MAVLink message ID| 2: Reserved (default:0)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_SET_MESSAGE_INTERVAL=511, /* Set the interval between messages for a particular MAVLink message ID. This interface replaces REQUEST_DATA_STREAM. |1: The MAVLink message ID| 2: The interval between two messages. Set to -1 to disable and 0 to request default rate.| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Target address of message stream (if message has target address fields). 0: Flight-stack default (recommended), 1: address of requestor, 2: broadcast.|  */
   MAV_CMD_REQUEST_MESSAGE=512, /* Request the target system(s) emit a single instance of a specified message (i.e. a "one-shot" version of MAV_CMD_SET_MESSAGE_INTERVAL). |1: The MAVLink message ID of the requested message.| 2: Use for index ID, if required. Otherwise, the use of this parameter (if any) must be defined in the requested message. By default assumed not used (0).| 3: The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| 4: The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| 5: The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| 6: The use of this parameter (if any), must be defined in the requested message. By default assumed not used (0).| 7: Target address for requested message (if message has target address fields). 0: Flight-stack default, 1: address of requestor, 2: broadcast.|  */
   MAV_CMD_REQUEST_PROTOCOL_VERSION=519, /* Request MAVLink protocol version compatibility. All receivers should ACK the command and then emit their capabilities in an PROTOCOL_VERSION message |1: 1: Request supported protocol versions by all nodes on the network| 2: Reserved (all remaining params)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES=520, /* Request autopilot capabilities. The receiver should ACK the command and then emit its capabilities in an AUTOPILOT_VERSION message |1: 1: Request autopilot version| 2: Reserved (all remaining params)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_REQUEST_CAMERA_INFORMATION=521, /* Request camera information (CAMERA_INFORMATION). |1: 0: No action 1: Request camera capabilities| 2: Reserved (all remaining params)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_REQUEST_CAMERA_SETTINGS=522, /* Request camera settings (CAMERA_SETTINGS). |1: 0: No Action 1: Request camera settings| 2: Reserved (all remaining params)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_REQUEST_STORAGE_INFORMATION=525, /* Request storage information (STORAGE_INFORMATION). Use the command's target_component to target a specific component's storage. |1: Storage ID (0 for all, 1 for first, 2 for second, etc.)| 2: 0: No Action 1: Request storage information| 3: Reserved (all remaining params)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_STORAGE_FORMAT=526, /* Format a storage medium. Once format is complete, a STORAGE_INFORMATION message is sent. Use the command's target_component to target a specific component's storage. |1: Storage ID (1 for first, 2 for second, etc.)| 2: Format storage (and reset image log). 0: No action 1: Format storage| 3: Reset Image Log (without formatting storage medium). This will reset CAMERA_CAPTURE_STATUS.image_count and CAMERA_IMAGE_CAPTURED.image_index. 0: No action 1: Reset Image Log| 4: Reserved (all remaining params)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS=527, /* Request camera capture status (CAMERA_CAPTURE_STATUS) |1: 0: No Action 1: Request camera capture status| 2: Reserved (all remaining params)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_REQUEST_FLIGHT_INFORMATION=528, /* Request flight information (FLIGHT_INFORMATION) |1: 1: Request flight information| 2: Reserved (all remaining params)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_RESET_CAMERA_SETTINGS=529, /* Reset all camera settings to Factory Default |1: 0: No Action 1: Reset all settings| 2: Reserved (all remaining params)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_SET_CAMERA_MODE=530, /* Set camera running mode. Use NaN for reserved values. GCS will send a MAV_CMD_REQUEST_VIDEO_STREAM_STATUS command after a mode change if the camera supports video streaming. |1: Reserved (Set to 0)| 2: Camera mode| 3: Reserved (default:NaN)| 4: Reserved (default:NaN)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:NaN)|  */
   MAV_CMD_SET_CAMERA_ZOOM=531, /* Set camera zoom. Camera must respond with a CAMERA_SETTINGS message (on success). |1: Zoom type| 2: Zoom value. The range of valid values depend on the zoom type.| 3: Reserved (default:NaN)| 4: Reserved (default:NaN)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:NaN)|  */
   MAV_CMD_SET_CAMERA_FOCUS=532, /* Set camera focus. Camera must respond with a CAMERA_SETTINGS message (on success). |1: Focus type| 2: Focus value| 3: Reserved (default:NaN)| 4: Reserved (default:NaN)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:NaN)|  */
   MAV_CMD_JUMP_TAG=600, /* Tagged jump target. Can be jumped to with MAV_CMD_DO_JUMP_TAG. |1: Tag.| 2: Reserved (default:0)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_DO_JUMP_TAG=601, /* Jump to the matching tag in the mission list. Repeat this action for the specified number of times. A mission should contain a single matching tag for each jump. If this is not the case then a jump to a missing tag should complete the mission, and a jump where there are multiple matching tags should always select the one with the lowest mission sequence number. |1: Target tag to jump to.| 2: Repeat count.| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_PARAM_TRANSACTION=900, /* Request to start or end a parameter transaction. Multiple kinds of transport layers can be used to exchange parameters in the transaction (param, param_ext and mavftp). The command response can either be a success/failure or an in progress in case the receiving side takes some time to apply the parameters. |1: Action to be performed (start, commit, cancel, etc.)| 2: a mavlink during a parameter transaction.| 3: Identifier for a specific transaction.| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW=1000, /* High level setpoint to be sent to a gimbal manager to set a gimbal attitude. It is possible to set combinations of the values below. E.g. an angle as well as a desired angular rate can be used to get to this angle at a certain angular rate, or an angular rate only will result in continuous turning. NaN is to be used to signal unset. Note: a gimbal is never to react to this command but only the gimbal manager. |1: Pitch angle (positive to pitch up, relative to vehicle for FOLLOW mode, relative to world horizon for LOCK mode).| 2: Yaw angle (positive to yaw to the right, relative to vehicle for FOLLOW mode, absolute to North for LOCK mode).| 3: Pitch rate (positive to pitch up).| 4: Yaw rate (positive to yaw to the right).| 5: Gimbal manager flags to use.| 6: Reserved (default:0)| 7: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).|  */
   MAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE=1001, /* Gimbal configuration to set which sysid/compid is in primary and secondary control. |1: Sysid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).| 2: Compid for primary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).| 3: Sysid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).| 4: Compid for secondary control (0: no one in control, -1: leave unchanged, -2: set itself in control (for missions where the own sysid is still unknown), -3: remove control if currently in control).| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Component ID of gimbal device to address (or 1-6 for non-MAVLink gimbal), 0 for all gimbal device components. Send command multiple times for more than one gimbal (but not all gimbals).|  */
   MAV_CMD_IMAGE_START_CAPTURE=2000, /* Start image capture sequence. Sends CAMERA_IMAGE_CAPTURED after each capture. Use NaN for reserved values. |1: Reserved (Set to 0)| 2: Desired elapsed time between two consecutive pictures (in seconds). Minimum values depend on hardware (typically greater than 2 seconds).| 3: Total number of images to capture. 0 to capture forever/until MAV_CMD_IMAGE_STOP_CAPTURE.| 4: Capture sequence number starting from 1. This is only valid for single-capture (param3 == 1), otherwise set to 0. Increment the capture ID for each capture command to prevent double captures when a command is re-transmitted.| 5: Reserved (default:NaN)| 6: Reserved (default:NaN)| 7: Reserved (default:NaN)|  */
   MAV_CMD_IMAGE_STOP_CAPTURE=2001, /* Stop image capture sequence Use NaN for reserved values. |1: Reserved (Set to 0)| 2: Reserved (default:NaN)| 3: Reserved (default:NaN)| 4: Reserved (default:NaN)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:NaN)|  */
   MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE=2002, /* Re-request a CAMERA_IMAGE_CAPTURED message. |1: Sequence number for missing CAMERA_IMAGE_CAPTURED message| 2: Reserved (default:NaN)| 3: Reserved (default:NaN)| 4: Reserved (default:NaN)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:NaN)|  */
   MAV_CMD_DO_TRIGGER_CONTROL=2003, /* Enable or disable on-board camera triggering system. |1: able/disable (0 for disable, 1 for start), -1 to ignore| 2: 1 to reset the trigger sequence, -1 or 0 to ignore| 3: 1 to pause triggering, but without switching the camera off or retracting it. -1 to ignore| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_CAMERA_TRACK_POINT=2004, /* If the camera supports point visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_POINT is set), this command allows to initiate the tracking. |1: Point to track x value (normalized 0..1, 0 is left, 1 is right).| 2: Point to track y value (normalized 0..1, 0 is top, 1 is bottom).| 3: Point radius (normalized 0..1, 0 is image left, 1 is image right).| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_CAMERA_TRACK_RECTANGLE=2005, /* If the camera supports rectangle visual tracking (CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE is set), this command allows to initiate the tracking. |1: Top left corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).| 2: Top left corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).| 3: Bottom right corner of rectangle x value (normalized 0..1, 0 is left, 1 is right).| 4: Bottom right corner of rectangle y value (normalized 0..1, 0 is top, 1 is bottom).| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_CAMERA_STOP_TRACKING=2010, /* Stops ongoing tracking. |1: Reserved (default:0)| 2: Reserved (default:0)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_VIDEO_START_CAPTURE=2500, /* Starts video capture (recording). |1: Video Stream ID (0 for all streams)| 2: Frequency CAMERA_CAPTURE_STATUS messages should be sent while recording (0 for no messages, otherwise frequency)| 3: Reserved (default:NaN)| 4: Reserved (default:NaN)| 5: Reserved (default:NaN)| 6: Reserved (default:NaN)| 7: Reserved (default:NaN)|  */
   MAV_CMD_VIDEO_STOP_CAPTURE=2501, /* Stop the current video capture (recording). |1: Video Stream ID (0 for all streams)| 2: Reserved (default:NaN)| 3: Reserved (default:NaN)| 4: Reserved (default:NaN)| 5: Reserved (default:NaN)| 6: Reserved (default:NaN)| 7: Reserved (default:NaN)|  */
   MAV_CMD_VIDEO_START_STREAMING=2502, /* Start video streaming |1: Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| 2: Reserved (default:0)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_VIDEO_STOP_STREAMING=2503, /* Stop the given video stream |1: Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| 2: Reserved (default:0)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION=2504, /* Request video stream information (VIDEO_STREAM_INFORMATION) |1: Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| 2: Reserved (default:0)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_REQUEST_VIDEO_STREAM_STATUS=2505, /* Request video stream status (VIDEO_STREAM_STATUS) |1: Video Stream ID (0 for all streams, 1 for first, 2 for second, etc.)| 2: Reserved (default:0)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_LOGGING_START=2510, /* Request to start streaming logging data over MAVLink (see also LOGGING_DATA message) |1: Format: 0: ULog| 2: Reserved (set to 0)| 3: Reserved (set to 0)| 4: o 0)| 5: Reserved (set to 0)| 6: Reserved (set to 0)| 7: Reserved (set to 0)|  */
   MAV_CMD_LOGGING_STOP=2511, /* Request to stop streaming log data over MAVLink |1: Reserved (set to 0)| 2: Reserved (set to 0)| 3: Reserved (set to 0)| 4: Reserved (set to 0)| 5: Reserved (set to 0)| 6: Reserved (set to 0)| 7: Reserved (set to 0)|  */
   MAV_CMD_AIRFRAME_CONFIGURATION=2520, /*  |1: Landing gear ID (default: 0, -1 for all)| 2: Landing gear position (Down: 0, Up: 1, NaN for no change)| 3: Reserved (default:NaN)| 4: Reserved (default:NaN)| 5: Reserved (default:NaN)| 6: Reserved (default:NaN)| 7: Reserved (default:NaN)|  */
   MAV_CMD_CONTROL_HIGH_LATENCY=2600, /* Request to start/stop transmitting over the high latency telemetry |1: Control transmission over high latency telemetry (0: stop, 1: start)| 2: Empty| 3: Empty| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_PANORAMA_CREATE=2800, /* Create a panorama at the current position |1: Viewing angle horizontal of the panorama (+- 0.5 the total angle)| 2: Viewing angle vertical of panorama.| 3: Speed of the horizontal rotation.| 4: Speed of the vertical rotation.| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_DO_VTOL_TRANSITION=3000, /* Request VTOL transition |1: The target VTOL state. Only MAV_VTOL_STATE_MC and MAV_VTOL_STATE_FW can be used.| 2: Reserved (default:0)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_ARM_AUTHORIZATION_REQUEST=3001, /* Request authorization to arm the vehicle to a external entity, the arm authorizer is responsible to request all data that is needs from the vehicle before authorize or deny the request. If approved the progress of command_ack message should be set with period of time that this authorization is valid in seconds or in case it was denied it should be set with one of the reasons in ARM_AUTH_DENIED_REASON.
         |1: Vehicle system id, this way ground station can request arm authorization on behalf of any vehicle| 2: Reserved (default:0)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_SET_GUIDED_SUBMODE_STANDARD=4000, /* This command sets the submode to standard guided when vehicle is in guided mode. The vehicle holds position and altitude and the user can input the desired velocities along all three axes.
                   |1: Reserved (default:0)| 2: Reserved (default:0)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE=4001, /* This command sets submode circle when vehicle is in guided mode. Vehicle flies along a circle facing the center of the circle. The user can input the velocity along the circle and change the radius. If no input is given the vehicle will hold position.
                   |1: Radius of desired circle in CIRCLE_MODE| 2: User defined| 3: User defined| 4: User defined| 5: Target latitude of center of circle in CIRCLE_MODE| 6: Target longitude of center of circle in CIRCLE_MODE| 7: Reserved (default:0)|  */
   MAV_CMD_CONDITION_GATE=4501, /* Delay mission state machine until gate has been reached. |1: Geometry: 0: orthogonal to path between previous and next waypoint.| 2: Altitude: 0: ignore altitude| 3: Empty| 4: Empty| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_NAV_FENCE_RETURN_POINT=5000, /* Fence return point. There can only be one fence return point.
         |1: Reserved| 2: Reserved| 3: Reserved| 4: Reserved| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION=5001, /* Fence vertex for an inclusion polygon (the polygon must not be self-intersecting). The vehicle must stay within this area. Minimum of 3 vertices required.
         |1: Polygon vertex count| 2: Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group, must be the same for all points in each polygon| 3: Reserved| 4: Reserved| 5: Latitude| 6: Longitude| 7: Reserved|  */
   MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION=5002, /* Fence vertex for an exclusion polygon (the polygon must not be self-intersecting). The vehicle must stay outside this area. Minimum of 3 vertices required.
         |1: Polygon vertex count| 2: Reserved| 3: Reserved| 4: Reserved| 5: Latitude| 6: Longitude| 7: Reserved|  */
   MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION=5003, /* Circular fence area. The vehicle must stay inside this area.
         |1: Radius.| 2: Vehicle must be inside ALL inclusion zones in a single group, vehicle must be inside at least one group| 3: Reserved| 4: Reserved| 5: Latitude| 6: Longitude| 7: Reserved|  */
   MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION=5004, /* Circular fence area. The vehicle must stay outside this area.
         |1: Radius.| 2: Reserved| 3: Reserved| 4: Reserved| 5: Latitude| 6: Longitude| 7: Reserved|  */
   MAV_CMD_NAV_RALLY_POINT=5100, /* Rally point. You can have multiple rally points defined.
         |1: Reserved| 2: Reserved| 3: Reserved| 4: Reserved| 5: Latitude| 6: Longitude| 7: Altitude|  */
   MAV_CMD_UAVCAN_GET_NODE_INFO=5200, /* Commands the vehicle to respond with a sequence of messages UAVCAN_NODE_INFO, one message per every UAVCAN node that is online. Note that some of the response messages can be lost, which the receiver can detect easily by checking whether every received UAVCAN_NODE_STATUS has a matching message UAVCAN_NODE_INFO received earlier; if not, this command should be sent again in order to request re-transmission of the node information messages. |1: Reserved (set to 0)| 2: Reserved (set to 0)| 3: Reserved (set to 0)| 4: Reserved (set to 0)| 5: Reserved (set to 0)| 6: Reserved (set to 0)| 7: Reserved (set to 0)|  */
   MAV_CMD_PAYLOAD_PREPARE_DEPLOY=30001, /* Deploy payload on a Lat / Lon / Alt position. This includes the navigation to reach the required release position and velocity. |1: Operation mode. 0: prepare single payload deploy (overwriting previous requests), but do not execute it. 1: execute payload deploy immediately (rejecting further deploy commands during execution, but allowing abort). 2: add payload deploy to existing deployment list.| 2: compass heading. A negative value indicates the system can define the approach vector at will.| 3: Desired ground speed at release time. This can be overridden by the airframe in case it needs to meet minimum airspeed. A negative value indicates the system can define the ground speed at will.| 4: Minimum altitude clearance to the release position. A negative value indicates the system can define the clearance at will.| 5: Latitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)| 6: Longitude. Note, if used in MISSION_ITEM (deprecated) the units are degrees (unscaled)| 7: Altitude (MSL)|  */
   MAV_CMD_PAYLOAD_CONTROL_DEPLOY=30002, /* Control the payload deployment. |1: Operation mode. 0: Abort deployment, continue normal mission. 1: switch to payload deployment mode. 100: delete first payload deployment request. 101: delete all payload deployment requests.| 2: Reserved| 3: Reserved| 4: Reserved| 5: Reserved| 6: Reserved| 7: Reserved|  */
   MAV_CMD_WAYPOINT_USER_1=31000, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: Latitude unscaled| 6: Longitude unscaled| 7: Altitude (MSL)|  */
   MAV_CMD_WAYPOINT_USER_2=31001, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: Latitude unscaled| 6: Longitude unscaled| 7: Altitude (MSL)|  */
   MAV_CMD_WAYPOINT_USER_3=31002, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: Latitude unscaled| 6: Longitude unscaled| 7: Altitude (MSL)|  */
   MAV_CMD_WAYPOINT_USER_4=31003, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: Latitude unscaled| 6: Longitude unscaled| 7: Altitude (MSL)|  */
   MAV_CMD_WAYPOINT_USER_5=31004, /* User defined waypoint item. Ground Station will show the Vehicle as flying through this item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: Latitude unscaled| 6: Longitude unscaled| 7: Altitude (MSL)|  */
   MAV_CMD_SPATIAL_USER_1=31005, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: Latitude unscaled| 6: Longitude unscaled| 7: Altitude (MSL)|  */
   MAV_CMD_SPATIAL_USER_2=31006, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: Latitude unscaled| 6: Longitude unscaled| 7: Altitude (MSL)|  */
   MAV_CMD_SPATIAL_USER_3=31007, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: Latitude unscaled| 6: Longitude unscaled| 7: Altitude (MSL)|  */
   MAV_CMD_SPATIAL_USER_4=31008, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: Latitude unscaled| 6: Longitude unscaled| 7: Altitude (MSL)|  */
   MAV_CMD_SPATIAL_USER_5=31009, /* User defined spatial item. Ground Station will not show the Vehicle as flying through this item. Example: ROI item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: Latitude unscaled| 6: Longitude unscaled| 7: Altitude (MSL)|  */
   MAV_CMD_USER_1=31010, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: User defined| 6: User defined| 7: User defined|  */
   MAV_CMD_USER_2=31011, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: User defined| 6: User defined| 7: User defined|  */
   MAV_CMD_USER_3=31012, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: User defined| 6: User defined| 7: User defined|  */
   MAV_CMD_USER_4=31013, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: User defined| 6: User defined| 7: User defined|  */
   MAV_CMD_USER_5=31014, /* User defined command. Ground Station will not show the Vehicle as flying through this item. Example: MAV_CMD_DO_SET_PARAMETER item. |1: User defined| 2: User defined| 3: User defined| 4: User defined| 5: User defined| 6: User defined| 7: User defined|  */
   MAV_CMD_POWER_OFF_INITIATED=42000, /* A system wide power-off event has been initiated. |1: Empty.| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_SOLO_BTN_FLY_CLICK=42001, /* FLY button has been clicked. |1: Empty.| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_SOLO_BTN_FLY_HOLD=42002, /* FLY button has been held for 1.5 seconds. |1: Takeoff altitude.| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_SOLO_BTN_PAUSE_CLICK=42003, /* PAUSE button has been clicked. |1: 1 if Solo is in a shot mode, 0 otherwise.| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_FIXED_MAG_CAL=42004, /* Magnetometer calibration based on fixed position
        in earth field given by inclination, declination and intensity. |1: Magnetic declination.| 2: Magnetic inclination.| 3: Magnetic intensity.| 4: Yaw.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_FIXED_MAG_CAL_FIELD=42005, /* Magnetometer calibration based on fixed expected field values. |1: Field strength X.| 2: Field strength Y.| 3: Field strength Z.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_FIXED_MAG_CAL_YAW=42006, /* Magnetometer calibration based on provided known yaw. This allows for fast calibration using WMM field tables in the vehicle, given only the known yaw of the vehicle. If Latitude and longitude are both zero then use the current vehicle location. |1: Yaw of vehicle in earth frame.| 2: CompassMask, 0 for all.| 3: Latitude.| 4: Longitude.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_DO_START_MAG_CAL=42424, /* Initiate a magnetometer calibration. |1: Bitmask of magnetometers to calibrate. Use 0 to calibrate all sensors that can be started (sensors may not start if disabled, unhealthy, etc.). The command will NACK if calibration does not start for a sensor explicitly specified by the bitmask.| 2: Automatically retry on failure (0=no retry, 1=retry).| 3: Save without user input (0=require input, 1=autosave).| 4: Delay.| 5: Autoreboot (0=user reboot, 1=autoreboot).| 6: Empty.| 7: Empty.|  */
   MAV_CMD_DO_ACCEPT_MAG_CAL=42425, /* Accept a magnetometer calibration. |1: Bitmask of magnetometers that calibration is accepted (0 means all).| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_DO_CANCEL_MAG_CAL=42426, /* Cancel a running magnetometer calibration. |1: Bitmask of magnetometers to cancel a running calibration (0 means all).| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_SET_FACTORY_TEST_MODE=42427, /* Command autopilot to get into factory test/diagnostic mode. |1: 0: activate test mode, 1: exit test mode.| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_DO_SEND_BANNER=42428, /* Reply with the version banner. |1: Empty.| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_ACCELCAL_VEHICLE_POS=42429, /* Used when doing accelerometer calibration. When sent to the GCS tells it what position to put the vehicle in. When sent to the vehicle says what position the vehicle is in. |1: Position.| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_GIMBAL_RESET=42501, /* Causes the gimbal to reset and boot as if it was just powered on. |1: Empty.| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_GIMBAL_AXIS_CALIBRATION_STATUS=42502, /* Reports progress and success or failure of gimbal axis calibration procedure. |1: Gimbal axis we're reporting calibration progress for.| 2: Current calibration progress for this axis.| 3: Status of the calibration.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_GIMBAL_REQUEST_AXIS_CALIBRATION=42503, /* Starts commutation calibration on the gimbal. |1: Empty.| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_GIMBAL_FULL_RESET=42505, /* Erases gimbal application and parameters. |1: Magic number.| 2: Magic number.| 3: Magic number.| 4: Magic number.| 5: Magic number.| 6: Magic number.| 7: Magic number.|  */
   MAV_CMD_DO_WINCH=42600, /* Command to operate winch. |1: Winch instance number.| 2: Action to perform.| 3: Length of cable to release (negative to wind).| 4: Release rate (negative to wind).| 5: Empty.| 6: Empty.| 7: Empty.|  */
   MAV_CMD_FLASH_BOOTLOADER=42650, /* Update the bootloader |1: Empty| 2: Empty| 3: Empty| 4: Empty| 5: Magic number - set to 290876 to actually flash| 6: Empty| 7: Empty|  */
   MAV_CMD_BATTERY_RESET=42651, /* Reset battery capacity for batteries that accumulate consumed battery via integration. |1: Bitmask of batteries to reset. Least significant bit is for the first battery.| 2: Battery percentage remaining to set.| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_DEBUG_TRAP=42700, /* Issue a trap signal to the autopilot process, presumably to enter the debugger. |1: Magic number - set to 32451 to actually trap.| 2: Empty.| 3: Empty.| 4: Empty.| 5: Empty.| 6: Empty.| 7: pty.|  */
   MAV_CMD_SCRIPTING=42701, /* Control onboard scripting. |1: Scripting command to execute| 2: Reserved (default:0)| 3: Reserved (default:0)| 4: Reserved (default:0)| 5: Reserved (default:0)| 6: Reserved (default:0)| 7: Reserved (default:0)|  */
   MAV_CMD_GUIDED_CHANGE_SPEED=43000, /* Change flight speed at a given rate. This slews the vehicle at a controllable rate between it's previous speed and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.) |1: Airspeed or groundspeed.| 2: Target Speed| 3: Acceleration rate, 0 to take effect instantly| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_GUIDED_CHANGE_ALTITUDE=43001, /* Change target altitude at a given rate. This slews the vehicle at a controllable rate between it's previous altitude and the new one. (affects GUIDED only. Outside GUIDED, aircraft ignores these commands. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.) |1: Empty| 2: Empty| 3: Rate of change, toward new altitude. 0 for maximum rate change. Positive numbers only, as negative numbers will not converge on the new target alt.| 4: Empty| 5: Empty| 6: mpty| 7: Target Altitude|  */
   MAV_CMD_GUIDED_CHANGE_HEADING=43002, /* Change to target heading at a given rate, overriding previous heading/s. This slews the vehicle at a controllable rate between it's previous heading and the new one. (affects GUIDED only. Exiting GUIDED returns aircraft to normal behaviour defined elsewhere. Designed for onboard companion-computer command-and-control, not normally operator/GCS control.) |1: course-over-ground or raw vehicle heading.| 2: Target heading.| 3: Maximum centripetal accelearation, ie rate of change,  toward new heading.| 4: Empty| 5: Empty| 6: Empty| 7: Empty|  */
   MAV_CMD_ENUM_END=43003, /*  | */
} MAV_CMD;
#endif

/** @brief  */
#ifndef HAVE_ENUM_SCRIPTING_CMD
#define HAVE_ENUM_SCRIPTING_CMD
typedef enum SCRIPTING_CMD
{
   SCRIPTING_CMD_REPL_START=0, /* Start a REPL session. | */
   SCRIPTING_CMD_REPL_STOP=1, /* End a REPL session. | */
   SCRIPTING_CMD_ENUM_END=2, /*  | */
} SCRIPTING_CMD;
#endif

/** @brief  */
#ifndef HAVE_ENUM_LIMITS_STATE
#define HAVE_ENUM_LIMITS_STATE
typedef enum LIMITS_STATE
{
   LIMITS_INIT=0, /* Pre-initialization. | */
   LIMITS_DISABLED=1, /* Disabled. | */
   LIMITS_ENABLED=2, /* Checking limits. | */
   LIMITS_TRIGGERED=3, /* A limit has been breached. | */
   LIMITS_RECOVERING=4, /* Taking action e.g. Return/RTL. | */
   LIMITS_RECOVERED=5, /* We're no longer in breach of a limit. | */
   LIMITS_STATE_ENUM_END=6, /*  | */
} LIMITS_STATE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_LIMIT_MODULE
#define HAVE_ENUM_LIMIT_MODULE
typedef enum LIMIT_MODULE
{
   LIMIT_GPSLOCK=1, /* Pre-initialization. | */
   LIMIT_GEOFENCE=2, /* Disabled. | */
   LIMIT_ALTITUDE=4, /* Checking limits. | */
   LIMIT_MODULE_ENUM_END=5, /*  | */
} LIMIT_MODULE;
#endif

/** @brief Flags in RALLY_POINT message. */
#ifndef HAVE_ENUM_RALLY_FLAGS
#define HAVE_ENUM_RALLY_FLAGS
typedef enum RALLY_FLAGS
{
   FAVORABLE_WIND=1, /* Flag set when requiring favorable winds for landing. | */
   LAND_IMMEDIATELY=2, /* Flag set when plane is to immediately descend to break altitude and land without GCS intervention. Flag not set when plane is to loiter at Rally point until commanded to land. | */
   RALLY_FLAGS_ENUM_END=3, /*  | */
} RALLY_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CAMERA_STATUS_TYPES
#define HAVE_ENUM_CAMERA_STATUS_TYPES
typedef enum CAMERA_STATUS_TYPES
{
   CAMERA_STATUS_TYPE_HEARTBEAT=0, /* Camera heartbeat, announce camera component ID at 1Hz. | */
   CAMERA_STATUS_TYPE_TRIGGER=1, /* Camera image triggered. | */
   CAMERA_STATUS_TYPE_DISCONNECT=2, /* Camera connection lost. | */
   CAMERA_STATUS_TYPE_ERROR=3, /* Camera unknown error. | */
   CAMERA_STATUS_TYPE_LOWBATT=4, /* Camera battery low. Parameter p1 shows reported voltage. | */
   CAMERA_STATUS_TYPE_LOWSTORE=5, /* Camera storage low. Parameter p1 shows reported shots remaining. | */
   CAMERA_STATUS_TYPE_LOWSTOREV=6, /* Camera storage low. Parameter p1 shows reported video minutes remaining. | */
   CAMERA_STATUS_TYPES_ENUM_END=7, /*  | */
} CAMERA_STATUS_TYPES;
#endif

/** @brief  */
#ifndef HAVE_ENUM_CAMERA_FEEDBACK_FLAGS
#define HAVE_ENUM_CAMERA_FEEDBACK_FLAGS
typedef enum CAMERA_FEEDBACK_FLAGS
{
   CAMERA_FEEDBACK_PHOTO=0, /* Shooting photos, not video. | */
   CAMERA_FEEDBACK_VIDEO=1, /* Shooting video, not stills. | */
   CAMERA_FEEDBACK_BADEXPOSURE=2, /* Unable to achieve requested exposure (e.g. shutter speed too low). | */
   CAMERA_FEEDBACK_CLOSEDLOOP=3, /* Closed loop feedback from camera, we know for sure it has successfully taken a picture. | */
   CAMERA_FEEDBACK_OPENLOOP=4, /* Open loop camera, an image trigger has been requested but we can't know for sure it has successfully taken a picture. | */
   CAMERA_FEEDBACK_FLAGS_ENUM_END=5, /*  | */
} CAMERA_FEEDBACK_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_MAV_MODE_GIMBAL
#define HAVE_ENUM_MAV_MODE_GIMBAL
typedef enum MAV_MODE_GIMBAL
{
   MAV_MODE_GIMBAL_UNINITIALIZED=0, /* Gimbal is powered on but has not started initializing yet. | */
   MAV_MODE_GIMBAL_CALIBRATING_PITCH=1, /* Gimbal is currently running calibration on the pitch axis. | */
   MAV_MODE_GIMBAL_CALIBRATING_ROLL=2, /* Gimbal is currently running calibration on the roll axis. | */
   MAV_MODE_GIMBAL_CALIBRATING_YAW=3, /* Gimbal is currently running calibration on the yaw axis. | */
   MAV_MODE_GIMBAL_INITIALIZED=4, /* Gimbal has finished calibrating and initializing, but is relaxed pending reception of first rate command from copter. | */
   MAV_MODE_GIMBAL_ACTIVE=5, /* Gimbal is actively stabilizing. | */
   MAV_MODE_GIMBAL_RATE_CMD_TIMEOUT=6, /* Gimbal is relaxed because it missed more than 10 expected rate command messages in a row. Gimbal will move back to active mode when it receives a new rate command. | */
   MAV_MODE_GIMBAL_ENUM_END=7, /*  | */
} MAV_MODE_GIMBAL;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GIMBAL_AXIS
#define HAVE_ENUM_GIMBAL_AXIS
typedef enum GIMBAL_AXIS
{
   GIMBAL_AXIS_YAW=0, /* Gimbal yaw axis. | */
   GIMBAL_AXIS_PITCH=1, /* Gimbal pitch axis. | */
   GIMBAL_AXIS_ROLL=2, /* Gimbal roll axis. | */
   GIMBAL_AXIS_ENUM_END=3, /*  | */
} GIMBAL_AXIS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GIMBAL_AXIS_CALIBRATION_STATUS
#define HAVE_ENUM_GIMBAL_AXIS_CALIBRATION_STATUS
typedef enum GIMBAL_AXIS_CALIBRATION_STATUS
{
   GIMBAL_AXIS_CALIBRATION_STATUS_IN_PROGRESS=0, /* Axis calibration is in progress. | */
   GIMBAL_AXIS_CALIBRATION_STATUS_SUCCEEDED=1, /* Axis calibration succeeded. | */
   GIMBAL_AXIS_CALIBRATION_STATUS_FAILED=2, /* Axis calibration failed. | */
   GIMBAL_AXIS_CALIBRATION_STATUS_ENUM_END=3, /*  | */
} GIMBAL_AXIS_CALIBRATION_STATUS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GIMBAL_AXIS_CALIBRATION_REQUIRED
#define HAVE_ENUM_GIMBAL_AXIS_CALIBRATION_REQUIRED
typedef enum GIMBAL_AXIS_CALIBRATION_REQUIRED
{
   GIMBAL_AXIS_CALIBRATION_REQUIRED_UNKNOWN=0, /* Whether or not this axis requires calibration is unknown at this time. | */
   GIMBAL_AXIS_CALIBRATION_REQUIRED_TRUE=1, /* This axis requires calibration. | */
   GIMBAL_AXIS_CALIBRATION_REQUIRED_FALSE=2, /* This axis does not require calibration. | */
   GIMBAL_AXIS_CALIBRATION_REQUIRED_ENUM_END=3, /*  | */
} GIMBAL_AXIS_CALIBRATION_REQUIRED;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_HEARTBEAT_STATUS
#define HAVE_ENUM_GOPRO_HEARTBEAT_STATUS
typedef enum GOPRO_HEARTBEAT_STATUS
{
   GOPRO_HEARTBEAT_STATUS_DISCONNECTED=0, /* No GoPro connected. | */
   GOPRO_HEARTBEAT_STATUS_INCOMPATIBLE=1, /* The detected GoPro is not HeroBus compatible. | */
   GOPRO_HEARTBEAT_STATUS_CONNECTED=2, /* A HeroBus compatible GoPro is connected. | */
   GOPRO_HEARTBEAT_STATUS_ERROR=3, /* An unrecoverable error was encountered with the connected GoPro, it may require a power cycle. | */
   GOPRO_HEARTBEAT_STATUS_ENUM_END=4, /*  | */
} GOPRO_HEARTBEAT_STATUS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_HEARTBEAT_FLAGS
#define HAVE_ENUM_GOPRO_HEARTBEAT_FLAGS
typedef enum GOPRO_HEARTBEAT_FLAGS
{
   GOPRO_FLAG_RECORDING=1, /* GoPro is currently recording. | */
   GOPRO_HEARTBEAT_FLAGS_ENUM_END=2, /*  | */
} GOPRO_HEARTBEAT_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_REQUEST_STATUS
#define HAVE_ENUM_GOPRO_REQUEST_STATUS
typedef enum GOPRO_REQUEST_STATUS
{
   GOPRO_REQUEST_SUCCESS=0, /* The write message with ID indicated succeeded. | */
   GOPRO_REQUEST_FAILED=1, /* The write message with ID indicated failed. | */
   GOPRO_REQUEST_STATUS_ENUM_END=2, /*  | */
} GOPRO_REQUEST_STATUS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_COMMAND
#define HAVE_ENUM_GOPRO_COMMAND
typedef enum GOPRO_COMMAND
{
   GOPRO_COMMAND_POWER=0, /* (Get/Set). | */
   GOPRO_COMMAND_CAPTURE_MODE=1, /* (Get/Set). | */
   GOPRO_COMMAND_SHUTTER=2, /* (___/Set). | */
   GOPRO_COMMAND_BATTERY=3, /* (Get/___). | */
   GOPRO_COMMAND_MODEL=4, /* (Get/___). | */
   GOPRO_COMMAND_VIDEO_SETTINGS=5, /* (Get/Set). | */
   GOPRO_COMMAND_LOW_LIGHT=6, /* (Get/Set). | */
   GOPRO_COMMAND_PHOTO_RESOLUTION=7, /* (Get/Set). | */
   GOPRO_COMMAND_PHOTO_BURST_RATE=8, /* (Get/Set). | */
   GOPRO_COMMAND_PROTUNE=9, /* (Get/Set). | */
   GOPRO_COMMAND_PROTUNE_WHITE_BALANCE=10, /* (Get/Set) Hero 3+ Only. | */
   GOPRO_COMMAND_PROTUNE_COLOUR=11, /* (Get/Set) Hero 3+ Only. | */
   GOPRO_COMMAND_PROTUNE_GAIN=12, /* (Get/Set) Hero 3+ Only. | */
   GOPRO_COMMAND_PROTUNE_SHARPNESS=13, /* (Get/Set) Hero 3+ Only. | */
   GOPRO_COMMAND_PROTUNE_EXPOSURE=14, /* (Get/Set) Hero 3+ Only. | */
   GOPRO_COMMAND_TIME=15, /* (Get/Set). | */
   GOPRO_COMMAND_CHARGING=16, /* (Get/Set). | */
   GOPRO_COMMAND_ENUM_END=17, /*  | */
} GOPRO_COMMAND;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_CAPTURE_MODE
#define HAVE_ENUM_GOPRO_CAPTURE_MODE
typedef enum GOPRO_CAPTURE_MODE
{
   GOPRO_CAPTURE_MODE_VIDEO=0, /* Video mode. | */
   GOPRO_CAPTURE_MODE_PHOTO=1, /* Photo mode. | */
   GOPRO_CAPTURE_MODE_BURST=2, /* Burst mode, Hero 3+ only. | */
   GOPRO_CAPTURE_MODE_TIME_LAPSE=3, /* Time lapse mode, Hero 3+ only. | */
   GOPRO_CAPTURE_MODE_MULTI_SHOT=4, /* Multi shot mode, Hero 4 only. | */
   GOPRO_CAPTURE_MODE_PLAYBACK=5, /* Playback mode, Hero 4 only, silver only except when LCD or HDMI is connected to black. | */
   GOPRO_CAPTURE_MODE_SETUP=6, /* Playback mode, Hero 4 only. | */
   GOPRO_CAPTURE_MODE_UNKNOWN=255, /* Mode not yet known. | */
   GOPRO_CAPTURE_MODE_ENUM_END=256, /*  | */
} GOPRO_CAPTURE_MODE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_RESOLUTION
#define HAVE_ENUM_GOPRO_RESOLUTION
typedef enum GOPRO_RESOLUTION
{
   GOPRO_RESOLUTION_480p=0, /* 848 x 480 (480p). | */
   GOPRO_RESOLUTION_720p=1, /* 1280 x 720 (720p). | */
   GOPRO_RESOLUTION_960p=2, /* 1280 x 960 (960p). | */
   GOPRO_RESOLUTION_1080p=3, /* 1920 x 1080 (1080p). | */
   GOPRO_RESOLUTION_1440p=4, /* 1920 x 1440 (1440p). | */
   GOPRO_RESOLUTION_2_7k_17_9=5, /* 2704 x 1440 (2.7k-17:9). | */
   GOPRO_RESOLUTION_2_7k_16_9=6, /* 2704 x 1524 (2.7k-16:9). | */
   GOPRO_RESOLUTION_2_7k_4_3=7, /* 2704 x 2028 (2.7k-4:3). | */
   GOPRO_RESOLUTION_4k_16_9=8, /* 3840 x 2160 (4k-16:9). | */
   GOPRO_RESOLUTION_4k_17_9=9, /* 4096 x 2160 (4k-17:9). | */
   GOPRO_RESOLUTION_720p_SUPERVIEW=10, /* 1280 x 720 (720p-SuperView). | */
   GOPRO_RESOLUTION_1080p_SUPERVIEW=11, /* 1920 x 1080 (1080p-SuperView). | */
   GOPRO_RESOLUTION_2_7k_SUPERVIEW=12, /* 2704 x 1520 (2.7k-SuperView). | */
   GOPRO_RESOLUTION_4k_SUPERVIEW=13, /* 3840 x 2160 (4k-SuperView). | */
   GOPRO_RESOLUTION_ENUM_END=14, /*  | */
} GOPRO_RESOLUTION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_FRAME_RATE
#define HAVE_ENUM_GOPRO_FRAME_RATE
typedef enum GOPRO_FRAME_RATE
{
   GOPRO_FRAME_RATE_12=0, /* 12 FPS. | */
   GOPRO_FRAME_RATE_15=1, /* 15 FPS. | */
   GOPRO_FRAME_RATE_24=2, /* 24 FPS. | */
   GOPRO_FRAME_RATE_25=3, /* 25 FPS. | */
   GOPRO_FRAME_RATE_30=4, /* 30 FPS. | */
   GOPRO_FRAME_RATE_48=5, /* 48 FPS. | */
   GOPRO_FRAME_RATE_50=6, /* 50 FPS. | */
   GOPRO_FRAME_RATE_60=7, /* 60 FPS. | */
   GOPRO_FRAME_RATE_80=8, /* 80 FPS. | */
   GOPRO_FRAME_RATE_90=9, /* 90 FPS. | */
   GOPRO_FRAME_RATE_100=10, /* 100 FPS. | */
   GOPRO_FRAME_RATE_120=11, /* 120 FPS. | */
   GOPRO_FRAME_RATE_240=12, /* 240 FPS. | */
   GOPRO_FRAME_RATE_12_5=13, /* 12.5 FPS. | */
   GOPRO_FRAME_RATE_ENUM_END=14, /*  | */
} GOPRO_FRAME_RATE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_FIELD_OF_VIEW
#define HAVE_ENUM_GOPRO_FIELD_OF_VIEW
typedef enum GOPRO_FIELD_OF_VIEW
{
   GOPRO_FIELD_OF_VIEW_WIDE=0, /* 0x00: Wide. | */
   GOPRO_FIELD_OF_VIEW_MEDIUM=1, /* 0x01: Medium. | */
   GOPRO_FIELD_OF_VIEW_NARROW=2, /* 0x02: Narrow. | */
   GOPRO_FIELD_OF_VIEW_ENUM_END=3, /*  | */
} GOPRO_FIELD_OF_VIEW;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_VIDEO_SETTINGS_FLAGS
#define HAVE_ENUM_GOPRO_VIDEO_SETTINGS_FLAGS
typedef enum GOPRO_VIDEO_SETTINGS_FLAGS
{
   GOPRO_VIDEO_SETTINGS_TV_MODE=1, /* 0=NTSC, 1=PAL. | */
   GOPRO_VIDEO_SETTINGS_FLAGS_ENUM_END=2, /*  | */
} GOPRO_VIDEO_SETTINGS_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_PHOTO_RESOLUTION
#define HAVE_ENUM_GOPRO_PHOTO_RESOLUTION
typedef enum GOPRO_PHOTO_RESOLUTION
{
   GOPRO_PHOTO_RESOLUTION_5MP_MEDIUM=0, /* 5MP Medium. | */
   GOPRO_PHOTO_RESOLUTION_7MP_MEDIUM=1, /* 7MP Medium. | */
   GOPRO_PHOTO_RESOLUTION_7MP_WIDE=2, /* 7MP Wide. | */
   GOPRO_PHOTO_RESOLUTION_10MP_WIDE=3, /* 10MP Wide. | */
   GOPRO_PHOTO_RESOLUTION_12MP_WIDE=4, /* 12MP Wide. | */
   GOPRO_PHOTO_RESOLUTION_ENUM_END=5, /*  | */
} GOPRO_PHOTO_RESOLUTION;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_PROTUNE_WHITE_BALANCE
#define HAVE_ENUM_GOPRO_PROTUNE_WHITE_BALANCE
typedef enum GOPRO_PROTUNE_WHITE_BALANCE
{
   GOPRO_PROTUNE_WHITE_BALANCE_AUTO=0, /* Auto. | */
   GOPRO_PROTUNE_WHITE_BALANCE_3000K=1, /* 3000K. | */
   GOPRO_PROTUNE_WHITE_BALANCE_5500K=2, /* 5500K. | */
   GOPRO_PROTUNE_WHITE_BALANCE_6500K=3, /* 6500K. | */
   GOPRO_PROTUNE_WHITE_BALANCE_RAW=4, /* Camera Raw. | */
   GOPRO_PROTUNE_WHITE_BALANCE_ENUM_END=5, /*  | */
} GOPRO_PROTUNE_WHITE_BALANCE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_PROTUNE_COLOUR
#define HAVE_ENUM_GOPRO_PROTUNE_COLOUR
typedef enum GOPRO_PROTUNE_COLOUR
{
   GOPRO_PROTUNE_COLOUR_STANDARD=0, /* Auto. | */
   GOPRO_PROTUNE_COLOUR_NEUTRAL=1, /* Neutral. | */
   GOPRO_PROTUNE_COLOUR_ENUM_END=2, /*  | */
} GOPRO_PROTUNE_COLOUR;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_PROTUNE_GAIN
#define HAVE_ENUM_GOPRO_PROTUNE_GAIN
typedef enum GOPRO_PROTUNE_GAIN
{
   GOPRO_PROTUNE_GAIN_400=0, /* ISO 400. | */
   GOPRO_PROTUNE_GAIN_800=1, /* ISO 800 (Only Hero 4). | */
   GOPRO_PROTUNE_GAIN_1600=2, /* ISO 1600. | */
   GOPRO_PROTUNE_GAIN_3200=3, /* ISO 3200 (Only Hero 4). | */
   GOPRO_PROTUNE_GAIN_6400=4, /* ISO 6400. | */
   GOPRO_PROTUNE_GAIN_ENUM_END=5, /*  | */
} GOPRO_PROTUNE_GAIN;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_PROTUNE_SHARPNESS
#define HAVE_ENUM_GOPRO_PROTUNE_SHARPNESS
typedef enum GOPRO_PROTUNE_SHARPNESS
{
   GOPRO_PROTUNE_SHARPNESS_LOW=0, /* Low Sharpness. | */
   GOPRO_PROTUNE_SHARPNESS_MEDIUM=1, /* Medium Sharpness. | */
   GOPRO_PROTUNE_SHARPNESS_HIGH=2, /* High Sharpness. | */
   GOPRO_PROTUNE_SHARPNESS_ENUM_END=3, /*  | */
} GOPRO_PROTUNE_SHARPNESS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_PROTUNE_EXPOSURE
#define HAVE_ENUM_GOPRO_PROTUNE_EXPOSURE
typedef enum GOPRO_PROTUNE_EXPOSURE
{
   GOPRO_PROTUNE_EXPOSURE_NEG_5_0=0, /* -5.0 EV (Hero 3+ Only). | */
   GOPRO_PROTUNE_EXPOSURE_NEG_4_5=1, /* -4.5 EV (Hero 3+ Only). | */
   GOPRO_PROTUNE_EXPOSURE_NEG_4_0=2, /* -4.0 EV (Hero 3+ Only). | */
   GOPRO_PROTUNE_EXPOSURE_NEG_3_5=3, /* -3.5 EV (Hero 3+ Only). | */
   GOPRO_PROTUNE_EXPOSURE_NEG_3_0=4, /* -3.0 EV (Hero 3+ Only). | */
   GOPRO_PROTUNE_EXPOSURE_NEG_2_5=5, /* -2.5 EV (Hero 3+ Only). | */
   GOPRO_PROTUNE_EXPOSURE_NEG_2_0=6, /* -2.0 EV. | */
   GOPRO_PROTUNE_EXPOSURE_NEG_1_5=7, /* -1.5 EV. | */
   GOPRO_PROTUNE_EXPOSURE_NEG_1_0=8, /* -1.0 EV. | */
   GOPRO_PROTUNE_EXPOSURE_NEG_0_5=9, /* -0.5 EV. | */
   GOPRO_PROTUNE_EXPOSURE_ZERO=10, /* 0.0 EV. | */
   GOPRO_PROTUNE_EXPOSURE_POS_0_5=11, /* +0.5 EV. | */
   GOPRO_PROTUNE_EXPOSURE_POS_1_0=12, /* +1.0 EV. | */
   GOPRO_PROTUNE_EXPOSURE_POS_1_5=13, /* +1.5 EV. | */
   GOPRO_PROTUNE_EXPOSURE_POS_2_0=14, /* +2.0 EV. | */
   GOPRO_PROTUNE_EXPOSURE_POS_2_5=15, /* +2.5 EV (Hero 3+ Only). | */
   GOPRO_PROTUNE_EXPOSURE_POS_3_0=16, /* +3.0 EV (Hero 3+ Only). | */
   GOPRO_PROTUNE_EXPOSURE_POS_3_5=17, /* +3.5 EV (Hero 3+ Only). | */
   GOPRO_PROTUNE_EXPOSURE_POS_4_0=18, /* +4.0 EV (Hero 3+ Only). | */
   GOPRO_PROTUNE_EXPOSURE_POS_4_5=19, /* +4.5 EV (Hero 3+ Only). | */
   GOPRO_PROTUNE_EXPOSURE_POS_5_0=20, /* +5.0 EV (Hero 3+ Only). | */
   GOPRO_PROTUNE_EXPOSURE_ENUM_END=21, /*  | */
} GOPRO_PROTUNE_EXPOSURE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_CHARGING
#define HAVE_ENUM_GOPRO_CHARGING
typedef enum GOPRO_CHARGING
{
   GOPRO_CHARGING_DISABLED=0, /* Charging disabled. | */
   GOPRO_CHARGING_ENABLED=1, /* Charging enabled. | */
   GOPRO_CHARGING_ENUM_END=2, /*  | */
} GOPRO_CHARGING;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_MODEL
#define HAVE_ENUM_GOPRO_MODEL
typedef enum GOPRO_MODEL
{
   GOPRO_MODEL_UNKNOWN=0, /* Unknown gopro model. | */
   GOPRO_MODEL_HERO_3_PLUS_SILVER=1, /* Hero 3+ Silver (HeroBus not supported by GoPro). | */
   GOPRO_MODEL_HERO_3_PLUS_BLACK=2, /* Hero 3+ Black. | */
   GOPRO_MODEL_HERO_4_SILVER=3, /* Hero 4 Silver. | */
   GOPRO_MODEL_HERO_4_BLACK=4, /* Hero 4 Black. | */
   GOPRO_MODEL_ENUM_END=5, /*  | */
} GOPRO_MODEL;
#endif

/** @brief  */
#ifndef HAVE_ENUM_GOPRO_BURST_RATE
#define HAVE_ENUM_GOPRO_BURST_RATE
typedef enum GOPRO_BURST_RATE
{
   GOPRO_BURST_RATE_3_IN_1_SECOND=0, /* 3 Shots / 1 Second. | */
   GOPRO_BURST_RATE_5_IN_1_SECOND=1, /* 5 Shots / 1 Second. | */
   GOPRO_BURST_RATE_10_IN_1_SECOND=2, /* 10 Shots / 1 Second. | */
   GOPRO_BURST_RATE_10_IN_2_SECOND=3, /* 10 Shots / 2 Second. | */
   GOPRO_BURST_RATE_10_IN_3_SECOND=4, /* 10 Shots / 3 Second (Hero 4 Only). | */
   GOPRO_BURST_RATE_30_IN_1_SECOND=5, /* 30 Shots / 1 Second. | */
   GOPRO_BURST_RATE_30_IN_2_SECOND=6, /* 30 Shots / 2 Second. | */
   GOPRO_BURST_RATE_30_IN_3_SECOND=7, /* 30 Shots / 3 Second. | */
   GOPRO_BURST_RATE_30_IN_6_SECOND=8, /* 30 Shots / 6 Second. | */
   GOPRO_BURST_RATE_ENUM_END=9, /*  | */
} GOPRO_BURST_RATE;
#endif

/** @brief  */
#ifndef HAVE_ENUM_LED_CONTROL_PATTERN
#define HAVE_ENUM_LED_CONTROL_PATTERN
typedef enum LED_CONTROL_PATTERN
{
   LED_CONTROL_PATTERN_OFF=0, /* LED patterns off (return control to regular vehicle control). | */
   LED_CONTROL_PATTERN_FIRMWAREUPDATE=1, /* LEDs show pattern during firmware update. | */
   LED_CONTROL_PATTERN_CUSTOM=255, /* Custom Pattern using custom bytes fields. | */
   LED_CONTROL_PATTERN_ENUM_END=256, /*  | */
} LED_CONTROL_PATTERN;
#endif

/** @brief Flags in EKF_STATUS message. */
#ifndef HAVE_ENUM_EKF_STATUS_FLAGS
#define HAVE_ENUM_EKF_STATUS_FLAGS
typedef enum EKF_STATUS_FLAGS
{
   EKF_ATTITUDE=1, /* Set if EKF's attitude estimate is good. | */
   EKF_VELOCITY_HORIZ=2, /* Set if EKF's horizontal velocity estimate is good. | */
   EKF_VELOCITY_VERT=4, /* Set if EKF's vertical velocity estimate is good. | */
   EKF_POS_HORIZ_REL=8, /* Set if EKF's horizontal position (relative) estimate is good. | */
   EKF_POS_HORIZ_ABS=16, /* Set if EKF's horizontal position (absolute) estimate is good. | */
   EKF_POS_VERT_ABS=32, /* Set if EKF's vertical position (absolute) estimate is good. | */
   EKF_POS_VERT_AGL=64, /* Set if EKF's vertical position (above ground) estimate is good. | */
   EKF_CONST_POS_MODE=128, /* EKF is in constant position mode and does not know it's absolute or relative position. | */
   EKF_PRED_POS_HORIZ_REL=256, /* Set if EKF's predicted horizontal position (relative) estimate is good. | */
   EKF_PRED_POS_HORIZ_ABS=512, /* Set if EKF's predicted horizontal position (absolute) estimate is good. | */
   EKF_UNINITIALIZED=1024, /* Set if EKF has never been healthy. | */
   EKF_STATUS_FLAGS_ENUM_END=1025, /*  | */
} EKF_STATUS_FLAGS;
#endif

/** @brief  */
#ifndef HAVE_ENUM_PID_TUNING_AXIS
#define HAVE_ENUM_PID_TUNING_AXIS
typedef enum PID_TUNING_AXIS
{
   PID_TUNING_ROLL=1, /*  | */
   PID_TUNING_PITCH=2, /*  | */
   PID_TUNING_YAW=3, /*  | */
   PID_TUNING_ACCZ=4, /*  | */
   PID_TUNING_STEER=5, /*  | */
   PID_TUNING_LANDING=6, /*  | */
   PID_TUNING_AXIS_ENUM_END=7, /*  | */
} PID_TUNING_AXIS;
#endif

/** @brief Special ACK block numbers control activation of dataflash log streaming. */
#ifndef HAVE_ENUM_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS
#define HAVE_ENUM_MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS
typedef enum MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS
{
   MAV_REMOTE_LOG_DATA_BLOCK_STOP=2147483645, /* UAV to stop sending DataFlash blocks. | */
   MAV_REMOTE_LOG_DATA_BLOCK_START=2147483646, /* UAV to start sending DataFlash blocks. | */
   MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS_ENUM_END=2147483647, /*  | */
} MAV_REMOTE_LOG_DATA_BLOCK_COMMANDS;
#endif

/** @brief Possible remote log data block statuses. */
#ifndef HAVE_ENUM_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES
#define HAVE_ENUM_MAV_REMOTE_LOG_DATA_BLOCK_STATUSES
typedef enum MAV_REMOTE_LOG_DATA_BLOCK_STATUSES
{
   MAV_REMOTE_LOG_DATA_BLOCK_NACK=0, /* This block has NOT been received. | */
   MAV_REMOTE_LOG_DATA_BLOCK_ACK=1, /* This block has been received. | */
   MAV_REMOTE_LOG_DATA_BLOCK_STATUSES_ENUM_END=2, /*  | */
} MAV_REMOTE_LOG_DATA_BLOCK_STATUSES;
#endif

/** @brief Bus types for device operations. */
#ifndef HAVE_ENUM_DEVICE_OP_BUSTYPE
#define HAVE_ENUM_DEVICE_OP_BUSTYPE
typedef enum DEVICE_OP_BUSTYPE
{
   DEVICE_OP_BUSTYPE_I2C=0, /* I2C Device operation. | */
   DEVICE_OP_BUSTYPE_SPI=1, /* SPI Device operation. | */
   DEVICE_OP_BUSTYPE_ENUM_END=2, /*  | */
} DEVICE_OP_BUSTYPE;
#endif

/** @brief Deepstall flight stage. */
#ifndef HAVE_ENUM_DEEPSTALL_STAGE
#define HAVE_ENUM_DEEPSTALL_STAGE
typedef enum DEEPSTALL_STAGE
{
   DEEPSTALL_STAGE_FLY_TO_LANDING=0, /* Flying to the landing point. | */
   DEEPSTALL_STAGE_ESTIMATE_WIND=1, /* Building an estimate of the wind. | */
   DEEPSTALL_STAGE_WAIT_FOR_BREAKOUT=2, /* Waiting to breakout of the loiter to fly the approach. | */
   DEEPSTALL_STAGE_FLY_TO_ARC=3, /* Flying to the first arc point to turn around to the landing point. | */
   DEEPSTALL_STAGE_ARC=4, /* Turning around back to the deepstall landing point. | */
   DEEPSTALL_STAGE_APPROACH=5, /* Approaching the landing point. | */
   DEEPSTALL_STAGE_LAND=6, /* Stalling and steering towards the land point. | */
   DEEPSTALL_STAGE_ENUM_END=7, /*  | */
} DEEPSTALL_STAGE;
#endif

/** @brief A mapping of plane flight modes for custom_mode field of heartbeat. */
#ifndef HAVE_ENUM_PLANE_MODE
#define HAVE_ENUM_PLANE_MODE
typedef enum PLANE_MODE
{
   PLANE_MODE_MANUAL=0, /*  | */
   PLANE_MODE_CIRCLE=1, /*  | */
   PLANE_MODE_STABILIZE=2, /*  | */
   PLANE_MODE_TRAINING=3, /*  | */
   PLANE_MODE_ACRO=4, /*  | */
   PLANE_MODE_FLY_BY_WIRE_A=5, /*  | */
   PLANE_MODE_FLY_BY_WIRE_B=6, /*  | */
   PLANE_MODE_CRUISE=7, /*  | */
   PLANE_MODE_AUTOTUNE=8, /*  | */
   PLANE_MODE_AUTO=10, /*  | */
   PLANE_MODE_RTL=11, /*  | */
   PLANE_MODE_LOITER=12, /*  | */
   PLANE_MODE_TAKEOFF=13, /*  | */
   PLANE_MODE_AVOID_ADSB=14, /*  | */
   PLANE_MODE_GUIDED=15, /*  | */
   PLANE_MODE_INITIALIZING=16, /*  | */
   PLANE_MODE_QSTABILIZE=17, /*  | */
   PLANE_MODE_QHOVER=18, /*  | */
   PLANE_MODE_QLOITER=19, /*  | */
   PLANE_MODE_QLAND=20, /*  | */
   PLANE_MODE_QRTL=21, /*  | */
   PLANE_MODE_QAUTOTUNE=22, /*  | */
   PLANE_MODE_QACRO=23, /*  | */
   PLANE_MODE_ENUM_END=24, /*  | */
} PLANE_MODE;
#endif

/** @brief A mapping of copter flight modes for custom_mode field of heartbeat. */
#ifndef HAVE_ENUM_COPTER_MODE
#define HAVE_ENUM_COPTER_MODE
typedef enum COPTER_MODE
{
   COPTER_MODE_STABILIZE=0, /*  | */
   COPTER_MODE_ACRO=1, /*  | */
   COPTER_MODE_ALT_HOLD=2, /*  | */
   COPTER_MODE_AUTO=3, /*  | */
   COPTER_MODE_GUIDED=4, /*  | */
   COPTER_MODE_LOITER=5, /*  | */
   COPTER_MODE_RTL=6, /*  | */
   COPTER_MODE_CIRCLE=7, /*  | */
   COPTER_MODE_LAND=9, /*  | */
   COPTER_MODE_DRIFT=11, /*  | */
   COPTER_MODE_SPORT=13, /*  | */
   COPTER_MODE_FLIP=14, /*  | */
   COPTER_MODE_AUTOTUNE=15, /*  | */
   COPTER_MODE_POSHOLD=16, /*  | */
   COPTER_MODE_BRAKE=17, /*  | */
   COPTER_MODE_THROW=18, /*  | */
   COPTER_MODE_AVOID_ADSB=19, /*  | */
   COPTER_MODE_GUIDED_NOGPS=20, /*  | */
   COPTER_MODE_SMART_RTL=21, /*  | */
   COPTER_MODE_FLOWHOLD=22, /*  | */
   COPTER_MODE_FOLLOW=23, /*  | */
   COPTER_MODE_ZIGZAG=24, /*  | */
   COPTER_MODE_SYSTEMID=25, /*  | */
   COPTER_MODE_AUTOROTATE=26, /*  | */
   COPTER_MODE_ENUM_END=27, /*  | */
} COPTER_MODE;
#endif

/** @brief A mapping of sub flight modes for custom_mode field of heartbeat. */
#ifndef HAVE_ENUM_SUB_MODE
#define HAVE_ENUM_SUB_MODE
typedef enum SUB_MODE
{
   SUB_MODE_STABILIZE=0, /*  | */
   SUB_MODE_ACRO=1, /*  | */
   SUB_MODE_ALT_HOLD=2, /*  | */
   SUB_MODE_AUTO=3, /*  | */
   SUB_MODE_GUIDED=4, /*  | */
   SUB_MODE_CIRCLE=7, /*  | */
   SUB_MODE_SURFACE=9, /*  | */
   SUB_MODE_POSHOLD=16, /*  | */
   SUB_MODE_MANUAL=19, /*  | */
   SUB_MODE_ENUM_END=20, /*  | */
} SUB_MODE;
#endif

/** @brief A mapping of rover flight modes for custom_mode field of heartbeat. */
#ifndef HAVE_ENUM_ROVER_MODE
#define HAVE_ENUM_ROVER_MODE
typedef enum ROVER_MODE
{
   ROVER_MODE_MANUAL=0, /*  | */
   ROVER_MODE_ACRO=1, /*  | */
   ROVER_MODE_STEERING=3, /*  | */
   ROVER_MODE_HOLD=4, /*  | */
   ROVER_MODE_LOITER=5, /*  | */
   ROVER_MODE_FOLLOW=6, /*  | */
   ROVER_MODE_SIMPLE=7, /*  | */
   ROVER_MODE_AUTO=10, /*  | */
   ROVER_MODE_RTL=11, /*  | */
   ROVER_MODE_SMART_RTL=12, /*  | */
   ROVER_MODE_GUIDED=15, /*  | */
   ROVER_MODE_INITIALIZING=16, /*  | */
   ROVER_MODE_ENUM_END=17, /*  | */
} ROVER_MODE;
#endif

/** @brief A mapping of antenna tracker flight modes for custom_mode field of heartbeat. */
#ifndef HAVE_ENUM_TRACKER_MODE
#define HAVE_ENUM_TRACKER_MODE
typedef enum TRACKER_MODE
{
   TRACKER_MODE_MANUAL=0, /*  | */
   TRACKER_MODE_STOP=1, /*  | */
   TRACKER_MODE_SCAN=2, /*  | */
   TRACKER_MODE_SERVO_TEST=3, /*  | */
   TRACKER_MODE_AUTO=10, /*  | */
   TRACKER_MODE_INITIALIZING=16, /*  | */
   TRACKER_MODE_ENUM_END=17, /*  | */
} TRACKER_MODE;
#endif

/** @brief The type of parameter for the OSD parameter editor. */
#ifndef HAVE_ENUM_OSD_PARAM_CONFIG_TYPE
#define HAVE_ENUM_OSD_PARAM_CONFIG_TYPE
typedef enum OSD_PARAM_CONFIG_TYPE
{
   OSD_PARAM_NONE=0, /*  | */
   OSD_PARAM_SERIAL_PROTOCOL=1, /*  | */
   OSD_PARAM_SERVO_FUNCTION=2, /*  | */
   OSD_PARAM_AUX_FUNCTION=3, /*  | */
   OSD_PARAM_FLIGHT_MODE=4, /*  | */
   OSD_PARAM_FAILSAFE_ACTION=5, /*  | */
   OSD_PARAM_FAILSAFE_ACTION_1=6, /*  | */
   OSD_PARAM_FAILSAFE_ACTION_2=7, /*  | */
   OSD_PARAM_NUM_TYPES=8, /*  | */
   OSD_PARAM_CONFIG_TYPE_ENUM_END=9, /*  | */
} OSD_PARAM_CONFIG_TYPE;
#endif

/** @brief The error type for the OSD parameter editor. */
#ifndef HAVE_ENUM_OSD_PARAM_CONFIG_ERROR
#define HAVE_ENUM_OSD_PARAM_CONFIG_ERROR
typedef enum OSD_PARAM_CONFIG_ERROR
{
   OSD_PARAM_SUCCESS=0, /*  | */
   OSD_PARAM_INVALID_SCREEN=1, /*  | */
   OSD_PARAM_INVALID_PARAMETER_INDEX=2, /*  | */
   OSD_PARAM_INVALID_PARAMETER=3, /*  | */
   OSD_PARAM_CONFIG_ERROR_ENUM_END=4, /*  | */
} OSD_PARAM_CONFIG_ERROR;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_sensor_offsets.h"
#include "./mavlink_msg_set_mag_offsets.h"
#include "./mavlink_msg_meminfo.h"
#include "./mavlink_msg_ap_adc.h"
#include "./mavlink_msg_digicam_configure.h"
#include "./mavlink_msg_digicam_control.h"
#include "./mavlink_msg_mount_configure.h"
#include "./mavlink_msg_mount_control.h"
#include "./mavlink_msg_mount_status.h"
#include "./mavlink_msg_fence_point.h"
#include "./mavlink_msg_fence_fetch_point.h"
#include "./mavlink_msg_ahrs.h"
#include "./mavlink_msg_simstate.h"
#include "./mavlink_msg_hwstatus.h"
#include "./mavlink_msg_radio.h"
#include "./mavlink_msg_limits_status.h"
#include "./mavlink_msg_wind.h"
#include "./mavlink_msg_data16.h"
#include "./mavlink_msg_data32.h"
#include "./mavlink_msg_data64.h"
#include "./mavlink_msg_data96.h"
#include "./mavlink_msg_rangefinder.h"
#include "./mavlink_msg_airspeed_autocal.h"
#include "./mavlink_msg_rally_point.h"
#include "./mavlink_msg_rally_fetch_point.h"
#include "./mavlink_msg_compassmot_status.h"
#include "./mavlink_msg_ahrs2.h"
#include "./mavlink_msg_camera_status.h"
#include "./mavlink_msg_camera_feedback.h"
#include "./mavlink_msg_battery2.h"
#include "./mavlink_msg_ahrs3.h"
#include "./mavlink_msg_autopilot_version_request.h"
#include "./mavlink_msg_remote_log_data_block.h"
#include "./mavlink_msg_remote_log_block_status.h"
#include "./mavlink_msg_led_control.h"
#include "./mavlink_msg_mag_cal_progress.h"
#include "./mavlink_msg_ekf_status_report.h"
#include "./mavlink_msg_pid_tuning.h"
#include "./mavlink_msg_deepstall.h"
#include "./mavlink_msg_gimbal_report.h"
#include "./mavlink_msg_gimbal_control.h"
#include "./mavlink_msg_gimbal_torque_cmd_report.h"
#include "./mavlink_msg_gopro_heartbeat.h"
#include "./mavlink_msg_gopro_get_request.h"
#include "./mavlink_msg_gopro_get_response.h"
#include "./mavlink_msg_gopro_set_request.h"
#include "./mavlink_msg_gopro_set_response.h"
#include "./mavlink_msg_rpm.h"
#include "./mavlink_msg_device_op_read.h"
#include "./mavlink_msg_device_op_read_reply.h"
#include "./mavlink_msg_device_op_write.h"
#include "./mavlink_msg_device_op_write_reply.h"
#include "./mavlink_msg_adap_tuning.h"
#include "./mavlink_msg_vision_position_delta.h"
#include "./mavlink_msg_aoa_ssa.h"
#include "./mavlink_msg_esc_telemetry_1_to_4.h"
#include "./mavlink_msg_esc_telemetry_5_to_8.h"
#include "./mavlink_msg_esc_telemetry_9_to_12.h"
#include "./mavlink_msg_osd_param_config.h"
#include "./mavlink_msg_osd_param_config_reply.h"
#include "./mavlink_msg_osd_param_show_config.h"
#include "./mavlink_msg_osd_param_show_config_reply.h"

// base include
#include "../common/common.h"
#include "../uAvionix/uAvionix.h"
#include "../icarous/icarous.h"

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 1

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_SENSOR_OFFSETS, MAVLINK_MESSAGE_INFO_SET_MAG_OFFSETS, MAVLINK_MESSAGE_INFO_MEMINFO, MAVLINK_MESSAGE_INFO_AP_ADC, MAVLINK_MESSAGE_INFO_DIGICAM_CONFIGURE, MAVLINK_MESSAGE_INFO_DIGICAM_CONTROL, MAVLINK_MESSAGE_INFO_MOUNT_CONFIGURE, MAVLINK_MESSAGE_INFO_MOUNT_CONTROL, MAVLINK_MESSAGE_INFO_MOUNT_STATUS, MAVLINK_MESSAGE_INFO_FENCE_POINT, MAVLINK_MESSAGE_INFO_FENCE_FETCH_POINT, MAVLINK_MESSAGE_INFO_AHRS, MAVLINK_MESSAGE_INFO_SIMSTATE, MAVLINK_MESSAGE_INFO_HWSTATUS, MAVLINK_MESSAGE_INFO_RADIO, MAVLINK_MESSAGE_INFO_LIMITS_STATUS, MAVLINK_MESSAGE_INFO_WIND, MAVLINK_MESSAGE_INFO_DATA16, MAVLINK_MESSAGE_INFO_DATA32, MAVLINK_MESSAGE_INFO_DATA64, MAVLINK_MESSAGE_INFO_DATA96, MAVLINK_MESSAGE_INFO_RANGEFINDER, MAVLINK_MESSAGE_INFO_AIRSPEED_AUTOCAL, MAVLINK_MESSAGE_INFO_RALLY_POINT, MAVLINK_MESSAGE_INFO_RALLY_FETCH_POINT, MAVLINK_MESSAGE_INFO_COMPASSMOT_STATUS, MAVLINK_MESSAGE_INFO_AHRS2, MAVLINK_MESSAGE_INFO_CAMERA_STATUS, MAVLINK_MESSAGE_INFO_CAMERA_FEEDBACK, MAVLINK_MESSAGE_INFO_BATTERY2, MAVLINK_MESSAGE_INFO_AHRS3, MAVLINK_MESSAGE_INFO_AUTOPILOT_VERSION_REQUEST, MAVLINK_MESSAGE_INFO_REMOTE_LOG_DATA_BLOCK, MAVLINK_MESSAGE_INFO_REMOTE_LOG_BLOCK_STATUS, MAVLINK_MESSAGE_INFO_LED_CONTROL, MAVLINK_MESSAGE_INFO_MAG_CAL_PROGRESS, MAVLINK_MESSAGE_INFO_EKF_STATUS_REPORT, MAVLINK_MESSAGE_INFO_PID_TUNING, MAVLINK_MESSAGE_INFO_DEEPSTALL, MAVLINK_MESSAGE_INFO_GIMBAL_REPORT, MAVLINK_MESSAGE_INFO_GIMBAL_CONTROL, MAVLINK_MESSAGE_INFO_GIMBAL_TORQUE_CMD_REPORT, MAVLINK_MESSAGE_INFO_GOPRO_HEARTBEAT, MAVLINK_MESSAGE_INFO_GOPRO_GET_REQUEST, MAVLINK_MESSAGE_INFO_GOPRO_GET_RESPONSE, MAVLINK_MESSAGE_INFO_GOPRO_SET_REQUEST, MAVLINK_MESSAGE_INFO_GOPRO_SET_RESPONSE, MAVLINK_MESSAGE_INFO_RPM, MAVLINK_MESSAGE_INFO_DEVICE_OP_READ, MAVLINK_MESSAGE_INFO_DEVICE_OP_READ_REPLY, MAVLINK_MESSAGE_INFO_DEVICE_OP_WRITE, MAVLINK_MESSAGE_INFO_DEVICE_OP_WRITE_REPLY, MAVLINK_MESSAGE_INFO_ADAP_TUNING, MAVLINK_MESSAGE_INFO_VISION_POSITION_DELTA, MAVLINK_MESSAGE_INFO_AOA_SSA, MAVLINK_MESSAGE_INFO_ESC_TELEMETRY_1_TO_4, MAVLINK_MESSAGE_INFO_ESC_TELEMETRY_5_TO_8, MAVLINK_MESSAGE_INFO_ESC_TELEMETRY_9_TO_12, MAVLINK_MESSAGE_INFO_OSD_PARAM_CONFIG, MAVLINK_MESSAGE_INFO_OSD_PARAM_CONFIG_REPLY, MAVLINK_MESSAGE_INFO_OSD_PARAM_SHOW_CONFIG, MAVLINK_MESSAGE_INFO_OSD_PARAM_SHOW_CONFIG_REPLY}
# define MAVLINK_MESSAGE_NAMES {{ "ADAP_TUNING", 11010 }, { "AHRS", 163 }, { "AHRS2", 178 }, { "AHRS3", 182 }, { "AIRSPEED_AUTOCAL", 174 }, { "AOA_SSA", 11020 }, { "AP_ADC", 153 }, { "AUTOPILOT_VERSION_REQUEST", 183 }, { "BATTERY2", 181 }, { "CAMERA_FEEDBACK", 180 }, { "CAMERA_STATUS", 179 }, { "COMPASSMOT_STATUS", 177 }, { "DATA16", 169 }, { "DATA32", 170 }, { "DATA64", 171 }, { "DATA96", 172 }, { "DEEPSTALL", 195 }, { "DEVICE_OP_READ", 11000 }, { "DEVICE_OP_READ_REPLY", 11001 }, { "DEVICE_OP_WRITE", 11002 }, { "DEVICE_OP_WRITE_REPLY", 11003 }, { "DIGICAM_CONFIGURE", 154 }, { "DIGICAM_CONTROL", 155 }, { "EKF_STATUS_REPORT", 193 }, { "ESC_TELEMETRY_1_TO_4", 11030 }, { "ESC_TELEMETRY_5_TO_8", 11031 }, { "ESC_TELEMETRY_9_TO_12", 11032 }, { "FENCE_FETCH_POINT", 161 }, { "FENCE_POINT", 160 }, { "GIMBAL_CONTROL", 201 }, { "GIMBAL_REPORT", 200 }, { "GIMBAL_TORQUE_CMD_REPORT", 214 }, { "GOPRO_GET_REQUEST", 216 }, { "GOPRO_GET_RESPONSE", 217 }, { "GOPRO_HEARTBEAT", 215 }, { "GOPRO_SET_REQUEST", 218 }, { "GOPRO_SET_RESPONSE", 219 }, { "HWSTATUS", 165 }, { "LED_CONTROL", 186 }, { "LIMITS_STATUS", 167 }, { "MAG_CAL_PROGRESS", 191 }, { "MEMINFO", 152 }, { "MOUNT_CONFIGURE", 156 }, { "MOUNT_CONTROL", 157 }, { "MOUNT_STATUS", 158 }, { "OSD_PARAM_CONFIG", 11033 }, { "OSD_PARAM_CONFIG_REPLY", 11034 }, { "OSD_PARAM_SHOW_CONFIG", 11035 }, { "OSD_PARAM_SHOW_CONFIG_REPLY", 11036 }, { "PID_TUNING", 194 }, { "RADIO", 166 }, { "RALLY_FETCH_POINT", 176 }, { "RALLY_POINT", 175 }, { "RANGEFINDER", 173 }, { "REMOTE_LOG_BLOCK_STATUS", 185 }, { "REMOTE_LOG_DATA_BLOCK", 184 }, { "RPM", 226 }, { "SENSOR_OFFSETS", 150 }, { "SET_MAG_OFFSETS", 151 }, { "SIMSTATE", 164 }, { "VISION_POSITION_DELTA", 11011 }, { "WIND", 168 }}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_ARDUPILOTMEGA_H
