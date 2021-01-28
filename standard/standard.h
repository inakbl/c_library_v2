/** @file
 *  @brief MAVLink comm protocol generated from standard.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_STANDARD_H
#define MAVLINK_STANDARD_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_STANDARD.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 5

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_STANDARD

// ENUM DEFINITIONS



// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS


// base include
#include "../common/common.h"

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 5

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {}
# define MAVLINK_MESSAGE_NAMES {}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_STANDARD_H
