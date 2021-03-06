/*
 * common.h
 *
 *  Created on: 2021.02.01
 *      Author: JackyPan
 */
#pragma once
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMON_H
#define __COMMON_H

#include "hal.h"

// used to pack structures
#define PACKED __attribute__((__packed__))

// used to mark a function that may be unused in some builds
#define UNUSED_FUNCTION __attribute__((unused))

// this can be used to optimize individual functions
#define OPTIMIZE(level) __attribute__((optimize(level)))

// sometimes we need to prevent inlining to prevent large stack usage
#define NOINLINE __attribute__((noinline))

#ifdef __has_cpp_attribute
#  if __has_cpp_attribute(fallthrough)
#    define FALLTHROUGH [[fallthrough]]
#  elif __has_cpp_attribute(gnu::fallthrough)
#    define FALLTHROUGH [[gnu::fallthrough]]
#  endif
#endif
#ifndef FALLTHROUGH
#  define FALLTHROUGH
#endif

#define ToRad(x) radians(x)	// *pi/180
#define ToDeg(x) degrees(x)	// *180/pi

/* Declare and implement const and non-const versions of the array subscript
 * operator. The object is treated as an array of type_ values. */
#define DEFINE_BYTE_ARRAY_METHODS                                                                   \
    inline uint8_t &operator[](size_t i) { return reinterpret_cast<uint8_t *>(this)[i]; }           \
    inline uint8_t operator[](size_t i) const { return reinterpret_cast<const uint8_t *>(this)[i]; }

#define LOCATION_ALT_MAX_M  83000   // maximum altitude (in meters) that can be fit into Location structure's alt field

/*
  check if bit bitnumber is set in value, returned as a
  bool. Bitnumber starts at 0 for the first bit
 */
#define BIT_IS_SET(value, bitnumber) (((value) & (1U<<(bitnumber))) != 0)

// get high or low bytes from 2 byte integer
#define LOWBYTE(i) ((uint8_t)(i))
#define HIGHBYTE(i) ((uint8_t)(((uint16_t)(i))>>8))

#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))

/*
 * See UNUSED_RESULT. The difference is that it receives @uniq_ as the name to
 * be used for its internal variable.
 *
 * @uniq_: a unique name to use for variable name
 * @expr_: the expression to be evaluated
 */
#define _UNUSED_RESULT(uniq_, expr_)                      \
    do {                                                  \
        decltype(expr_) uniq_ __attribute__((unused));    \
        uniq_ = expr_;                                    \
    } while (0)

/*
 * Allow to call a function annotated with warn_unused_result attribute
 * without getting a warning, because sometimes this is what we want to do.
 *
 * @expr_: the expression to be evaluated
 */
#define UNUSED_RESULT(expr_) _UNUSED_RESULT(__unique_name_##__COUNTER__, expr_)

// @}


////////////////////////////////////////////////////////////////////////////////
/// @name	Types
///
/// Data structures and types used throughout the libraries and applications. 0 = default
/// bit 0: Altitude is stored               0: Absolute,	1: Relative
/// bit 1: Change Alt between WP            0: Gradually,	1: ASAP
/// bit 2: Direction of loiter command      0: Clockwise	1: Counter-Clockwise
/// bit 3: Req.to hit WP.alt to continue    0: No,          1: Yes
/// bit 4: Relative to Home					0: No,          1: Yes
/// bit 5: Loiter crosstrack reference      0: WP center    1: Tangent exit point
/// bit 6:
/// bit 7: Move to next Command             0: YES,         1: Loiter until commanded

//@{

struct PACKED Location_Option_Flags {
    uint8_t relative_alt : 1;           // 1 if altitude is relative to home
    uint8_t unused1      : 1;           // unused flag (defined so that loiter_ccw uses the correct bit)
    uint8_t loiter_ccw   : 1;           // 0 if clockwise, 1 if counter clockwise
    uint8_t terrain_alt  : 1;           // this altitude is above terrain
    uint8_t origin_alt   : 1;           // this altitude is above ekf origin
    uint8_t loiter_xtrack : 1;          // 0 to crosstrack from center of waypoint, 1 to crosstrack from tangent exit location
};

struct PACKED Location {
    union {
        Location_Option_Flags flags;                    ///< options bitmask (1<<0 = relative altitude)
        uint8_t options;                                /// allows writing all flags to eeprom as one byte
    };
    // by making alt 24 bit we can make p1 in a command 16 bit,
    // allowing an accurate angle in centi-degrees. This keeps the
    // storage cost per mission item at 15 bytes, and allows mission
    // altitudes of up to +/- 83km
    int32_t alt:24;                                     ///< param 2 - Altitude in centimeters (meters * 100) see LOCATION_ALT_MAX_M
    int32_t lat;                                        ///< param 3 - Latitude * 10**7
    int32_t lng;                                        ///< param 4 - Longitude * 10**7
};

//@}

////////////////////////////////////////////////////////////////////////////////
/// @name	Conversions
///
/// Conversion macros and factors.
///
//@{

/*
  Return true if value is between lower and upper bound inclusive.
  False otherwise.
*/
bool is_bounded_int32(int32_t value, int32_t lower_bound, int32_t upper_bound);

#endif /* __COMMON_H */
