#ifndef AIGLE_TYPES_H
#define AIGLE_TYPES_H

#include <cstdint>

const unsigned n_out_max = 16;

typedef struct _gps_data {
	uint64_t time_usec; /*< Timestamp (microseconds since UNIX epoch or microseconds since system boot)*/
	int32_t lat; /*< Latitude (WGS84), in degrees * 1E7*/
	int32_t lon; /*< Longitude (WGS84), in degrees * 1E7*/
	int32_t alt; /*< Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)*/
	uint16_t eph; /*< GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535*/
	uint16_t epv; /*< GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535*/
	uint16_t vel; /*< GPS ground speed in cm/s. If unknown, set to: 65535*/
	int16_t vn; /*< GPS velocity in cm/s in NORTH direction in earth-fixed NED frame*/
	int16_t ve; /*< GPS velocity in cm/s in EAST direction in earth-fixed NED frame*/
	int16_t vd; /*< GPS velocity in cm/s in DOWN direction in earth-fixed NED frame*/
	uint16_t cog; /*< Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535*/
	uint8_t fix_type; /*< 0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix.*/
	uint8_t satellites_visible; /*< Number of satellites visible. If unknown, set to 255*/
} gps_data;

typedef struct _motor_data{
	uint8_t is_armed;
	uint32_t motor_values[n_out_max];
	// uint32_t motor_values_size; 
} motor_data;

typedef struct _topology {
	int x_max;
	int x_min;
	int y_max;
	int y_min;
	int z_max;
	int z_min;
} topology;

typedef enum {
	MODE_SAFE,
	MODE_UNSAFE
} MODE;

#endif //AIGLE_TYPES_H