#ifndef NOVATEL_MSGS_SATELLITE_H
#define NOVATEL_MSGS_SATELLITE_H
#include <stdint.h>
#include <string>

#include "base.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Satellite
	{
		/* Satellite data structure used in GPGSV messages

			# PRN number of the satellite
			# GPS = 1..32
			# SBAS = 33..64
			# GLO = 65..96*/
		uint8 prn;

		// Elevation, degrees.Maximum 90
		uint8 elevation;

		// Azimuth, True North degrees.[0, 359]
		uint16 azimuth;

		// Signal to noise ratio, 0 - 99 dB. - 1 when null in NMEA sentence(not tracking)
		int8 snr;
	}SATELLITE;
}// end of namespace
#endif

