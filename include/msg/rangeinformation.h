#ifndef NOVATEL_MSGS_RANGEINFORMATION_H
#define NOVATEL_MSGS_RANGEINFORMATION_H
#include <stdint.h>
#include <string>

#include "base.h"
#include "messageheader.h"

using namespace base;

namespace novatel_gps_msgs
{
	struct  TRUCKING_STATUS
	{
		int sys;
		int code;
		int track;
		int plock;
		int clock;
		int parity;
		int halfc;
	};
	typedef struct RangeInformation
	{
		/*Satellite Range information structure used in range messages*/

		/*Satelite PRN number of range measurement*/
		uint16 prn_number;

		//GLONASS Frequency
		uint16 glofreq;

		//Pseudorange measurement(m)
		float64 psr;

		//Pseudorange measurement standard deviation(m)
		float32 psr_std;

		//Carrier phase, in cycles
		float64 adr;

		//Estimated carrier phase standard deviation(cycles)
		float32 adr_std;

		//Instantaneous carrier Doppler frequency(Hz)
		float32 dopp;

		//Carrier to noise density ratio
		float32 noise_density_ratio;

        //of seconds of continous tracking
		float32 locktime;

		//Tracking status
		uint32 tracking_status;
		TRUCKING_STATUS trackstat;

	}RANGEINFORMATION;
}// end of namespace
#endif

