#ifndef NOVATEL_MSGS_CLOCKSTEERING_H
#define NOVATEL_MSGS_CLOCKSTEERING_H
#include <stdint.h>
#include <string>

#include "base.h"
#include <boost/shared_ptr.hpp>
#include "messageheader.h"
#include "extendedsolutionstatus.h"
#include "signalmark.h"


using namespace base;

namespace novatel_gps_msgs
{
	typedef struct ClockSteering
	{
		//# The CLOCKSTEERING log is used to monitor the current state of the clock steering process.

		int8 INTERNAL_SOURCE = 0;
		int8 EXTERNAL_SOURCE = 1;

		int8 FIRST_ORDER_STEERING_STATE = 0;
		int8 SECOND_ORDER_STEERING_STATE = 1;
		int8 CALIBRATE_HIGH_STEERING_STATE = 2;
		int8 CALIBRATE_LOW_STEERING_STATE = 3;
		int8 CALIBRATE_CENTER_STEERING_STATE = 4;

			//# Clock source
		std::string source;

		//# Steering state
			std::string steering_state;

		//	# Period of the FREQUENCYOUT signal used to control the oscillator
			uint32 period;

			//# Current pulse width of the FREQUENCYOUT signal.
			float64 pulse_width;

			//# The current band width of the clock steering tracking loop in Hz.
			float64 bandwidth;

			//# The current clock drift change in m / s / bit for a 1 LSB pulse width.
			float32 slope;

			//# The last valid receiver clock offset computed(m).
			float64 offset;

			//# The last valid receiver clock drift rate received(m / s).
			float64 drift_rate;

	}CLOCKSTEERING;

	typedef boost::shared_ptr< novatel_gps_msgs::ClockSteering > ClockSteeringPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::ClockSteering const> ClockSteeringConstPtr;
}// end of namespace
#endif