#ifndef NOVATEL_MSGS_TIME_H
#define NOVATEL_MSGS_TIME_H
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
	typedef struct Time
	{
		/*
		     Clock model status
			# see Table 65 on pg 322 of the OEM6 Family Firmware Reference Manual, Rev3
			*/
		std::string clock_status;

		/*
			# Receiver clock offset, in seconds, from GPS reference time.A positive offset
			# implies that the receiver clock is ahead of GPS reference time.To derive
			# GPS reference time, use the following formula :
		# GPS reference time = receiver time - offset*/
		float64 offset;

		/*	# Standard deviation of the offset*/
		float64 offset_std;

		/*	# The offset of the GPS reference time from UTC time, computed using almanac
			# parameters.UTC time is GPS reference time plus the current UTC offset plus
			# the receiver clock offset :
		# UTC time = GPS reference time + offset + UTC offset*/
		float64 utc_offset;

		uint32 utc_year;
		uint8 utc_month;
		uint8 utc_day;
		uint8 utc_hour;
		uint8 utc_minute;
		uint32 utc_millisecond;

		std::string utc_status;

	}TIME;

	typedef boost::shared_ptr< novatel_gps_msgs::Time > TimePtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Time const> TimeConstPtr;
}// end of namespace
#endif