#ifndef NOVATEL_MSGS_GPGSV_H
#define NOVATEL_MSGS_GPGSV_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <vector>
#include "base.h"
#include "satellite.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Gpgsv
	{
		std::string message_id;

		//Number of messages in this sequence
		uint8 n_msgs;
		// This messages number in its sequence.The first message is number 1.
		uint8 msg_number;


		//Number of satellites currently visible
		uint8 n_satellites;

		//Up to 4 satellites are described in each message
		std::vector<Satellite> satellites;

	}GPGSV;

	typedef boost::shared_ptr< novatel_gps_msgs::Gpgsv > GpgsvPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Gpgsv const> GpgsvConstPtr;
} // end of namespace
#endif