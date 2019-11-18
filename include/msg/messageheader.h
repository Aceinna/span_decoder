#ifndef NOVATEL_MSGS_MESSAGEHEADER_H
#define NOVATEL_MSGS_MESSAGEHEADER_H
#include <stdint.h>
#include <string>
#include "base.h"
#include "receiverstatus.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct MessageHeader
	{
		std::string message_name;
		std::string port;

		uint32 sequence_num;
		float32 percent_idle_time;
		std::string gps_time_status;
		uint32 gps_week_num;
		float64 gps_seconds; 
		ReceiverStatus receiver_status;
		uint32 receiver_software_version;

	}MESSAGEHEADER;

	typedef struct ShortMessageHeader
	{
		std::string message_name;

		uint32 gps_week_num;
		float64 gps_seconds;

	}SHORTMESSAGEHEADER;


}// end of namespace
#endif