#ifndef NOVATEL_MSGS_GPGSA_H
#define NOVATEL_MSGS_GPGSA_H
#include <stdint.h>
#include <string>
#include <boost/shared_ptr.hpp>
#include <vector>
#include "base.h"

using namespace base;

namespace novatel_gps_msgs
{
	typedef struct Gpgsa
	{
		std::string message_id;

		std::string auto_manual_mode;

		uint8 fix_mode;

		std::vector<uint8> sv_ids;

		float32 pdop;
		float32 hdop;
		float32 vdop;
	}GPGSA;

	typedef boost::shared_ptr< novatel_gps_msgs::Gpgsa > GpgsaPtr;
	typedef boost::shared_ptr< novatel_gps_msgs::Gpgsa const> GpgsaConstPtr;
} // end of namespace
#endif