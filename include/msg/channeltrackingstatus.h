#ifndef NOVATEL_MSGS_CHANNELTRACKINGSTATUS_H
#define NOVATEL_MSGS_CHANNELTRACKINGSTATUS_H
#include <stdint.h>
#include <string>

#include "base.h"
#include "messageheader.h"

using namespace base;

namespace novatel_gps_msgs
{
	enum TRCKING_STATE
	{
		Idle = 0,
		Sky_Search,
		Wide_frequency_band_pull_in,
		Narrow_frequency_band_pull_in,
		Phase_lock_loop,
		Channel_steering,
		Frequency_lock_loop,
		Channel_alignment,
		Code_search,
		Aided_phase_lock_loop,
		Side_peak_detection
	};
	enum CORRELATOR_TYPE
	{
		TYPE_NULL = 0,
		Standard_correlator,  //spacing = 1 chip
		Narrow_Correlator,    //spacing < 1 chip
		Reserved,
		Pulse_Aperture_Correlator
	};
	enum STATELLITE_SYSTEM
	{
		GPS = 0,
		GLONASS,
		SBAS,
		Galileo,
		BeiDou,
		QZSS,
		NavIC,
		Other
	};
	enum SIGNAL_TYPE
	{
		GPS_L1CA = 0,GPS_L2P,GPS_L2PY,GPS_L5Q,GPS_L1CP,GPS_L2CM,
		GLONASS_L1CA,GLONASS_L2CA,GLONASS_L2P,GLONASS_L3Q,
		BEIDOU_B1_1,BEIDOU_B2_1,BEIDOU_B3_1,BEIDOU_B1_2,BEIDOU_B2_2,BEIDOU_B3_2,BEIDOU_B1C,BEIDOU_B2C,
		GALILEO_E1C,GALILEO_E6B,GALILEO_E6C, GALILEO_E5A, GALILEO_E5B, GALILEO_E5AltBOC,
		QZSS_L1CA,QZSS_L5Q,QZSS_L1CP,QZSS_L2CM,QZSS_L6P,
		SBAS_L1CA,SBAS_L5I,
		NAVIC_L5_SPS,
		OTHER_L_BAND
	};
	typedef struct ChannelTrackingStatus
	{
		uint32 tacking_state;
		uint32 sv_channel_number;
		
		bool is_phase_lock_flag; // 0 = not locked, 1 = locked
		bool is_code_locked_flah; // 0 = not locked, 1 = locked

		uint8 correlator_type;
		/*0 = GPS 1 = GLONASS 2 = SBAS 3 = Galileo 4 = BeiDou 5 = QZSS 6 = NavIC 7 = Other*/
		uint8 statellite_system;  
		bool is_grouping; //0 = Not grouped, 1 = Grouped

		uint32 signal_type;

		bool is_primary_l1_channel; //0 = Not primary, 1 = Primary
		bool is_carrier_phase_measurement; //0 = Half Cycle Not Added ,1 = Half Cycle Added
		bool is_digital_filtering_on_signal; //0 = No digital filter, 1 = Digital filter
		bool is_PRN_lock_flag;              //0 = PRN Not Locked Out,1 = PRN Locked Out
		bool is_Channel_assignment;         //0 = Automatic, 1 = Forced
	}CHANNELTRACKINGSTATUS;

}// end of namespace
#endif
