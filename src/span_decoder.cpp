#include "include/novatel_gps.h"
#include <swri_util/string_util.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <vector>

#define	grav_WGS84 9.7803267714e0
#ifndef PI
#define	PI 3.14159265358979
#endif

#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_27_F     2.270936965942383E-09 /* 2^-27 FEET */
#define P2_29_F     5.677342414855957E-10 /* 2^-29 FEET */

#define P2_29       1.862645149230957E-09 /* 2^-29 */
enum DataInputType
{
	ASCII = 0,
	BINARY
};

bool is_acena = true;
DataInputType DataInputType_ = ASCII;

std::ifstream iput_file;
std::ofstream output_gpgga;
std::ofstream output_gps;
std::ofstream output_imu;
std::ofstream output_ins;
std::ofstream output_kml;
std::ofstream output_pos;

std::ofstream output_process;


bool publish_gpgga_ = TRUE;
bool publish_gpgsa_ = TRUE;
bool publish_gpgsv_ = TRUE;
bool publish_gphdt_ = TRUE;
bool publish_clock_steering_ = TRUE;
bool publish_imu_messages_ = TRUE;
bool publish_novatel_positions_ = TRUE;
bool publish_novatel_xyz_positions_ = TRUE;
bool publish_novatel_utm_positions_ = TRUE;
bool publish_novatel_velocity_ = TRUE;
bool publish_novatel_heading2_ = TRUE;
bool publish_novatel_dual_antenna_heading_ = TRUE;
bool publish_nmea_messages_ = TRUE;
bool publish_range_messages_ = TRUE;
bool publish_time_messages_ = TRUE;
bool publish_trackstat_ = TRUE;
bool publish_diagnostics_ = TRUE;
bool publish_sync_diagnostic_ = TRUE;
bool publish_gps_bin = TRUE;
bool publish_inspvax_ = TRUE;
bool publish_kml_ = TRUE;

static std::map<uint8, std::pair<std::vector<double>, std::string>> rates = {
  { 0,  std::pair<std::vector<double>, std::string>({100,2.0,100}, "Unknown") },
  { 1,  std::pair<std::vector<double>, std::string>({100,100,100}, "Honeywell HG1700 AG11") },
  { 4,  std::pair<std::vector<double>, std::string>({100,100,100}, "Honeywell HG1700 AG17") },
  { 5,  std::pair<std::vector<double>, std::string>({100,100,100}, "Honeywell HG1700 CA29") },
  { 8,  std::pair<std::vector<double>, std::string>({200,100,100}, "Litton LN-200 (200hz model)") },
  { 11, std::pair<std::vector<double>, std::string>({100,P2_33,P2_27_F}, "Honeywell HG1700 AG58") },
  { 12, std::pair<std::vector<double>, std::string>({100,100,100}, "Honeywell HG1700 AG62") },
  { 13, std::pair<std::vector<double>, std::string>({200,100,100}, "iMAR ilMU-FSAS") },
  { 16, std::pair<std::vector<double>, std::string>({200,100,100}, "KVH 1750 IMU") },
  { 19, std::pair<std::vector<double>, std::string>({200,100,100}, "Northrop Grumman Litef LCI-1") },
  { 20, std::pair<std::vector<double>, std::string>({100,100,100}, "Honeywell HG1930 AA99") },
  { 26, std::pair<std::vector<double>, std::string>({100,100,100}, "Northrop Grumman Litef ISA-100C") },
  { 27, std::pair<std::vector<double>, std::string>({100,100,100}, "Honeywell HG1900 CA50") },
  { 28, std::pair<std::vector<double>, std::string>({100,100,100}, "Honeywell HG1930 CA50") },
  { 31, std::pair<std::vector<double>, std::string>({200,100,100}, "Analog Devices ADIS16488") },
  { 32, std::pair<std::vector<double>, std::string>({125,100,100}, "Sensonor STIM300") },
  { 33, std::pair<std::vector<double>, std::string>({200,100,100}, "KVH1750 IMU") },
  { 34, std::pair<std::vector<double>, std::string>({200,100,100}, "Northrop Grumman Litef ISA-100") },
  { 38, std::pair<std::vector<double>, std::string>({400,100,100}, "Northrop Grumman Litef ISA-100 400Hz") },
  { 39, std::pair<std::vector<double>, std::string>({400,100,100}, "Northrop Grumman Litef ISA-100C 400Hz") },
  { 41, std::pair<std::vector<double>, std::string>({125,100,100}, "Epson G320N") },
  { 45, std::pair<std::vector<double>, std::string>({200,100,100}, "KVH 1725 IMU?") }, //(This was a guess based on the 1750
				 // as the actual rate is not documented and the specs are similar)
  { 52, std::pair<std::vector<double>, std::string>({200,100,100}, "Litef microIMU") },
  { 56, std::pair<std::vector<double>, std::string>({125,100,100}, "Sensonor STIM300, Direct Connection") },
  { 58, std::pair<std::vector<double>, std::string>({100,P2_33,P2_29}, "Honeywell HG4930 AN01") },
};

void decode_span(const std::string fname, int sensortype, double sampleRate, int isKMZ)
{
	std::string filename = fname.substr(0, fname.find_last_of('.'));
	if (publish_gpgga_)
	{
		output_gpgga.open(filename + ".gga");
	}

	if (publish_gps_bin)
	{
		output_gps.open(filename + "-gps.bin");
	}

	if (publish_novatel_positions_)
	{
		output_pos.open(filename + "-pos.csv");
	}

	if (publish_imu_messages_)
	{
		output_imu.open(filename + "-imu.csv");
	}

	if (publish_inspvax_)
	{
		output_ins.open(filename + "-ins.csv");
	}

	if (publish_kml_)
	{
		output_kml.open(filename + ".kml");
	}

	output_process.open(filename + "process.csv");
	//iput_file.open(fname.c_str());
	iput_file.open(fname.c_str(), std::ios::in | std::ios::binary);
	iput_file.seekg(0, std::ios::beg);
	if (!iput_file.is_open())
	{
		std::cout << "Cannt open iput data file" << std::endl;
	}
	novatel_gps_driver::NovatelGps gps_;

	std::vector<uint8> buffer_read_;

	iput_file.seekg(0, std::ios::end);

	int32 length = iput_file.tellg();

	iput_file.seekg(0, std::ios::beg);

	int readlength = 100;

	int32 tcoutwhile = (floor)(length / readlength);
	buffer_read_.reserve(readlength);


	while (!iput_file.eof())
	{
		//std::string line;
		//std::string line = "%RAWIMUSXA,1692,484620.664;00,11,1692,484620.664389000,00801503,43110635,-817242,-202184,-215194,-41188,-9895*a5db8c7b";
		//std::string line = "#RAWIMUA,COM1,0,68.5,FINESTEERING,1724,219418.009,024c0040,6125,30019;1724,219418.008755000,00000077,64732,56,298,8,28,-3*7378486f";
		//std::string line = "#RAWIMUXA,COM1,0,81.5,FINESTEERING,1691,410338.819,024c0020,3fd1,43495;00,5,1691,410338.818721000,00170705,-113836,-464281,43146813,89,11346,181*01cd06bf";
		//std::string line = "#BESTPOSA,COM1,0,74.0,FINESTEERING,1687,420221.000,00000000,6145,3153;SOL_COMPUTED,SINGLE,32.59536697319,-85.29568091247,199.8451,-29.2000,WGS84,1.2596,1.1674,2.6585,\"\",0.000,0.000,12,11,0,0,0,06,00,03*703fb123";
		//std::string line = "#BESTPOSA,COM1_30,0,65.0,FINESTEERING,1687,419234.000,00000000,6145,3153;SOL_COMPUTED,SINGLE,32.59535998647,-85.29567377157,200.9142,-29.2000,WGS84,1.3042,1.2961,3.1010,\"\",0.000,0.000,11,10,0,0,0,06,00,03*4d0c2769";
		//std::string line = "#CORRIMUDATAA,COM1,0,77.5,FINESTEERING,1769,237601.000,02000020,"
		//	"bdba,12597;1769,237601.000000000,-0.000003356,0.000002872,0.000001398,0.000151593,"
		//	"0.000038348,-0.000078820*e370e1d9";

		//std::string line = "#RANGECMPA,SPECIAL,0,29.5,FINESTEERING,2076,544.100,02004000,9691,15826;60,04bc10189ec9f5ff8c4f5c0a5073c0b1201f7e3ae0030000,0b3c3011d20af83f854f5c0a196e67b2221f7d3ae0020000,24bc10082d4208c098b5d50a26e201e2401a242f40030000,2b3c30016a6f0600b7b5d50a41e743f4521a242fc0020000,44bc101801cef5bf25a6940a398ebe8c30013e2ea0030000,4b3c30113d0ef86f40a6940aab22919532013c2e00030000,64bc1008b266f79f15bffc0ad5625dc8400ed3a240030000,6b3c3001b34cf94f18bffc0a85c748e0640e97a220020000,84bc10089e11f4dfd345960bbc3884e3500b0057e0020000,8b3c3001f6b3f62fd945960bae7eb391940bff5620020000,a4bc1018ca79f5af4e91f10b67048ca7f5208303c0010000,ab3c30119dccf75f6a91f10bf19af8e2f920100020010000,c4bc1008308b0ac0146dd80b4dcc0fb85009aa2ae0020000,cb3c3001453708502f6dd80ba507d7ef94095e2a20020000,e4bc1008f92101a0d51cd309f6d6df8b2016312ee0030000,eb3c3001f4e10070bf1cd3093d1da1f82116302e80030000,04bd1018d3fd0c6067f2720b069ab8fa8010f52de0020000,0b3d30118e1f0ac067f2720b4f5dc8a3c410b02ce0010000,44bd10182b4407305b59bf0a1d05b2f02017362ec0030000,4b3d30117ca905504f59bf0abff5b5ff33172e2ea0020000,c4bd10187f4c06306d47e309564e41812003352ee0030000,cb3d30117de804107e47e309a5c35af02103342e60030000,049f1108205e0010586a6c099df28eb8203392a8e0030000,0b9f3100364900306e6a6c09747cc49d113390a860030000,249f1108a2b50e60df2c110a5f608bc62034808bc01f0000,2b9f3110d3700b00f82c110a160d17c52234a356e01e0000,449f11081d0b06f04c7d5c0bc9ab82e6743ae428402e0000,4b9f31104fb304d05b7d5c0b2d6bd796753afa11c02d0000,a49f1118e070f24fc707b60a04c6b6d96432112b60160000,ab9f31003c74f58fe207b60a69b071f07432102b00160000,049c111856b609a004a32f0a100096b1212a2a2ea0230000,0b9c3100d18d070017a32f0af8f5c9b4222a292ec0220000,249c11188215f98f24edbb0a5f4e33d34339352ea0260000,2b9c3100f39efa7f55edbb0ae5cb60eb4239312ec0260000,849c11182d93f6ff9235af0ab2b6c6d932288b3100330000,8b9c31105cabf8cfa535af0a551d7ef043288a3160320000,c49c53084e7af98f922d690cd42bfad83007a6a2a0030000,c43c3302a900fbdfa52d690c6f48dac03007a6a220030000,e49c530823d7f6bf0004870c8f9f60c53021202ea0030000,e43c330258fbf86f0c04870c0fa8d5b120211f2e60030000,049d5308f4c504805862b10b231cb5d1201b6da7e0030000,043d330268a803f06662b10b3b345c9d201b6aa8c0030000,249d5308921406408a87fd0b0674b09f4013042e20030000,243d3302b6a804108e87fd0b6edd08f73013002e20030000,449d5308b283fa1f3035fb0bf8e036a1301eed2a40030000,443d33021eccfb3f3b35fb0bf2ff33f8201e883a20030000,649d5308962dffefeebabd0bee0599c9410c062ee0020000,643d3302ca5eff4fe5babd0b0f7a2597400c052ea0020000,a49d530800de0b50c099a90cedc1a8ae4015573760030000,a43d3302c81709d0d299a90ce7586da02015553740030000,a49e0418f41b07c05086ce0a04543ff720152f2e80030000,449f0408454ffd5f8f03840a5343bfa7201689a8e0030000,649f04480c260830782dda0925764b96202d55a7a0030000,e49f04489cfbfb6fa098ec0aa972ace3202463a7a0030000,249c1418831f04a0adbe7013561299d8e208de28a0010000,249c3410063003c099be70135cb916b78108072b20020000,649c0418185ff62f148f150ce9d660a26013dd2960020000,849c1418b777fc5f9ba98a0c22a727d6910be02a00020000,849c3410bd44fd9fa1a98a0cfbccffaf400bde2aa0020000,e49c04482004f9dfdfc8900c0d6c2bd24022302ea0020000*466dc122";

		//std::vector<uint8> buffer_read_;

		char buffer;
		iput_file.get(buffer);
		buffer_read_.push_back(buffer);
		if (tcoutwhile == 0)
		{
			int readbuffercount = iput_file.tellg();
			if (readbuffercount < readlength)
			{
				continue;
			}
			else if (readbuffercount == readlength)
			{
				gps_.set_data_buffer_(buffer_read_);
				buffer_read_.clear();
			}
		}
		else if (tcoutwhile > 0)
		{
			if (buffer_read_.size() < readlength)
			{
				continue;
			}
			else if (buffer_read_.size() == readlength)
			{
				gps_.set_data_buffer_(buffer_read_);
				buffer_read_.clear();
				std::cout << "Date process :" << (int)(1000 - tcoutwhile * readlength * 1000.0 / length)/10.0 <<  "% " <<"\r";
				tcoutwhile--;
			}
		}
		//gps_.set_data_buffer_(buffer);

		
		//if (DataInputType_ == ASCII)
		//{
		//	std::getline(iput_file, line);

		//}
		//else if (DataInputType_ == BINARY)
		//{
		//	iput_file.read(c,100);

		//}
		//gps_.set_data_buffer_(line);

		novatel_gps_driver::NovatelGps::ReadResult result = gps_.ProcessData();

		if (publish_inspvax_)
		{
			std::vector<novatel_gps_msgs::InspvaxPtr> inspvax_msgs;
			gps_.GetInspvaxMessages(inspvax_msgs);
			for (const auto& msg : inspvax_msgs)
			{
				output_ins << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
					<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
					<< std::setw(14) << std::setprecision(9) << msg->latitude << ","
					<< std::setw(14) << std::setprecision(9) << msg->longitude << ","
					<< std::setw(10) << std::setprecision(4) << msg->altitude + msg->undulation << ","
					<< std::setw(10) << std::setprecision(4) << msg->north_velocity << ","
					<< std::setw(10) << std::setprecision(4) << msg->east_velocity << ","
					<< std::setw(10) << std::setprecision(4) << -msg->up_velocity << ","
					<< std::setw(14) << std::setprecision(9) << msg->roll << ","
					<< std::setw(14) << std::setprecision(9) << msg->pitch << ","
					<< std::setw(14) << std::setprecision(9) << msg->azimuth << ","
					<< std::setw(10) << std::setprecision(4) << msg->latitude_std << ","
					<< std::setw(10) << std::setprecision(4) << msg->longitude_std << ","
					<< std::setw(10) << std::setprecision(4) << msg->altitude_std << ","
					<< std::setw(10) << std::setprecision(4) << msg->north_velocity_std << ","
					<< std::setw(10) << std::setprecision(4) << msg->east_velocity_std << ","
					<< std::setw(10) << std::setprecision(4) << msg->up_velocity_std << ","
					<< std::setw(10) << std::setprecision(4) << msg->roll_std << ","
					<< std::setw(10) << std::setprecision(4) << msg->pitch_std << ","
					<< std::setw(10) << std::setprecision(4) << msg->altitude_std << ","
					<< std::setw(10) << std::setprecision(4) << msg->longitude << ","
					<< std::endl;				      
			}
		} //GNSS/INS data

		if (publish_imu_messages_ & (!is_acena))
		{
			std::vector<novatel_gps_msgs::RawimusxPtr> rawimusx_msgs;
			gps_.GetRawimusxMessages(rawimusx_msgs);
			double fxyz_scale;
			double wxyz_scale;
			double sample_rate;
			for (const auto& msg : rawimusx_msgs)
			{
				if (rates.find(msg->imutype) != rates.end())
				{
					sample_rate = rates[msg->imutype].first[0];
					fxyz_scale = rates[msg->imutype].first[2];
					wxyz_scale = rates[msg->imutype].first[1];
				}
				output_imu << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
					<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
					<< std::setw(14) << std::setprecision(10) << msg->x_accel * fxyz_scale * sample_rate << ","
					<< std::setw(14) << std::setprecision(10) << -msg->y_accel * fxyz_scale * sample_rate << ","
					<< std::setw(10) << std::setprecision(10) << msg->z_accel * fxyz_scale * sample_rate << ","
					<< std::setw(10) << std::setprecision(10) << msg->x_gyro * wxyz_scale * sample_rate *  PI/180<< ","
					<< std::setw(10) << std::setprecision(10) << -msg->y_gyro * wxyz_scale * sample_rate *  PI / 180 << ","
					<< std::setw(10) << std::setprecision(10) << msg->z_gyro * wxyz_scale * sample_rate *  PI / 180
					<< std::endl;

			}
		}
				if (publish_imu_messages_ & (!is_acena))
		{
			std::vector<novatel_gps_msgs::RawimusxPtr> rawimusx_msgs;
			gps_.GetRawimusxMessages(rawimusx_msgs);
			double fxyz_scale;
			double wxyz_scale;
			double sample_rate;
			for (const auto& msg : rawimusx_msgs)
			{
				if (rates.find(msg->imutype) != rates.end())
				{
					sample_rate = rates[msg->imutype].second[0];
					fxyz_scale = rates[msg->imutype].second[1];
					wxyz_scale = rates[msg->imutype].second[2];					
				}
				output_imu << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
					<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
					<< std::setw(14) << std::setprecision(10) << msg->x_accel * fxyz_scale * sample_rate << ","
					<< std::setw(14) << std::setprecision(10) << msg->y_accel * fxyz_scale * sample_rate << ","
					<< std::setw(10) << std::setprecision(10) << msg->z_accel * fxyz_scale * sample_rate << ","
					<< std::setw(10) << std::setprecision(10) << msg->x_gyro * wxyz_scale * sample_rate *  PI/180<< ","
					<< std::setw(10) << std::setprecision(10) << msg->y_gyro * wxyz_scale * sample_rate *  PI / 180 << ","
					<< std::setw(10) << std::setprecision(10) << -msg->z_gyro * wxyz_scale * sample_rate *  PI / 180
					<< std::endl;

			}
		}

		if (publish_imu_messages_ & is_acena)
		{
			std::vector<novatel_gps_msgs::RawimuPtr> rawimu_msgs;
			gps_.GetRawimuMessages(rawimu_msgs);
			double fxyz_scale;
			double wxyz_scale;
			double sample_rate;
			for (const auto& msg : rawimu_msgs)
			{

				output_imu << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
					<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
					<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->x_accel))) * grav_WGS84 << ","
					<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->y_accel))) * grav_WGS84 << ","
					<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->z_accel))) * grav_WGS84 << ","
					<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->x_gyro))) << ","
					<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->y_gyro))) << ","
					<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->z_gyro)))
					<< std::endl;
				output_process << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
					<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
					<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->x_accel))) * grav_WGS84 << ","
					<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->y_accel))) * grav_WGS84 << ","
					<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->z_accel))) * grav_WGS84 << ","
					<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->x_gyro))) << ","
					<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->y_gyro))) << ","
					<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->z_gyro)))
					<< std::endl;

			}
		}


		if (publish_novatel_positions_)   //GNSS  position data
		{
			std::vector<novatel_gps_msgs::BestPosPtr> bestpos_msgs;
			gps_.GetNovatelPositions(bestpos_msgs);

			for (const auto& msg : bestpos_msgs)
			{
				output_pos << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
					<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
					<< std::setw(14) << std::setprecision(9) << msg->lat << ","
					<< std::setw(14) << std::setprecision(9) << msg->lon << ","
					<< std::setw(10) << std::setprecision(4) << msg->height << ","
					<< std::setw(10) << std::setprecision(4) << msg->lat_sigma << ","
					<< std::setw(10) << std::setprecision(4) << msg->lon_sigma << ","
					<< std::setw(10) << std::setprecision(4) << msg->height_sigma
					<< std::endl;
				output_process << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
					<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
					<< std::setw(14) << std::setprecision(9) << msg->lat << ","
					<< std::setw(14) << std::setprecision(9) << msg->lon << ","
					<< std::setw(10) << std::setprecision(4) << msg->height << ","
					<< std::setw(10) << std::setprecision(4) << msg->lat_sigma << ","
					<< std::setw(10) << std::setprecision(4) << msg->lon_sigma << ","
					<< std::setw(10) << std::setprecision(4) << msg->height_sigma
					<< std::endl;
			}


		}

		if (publish_gps_bin)
		{
			std::vector<novatel_gps_msgs::RangecmpPtr> rangecmp_msgs;
			gps_.GetRangecmpMessages(rangecmp_msgs);
			for (const auto& msg : rangecmp_msgs)
			{
				output_gps << msg->numb_of_observ << std::endl;
			}
		}





	}

	{
		 iput_file.close();
		 output_gpgga.close();
		 output_gps.close();
		 output_imu.close();
		 output_ins.close();
		 output_kml.close();
		 output_pos.close();
		 output_process.close();
	}


	



	

}


int main(int argc, char **argv)
{
	std::string inutfilename = "imugpslog5"; //ParsingData.ASC imugpslog ParsingData.GPSnovatel_CPT7-2019_10_19_17_09_13.ASC
	double sampleRate = 1.0;  
	publish_kml_ = TRUE;
	int sensortype = 0;  
	std::string imu_type = "Unknown";

	is_acena = true;

	//std::map<uint8, std::pair<std::vector<double>, std::string>>::iterator iter = rates.begin();
	//for (;iter != rates.end(); iter++)
	//{
	//	if (iter->second.second == imu_type)
	//	{
	//		sensortype = iter->first;
	//		sampleRate = iter->second.first[0];
	//		break;
	//	}
	//}
	//if (iter == rates.end())
	//{
	//	std::cout << "error imu type" << std::endl;
	//}

	decode_span(inutfilename, sensortype, sampleRate, publish_kml_);
	

}

