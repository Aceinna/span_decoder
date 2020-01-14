#include <swri_util/string_util.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <vector>

#include "include/novatel_gps.h"
#include "include/file.h"


int8_t is_equal_ignore_case(const char* s1, const char* s2)
{
	assert(s1 != NULL && s2 != NULL);

	do {
		char c1 = *s1, c2 = *s2;
		if (c1 >= 'A' && c1 <= 'Z') {
			c1 -= ('A' - 'a');
		}
		if (c2 >= 'A' && c2 <= 'Z') {
			c2 -= ('A' - 'a');
		}
		if (c1 != c2) {
			return 0;
		}
		s1++, s2++;
	} while (*s1 != 0 || *s2 != 0);

	return 1;
}

void decode_span(const std::string fname)
{
	std::string filename = fname.substr(0, fname.find_last_of('.'));

	file_open(filename);

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
	int readlength = 1024*200;
	int32 tcoutwhile = (floor)(length / readlength);
	buffer_read_.reserve(readlength);
	while (!iput_file.eof())
	{
		char buffer;
		iput_file.get(buffer);
		buffer_read_.push_back(buffer);
		if (tcoutwhile == 0)
		{
			int a = iput_file.tellg();
			int readbuffercount = length - iput_file.tellg();
			if (readbuffercount != 0 )
			{
				continue;
			}
			else if (readbuffercount == 0)
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

		novatel_gps_driver::NovatelGps::ReadResult result = gps_.ProcessData();
/*
		if (publish_aceinna_imu_)
		{
			std::vector<novatel_gps_msgs::RawimuPtr> rawimu_msgs;
			gps_.GetRawimuMessages(rawimu_msgs);
			for (const auto& msg : rawimu_msgs)
			{
				traceimu(msg);
			}
		}
		if (publish_imu_messages_)
		{
			std::vector<novatel_gps_msgs::RawimusxPtr> rawimusx_msgs;
			gps_.GetRawimusxMessages(rawimusx_msgs);

			for (const auto& msg : rawimusx_msgs)
			{

				tracenovatelimu(msg);
			}
			
		}
		if (publish_gnss_positions_)
		{
			std::vector<novatel_gps_msgs::BestPosPtr> bestpos_msgs;
			gps_.GetNovatelPositions(bestpos_msgs);
			for (const auto& msg : bestpos_msgs)
			{
				tracegnss(msg);
			}
		}
		if (pubilsh_gnss_vel_)
		{
			std::vector<novatel_gps_msgs::VelocityPtr> vel_msgs;
			gps_.GetNovatelVelocities(vel_msgs);
			for (const auto& msg : vel_msgs)
			{
				tracegnssvel(msg);
			}
		}
		if (publish_ins_)
		{
			std::vector<novatel_gps_msgs::InspvaPtr> inspva_msgs;
			gps_.GetInspvaMessages(inspva_msgs);
			for (const auto& msg : inspva_msgs)
			{
				traceins(msg);
			}
		} 
		if (publish_inspvax_)
		{
			std::vector<novatel_gps_msgs::InspvaxPtr> inspvax_msgs;
			gps_.GetInspvaxMessages(inspvax_msgs);
			for (const auto& msg : inspvax_msgs)
			{
				traceinspvax(msg);
			}

		}
		*/
	}
	int tcolor = 2;
	int pcolor = 5;
	int outalt = 0;
	int outtime = 2;
	if (publish_inspvax_)
	{
		saveinspvaxkml(inspvax_msgs_, tcolor, pcolor, outalt, outtime);
		savegnssposkml(gnss_msgs_, tcolor, pcolor, outalt, outtime);
	}
	else
	{
		savegnsskml(gnss_msgs_, gnssvel_msgs_, tcolor, pcolor, outalt, outtime);
		saveinskml(ins_msgs_, tcolor, pcolor, outalt, outtime);
	}
	file_close();
}

int main(int argc, char **argv)
{
	//std::string inutfilename = "novatel_FLX6"; 
	std::string inutfilename = argv[1];
	
	if (is_equal_ignore_case(argv[2], "0"))
	{
		//transfer Aceinna binnary data
		FILE_TYPE = ACEINNA;
	}
	else if (is_equal_ignore_case(argv[2], "1"))
	{
		FILE_TYPE = NOVATEL;
		//transfer novatel binnary data
	}
	if (FILE_TYPE == ACEINNA)
	{
		publish_gnss_positions_ = true;
		pubilsh_gnss_vel_ = true;
		publish_aceinna_imu_ = true;
		publish_ins_ = true;
		publish_process_ = true;
		publish_kml_ = true;

		publish_inspvax_ = false;
		publish_imu_messages_ = false;
		publish_gps_bin = false;
	}
	else if (FILE_TYPE = NOVATEL)
	{
		publish_gnss_positions_ = true;
		pubilsh_gnss_vel_ = false;
		publish_aceinna_imu_ = false;
		publish_ins_ = false;
		publish_process_ = true;
		publish_kml_ = true;

		publish_inspvax_ = true;
		publish_imu_messages_ = true;
		publish_gps_bin = false;
	}

	decode_span(inutfilename);
	
}

