
// *****************************************************************************
//
// Copyright (c) 2017, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"


#include "include/file.h"

#include <iostream>
#include <map>
#define	grav_WGS84 9.7803267714e0
#ifndef PI
#define	PI 3.14159265358979
#endif

#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_33_DEG   6.670106611340576E-09 /* 2^-33 */


#define P2_27_F     2.270936965942383E-09 /* 2^-27 FEET */
#define P2_29_F     5.677342414855957E-10 /* 2^-29 FEET */

#define P2_29       1.862645149230957E-09 /* 2^-29 */

#define FSAS_GYRO   1.085069444444445E-07  //0.1x 2-8 arcsec/LSB
#define FSAS_ACC    1.525878906250000E-06  //0.05 x 2-15 m/s/LSB

#define CPT_GYRO   6.670106611340576E-09  /* 2^-33 * 180 /PI */


static std::map<uint8, std::pair<std::vector<double>, std::string>> rates = {
  { 0,  std::pair<std::vector<double>, std::string>({100,2.0,100}, "Unknown") },
  { 1,  std::pair<std::vector<double>, std::string>({100,100,100}, "Honeywell HG1700 AG11") },
  { 4,  std::pair<std::vector<double>, std::string>({100,100,100}, "Honeywell HG1700 AG17") },
  { 5,  std::pair<std::vector<double>, std::string>({100,100,100}, "Honeywell HG1700 CA29") },
  { 8,  std::pair<std::vector<double>, std::string>({200,100,100}, "Litton LN-200 (200hz model)") },
  { 11, std::pair<std::vector<double>, std::string>({100,P2_33,P2_27_F}, "Honeywell HG1700 AG58") },
  { 12, std::pair<std::vector<double>, std::string>({100,100,100}, "Honeywell HG1700 AG62") },
  { 13, std::pair<std::vector<double>, std::string>({200,FSAS_GYRO,FSAS_ACC}, "iMAR ilMU-FSAS") },
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
  { 58, std::pair<std::vector<double>, std::string>({100,P2_33_DEG,P2_29}, "Honeywell HG4930 AN01") },
};

const std::string SOLUTION_STATUSES[] = {
	"SOL_COMPUTED", "INSUFFICIENT_OBS", "NO_CONVERGENCE", "SINGULARITY", "COV_TRACE",
	"TEST_DIST", "COLD_START", "V_H_LIMIT", "VARIANCE", "RESIDUALS",
	"RESERVED", "RESERVED", "RESERVED", "INTEGRITY_WARNING", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED", "PENDING", "INVALID_FIX",
	"UNAUTHORIZED", "RESERVED", "INVALID_RATE" };

const std::string POSITION_TYPES[] = {
	"NONE", "FIXEDPOS", "FIXEDHEIGHT", "RESERVED", "FLOATCONV",
	"WIDELANE", "NARROWLANE", "RESERVED", "DOPPLER_VELOCITY", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED",
	"RESERVED", "SINGLE", "PSRDIFF", "WAAS", "PROPOGATED",
	"RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED",
	"RESERVED", "RESERVED", "L1_FLOAT", "IONOFREE_FLOAT", "NARROW_FLOAT",
	"RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED", "L1_INT", "WIDE_INT",
	"NARROW_INT", "RTK_DIRECT_INS", "INS_SBAS", "INS_PSRSP", "INS_PSRDIFF",
	"INS_RTKFLOAT", "INS_RTKFIXED", "RESERVED", "RESERVED", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED", "PPP_CONVERGING", "PPP",
	"OPERATIONAL", "WARNING", "OUT_OF_BOUNDS", "INS_PPP_CONVERGING", "INS_PPP",
	"UNKNOWN", "UNKNOWN", "PPP_BASIC_CONVERGING", "PPP_BASIC", "INS_PPP_BASIC",
	"INS_PPP_BASIC_CONVERGING" };

const std::string INERTIAL_STATUSES[] = {
	"INS_INACTIVE","INS_ALIGNING","INS_HIGH_VARIANCE","INS_SOLUTION_GOOD",
	 " " , " " ,"INS_SOLUTION_FREE","INS_ALIGNMENT_COMPLETE","DETERMINING_ORIENTATION",
	 "WAITING_INITIALPOS","WAITING_AZIMUTH","INITIALIZING_BIASES","MOTION_DETECT"
};

int FILE_TYPE = 0;


bool publish_gnss_positions_ = true;
bool pubilsh_gnss_vel_ = true;
bool publish_aceinna_imu_ = true;
bool publish_ins_ = true;
bool publish_process_ = true;
bool publish_kml_ = true;

bool publish_inspvax_ = false;
bool publish_imu_messages_ = false;
bool publish_gps_bin = false;


std::ifstream iput_file;
std::ofstream output_gnss;
std::ofstream output_gnss_vel;
std::ofstream output_imu;
std::ofstream output_ins;
std::ofstream output_process;
std::ofstream output_gnss_kml;
std::ofstream output_ins_kml;

std::vector <novatel_gps_msgs::BestPos> gnss_msgs_;
std::vector <novatel_gps_msgs::Velocity> gnssvel_msgs_;
std::vector <novatel_gps_msgs::Inspva> ins_msgs_;
std::vector <novatel_gps_msgs::Inspvax> inspvax_msgs_;


#define HEADKML1 "<?xml version=\"1.0\" encoding=\"UTF-8\"?>"
#define HEADKML2 "<kml xmlns=\"http://www.opengis.net/kml/2.2\">"
#define MARKICON "http://maps.google.com/mapfiles/kml/shapes/track.png"
//#define MARKICON1 "http://maps.google.com/mapfiles/kml/pal2/icon18.png"


#define R2D   (180/3.1415926)
#define SIZP     0.3            /* mark size of rover positions */
#define SIZR     0.3            /* mark size of reference position */
#define TINT     30.0           /* time label interval (sec) */

inline void  DegtoDMS(const double mdeg, int &Deg, int &Min, double &Sec)
{
	double AM;
	Deg = (int)mdeg;
	AM = (mdeg - Deg) * 60.0;
	Min = (int)AM;
	Sec = (AM - Min) * 60.0;
}

bool file_open(const std::string input_fname)
{
	bool ret = false;
	std::string fname = input_fname.substr(0, input_fname.find_last_of('.'));
	if (publish_gnss_positions_) output_gnss.open(fname + "-gnss.csv");
	if (pubilsh_gnss_vel_) output_gnss_vel.open(fname + "-gnssvel.csv");

	if (publish_aceinna_imu_)   output_imu.open(fname + "-imu.csv");
	if (publish_imu_messages_)   output_imu.open(fname + "-imu.csv");

	if (publish_ins_)  output_ins.open(fname + "-ins.csv");
	if (publish_inspvax_)  output_ins.open(fname + "-ins.csv");

	if (publish_process_) output_process.open(fname + "-process");

	if (publish_kml_)
	{
		output_gnss_kml.open(fname + "-gnss.kml");
		output_ins_kml.open(fname + "-ins.kml");
	}
	ret = true;
	return ret;
}

static double lastlctime = 0;
static double lastgnsstime = -1;
//static int32_t firstgnssweek = -1;
bool traceimu(novatel_gps_msgs::RawimuPtr msg)
{
	bool ret = false;
	//if (firstgnssweek == -1)
	//{
	//	firstgnssweek = msg->novatel_msg_header.gps_week_num;
	//}
	//else
	//{
	//	msg->novatel_msg_header.gps_seconds += 604800 * (msg->gps_week_num - firstgnssweek);
	//	msg->gps_seconds += 604800 * (msg->gps_week_num - firstgnssweek);
	//	msg->gps_week_num = firstgnssweek;
	//}
	if (publish_aceinna_imu_)
	{
		output_imu << std::setw(4) << msg->gps_week_num << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->gps_seconds / 1000 << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
			<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->x_accel))) * grav_WGS84 << ","
			<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->y_accel))) * grav_WGS84 << ","
			<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->z_accel))) * grav_WGS84 << ","
			<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->x_gyro))) << ","
			<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->y_gyro))) << ","
			<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->z_gyro)))
			<< std::endl;
	}
	if (publish_process_)
	{
		output_process << "$GPIMU,"
			<< std::setw(4) << msg->gps_week_num << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->gps_seconds / 1000 << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
			<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->x_accel))) * grav_WGS84 << ","
			<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->y_accel))) * grav_WGS84 << ","
			<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->z_accel))) * grav_WGS84 << ","
			<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->x_gyro))) << ","
			<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->y_gyro))) << ","
			<< std::setw(14) << std::setprecision(10) << *(reinterpret_cast<float *>(&(msg->z_gyro)))
			<< std::endl;
	}
	ret = true;
	return ret;
}


bool tracenovatelimu(novatel_gps_msgs::RawimusxPtr msg)
{
	bool ret = false;
	double fxyz_scale;
	double wxyz_scale;
	double sample_rate;
	if (rates.find(msg->imutype) != rates.end())
	{
		sample_rate = rates[msg->imutype].first[0];
		wxyz_scale = rates[msg->imutype].first[1];
		fxyz_scale = rates[msg->imutype].first[2];
	}
	double x_accel = msg->x_accel * fxyz_scale * sample_rate;
	double y_accel = -msg->y_accel *fxyz_scale * sample_rate;
	double z_accel = msg->z_accel * fxyz_scale * sample_rate;
	double x_gyro = msg->x_gyro * wxyz_scale * sample_rate;
	double y_gyro = -msg->y_gyro * wxyz_scale * sample_rate;
	double z_gyro = msg->z_gyro * wxyz_scale * sample_rate;
	//if (firstgnssweek == -1)
	//{
	//	firstgnssweek = msg->novatel_msg_header.gps_week_num;
	//}
	//else
	//{
	//	msg->novatel_msg_header.gps_seconds += 604800*(msg->novatel_msg_header.gps_week_num - firstgnssweek);
	//	msg->novatel_msg_header.gps_week_num = firstgnssweek;
	//}
	double lctime = msg->novatel_msg_header.gps_seconds;
	if (fmod(lctime + 0.02, 1) < 0.01 & lctime - lastlctime > 0.98)
	{
		msg->novatel_msg_header.gps_seconds = floor(lctime + 0.02);
		lastlctime = lctime;
	}
	else
	{
		lctime = 0;
	}



	//if (fmod(msg->novatel_msg_header.gps_seconds + 0.001, 1) < 0.3 && 
	//	fmod(msg->novatel_msg_header.gps_seconds + 0.001, 1) > 0.1 &&
	//	msg->novatel_msg_header.gps_seconds > lastgnsstime + 1.0
	//	&&lastgnsstime != -1)
	//{
	//	novatel_gps_msgs::BestPosPtr gnssmsg;
	//	gnssmsg->novatel_msg_header.gps_week_num = 1;// msg->gps_week_num;
	//	gnssmsg->novatel_msg_header.gps_seconds = 2.0;// floor(lastlctime + 0.02);
	//	gnssmsg->position_type_int = 0;
	//	gnssmsg->lon = 0;
	//	gnssmsg->height = 0;
	//	gnssmsg->lon_sigma = 0;
	//	gnssmsg->lat_sigma = 0;
	//	gnssmsg->lon_sigma = 0;
	//	gnssmsg->lon_sigma = 0;
	//	lastgnsstime = gnssmsg->novatel_msg_header.gps_seconds;
 //    	tracegnss(gnssmsg);
	//}

	if (publish_imu_messages_)
	{
		output_imu << std::setw(4) << msg->gps_week_num << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->gps_seconds << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << floor(lctime+0.02) << ","
			<< std::setw(14) << std::setprecision(10) << x_accel << ","
			<< std::setw(14) << std::setprecision(10) << y_accel << ","
			<< std::setw(14) << std::setprecision(10) << z_accel << ","
			<< std::setw(14) << std::setprecision(10) << x_gyro << ","
			<< std::setw(14) << std::setprecision(10) << y_gyro << ","
			<< std::setw(14) << std::setprecision(10) << z_gyro
			<< std::endl;
	}
	if (publish_process_)
	{
		output_process << "$GPIMU,"
			<< std::setw(4) << msg->gps_week_num << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->gps_seconds << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << floor(lctime + 0.02) << ","
			<< std::setw(14) << std::setprecision(10) << x_accel << ","
			<< std::setw(14) << std::setprecision(10) << y_accel << ","
			<< std::setw(14) << std::setprecision(10) << z_accel << ","
			<< std::setw(14) << std::setprecision(10) << x_gyro << ","
			<< std::setw(14) << std::setprecision(10) << y_gyro << ","
			<< std::setw(14) << std::setprecision(10) << z_gyro
			<< std::endl;
	}
	ret = true;
	return ret;
}

int32_t getpostype(int position_type, int file_type)
{
	int32_t ret = 0;
	if (file_type == ACEINNA)
	{
		switch (position_type)
		{
		case 4:
			//rtk;
			ret = 4;
			break;
		case 5:
			//rtd;
			ret = 5;
			break;;
		default:
			break;
		}
	}
	else if (file_type == NOVATEL)
	{
		switch (position_type)
		{
		case 16:
			//spp;
			ret = 1;
			break;
		case 17:
			//rtd;
			ret = 2;
			break;
			//case 3:
			//	//udr;
			//	pcolor = 4;
			//	break;
		case 50:
			//fix;
			ret = 4;
			break;
		case 34:
			//float;
			ret = 5;
			break;
		default:
			break;
		}

	}

	return ret;
}

bool tracegnss(novatel_gps_msgs::BestPosPtr msg)
{
	bool ret = false;
	if (publish_kml_)
	{
		if (fabs(msg->lat) > 0.001)
		{
			gnss_msgs_.push_back(*msg);
		}
	}

	int type = getpostype(msg->position_type_int,FILE_TYPE);

    if (msg->solution_status_int != 0)
	{
		type = 0;
	}

	//if (firstgnssweek == -1)
	//{
	//	firstgnssweek = msg->novatel_msg_header.gps_week_num;
	//}
	//else
	//{
	//	msg->novatel_msg_header.gps_seconds += 604800 * (msg->novatel_msg_header.gps_week_num - firstgnssweek);
	//	msg->novatel_msg_header.gps_week_num = firstgnssweek;
	//}
	if (publish_gnss_positions_)
	{
		if (FILE_TYPE != NOVATEL ||fmod(msg->novatel_msg_header.gps_seconds + 0.001, 1) < 0.01)
		{
			output_gnss << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
				<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
				<< std::setw(14) << std::setprecision(9) << msg->lat << ","
				<< std::setw(14) << std::setprecision(9) << msg->lon << ","
				<< std::setw(10) << std::setprecision(4) << msg->height + msg->undulation << ","
				<< std::setw(10) << std::setprecision(4) << msg->lat_sigma << ","
				<< std::setw(10) << std::setprecision(4) << msg->lon_sigma << ","
				<< std::setw(10) << std::setprecision(4) << msg->height_sigma << ","
				<< type
				<< std::endl;
		}
	}
	if (publish_process_)
	{
		if (FILE_TYPE != NOVATEL || fmod(msg->novatel_msg_header.gps_seconds + 0.001, 1) < 0.01)
		{
			lastgnsstime = msg->novatel_msg_header.gps_seconds;
			output_process << "$GPGNSS," << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
				<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
				<< std::setw(14) << std::setprecision(9) << msg->lat << ","
				<< std::setw(14) << std::setprecision(9) << msg->lon << ","
				<< std::setw(10) << std::setprecision(4) << msg->height + msg->undulation << ","
				<< std::setw(10) << std::setprecision(4) << msg->lat_sigma << ","
				<< std::setw(10) << std::setprecision(4) << msg->lon_sigma << ","
				<< std::setw(10) << std::setprecision(4) << msg->height_sigma << ","
				<< type
				<< std::endl;
		}
	}
	
	ret = true;
	return ret;
}

bool tracegnssvel(novatel_gps_msgs::VelocityPtr msg)
{
	int ret = false;
	if (publish_kml_)
	{
		if (fabs(msg->horizontal_speed) > 0.0001 ||
			fabs(msg->vertical_speed) > 0.0001 || 
			fabs(msg->track_ground) > 0.0001  )
		{
			gnssvel_msgs_.push_back(*msg);
		}
	}
	//if (firstgnssweek == -1)
	//{
	//	firstgnssweek = msg->novatel_msg_header.gps_week_num;
	//}
	//else
	//{
	//	msg->novatel_msg_header.gps_seconds += 604800 * (msg->novatel_msg_header.gps_week_num - firstgnssweek);
	//	msg->novatel_msg_header.gps_week_num = firstgnssweek;
	//}
	if (pubilsh_gnss_vel_)
	{
		if (FILE_TYPE != NOVATEL || fmod(msg->novatel_msg_header.gps_seconds + 0.001, 1) < 0.01)
		{
			output_gnss_vel << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
				<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
				<< std::setw(14) << std::setprecision(9) << msg->horizontal_speed << ","
				<< std::setw(14) << std::setprecision(9) << msg->track_ground << ","
				<< std::setw(10) << std::setprecision(4) << msg->vertical_speed
				<< std::endl;
		}
	}
	if (publish_process_)
	{
		if (FILE_TYPE != NOVATEL || fmod(msg->novatel_msg_header.gps_seconds + 0.001, 1) < 0.01)
		{
			output_process << "$GPVEL,"
				<< std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
				<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
				<< std::setw(14) << std::setprecision(9) << msg->horizontal_speed << ","
				<< std::setw(14) << std::setprecision(9) << msg->track_ground << ","
				<< std::setw(10) << std::setprecision(4) << msg->vertical_speed
				<< std::endl;
		}
	}
	ret = true;
	return ret;
}

bool traceins(novatel_gps_msgs::InspvaPtr msg)
{
	if (publish_kml_)
	{
		if (fabs(msg->latitude) > 0.001)
		{
			ins_msgs_.push_back(*msg);
		}
	}
	//if (firstgnssweek == -1)
	//{
	//	firstgnssweek = msg->novatel_msg_header.gps_week_num;
	//}
	//else
	//{
	//	msg->novatel_msg_header.gps_seconds += 604800 * (msg->novatel_msg_header.gps_week_num - firstgnssweek);
	//	msg->novatel_msg_header.gps_week_num = firstgnssweek;
	//}
	int ret = false;
	if (publish_ins_ && fabs(msg->latitude) > 0.001)
	{
		output_ins << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
			<< std::setw(14) << std::setprecision(9) << msg->latitude << ","
			<< std::setw(14) << std::setprecision(9) << msg->longitude << ","
			<< std::setw(10) << std::setprecision(4) << msg->height << ","
			<< std::setw(10) << std::setprecision(4) << msg->north_velocity << ","
			<< std::setw(10) << std::setprecision(4) << msg->east_velocity << ","
			<< std::setw(10) << std::setprecision(4) << msg->up_velocity << ","
			<< std::setw(14) << std::setprecision(9) << msg->roll << ","
			<< std::setw(14) << std::setprecision(9) << msg->pitch << ","
			<< std::setw(14) << std::setprecision(9) << msg->azimuth << ","
			<< msg->status_int
			<< std::endl;
	}
	if (publish_process_)
	{
		output_process << "$GPINS," << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
			<< std::setw(14) << std::setprecision(9) << msg->latitude << ","
			<< std::setw(14) << std::setprecision(9) << msg->longitude << ","
			<< std::setw(10) << std::setprecision(4) << msg->height << ","
			<< std::setw(10) << std::setprecision(4) << msg->north_velocity << ","
			<< std::setw(10) << std::setprecision(4) << msg->east_velocity << ","
			<< std::setw(10) << std::setprecision(4) << msg->up_velocity << ","
			<< std::setw(14) << std::setprecision(9) << msg->roll << ","
			<< std::setw(14) << std::setprecision(9) << msg->pitch << ","
			<< std::setw(14) << std::setprecision(9) << msg->azimuth << ","
			<< msg->status_int
			<< std::endl;
	}

	ret = true;
	return ret;
}

bool traceinspvax(novatel_gps_msgs::InspvaxPtr msg)
{
	int ret = false;
	if (publish_kml_)
	{
		if (fabs(msg->latitude) > 0.001)
		{
			inspvax_msgs_.push_back(*msg);
		}
	}
	//if (firstgnssweek == -1)
	//{
	//	firstgnssweek = msg->novatel_msg_header.gps_week_num;
	//}
	//else
	//{
	//	msg->novatel_msg_header.gps_seconds += 604800 * (msg->novatel_msg_header.gps_week_num - firstgnssweek);
	//	msg->novatel_msg_header.gps_week_num = firstgnssweek;
	//}
	if (publish_inspvax_)
	{
		output_ins << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
			<< std::setw(14) << std::setprecision(9) << msg->latitude << ","
			<< std::setw(14) << std::setprecision(9) << msg->longitude << ","
			<< std::setw(10) << std::setprecision(4) << msg->altitude + msg->undulation << ","
			<< std::setw(10) << std::setprecision(4) << msg->north_velocity << ","
			<< std::setw(10) << std::setprecision(4) << msg->east_velocity << ","
			<< std::setw(10) << std::setprecision(4) << msg->up_velocity << ","
			<< std::setw(14) << std::setprecision(9) << msg->roll << ","
			<< std::setw(14) << std::setprecision(9) << msg->pitch << ","
			<< std::setw(14) << std::setprecision(9) << msg->azimuth << ","
			<< msg->ins_status_int
			<< std::endl;
	}
	if (publish_process_)
	{
		output_process << "$GPINS," << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
			<< std::setw(14) << std::setprecision(9) << msg->latitude << ","
			<< std::setw(14) << std::setprecision(9) << msg->longitude << ","
			<< std::setw(10) << std::setprecision(4) << msg->altitude + msg->undulation << ","
			<< std::setw(10) << std::setprecision(4) << msg->north_velocity << ","
			<< std::setw(10) << std::setprecision(4) << msg->east_velocity << ","
			<< std::setw(10) << std::setprecision(4) << msg->up_velocity << ","
			<< std::setw(14) << std::setprecision(9) << msg->roll << ","
			<< std::setw(14) << std::setprecision(9) << msg->pitch << ","
			<< std::setw(14) << std::setprecision(9) << msg->azimuth << ","
			<< msg->ins_status_int
			<< std::endl;
	}

	ret = true;
	return ret;
}


void outtrack(std::ofstream& kmloutput, const std::vector<novatel_gps_msgs::Inspva> &msg,
	const std::string color, int outalt, int outtime)
{
	int i;
	kmloutput << "<Placemark>" << std::endl
		<< "<name>Rover Track</name>" << std::endl
		<< "<Style>" << std::endl
		<< "<LineStyle>" << std::endl
		<< "<color>" << color << "</color>" << std::endl
		<< "</LineStyle>" << std::endl
		<< "</Style>" << std::endl
		<< "<LineString>" << std::endl;

	if (outalt)
		kmloutput << "<altitudeMode>absolute</altitudeMode>" << std::endl;

	kmloutput << "<coordinates>" << std::endl;

	for (i = 0; i < (int)msg.size(); i++)
	{
		if(fmod(msg[i].novatel_msg_header.gps_seconds + 0.0005, 0.2) < 0.005)
        {
kmloutput << std::setiosflags(std::ios::fixed) << std::setprecision(9) << msg[i].longitude << ","
<< std::setprecision(9) << msg[i].latitude << ","
<< std::setprecision(3) << msg[i].height << std::endl;
		}
	}
	kmloutput << "</coordinates>" << std::endl
		<< "</LineString>" << std::endl
		<< "</Placemark>" << std::endl;
}

void outtrackinspvax(std::ofstream& kmloutput, const std::vector<novatel_gps_msgs::Inspvax> &msg,
	const std::string color, int outalt, int outtime)
{
	int i;
	kmloutput << "<Placemark>" << std::endl
		<< "<name>Rover Track</name>" << std::endl
		<< "<Style>" << std::endl
		<< "<LineStyle>" << std::endl
		<< "<color>" << color << "</color>" << std::endl
		<< "</LineStyle>" << std::endl
		<< "</Style>" << std::endl
		<< "<LineString>" << std::endl;

	if (outalt)
		kmloutput << "<altitudeMode>absolute</altitudeMode>" << std::endl;

	kmloutput << "<coordinates>" << std::endl;
	for (i = 0; i < (int)msg.size(); i++)
	{
		kmloutput << std::setiosflags(std::ios::fixed) << std::setprecision(9) << msg[i].longitude << ","
			<< std::setprecision(9) << msg[i].latitude << ","
			<< std::setprecision(3) << msg[i].altitude + msg[i].undulation << std::endl;
	}
	kmloutput << "</coordinates>" << std::endl
		<< "</LineString>" << std::endl
		<< "</Placemark>" << std::endl;
}

void outtrackgnss(std::ofstream& kmloutput, const std::vector<novatel_gps_msgs::BestPos> &msg,
	const std::string color, int outalt, int outtime)
{
	int i;
	kmloutput << "<Placemark>" << std::endl
		<< "<name>Rover Track</name>" << std::endl
		<< "<Style>" << std::endl
		<< "<LineStyle>" << std::endl
		<< "<color>" << color << "</color>" << std::endl
		<< "</LineStyle>" << std::endl
		<< "</Style>" << std::endl
		<< "<LineString>" << std::endl;

	if (outalt)
		kmloutput << "<altitudeMode>absolute</altitudeMode>" << std::endl;

	kmloutput << "<coordinates>" << std::endl;
	for (i = 0; i < (int)msg.size(); i++)
	{
		kmloutput << std::setiosflags(std::ios::fixed) << std::setprecision(9) << msg[i].lon << ","
			<< std::setprecision(9) << msg[i].lat << ","
			<< std::setprecision(3) << msg[i].height + msg[i].undulation << std::endl;
	}
	kmloutput << "</coordinates>" << std::endl
		<< "</LineString>" << std::endl
		<< "</Placemark>" << std::endl;
}

/*output point*/
void outpoint(std::ofstream& kmloutput, const novatel_gps_msgs::Inspva & msg,
	const char *label, int style, int outalt, int outtime, int number)
{
	double ep[6], alt = 0.0;

	kmloutput << "<Placemark>" << std::endl;
	if (*label)
		kmloutput << "<name>" << label << "</name>" << std::endl;
	gtime_t gpstime = gpst2time(msg.novatel_msg_header.gps_week_num, msg.novatel_msg_header.gps_seconds);
	gtime_t utctime = gpst2utc(gpstime);
	time2epoch(utctime, ep);


	if (outtime)
	{
		if (outtime == 2) {}
		else if (outtime == 3) {}
		if (number == 1)
		{
			kmloutput << "<name>" << "Start" << "</name>" << std::endl;
		}
		else if (number == -1)
		{
			kmloutput << "<name>" << "End" << "</name>" << std::endl;

		}
		else if (fmod(ep[5] + 0.025, TINT) < 0.05)
		{
			kmloutput << "<name>"
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[3]
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[4]
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[5]
				<< "</name>" << std::endl;
		}
		kmloutput << "<TimeStamp><when>"
			<< std::setfill('0') << std::setw(4) << std::setprecision(0) << ep[0] << "-"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[1] << "-"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[2] << "T"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[3] << ":"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[4] << ":"
			<< std::setfill('0') << std::setw(5) << std::setprecision(2) << ep[5] << "Z"
			<< "</when></TimeStamp>" << std::endl;
	}

	kmloutput << "<description><![CDATA[" << std::endl
		<< "<TABLE border=\"1\" width=\"100%\" Align=\"center\">" << std::endl
		<< "<TR ALIGN=RIGHT>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>"
		<< msg.novatel_msg_header.gps_week_num << "</TD><TD>"
		<< std::setiosflags(std::ios::fixed) << std::setprecision(3) << msg.novatel_msg_header.gps_seconds << "</TD><TD>"
		<< std::setprecision(0) << std::setfill(' ') << std::setw(2) << ep[3] << ":" << std::setw(2) << ep[4] << ":" << std::setiosflags(std::ios::fixed) << std::setprecision(4) << std::setw(7) << ep[5] << "</TD><TD>"
		<< std::setprecision(0) << std::setw(4) << ep[0] << "/" << std::setw(2) << ep[1] << "/" << std::setw(2) << ep[2] << "</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>";
	kmloutput << std::setiosflags(std::ios::fixed) << std::setprecision(8) << msg.latitude << "</TD><TD>";
	kmloutput << msg.longitude << "</TD><TD>" << std::setprecision(4) << msg.height << "</TD><TD>(DMS,m)</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>"
		<< msg.north_velocity << "</TD><TD>" << msg.east_velocity << "</TD><TD>" << -msg.up_velocity << "</TD><TD>(m/s)</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>"
		<< msg.roll << "</TD><TD>" << msg.pitch << "</TD><TD>" << msg.azimuth << "</TD><TD>(deg,approx)</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>"
		<< 0 << "</TD><TD>"
		<< msg.status_int << "</TD><TR>" << std::endl
		<< "</TABLE>" << std::endl
		<< "]]></description>" << std::endl;

	kmloutput << "<styleUrl>#P" << style << "</styleUrl>" << std::endl;

	kmloutput << "<Style>\n" << "<IconStyle>\n" << "<heading>" << msg.azimuth << "</heading>\n" << "</IconStyle>\n" << "</Style>\n";

	kmloutput << "<Point>" << std::endl;
	if (outalt)
	{
		kmloutput << "<extrude>1</extrude>" << std::endl
			<< "<altitudeMode>absolute</altitudeMode>" << std::endl;
	}
	kmloutput << "<coordinates>"
		<< std::setw(13) << std::setprecision(9) << msg.longitude << ","
		<< std::setw(12) << std::setprecision(9) << msg.latitude << ","
		<< std::setw(5) << std::setprecision(3) << msg.height 
		<< "</coordinates>" << std::endl
		<< "</Point>" << std::endl
		<< "</Placemark>" << std::endl;
	/*
	double ep[6], alt = 0.0;
	kmloutput << "<Placemark>" << std::endl;
	if (*label)
		kmloutput << "<name>" << label << "</name>" << std::endl;
	gtime_t gpstime = gpst2time(msg.novatel_msg_header.gps_week_num, msg.novatel_msg_header.gps_seconds);
	gtime_t utctime = gpst2utc(gpstime);
	time2epoch(utctime, ep);


	if (outtime)
	{
		if (outtime == 2) {}
		else if (outtime == 3) {}
		if (number == 1)
		{
			kmloutput << "<name>" << "Start" << "</name>" << std::endl;
		}
		else if (number == -1)
		{
			kmloutput << "<name>" << "End" << "</name>" << std::endl;

		}
		else if (fmod(ep[5] + 0.025, TINT) < 0.05)
		{
			kmloutput << "<name>"
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[3]
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[4]
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[5]
				<< "</name>" << std::endl;
		}
		kmloutput << "<TimeStamp><when>"
			<< std::setfill('0') << std::setw(4) << std::setprecision(0) << ep[0] << "-"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[1] << "-"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[2] << "T"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[3] << ":"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[4] << ":"
			<< std::setfill('0') << std::setw(5) << std::setprecision(2) << ep[5] << "Z"
			<< "</when></TimeStamp>" << std::endl;
	}


	kmloutput << "<description><![CDATA[" << std::endl
		<< "<TABLE border=\"1\" width=\"100%\" Align=\"center\">" << std::endl
		<< "<TR ALIGN=RIGHT>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>" << std::setiosflags(std::ios::fixed) << std::setprecision(3) << msg.novatel_msg_header.gps_seconds << "</TD><TD>" << std::setprecision(0) << 0
		<< "</TD><TD>" << std::setfill(' ') << std::setw(2) << ep[3] << ":" << std::setw(2) << ep[4] << ":" << std::setiosflags(std::ios::fixed) << std::setprecision(4) << std::setw(7) << ep[5]
		<< "</TD><TD>" << std::setprecision(0) << std::setw(4) << ep[0] << "/" << std::setw(2) << ep[1] << "/" << std::setw(2) << ep[2] << "</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>";
	int deg, min;
	double sec;
	DegtoDMS(msg.latitude, deg, min, sec);
	kmloutput << std::setiosflags(std::ios::fixed) << std::setprecision(4) << deg << " " << min << " " << sec << "</TD><TD>";
	DegtoDMS(msg.longitude, deg, min, sec);
	kmloutput << deg << " " << min << " " << sec << "</TD><TD>" << msg.height << "</TD><TD>(DMS,m)</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(E,N,D):</TD><TD>"
		<< msg.north_velocity << "</TD><TD>" << msg.east_velocity << "</TD><TD>" << -msg.up_velocity << "</TD><TD>(m/s)</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>"
		<< msg.roll << "</TD><TD>" << msg.pitch << "</TD><TD>" << msg.azimuth << "</TD><TD>(deg,approx)</TD></TR>" << std::endl
		<< "</TABLE>" << std::endl
		<< "]]></description>" << std::endl;

	kmloutput << "<styleUrl>#P" << style << "</styleUrl>" << std::endl;

	kmloutput << "<Style>\n" << "<IconStyle>\n" << "<heading>" << msg.azimuth << "</heading>\n" << "</IconStyle>\n" << "</Style>\n";

	kmloutput << "<Point>" << std::endl;
	if (outalt)
	{
		kmloutput << "<extrude>1</extrude>" << std::endl
			<< "<altitudeMode>absolute</altitudeMode>" << std::endl;
	}
	kmloutput << "<coordinates>"
		<< std::setw(13) << std::setprecision(9) << msg.longitude << ","
		<< std::setw(12) << std::setprecision(9) << msg.latitude << ","
		<< std::setw(5) << std::setprecision(3) << msg.height
		<< "</coordinates>" << std::endl
		<< "</Point>" << std::endl
		<< "</Placemark>" << std::endl;
		*/
}

void outpointinspvax(std::ofstream& kmloutput, const novatel_gps_msgs::Inspvax & msg,
	const char *label, int style, int outalt, int outtime, int number)
{
	double ep[6], alt = 0.0;

	kmloutput << "<Placemark>" << std::endl;
	if (*label)
		kmloutput << "<name>" << label << "</name>" << std::endl;
	gtime_t gpstime = gpst2time(msg.novatel_msg_header.gps_week_num, msg.novatel_msg_header.gps_seconds);
	gtime_t utctime = gpst2utc(gpstime);
	time2epoch(utctime, ep);


	if (outtime)
	{
		if (outtime == 2) {}
		else if (outtime == 3) {}
		if (number == 1)
		{
			kmloutput << "<name>" << "Start" << "</name>" << std::endl;
		}
		else if (number == -1)
		{
			kmloutput << "<name>" << "End" << "</name>" << std::endl;

		}
		else if (fmod(ep[5] + 0.025, TINT) < 0.05)
		{
			kmloutput << "<name>"
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[3]
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[4]
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[5]
				<< "</name>" << std::endl;
		}
		kmloutput << "<TimeStamp><when>"
			<< std::setfill('0') << std::setw(4) << std::setprecision(0) << ep[0] << "-"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[1] << "-"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[2] << "T"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[3] << ":"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[4] << ":"
			<< std::setfill('0') << std::setw(5) << std::setprecision(2) << ep[5] << "Z"
			<< "</when></TimeStamp>" << std::endl;
	}

	kmloutput << "<description><![CDATA[" << std::endl
		<< "<TABLE border=\"1\" width=\"100%\" Align=\"center\">" << std::endl
		<< "<TR ALIGN=RIGHT>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>"
		<< msg.novatel_msg_header.gps_week_num << "</TD><TD>"
		<< std::setiosflags(std::ios::fixed) << std::setprecision(3) << msg.novatel_msg_header.gps_seconds << "</TD><TD>"
		<< std::setprecision(0) << std::setfill(' ') << std::setw(2) << ep[3] << ":" << std::setw(2) << ep[4] << ":" << std::setiosflags(std::ios::fixed) << std::setprecision(4) << std::setw(7) << ep[5] << "</TD><TD>"
		<< std::setprecision(0) << std::setw(4) << ep[0] << "/" << std::setw(2) << ep[1] << "/" << std::setw(2) << ep[2] << "</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>";
	kmloutput << std::setiosflags(std::ios::fixed) << std::setprecision(8) << msg.latitude << "</TD><TD>";
	kmloutput << msg.longitude << "</TD><TD>" << std::setprecision(4) << msg.altitude + msg.undulation << "</TD><TD>(DMS,m)</TD></TR>" << std::endl	
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>"
		<< msg.north_velocity << "</TD><TD>" << msg.east_velocity << "</TD><TD>" << -msg.up_velocity << "</TD><TD>(m/s)</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>"
		<< msg.roll << "</TD><TD>" << msg.pitch << "</TD><TD>" << msg.azimuth << "</TD><TD>(deg,approx)</TD></TR>" << std::endl
	    << "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>"
		<< INERTIAL_STATUSES[msg.ins_status_int] << "</TD><TD>"
		<< POSITION_TYPES[msg.position_type_int] << "</TD><TR>" << std::endl
		<< "</TABLE>" << std::endl
		<< "]]></description>" << std::endl;

	kmloutput << "<styleUrl>#P" << style << "</styleUrl>" << std::endl;

	kmloutput << "<Style>\n" << "<IconStyle>\n" << "<heading>" << msg.azimuth << "</heading>\n" << "</IconStyle>\n" << "</Style>\n";

	kmloutput << "<Point>" << std::endl;
	if (outalt)
	{
		kmloutput << "<extrude>1</extrude>" << std::endl
			<< "<altitudeMode>absolute</altitudeMode>" << std::endl;
	}
	kmloutput << "<coordinates>"
		<< std::setw(13) << std::setprecision(9) << msg.longitude << ","
		<< std::setw(12) << std::setprecision(9) << msg.latitude << ","
		<< std::setw(5) << std::setprecision(3) << msg.altitude + msg.undulation
		<< "</coordinates>" << std::endl
		<< "</Point>" << std::endl
		<< "</Placemark>" << std::endl;
}

void outpointgnss(std::ofstream& kmloutput, const novatel_gps_msgs::BestPos &msg_pos,
	const novatel_gps_msgs::Velocity &msg_vel,
	const char *label, int style, int outalt, int outtime, int number)
{

	double ep[6], alt = 0.0;
	kmloutput << "<Placemark>" << std::endl;
	if (*label)
		kmloutput << "<name>" << label << "</name>" << std::endl;
	gtime_t gpstime = gpst2time(msg_pos.novatel_msg_header.gps_week_num, msg_pos.novatel_msg_header.gps_seconds);
	gtime_t utctime = gpst2utc(gpstime);
	time2epoch(utctime, ep);


	if (outtime)
	{
		if (outtime == 2) {}
		else if (outtime == 3) {}
		if (number == 1)
		{
			kmloutput << "<name>" << "Start" << "</name>" << std::endl;
		}
		else if (number == -1)
		{
			kmloutput << "<name>" << "End" << "</name>" << std::endl;

		}
		else if (fmod(ep[5] + 0.025, TINT) < 0.05)
		{
			kmloutput << "<name>"
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[3]
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[4]
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[5]
				<< "</name>" << std::endl;
		}
		kmloutput << "<TimeStamp><when>"
			<< std::setfill('0') << std::setw(4) << std::setprecision(0) << ep[0] << "-"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[1] << "-"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[2] << "T"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[3] << ":"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[4] << ":"
			<< std::setfill('0') << std::setw(5) << std::setprecision(2) << ep[5] << "Z"
			<< "</when></TimeStamp>" << std::endl;
	}
	double north_velocity = msg_vel.horizontal_speed * cos(msg_vel.track_ground * PI / 180);
	double east_velocity = msg_vel.horizontal_speed * sin(msg_vel.track_ground * PI / 180);
	double up_velocity = msg_vel.vertical_speed;
	kmloutput << "<description><![CDATA[" << std::endl
		<< "<TABLE border=\"1\" width=\"100%\" Align=\"center\">" << std::endl
		<< "<TR ALIGN=RIGHT>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>"
		<< msg_pos.novatel_msg_header.gps_week_num << "</TD><TD>"
		<< std::setiosflags(std::ios::fixed) << std::setprecision(3) << msg_pos.novatel_msg_header.gps_seconds << "</TD><TD>"
		<< std::setprecision(0) << std::setfill(' ') << std::setw(2) << ep[3] << ":" << std::setw(2) << ep[4] << ":" << std::setiosflags(std::ios::fixed) << std::setprecision(4) << std::setw(7) << ep[5] << "</TD><TD>"
		<< std::setprecision(0) << std::setw(4) << ep[0] << "/" << std::setw(2) << ep[1] << "/" << std::setw(2) << ep[2] << "</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>";
	kmloutput << std::setiosflags(std::ios::fixed) << std::setprecision(8) << msg_pos.lat << "</TD><TD>";
	kmloutput << msg_pos.lon << "</TD><TD>" << std::setprecision(4) << msg_pos.height + msg_pos.undulation << "</TD><TD>(DMS,m)</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(N,E,D):</TD><TD>"
		<< north_velocity << "</TD><TD>" << east_velocity << "</TD><TD>" << -up_velocity << "</TD><TD>(m/s)</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>"
		<< 0 << "</TD><TD>" << 0 << "</TD><TD>" << msg_vel.track_ground << "</TD><TD>(deg,approx)</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>"
		<< msg_pos.solution_status_int << "</TD><TD>"
		<< msg_pos.position_type_int << "</TD><TR>" << std::endl
		<< "</TABLE>" << std::endl
		<< "]]></description>" << std::endl;

	kmloutput << "<styleUrl>#P" << style << "</styleUrl>" << std::endl;
	kmloutput << "<Style>\n" << "<IconStyle>\n" << "<heading>" << msg_vel.track_ground << "</heading>\n" << "</IconStyle>\n" << "</Style>\n";

	kmloutput << "<Point>" << std::endl;
	if (outalt)
	{
		kmloutput << "<extrude>1</extrude>" << std::endl
			<< "<altitudeMode>absolute</altitudeMode>" << std::endl;
		alt = msg_pos.height;
	}
	kmloutput << "<coordinates>"
		<< std::setw(13) << std::setprecision(9) << msg_pos.lon << ","
		<< std::setw(12) << std::setprecision(9) << msg_pos.lat << ","
		<< std::setw(5) << std::setprecision(3) << msg_pos.height + msg_pos.undulation
		<< "</coordinates>" << std::endl
		<< "</Point>" << std::endl
		<< "</Placemark>" << std::endl;


/*
	kmloutput << "<description><![CDATA[" << std::endl
		<< "<TABLE border=\"1\" width=\"100%\" Align=\"center\">" << std::endl
		<< "<TR ALIGN=RIGHT>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>" << std::setiosflags(std::ios::fixed) << std::setprecision(3) << msg_pos.novatel_msg_header.gps_seconds << "</TD><TD>" << std::setprecision(0) << 0
		<< "</TD><TD>" << std::setfill(' ') << std::setw(2) << ep[3] << ":" << std::setw(2) << ep[4] << ":" << std::setiosflags(std::ios::fixed) << std::setprecision(4) << std::setw(7) << ep[5]
		<< "</TD><TD>" << std::setprecision(0) << std::setw(4) << ep[0] << "/" << std::setw(2) << ep[1] << "/" << std::setw(2) << ep[2] << "</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>";
	int deg, min;
	double sec;
	DegtoDMS(msg_pos.lat, deg, min, sec);
	kmloutput << std::setiosflags(std::ios::fixed) << std::setprecision(4) << deg << " " << min << " " << sec << "</TD><TD>";
	DegtoDMS(msg_pos.lon, deg, min, sec);
	double north_velocity = msg_vel.horizontal_speed * cos(msg_vel.track_ground * PI / 180);
	double east_velocity = msg_vel.horizontal_speed * sin(msg_vel.track_ground * PI / 180);
	double up_velocity = msg_vel.vertical_speed;
	kmloutput << deg << " " << min << " " << sec << "</TD><TD>" << msg_pos.height << "</TD><TD>(DMS,m)</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Vel(E,N,D):</TD><TD>"
		<< north_velocity << "</TD><TD>" << east_velocity << "</TD><TD>" << -up_velocity << "</TD><TD>(m/s)</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Att(r,p,h):</TD><TD>"
		<< 0 << "</TD><TD>" << 0 << "</TD><TD>" << msg_vel.track_ground << "</TD><TD>(deg,approx)</TD></TR>" << std::endl
		<< "</TABLE>" << std::endl
		<< "]]></description>" << std::endl;

	kmloutput << "<styleUrl>#P" << style << "</styleUrl>" << std::endl;

	kmloutput << "<Style>\n" << "<IconStyle>\n" << "<heading>" << msg_vel.track_ground << "</heading>\n" << "</IconStyle>\n" << "</Style>\n";

	kmloutput << "<Point>" << std::endl;
	if (outalt)
	{
		kmloutput << "<extrude>1</extrude>" << std::endl
			<< "<altitudeMode>absolute</altitudeMode>" << std::endl;
		alt = msg_pos.height;
	}
	kmloutput << "<coordinates>"
		<< std::setw(13) << std::setprecision(9) << msg_pos.lon << ","
		<< std::setw(12) << std::setprecision(9) << msg_pos.lat << ","
		<< std::setw(5) << std::setprecision(3) << msg_pos.height
		<< "</coordinates>" << std::endl
		<< "</Point>" << std::endl
		<< "</Placemark>" << std::endl;
		*/
}

void outpointgnsspos(std::ofstream& kmloutput, const novatel_gps_msgs::BestPos &msg_pos,
	const char *label, int style, int outalt, int outtime, int number)
{
	double ep[6], alt = 0.0;
	kmloutput << "<Placemark>" << std::endl;
	if (*label)
		kmloutput << "<name>" << label << "</name>" << std::endl;
	gtime_t gpstime = gpst2time(msg_pos.novatel_msg_header.gps_week_num, msg_pos.novatel_msg_header.gps_seconds);
	gtime_t utctime = gpst2utc(gpstime);
	time2epoch(utctime, ep);


	if (outtime)
	{
		if (outtime == 2) {}
		else if (outtime == 3) {}
		if (number == 1)
		{
			kmloutput << "<name>" << "Start" << "</name>" << std::endl;
		}
		else if (number == -1)
		{
			kmloutput << "<name>" << "End" << "</name>" << std::endl;

		}
		else if (fmod(ep[5] + 0.025, TINT) < 0.05)
		{
			kmloutput << "<name>"
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[3]
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[4]
				<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[5]
				<< "</name>" << std::endl;
		}
		kmloutput << "<TimeStamp><when>"
			<< std::setfill('0') << std::setw(4) << std::setprecision(0) << ep[0] << "-"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[1] << "-"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[2] << "T"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[3] << ":"
			<< std::setfill('0') << std::setw(2) << std::setprecision(0) << ep[4] << ":"
			<< std::setfill('0') << std::setw(5) << std::setprecision(2) << ep[5] << "Z"
			<< "</when></TimeStamp>" << std::endl;
	}

	int type =0 ;
	switch (msg_pos.position_type_int)
	{
	case 16:
		//spp;
		type = 1;
		break;
	case 17:
		//rtd;
		type = 2;
		break;
		//case 3:
		//	//udr;
		//	pcolor = 4;
		//	break;
	case 50:
		//fix;
		type = 4;
		break;
	case 34:
		//float;
		type = 5;
		break;
	default:
		break;
	}


	kmloutput << "<description><![CDATA[" << std::endl
		<< "<TABLE border=\"1\" width=\"100%\" Align=\"center\">" << std::endl
		<< "<TR ALIGN=RIGHT>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Time:</TD><TD>" 
		<< msg_pos.novatel_msg_header.gps_week_num<< "</TD><TD>"
	    << std::setiosflags(std::ios::fixed) << std::setprecision(3) << msg_pos.novatel_msg_header.gps_seconds << "</TD><TD>" 
		<< std::setprecision(0) << std::setfill(' ') << std::setw(2) << ep[3] << ":" << std::setw(2) << ep[4] << ":" << std::setiosflags(std::ios::fixed) << std::setprecision(4) << std::setw(7) << ep[5] << "</TD><TD>"
		<< std::setprecision(0) << std::setw(4) << ep[0] << "/" << std::setw(2) << ep[1] << "/" << std::setw(2) << ep[2] << "</TD></TR>" << std::endl
		<< "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Position:</TD><TD>";
	//int deg, min;
	//double sec;
	//DegtoDMS(msg_pos.lat, deg, min, sec);
	kmloutput << std::setiosflags(std::ios::fixed) << std::setprecision(8) << msg_pos.lat << "</TD><TD>";
	//DegtoDMS(msg_pos.lon, deg, min, sec);
	kmloutput << msg_pos.lon << "</TD><TD>" << std::setprecision(4) << msg_pos.height + msg_pos.undulation << "</TD><TD>(DMS,m)</TD></TR>" << std::endl
	      << "<TR ALIGN=RIGHT><TD ALIGN=LEFT>Mode:</TD><TD>"
		  << SOLUTION_STATUSES[msg_pos.solution_status_int] << "</TD><TD>"
		  << POSITION_TYPES[msg_pos.position_type_int] <<"</TD><TR>" << std::endl
		  << "</TABLE>" << std::endl
		  << "]]></description>" << std::endl;

	kmloutput << "<styleUrl>#P" << style << "</styleUrl>" << std::endl;

	kmloutput << "<Point>" << std::endl;
	if (outalt)
	{
		kmloutput << "<extrude>1</extrude>" << std::endl
			<< "<altitudeMode>absolute</altitudeMode>" << std::endl;
		alt = msg_pos.height;
	}
	kmloutput << "<coordinates>"
		<< std::setw(13) << std::setprecision(9) << msg_pos.lon << ","
		<< std::setw(12) << std::setprecision(9) << msg_pos.lat << ","
		<< std::setw(5) << std::setprecision(3) << msg_pos.height + msg_pos.undulation
		<< "</coordinates>" << std::endl
		<< "</Point>" << std::endl
		<< "</Placemark>" << std::endl;
}
/* convert to google earth kml file --------------------------------------------
* convert solutions to google earth kml file
* args   : char   *infile   I   input solutions file
*          char   *outfile  I   output google earth kml file ("":<infile>.kml)
*          gtime_t ts,te    I   start/end time (gpst)
*          int    tint      I   time interval (s) (0.0:all)
*          int    qflg      I   quality flag (0:all)
*          double *offset   I   add offset {east,north,up} (m)
*          int    tcolor    I   track color
*                               (0:none,1:white,2:green,3:orange,4:red,5:yellow)
*          int    pcolor    I   point color
*                               (0:none,1:white,2:green,3:orange,4:red,5:by qflag)
*          int    outalt    I   output altitude (0:off,1:elipsoidal,2:geodetic)
*          int    outtime   I   output time (0:off,1:gpst,2:utc,3:jst)
* return : status (0:ok,-1:file read,-2:file format,-3:no data,-4:file write)
* notes  : see ref [1] for google earth kml file format
*-----------------------------------------------------------------------------*/
bool saveinskml(const std::vector<novatel_gps_msgs::Inspva> &msg, int tcolor,
	int pcolor, int outalt, int outtime)
{
	int ret = false;
	std::string color[6] =
	{
	"ffffffff","ff0000ff","ffff00ff","50FF78F0","ff00ff00","ff00aaff"
	};//B-G-R 白色 绿色 浅黄  红色  黄色 青色
	output_ins_kml << HEADKML1 << std::endl << HEADKML2 << std::endl
		<< "<Document>" << std::endl;
	for (int i = 0; i < 6; i++)
	{
		output_ins_kml << "<Style id=\"P" << i << "\">\n" << std::endl
			<< "<IconStyle>\n" << std::endl
			<< "<color>" << color[i] << "</color>" << std::endl
			<< "<scale>" << std::setprecision(1) << SIZP << "</scale>" << std::endl
			<< "<Icon><href>" << MARKICON << "</href></Icon>" << std::endl
			<< "</IconStyle>" << std::endl
			<< "</Style>" << std::endl;
	}

	if (tcolor > 0)
	{
		outtrack(output_ins_kml, msg, color[tcolor - 1], outalt, outtime);
	}

	if (pcolor > 0)
	{
		output_ins_kml << "<Folder>" << std::endl
			<< "<name>Rover Position</name>" << std::endl;
		for (int i = 0; i < msg.size(); i++)
		{
			//设置输出频率,保持1Hz输出频率，并显示头尾
			if (i == 0 || i == msg.size() - 1 ||
				fmod(msg[i].novatel_msg_header.gps_seconds + 0.0005, 0.2) < 0.005)
			{
				int number = i + 1;
				if (i == 1) { number = 1; }
				if (i == msg.size() - 1) { number = -1; }
				int pcolor = 0;
				switch (msg[i].status_int)
				{
					//B-G-R 白色 绿色 浅黄  红色  黄色 青色
				case 0:
					//ros_msg->status = "INS_INACTIVE";
					pcolor = 0;
					break;
				case 1:
					//ros_msg->status = "INS_ALIGNING";
					pcolor = 2;
					break;
				case 2:
					//ros_msg->status = "INS_HIGH_VARIANCE";
					pcolor = 4;
					break;
				case 3:
					//ros_msg->status = "INS_SOLUTION_GOOD";
					pcolor = 1;
					break;
				case 6:
					//ros_msg->status = "INS_SOLUTION_FREE";
					pcolor = 3;
					break;
				case 7:
					//ros_msg->status = "INS_ALIGNMENT_COMPLETE";
					pcolor = 5;
					break;
				default:
					break;
				}
				pcolor = 4;

				outpoint(output_ins_kml, msg[i],
					"", pcolor, outalt, outtime, number);
			}
		}
		output_ins_kml << "</Folder>" << std::endl;
	}

	output_ins_kml << "</Document>" << std::endl
		<< "</kml>" << std::endl;
	output_ins_kml.close();
	ret = true;
	return ret;

}

bool saveinspvaxkml(const std::vector<novatel_gps_msgs::Inspvax> &msg, int tcolor,
	int pcolor, int outalt, int outtime)
{
	int ret = false;
	std::string color[6] =
	{
	"ffffffff","ff0000ff","ffff00ff","50FF78F0","ff00ff00","ff00aaff"
	};//B-G-R 白色 SPP RTD UDR FIX FLOAT
	output_ins_kml << HEADKML1 << std::endl << HEADKML2 << std::endl
		<< "<Document>" << std::endl;
	for (int i = 0; i < 6; i++)
	{
		output_ins_kml << "<Style id=\"P" << i << "\">\n" << std::endl
			<< "<IconStyle>\n" << std::endl
			<< "<color>" << color[i] << "</color>" << std::endl
			<< "<scale>" << std::setprecision(1) << SIZP << "</scale>" << std::endl
			<< "<Icon><href>" << MARKICON << "</href></Icon>" << std::endl
			<< "</IconStyle>" << std::endl
			<< "</Style>" << std::endl;
	}

	if (tcolor > 0)
	{
		tcolor = 1;
		outtrackinspvax(output_ins_kml, msg, color[tcolor - 1], outalt, outtime);
	}

	if (pcolor > 0)
	{
		output_ins_kml << "<Folder>" << std::endl
			<< "<name>Rover Position</name>" << std::endl;
		for (int i = 0; i < msg.size(); i++)
		{
			//设置输出频率,保持1Hz输出频率，并显示头尾
			if (i == 0 || i == msg.size() - 1 ||
				fmod(msg[i].novatel_msg_header.gps_seconds + 0.0005, 0.2) < 0.005)
			{
				int number = i + 1;
				if (i == 1) { number = 1; }
				if (i == msg.size() - 1) { number = -1; }
				int pcolor1 = 0;
				switch (msg[i].ins_status_int)
				{
				case 0:
					//ros_msg->status = "INS_INACTIVE";
					pcolor1 = 0;
					break;
				case 1:
					//ros_msg->status = "INS_ALIGNING";
					pcolor1 = 1;
					break;
				case 2:
					//ros_msg->status = "INS_HIGH_VARIANCE";
					pcolor1 = 1;
					break;
				case 3:
					//ros_msg->status = "INS_SOLUTION_GOOD";
					pcolor1 = 4;
					break;
				case 6:
					//ros_msg->status = "INS_SOLUTION_FREE";
					pcolor1 = 1;
					break;
				case 7:
					//ros_msg->status = "INS_ALIGNMENT_COMPLETE";
					pcolor1 = 1;
					break;
				default:
					break;
				}
				outpointinspvax(output_ins_kml, msg[i],
					"", pcolor1, 0, 1, number);
			}
		}
		output_ins_kml << "</Folder>" << std::endl;
	}

	output_ins_kml << "</Document>" << std::endl
		<< "</kml>" << std::endl;
	output_ins_kml.close();
	ret = true;
	return ret;

}

bool savegnsskml(const std::vector<novatel_gps_msgs::BestPos> &msg_pos,
	const std::vector<novatel_gps_msgs::Velocity> &msg_vel, int tcolor,
	int pcolor, int outalt, int outtime)
{
	int ret = false;
	std::string color[6] =
	{
	"ffffffff","ff0000ff","ffff00ff","50FF78F0","ff00ff00","ff00aaff"
	};//B-G-R 白色 SPP RTD UDR FIX FLOAT
	//             
	output_gnss_kml << HEADKML1 << std::endl << HEADKML2 << std::endl
		<< "<Document>" << std::endl;
	for (int i = 0; i < 6; i++)
	{
		output_gnss_kml << "<Style id=\"P" << i << "\">\n" << std::endl
			<< "<IconStyle>\n" << std::endl
			<< "<color>" << color[i] << "</color>" << std::endl
			<< "<scale>" << std::setprecision(1) << SIZP << "</scale>" << std::endl
			<< "<Icon><href>" << MARKICON << "</href></Icon>" << std::endl
			<< "</IconStyle>" << std::endl
			<< "</Style>" << std::endl;
	}

	if (tcolor > 0)
	{
		//outtrackgnss(output_gnss_kml, msg_pos, color[tcolor - 1], outalt, outtime);
		int tcolor1 = 1;
		outtrackgnss(output_gnss_kml, msg_pos, color[tcolor1 - 1], outalt, outtime);
	}

	if (pcolor > 0)
	{
		output_gnss_kml << "<Folder>" << std::endl
			<< "<name>Rover Position</name>" << std::endl;
		for (int i = 0, j = 0; i < msg_pos.size(); i++)
		{
			if (!msg_vel.empty() && j >= msg_vel.size())
			{
				break;
			}
			//设置输出频率,保持1Hz输出频率，并显示头尾
			if (FILE_TYPE != ACEINNA ||fmod(msg_pos[i].novatel_msg_header.gps_seconds + 0.005, 0.1) < 0.05)
			{
				int number = i + 1;
				if (i == 1) { number = 1; }
				if (i == msg_pos.size() - 1) { number = -1; }
				int pcolor = getpostype(msg_pos[i].position_type_int, FILE_TYPE);
				while (!msg_vel.empty())
				{
					if (fabs(msg_vel[j].novatel_msg_header.gps_seconds - msg_pos[i].novatel_msg_header.gps_seconds) < 0.1)
					{
						break;
					}
					else if (msg_vel[j].novatel_msg_header.gps_seconds - msg_pos[i].novatel_msg_header.gps_seconds < -0.1)
					{
						j++;
					}
					else if (msg_vel[j].novatel_msg_header.gps_seconds - msg_pos[i].novatel_msg_header.gps_seconds > 0.1)
					{
						i++;
					}
				}

				outpointgnss(output_gnss_kml, msg_pos[i], msg_vel[j],
					"", pcolor, outalt, outtime, number);
				j++;
			}
		}
		output_gnss_kml << "</Folder>" << std::endl;
	}

	output_gnss_kml << "</Document>" << std::endl
		<< "</kml>" << std::endl;
	output_gnss_kml.close();
	ret = true;
	return ret;

}

bool savegnssposkml(const std::vector<novatel_gps_msgs::BestPos> &msg_pos,
	int tcolor, int pcolor, int outalt, int outtime)
{
	int ret = false;
	std::string color[6] =
	{
	"ffffffff","ff0000ff","ffff00ff","50FF78F0","ff00ff00","ff00aaff"
	};//B-G-R 白色 SPP RTD UDR FIX FLOAT
	output_gnss_kml << HEADKML1 << std::endl << HEADKML2 << std::endl
		<< "<Document>" << std::endl;
	for (int i = 0; i < 6; i++)
	{
		output_gnss_kml << "<Style id=\"P" << i << "\">\n" << std::endl
			<< "<IconStyle>\n" << std::endl
			<< "<color>" << color[i] << "</color>" << std::endl
			<< "<scale>" << std::setprecision(1) << SIZP << "</scale>" << std::endl
			<< "<Icon><href>" << MARKICON << "</href></Icon>" << std::endl
			<< "</IconStyle>" << std::endl
			<< "</Style>" << std::endl;
	}

	if (tcolor > 0)
	{
		//outtrackgnss(output_gnss_kml, msg_pos, color[tcolor - 1], outalt, outtime);
		int tcolor1 = 2;
		outtrackgnss(output_gnss_kml, msg_pos, color[tcolor1 - 1], outalt, outtime);
	}

	if (pcolor > 0)
	{
		output_gnss_kml << "<Folder>" << std::endl
			<< "<name>Rover Position</name>" << std::endl;
		for (int i = 0; i < msg_pos.size(); i++)
		{
			//设置输出频率,保持1Hz输出频率，并显示头尾
			if (fmod(msg_pos[i].novatel_msg_header.gps_seconds + 0.025, 1) < 0.05)
			{

				int number = i + 1;
				if (i == 1) { number = 1; }
				if (i == msg_pos.size() - 1) { number = -1; }
				int 	pcolor1 = 0;
				if (FILE_TYPE == ACEINNA)
				{
					switch (msg_pos[i].position_type_int)
					{
					case 1:
						//spp;
						pcolor1 = 1;
						break;
					case 2:
						//rtd;
						pcolor1 = 2;
						break;
					case 3:
						//udr;
						pcolor1 = 3;
						break;
					case 4:
						//fix;
						pcolor1 = 4;
						break;
					case 5:
						//float;
						pcolor1 = 5;
						break;
					default:
						break;
					}
				}
				else if (FILE_TYPE == NOVATEL)
				{
					switch (msg_pos[i].position_type_int)
					{
					case 16:
						//spp;
						pcolor1 = 1;
						break;
					case 17:
						//rtd;
						pcolor1 = 2;
						break;
						//case 3:
						//	//udr;
						//	pcolor = 4;
						//	break;
					case 50:
						//fix;
						pcolor1 = 4;
						break;
					case 34:
						//float;
						pcolor1 = 5;
						break;
					default:
						break;
					}

					if (msg_pos[i].solution_status_int != 0)
					{
						pcolor1 = 0;
					}

				}
				outpointgnsspos(output_gnss_kml, msg_pos[i], "", pcolor1, 0, 1, number);

			}
		}
		output_gnss_kml << "</Folder>" << std::endl;
	}

	output_gnss_kml << "</Document>" << std::endl
		<< "</kml>" << std::endl;
	output_gnss_kml.close();
	ret = true;
	return ret;

}

bool file_close()
{

	bool ret = false;
	if (publish_gnss_positions_) output_gnss.close();
	if (pubilsh_gnss_vel_) output_gnss_vel.close();
	if (publish_aceinna_imu_)   output_imu.close();
	if (publish_imu_messages_)   output_imu.close();
	if (publish_ins_)  output_ins.close();
	if (publish_inspvax_)  output_ins.close();

	if (publish_process_) output_process.close();
	if (publish_kml_)
	{
		output_gnss_kml.close();
		output_ins_kml.close();
	}
	return ret;
}

