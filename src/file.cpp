
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

#define ISA100C_GYRO 5.729577951308233E-08
#define ISA100C_ACC 2.0E-8


#define CPT_GYRO   6.670106611340576E-09  /* 2^-33 * 180 /PI */

#define MAXLEAPS    64                  /* max number of leap seconds table */
const static double gpst0[] = { 1980,1,6,0,0,0 }; /* gps time reference */
static double leaps[MAXLEAPS + 1][7] = { /* leap seconds (y,m,d,h,m,s,utc-gpst) */
	{2017,1,1,0,0,0,-18},
	{2015,7,1,0,0,0,-17},
	{2012,7,1,0,0,0,-16},
	{2009,1,1,0,0,0,-15},
	{2006,1,1,0,0,0,-14},
	{1999,1,1,0,0,0,-13},
	{1997,7,1,0,0,0,-12},
	{1996,1,1,0,0,0,-11},
	{1994,7,1,0,0,0,-10},
	{1993,7,1,0,0,0, -9},
	{1992,7,1,0,0,0, -8},
	{1991,1,1,0,0,0, -7},
	{1990,1,1,0,0,0, -6},
	{1988,1,1,0,0,0, -5},
	{1985,7,1,0,0,0, -4},
	{1983,7,1,0,0,0, -3},
	{1982,7,1,0,0,0, -2},
	{1981,7,1,0,0,0, -1},
	{0}
};
static  gtime_t timeadd(gtime_t t, double sec)
{
	double tt;

	t.sec += sec; tt = floor(t.sec); t.time += (int)tt; t.sec -= tt;
	return t;
}
static void time2epoch(gtime_t t, double *ep)
{
	const int mday[] = { /* # of days in a month */
		31,28,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31,
		31,29,31,30,31,30,31,31,30,31,30,31,31,28,31,30,31,30,31,31,30,31,30,31
	};
	int days, sec, mon, day;

	/* leap year if year%4==0 in 1901-2099 */
	days = (int)(t.time / 86400);
	sec = (int)(t.time - (time_t)days * 86400);
	for (day = days % 1461, mon = 0; mon < 48; mon++) {
		if (day >= mday[mon]) day -= mday[mon]; else break;
	}
	ep[0] = 1970 + days / 1461 * 4 + mon / 12; ep[1] = mon % 12 + 1; ep[2] = day + 1;
	ep[3] = sec / 3600; ep[4] = sec % 3600 / 60; ep[5] = sec % 60 + t.sec;
}
static gtime_t epoch2time(const double *ep)
{
	const int doy[] = { 1,32,60,91,121,152,182,213,244,274,305,335 };
	gtime_t time = { 0 };
	int days, sec, year = (int)ep[0], mon = (int)ep[1], day = (int)ep[2];

	if (year < 1970 || 2099 < year || mon < 1 || 12 < mon) return time;

	/* leap year if year%4==0 in 1901-2099 */
	days = (year - 1970) * 365 + (year - 1969) / 4 + doy[mon - 1] + day - 2 + (year % 4 == 0 && mon >= 3 ? 1 : 0);
	sec = (int)floor(ep[5]);
	time.time = (time_t)days * 86400 + (int)ep[3] * 3600 + (int)ep[4] * 60 + sec;
	time.sec = ep[5] - sec;
	return time;
}
static gtime_t gpst2time(int week, double sec)
{
	gtime_t t = epoch2time(gpst0);

	if (sec < -1E9 || 1E9 < sec) sec = 0.0;
	t.time += 86400 * 7 * week + (int)sec;
	t.sec = sec - (int)sec;
	return t;
}
static gtime_t gpst2utc(gtime_t t)
{
	gtime_t tu;

	////for (i = 0; leaps[i][0] > 0; i++) {
	   //// tu = timeadd(t, leaps[i][6]);
	   //// if (timediff(tu, epoch2time(leaps[i])) >= 0.0) return tu;
	////}
	tu = timeadd(t, leaps[0][6]);
	return tu;
}
static void  DegtoDMS(const double mdeg, int* Deg, int* Min, double* Sec)
{
	double AM;
	*Deg = (int)mdeg;
	AM = (mdeg - *Deg) * 60.0;
	*Min = (int)AM;
	*Sec = (AM - *Min) * 60.0;
}

static void deg2dms(double deg, double* dms)
{
	double sign = deg < 0.0 ? -1.0 : 1.0, a = fabs(deg);
	dms[0] = floor(a); a = (a - dms[0]) * 60.0;
	dms[1] = floor(a); a = (a - dms[1]) * 60.0;
	dms[2] = a; dms[0] *= sign;
}
/* output solution in the form of nmea GGA sentence --------------------------*/
static int outnmea_gga(unsigned char* buff, double time, int type, double* blh, int ns, double dop, double age)
{
	double h, ep[6], dms1[3], dms2[3];
	char* p = (char*)buff, *q, sum;

	if (type != 1 && type != 4 && type != 5) {
		p += sprintf_s(p, 255, "$GPGGA,,,,,,,,,,,,,,");
		for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q;
		p += sprintf_s(p, 255, "*%02X%c%c", sum, 0x0D, 0x0A);
		return p - (char*)buff;
	}
	time -= 18.0;
	ep[2] = floor(time / (24 * 3600));
	time -= ep[2] * 24 * 3600.0;
	ep[3] = floor(time / 3600);
	time -= ep[3] * 3600;
	ep[4] = floor(time / 60);
	time -= ep[4] * 60;
	ep[5] = time;
	h = 0.0;
	deg2dms(fabs(blh[0]) * 180.0 / PI, dms1);
	deg2dms(fabs(blh[1]) * 180.0 / PI, dms2);
	p += sprintf_s(p, 255, "$GPGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,",
		ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, blh[0] >= 0 ? "N" : "S",
		dms2[0], dms2[1] + dms2[2] / 60.0, blh[1] >= 0 ? "E" : "W", type,
		ns, dop, blh[2] - h, h, age);
	for (q = (char*)buff + 1, sum = 0; *q; q++) sum ^= *q; /* check-sum */
	p += sprintf_s(p, 255, "*%02X%c%c", sum, 0x0D, 0x0A);
	return p - (char*)buff;
}


static void euler2dcm(const double eular[3], double dc[3][3])
{
	double roll = eular[0];
	double  pitch = eular[1];
	double  heading = eular[2];
	double  cr, cp, ch, sr, sp, sh;
	cr = cos(roll); cp = cos(pitch); ch = cos(heading);
	sr = sin(roll); sp = sin(pitch); sh = sin(heading);

	dc[0][0] = cp * ch;
	dc[0][1] = -cr * sh + sr * sp*ch;
	dc[0][2] = sr * sh + cr * sp*ch;

	dc[1][0] = cp * sh;
	dc[1][1] = cr * ch + sr * sp*sh;
	dc[1][2] = -sr * ch + cr * sp * sh;

	dc[2][0] = -sp;
	dc[2][1] = sr * cp;
	dc[2][2] = cr * cp;
}

static uint8_t MatrixMutiply(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_column, const int matrix_b_column, double *matrix_result)
{
	double sum = 0;
	double median = 0;
	for (int i = 0; i < matrix_a_row; i++)
	{
		for (int k = 0; k < matrix_b_column; k++)
		{
			for (int j = 0; j < matrix_a_column; j++)
			{
				median = matrix_a[matrix_a_column*i + j] * matrix_b[matrix_b_column*j + k];
				sum = sum + median;
			}
			matrix_result[matrix_b_column*i + k] = sum;
			sum = 0;
		}
	}
	return 1;

}

uint8_t MatrixAdd(const double *matrix_a, const double *matrix_b, const int matrix_a_row, const int matrix_a_colume, double *matrix_result)
{
	for (int i = 0; i < matrix_a_row*matrix_a_colume; i++)
	{
		*(matrix_result + i) = *(matrix_a + i) + *(matrix_b + i);
	}
	return 1;
}

typedef  struct EarthParameter
{
	double a;       //Ellipsoid long axis
	double b;       //Ellipsoid short axis
	double f;       //Ellipsoidal oblate 
	double e;       //first Eccentricity of Elliopsoid 
	double e2;
	//double ep;
	//double ep2;     //second Eccentricity of Elliopsoid 
	double wie;     //rotational angular velocity of the earths  
	double GM;      //geocentric gravitational constant 
} EarthParameter;
const  EarthParameter WGS84 = { 6378137.0, 6356752.3142, 0.0033528106643315515,0.081819190837555025,0.0066943799893122479 ,  7.2922115147e-5,398600441800000.00 };


uint8_t UpdateMN(const double *BLH, double *M, double *N)
{
	double sinB = sin(*BLH);
	double temp = 1 - WGS84.e2 * sinB * sinB;
	double sqrttemp = sqrt(temp);
	*M = WGS84.a * (1 - WGS84.e2) / (sqrttemp*temp);
	*N = WGS84.a / sqrttemp;
	return 1;
};


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
  { 26, std::pair<std::vector<double>, std::string>({200,ISA100C_GYRO,ISA100C_ACC}, "Northrop Grumman Litef ISA-100C") },
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


bool publish_gnss_positions_ = false;
bool pubilsh_gnss_vel_ = false;
bool publish_aceinna_imu_ = true;
bool publish_odometer_imu_ = true;
bool publish_ins_ = true;
bool publish_ins_gga_ = false;
bool publish_process_ = true;
bool publish_gnss_ = true;
bool publish_kml_ = true;
bool publish_gnss_gga_ = false;
bool publish_inspvax_ = false;
bool publish_imu_messages_ = false;
bool publish_gps_bin = false;
bool publish_gnss_csv = false;
bool publish_ins_csv = false;
std::ifstream iput_file;
std::ofstream output_gnss;
std::ofstream output_gnss_vel;
std::ofstream output_imu;
std::ofstream output_odo;
std::ofstream output_ins;
std::ofstream output_process;

std::ofstream output_gnssposvel;
std::ofstream output_gnss_kml;
std::ofstream output_ins_kml;

std::ofstream output_gnss_csv;
std::ofstream output_ins_csv;


std::ofstream output_usergga;


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

FILE* output_GGA_GPS = NULL; /* output GGA for GPS input */
FILE* output_GGA_INS = NULL; /* output GGA for INS input */


bool file_open(const std::string input_fname)
{
	bool ret = false;
	std::string fname = input_fname.substr(0, input_fname.find_last_of('.'));
	std::string outfilename = fname + "-gga.nmea";
	if(publish_gnss_gga_)
	fopen_s(&output_GGA_GPS, outfilename.c_str(), "w");

	if (publish_gnss_) output_gnssposvel.open(fname + "-gnssposvel.txt");
	if (publish_gnss_positions_) output_gnss.open(fname + "-gnss.txt");
	if (pubilsh_gnss_vel_) output_gnss_vel.open(fname + "-gnssvel.txt");

	if (publish_aceinna_imu_)   output_imu.open(fname + "-imu.txt");
	if (publish_odometer_imu_)   output_odo.open(fname + "-odo.txt");

	if (publish_imu_messages_)   output_imu.open(fname + "-imu.txt");

	if (publish_ins_)  output_ins.open(fname + "-ins.txt");
	outfilename = fname + "-ins-gga.nmea";
	if (publish_ins_gga_) 	fopen_s(&output_GGA_INS, outfilename.c_str(), "w");


	if (publish_inspvax_)  output_ins.open(fname + "-ins.txt");

	if (publish_process_) output_process.open(fname + "-process");

	if (publish_kml_)
	{
		output_gnss_kml.open(fname + "-gnss.kml");
		output_ins_kml.open(fname + "-ins.kml");
	}

	if (publish_gnss_csv)
	{
		output_gnss_csv.open(fname + "-gnss.csv");
		output_gnss_csv << "Week,GPSTime,Roll,Pitch,Heading,VX-ECEF,VY-ECEF,VZ-ECEF,VEast,VNorth,VUp,AngRateX,AngRateY,AngRateZ,AccBdyX,AccBdyY,AccBdyZ,X-ECEF,Y-ECEF,Z-ECEF,RollSD,PitchSD,HdngSD,SDEast,SDNorth,SDHeight,SD-VE,SD-VN,SD-VH,Latitude,Longitude,H-Ell,NS,HDOP" << std::endl;
		output_gnss_csv << "(weeks),(sec),(deg),(deg),(deg),(m/s),(m/s),(m/s),(m/s),(m/s),(m/s),(deg/s),(deg/s),(deg/s),(m/s^2),(m/s^2),(m/s^2),(m),(m),(m),(deg),(deg),(deg),(m),(m),(m),(m/s),(m/s),(m/s),(deg),(deg),(m),,(dop)" << std::endl;
	}

	if (publish_ins_csv)
	{
		output_ins_csv.open(fname + "-ins.csv");
		output_ins_csv << "Week,GPSTime,Roll,Pitch,Heading,VX-ECEF,VY-ECEF,VZ-ECEF,VEast,VNorth,VUp,AngRateX,AngRateY,AngRateZ,AccBdyX,AccBdyY,AccBdyZ,X-ECEF,Y-ECEF,Z-ECEF,RollSD,PitchSD,HdngSD,SDEast,SDNorth,SDHeight,SD-VE,SD-VN,SD-VH,Latitude,Longitude,H-Ell,NS,HDOP" << std::endl;
		output_ins_csv << "(weeks),(sec),(deg),(deg),(deg),(m/s),(m/s),(m/s),(m/s),(m/s),(m/s),(deg/s),(deg/s),(deg/s),(m/s^2),(m/s^2),(m/s^2),(m),(m),(m),(deg),(deg),(deg),(m),(m),(m),(m/s),(m/s),(m/s),(deg),(deg),(m),,(dop)" << std::endl;
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

bool traceodometer(novatel_gps_msgs::OdometerPtr msg)
{
	bool ret = false;
	if (publish_odometer_imu_)
	{
		output_odo << std::setw(4) << msg->gps_week_num << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->gps_seconds << ","
			<< std::setw(14) << (int)msg->mode << ","
			<< std::setw(14) << std::setprecision(10) << msg->speed << ","
			<< std::setw(14) << (int)msg->fwd << ","
			<< std::setw(14) << msg->wheel_tick
			<< std::endl;

	}
	if (publish_process_)
	{
		output_process << "$GPODO,"
			<< std::setw(4) << msg->gps_week_num << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->gps_seconds << ","
			<< std::setw(14) << (int)msg->mode << ","
			<< std::setw(14) << std::setprecision(10) << msg->speed << ","
			<< std::setw(14) << (int)msg->fwd << ","
			<< std::setw(14) << msg->wheel_tick
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
		case 1:
			//rtk;
			ret = 1;
			break;
		case 2:
			//rtd;
			ret = 2;
			break;
		case 4:
			//rtk;
			ret = 4;
			break;
		case 5:
			//rtd;
			ret = 5;
			break;
		case 6:
			//rtd;
			ret = 6;
			break;
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
		case 53:
			//spp;
			ret = 1;
			break;
		case 17:
			//rtd;
			ret = 2;
			break;
		case 54:
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
		case 56:
			//fix;
			ret = 4;
			break;
		case 55:
			//fix;
			ret = 5;
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

bool tracegnss(novatel_gps_msgs::BestPosPtr msg,int ID)
{
	bool ret = false;
	if (FILE_TYPE == NOVATEL &&ID != 1429)
	{
		return -1;
	}
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

	if (publish_gnss_csv)
	{
		output_gnss_csv << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
			<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
			<< "0" << "," << "0" << "," << "0" << "," // roll ,pitch ,heading
			<< "0" << "," << "0" << "," << "0" << "," // 
			<< "0" << "," << "0" << "," << "0" << "," // ve vn vd
			<< "0" << "," << "0" << "," << "0" << "," // wnb
			<< "0" << "," << "0" << "," << "0" << "," // a_n
			<< "0" << "," << "0" << "," << "0" << "," // 
			<< "0" << "," << "0" << "," << "0" << "," // p
			<< std::setw(10) << std::setprecision(4) << msg->lat_sigma << ","
			<< std::setw(10) << std::setprecision(4) << msg->lat_sigma << ","
			<< std::setw(10) << std::setprecision(4) << msg->height_sigma << ","
			<< "0" << "," << "0" << "," << "0" << "," // p
			<< std::setw(14) << std::setprecision(9) << msg->lat << ","
			<< std::setw(14) << std::setprecision(9) << msg->lon << ","
			<< std::setw(10) << std::setprecision(4) << msg->height + msg->undulation << ","
			<< "0" << "0" << std::endl;
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
			if (type >= 0)
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
	if (publish_ins_gga_ && fabs(msg->latitude) > 0.001
		&& (msg->status_int))
	{
		double leverarm_v[3] = { 0.0,0.0, 0.0 };
		double eular[3] = { msg->roll*PI / 180,msg->pitch * PI / 180,msg->azimuth * PI / 180 };
		double C_vn[3][3];
		euler2dcm(eular, C_vn);
		double leverarm_n[3];
		MatrixMutiply(*C_vn, leverarm_v, 3, 3, 1, leverarm_n);
		double d_leverarm[3];
		double pos[3] = { msg->latitude*PI / 180, msg->longitude*PI / 180, msg->height };
		double M, N;
		UpdateMN(pos, &M, &N);
		d_leverarm[0] = leverarm_n[0] / (M + pos[2]);
		d_leverarm[1] = leverarm_n[1] / ((N + pos[2])*cos(pos[0]));
		d_leverarm[2] = -leverarm_n[2];

		MatrixAdd(pos, d_leverarm, 3, 1, pos);
		unsigned char ggaBuffer[400] = { 0 };
		int type = msg->status_int;
		int len = outnmea_gga(ggaBuffer, msg->novatel_msg_header.gps_seconds, type, pos, 10, 1.0, 1.0);
		fprintf(output_GGA_INS, "%s", ggaBuffer);
	}
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

	if (publish_ins_gga_ &&fabs(msg->latitude) > 0.001
		&& (msg->ins_status_int == 3 || msg->ins_status_int == 6 || msg->ins_status_int == 7))
	{
		double leverarm_v[3] = { 0.0,0.0, 0.0};
		double eular[3] = { msg->roll*PI / 180,msg->pitch * PI / 180,msg->azimuth * PI / 180 };
		double C_vn[3][3];
		euler2dcm(eular, C_vn);
		double leverarm_n[3];
		MatrixMutiply(*C_vn, leverarm_v, 3, 3, 1, leverarm_n);
		double d_leverarm[3];
		double pos[3] = { msg->latitude*PI / 180, msg->longitude*PI / 180, msg->altitude + msg->undulation };
		double M, N;
		UpdateMN(pos, &M, &N);
		d_leverarm[0] = leverarm_n[0] / (M + pos[2]);
		d_leverarm[1] = leverarm_n[1] / ((N + pos[2])*cos(pos[0]));
		d_leverarm[2] = -leverarm_n[2];

		MatrixAdd(pos, d_leverarm, 3, 1, pos);
		unsigned char ggaBuffer[400] = { 0 };
		int type = getpostype(msg->position_type_int,  1);
		int len = outnmea_gga(ggaBuffer, msg->novatel_msg_header.gps_seconds, type, pos, 10, 1.0, 1.0);
		fprintf(output_GGA_INS, "%s", ggaBuffer);
	}

	if (fabs(msg->latitude) > 0.001
		&& (msg->ins_status_int == 3 || msg->ins_status_int == 6 || msg->ins_status_int == 7))
	{
		if (publish_ins_csv)
		{
			output_ins_csv << std::setw(4) << msg->novatel_msg_header.gps_week_num << ","
				<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg->novatel_msg_header.gps_seconds << ","
				<< std::setw(10) << std::setprecision(4) << msg->roll << ","
				<< std::setw(10) << std::setprecision(4) << msg->pitch << ","
				<< std::setw(10) << std::setprecision(4) << msg->azimuth << ","
				<< "0" << "," << "0" << "," << "0" << "," // 
				<< std::setw(10) << std::setprecision(4) << msg->east_velocity << ","
				<< std::setw(10) << std::setprecision(4) << msg->north_velocity << ","
				<< std::setw(10) << std::setprecision(4) << msg->up_velocity << ","
				<< "0" << "," << "0" << "," << "0" << "," // wnb
				<< "0" << "," << "0" << "," << "0" << "," // a_n
				<< "0" << "," << "0" << "," << "0" << "," // 
				<< std::setw(10) << std::setprecision(4) << msg->roll_std << ","
				<< std::setw(10) << std::setprecision(4) << msg->pitch_std << ","
				<< std::setw(10) << std::setprecision(4) << msg->azimuth_std << ","
				<< std::setw(10) << std::setprecision(4) << msg->longitude_std << ","
				<< std::setw(10) << std::setprecision(4) << msg->latitude_std << ","
				<< std::setw(10) << std::setprecision(4) << msg->altitude_std << ","
				<< std::setw(10) << std::setprecision(4) << msg->east_velocity_std << ","
				<< std::setw(10) << std::setprecision(4) << msg->north_velocity_std << ","
				<< std::setw(10) << std::setprecision(4) << msg->up_velocity_std << ","
				<< std::setw(14) << std::setprecision(9) << msg->latitude << ","
				<< std::setw(14) << std::setprecision(9) << msg->longitude << ","
				<< std::setw(10) << std::setprecision(4) << msg->altitude + msg->undulation << ","
				<< "0" << "0" << std::endl;
		}
	}


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





	output_gnssposvel << std::setw(4) << msg_pos.novatel_msg_header.gps_week_num << ","
		<< std::setw(10) << std::setiosflags(std::ios::fixed) << std::setprecision(4) << msg_pos.novatel_msg_header.gps_seconds << ","
		<< std::setw(14) << std::setprecision(9) << msg_pos.lat << ","
		<< std::setw(14) << std::setprecision(9) << msg_pos.lon << ","
		<< std::setw(10) << std::setprecision(4) << msg_pos.height + msg_pos.undulation << ","
		<< std::setw(10) << std::setprecision(4) << msg_pos.lat_sigma << ","
		<< std::setw(10) << std::setprecision(4) << msg_pos.lon_sigma << ","
		<< std::setw(10) << std::setprecision(4) << msg_pos.height_sigma << ","
		<< msg_pos.position_type_int << ","
		<< std::setw(10) << std::setprecision(4) << north_velocity << ","
		<< std::setw(10) << std::setprecision(4) << east_velocity << ","
		<< std::setw(10) << std::setprecision(4) << up_velocity << ","
		<< std::setw(10) << std::setprecision(4) << msg_vel.track_ground
		<< std::endl;
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

#define INS_KML_OUTPUT_DATARATE (1.0)
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
				fmod(msg[i].novatel_msg_header.gps_seconds + 0.0005, INS_KML_OUTPUT_DATARATE) < 0.005)
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
				fmod(msg[i].novatel_msg_header.gps_seconds + 0.0005, INS_KML_OUTPUT_DATARATE) < 0.005)
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
	if (publish_odometer_imu_)   output_odo.close();

	if (publish_imu_messages_)   output_imu.close();
	if (publish_ins_)  output_ins.close();
	if (publish_inspvax_)  output_ins.close();

	if (publish_process_) output_process.close();
	if (publish_gnss_) output_gnssposvel.close();
	if (publish_kml_)
	{
		output_gnss_kml.close();
		output_ins_kml.close();
	}
	if (output_GGA_GPS) fclose(output_GGA_GPS);
	if (output_GGA_INS) fclose(output_GGA_INS);
	if (output_gnss_csv.is_open()) output_gnss_csv.close();
	if (output_ins_csv.is_open()) output_ins_csv.close();

	return ret;
}

