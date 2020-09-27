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
#ifndef _FILE_H_
#define _FILE_H_
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>

#include "include/msg/rawimu.h"
#include "include/msg/rawimusx.h"
#include "include/msg/bestpos.h"
#include "include/msg/velocity.h"
#include "include/msg/inspva.h"
#include "include/msg/inspvax.h"
#include "include/msg/odometer.h"
#include "rtklib/rtklib.h"

#define ACEINNA 0
#define NOVATEL 1

extern bool publish_gnss_;
extern bool publish_gnss_positions_ ;
extern bool pubilsh_gnss_vel_ ;
extern bool publish_aceinna_imu_;
extern bool publish_ins_ ;
extern bool publish_process_ ;
extern bool publish_kml_ ;

extern bool publish_inspvax_ ;
extern bool publish_imu_messages_ ;
extern bool publish_gps_bin ;
extern bool publish_ins_gga_;

extern int FILE_TYPE ;

extern std::ifstream iput_file;
extern std::ofstream output_gnss;
extern std::ofstream output_gnss_vel;
extern std::ofstream output_imu;
extern std::ofstream output_ins;
extern std::ofstream output_process;
extern std::ofstream output_gnss_kml;
extern std::ofstream output_ins_kml;

extern std::vector <novatel_gps_msgs::BestPos> gnss_msgs_;
extern std::vector <novatel_gps_msgs::Velocity> gnssvel_msgs_;
extern std::vector <novatel_gps_msgs::Inspva> ins_msgs_;
extern std::vector <novatel_gps_msgs::Inspvax> inspvax_msgs_;

bool file_open(const std::string input_fname);

bool traceimu(novatel_gps_msgs::RawimuPtr msg);

bool tracenovatelimu(novatel_gps_msgs::RawimusxPtr msg);

bool traceodometer(novatel_gps_msgs::OdometerPtr msg);


bool tracegnss(novatel_gps_msgs::BestPosPtr msg,int ID);

bool tracegnssvel(novatel_gps_msgs::VelocityPtr msg);

bool traceins(novatel_gps_msgs::InspvaPtr msg);

bool traceinspvax(novatel_gps_msgs::InspvaxPtr msg);


void outtrackinspva(std::ofstream& kmloutput, const std::vector<novatel_gps_msgs::Inspva> &msg,
	const std::string color, int outalt, int outtime);

void outtrackinspvax(std::ofstream& kmloutput, const std::vector<novatel_gps_msgs::Inspvax> &msg,
	const std::string color, int outalt, int outtime);

void outtrackgnss(std::ofstream& kmloutput, const std::vector<novatel_gps_msgs::BestPos> &msg,
	const std::string color, int outalt, int outtime);

/*output point*/
void outpointinspva(std::ofstream& kmloutput, const novatel_gps_msgs::Inspva & msg,
	const char *label, int style, int outalt, int outtime, int number);
void outpointinspvax(std::ofstream& kmloutput, const novatel_gps_msgs::Inspvax & msg,
	const char *label, int style, int outalt, int outtime, int number);

void outpointgnss(std::ofstream& kmloutput, const novatel_gps_msgs::BestPos &msg_pos,
	const novatel_gps_msgs::Velocity &msg_vel,
	const char *label, int style, int outalt, int outtime, int number);

	void outpointgnsspos(std::ofstream& kmloutput, const novatel_gps_msgs::BestPos &msg_pos,
		const char *label, int style, int outalt, int outtime, int number);
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
	int pcolor, int outalt, int outtime);

bool saveinspvaxkml(const std::vector<novatel_gps_msgs::Inspvax> &msg, int tcolor,
	int pcolor, int outalt, int outtime);

bool savegnsskml(const std::vector<novatel_gps_msgs::BestPos> &msg_pos,
	const std::vector<novatel_gps_msgs::Velocity> &msg_vel, int tcolor,
	int pcolor, int outalt, int outtime);

bool savegnssposkml(const std::vector<novatel_gps_msgs::BestPos> &msg_pos,
	int tcolor, int pcolor, int outalt, int outtime);

bool file_close();

#endif