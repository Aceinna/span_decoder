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
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <include/parsers/rangecmp.h>
#include <include/parsers/header.h>

#include <boost/make_shared.hpp>
#include <iostream>
#include <sstream>
#include <vector>

#include <rtklib/rtklib.h>
#pragma warning(disable:4996)

#define SCALE_FACTOR_DOP (1.0/256)
#define SCALE_FACTOR_PSR (1.0/128)
#define SCALE_FACTOR_ADR (1.0/256)
#define SCALE_FACTOR_CLOCKTIME (1.0/32)

#define MAXVAL      8388608.0



const double STDDEV_PSR[] = { 0.05, 0.075,0.113,0.169,0.253,0.380,0.570,0.854,1.281,2.375,4.750,9.50,19.00,38.00,76.00,152.00 };

/* get fields (little-endian) ------------------------------------------------*/
#define U1(p) (*((unsigned char *)(p)))
#define I1(p) (*((signed char *)(p)))
static unsigned short U2(unsigned char *p) { unsigned short u; memcpy(&u, p, 2); return u; }
static unsigned int   U4(unsigned char *p) { unsigned int   u; memcpy(&u, p, 4); return u; }
static int            I4(unsigned char *p) { int            i; memcpy(&i, p, 4); return i; }
static float          R4(unsigned char *p) { float          r; memcpy(&r, p, 4); return r; }
static double         R8(unsigned char *p) { double         r; memcpy(&r, p, 8); return r; }

/* extend sign ---------------------------------------------------------------*/
static int exsign(unsigned int v, int bits)
{
	return (int)(v&(1 << (bits - 1)) ? v | (~0u << bits) : v);
}

/* decode oem4 tracking status -------------------------------------------------
* deocode oem4 tracking status
* args   : unsigned int stat I  tracking status field
*          int    *sys   O      system (SYS_???)
*          int    *code  O      signal code (CODE_L??)
*          int    *track O      tracking state
*                         (oem4/5)
*                         0=L1 idle                   8=L2 idle
*                         1=L1 sky search             9=L2 p-code align
*                         2=L1 wide freq pull-in     10=L2 search
*                         3=L1 narrow freq pull-in   11=L2 pll
*                         4=L1 pll                   12=L2 steering
*                         5=L1 reacq
*                         6=L1 steering
*                         7=L1 fll
*                         (oem6)
*                         0=idle                      7=freq-lock loop
*                         2=wide freq band pull-in    9=channel alignment
*                         3=narrow freq band pull-in 10=code search
*                         4=phase lock loop          11=aided phase lock loop
*          int    *plock O      phase-lock flag   (0=not locked, 1=locked)
*          int    *clock O      code-lock flag    (0=not locked, 1=locked)
*          int    *parity O     parity known flag (0=not known,  1=known)
*          int    *halfc O      phase measurement (0=half-cycle not added,
*                                                  1=added)
* return : signal frequency (0:L1,1:L2,2:L5,3:L6,4:L7,5:L8,-1:error)
* notes  : refer [1][3]
*-----------------------------------------------------------------------------*/
static int decode_trackstat(unsigned int stat, int *sys, int *code, int *track,
	int *plock, int *clock, int *parity, int *halfc)
{
	int satsys, sigtype, freq = 0;

	*track = stat & 0x1F;
	*plock = (stat >> 10) & 1;
	*parity = (stat >> 11) & 1;
	*clock = (stat >> 12) & 1;
	satsys = (stat >> 16) & 7;
	*halfc = (stat >> 28) & 1;
	sigtype = (stat >> 21) & 0x1F;

	switch (satsys) {
	case 0: *sys = SYS_GPS; break;
	case 1: *sys = SYS_GLO; break;
	case 2: *sys = SYS_SBS; break;
	case 3: *sys = SYS_GAL; break; /* OEM6 */
	case 4: *sys = SYS_CMP; break; /* OEM6 F/W 6.400 */
	case 5: *sys = SYS_QZS; break; /* OEM6 */
	default:
		//trace(2, "oem4 unknown system: sys=%d\n", satsys);
		return -1;
	}
	if (*sys == SYS_GPS || *sys == SYS_QZS) {
		switch (sigtype) {
		case  0: freq = 0; *code = CODE_L1C; break; /* L1C/A */
		case  5: freq = 0; *code = CODE_L1P; break; /* L1P */
		case  9: freq = 1; *code = CODE_L2W; break; /* L2Pcodeless */
		case 14: freq = 2; *code = CODE_L5Q; break; /* L5Q (OEM6) */
		case 17: freq = 1; *code = CODE_L2X; break; /* L2C(M+L) */
		default: freq = -1; break;
		}
	}
	else if (*sys == SYS_GLO) {
		switch (sigtype) {
		case  0: freq = 0; *code = CODE_L1C; break; /* L1C/A */
		case  1: freq = 1; *code = CODE_L2C; break; /* L2C/A (OEM6) */
		case  5: freq = 1; *code = CODE_L2P; break; /* L2P */
		default: freq = -1; break;
		}
	}
	else if (*sys == SYS_GAL) {
		switch (sigtype) {
		case  1: freq = 0; *code = CODE_L1B; break; /* E1B  (OEM6) */
		case  2: freq = 0; *code = CODE_L1C; break; /* E1C  (OEM6) */
		case 12: freq = 2; *code = CODE_L5Q; break; /* E5aQ (OEM6) */
		case 17: freq = 4; *code = CODE_L7Q; break; /* E5bQ (OEM6) */
		case 20: freq = 5; *code = CODE_L8Q; break; /* AltBOCQ (OEM6) */
		default: freq = -1; break;
		}
	}
	else if (*sys == SYS_CMP) {
		switch (sigtype) {
		case  0: freq = 0; *code = CODE_L1I; break; /* B1 with D1 (OEM6) */
		case  1: freq = 1; *code = CODE_L7I; break; /* B2 with D1 (OEM6) */
		case  4: freq = 0; *code = CODE_L1I; break; /* B1 with D2 (OEM6) */
		case  5: freq = 1; *code = CODE_L7I; break; /* B2 with D2 (OEM6) */
		default: freq = -1; break;
		}
	}
	else if (*sys == SYS_SBS) {
		switch (sigtype) {
		case  0: freq = 0; *code = CODE_L1C; break; /* L1C/A */
		case  6: freq = 2; *code = CODE_L5I; break; /* L5I (OEM6) */
		default: freq = -1; break;
		}
	}
	if (freq < 0) {
		//trace(2, "oem4 signal type error: sys=%d sigtype=%d\n", *sys, sigtype);
		return -1;
	}
	return freq;
}

/* check code priority and return obs position -------------------------------*/
static int checkpri(const char *opt, int sys, int code, int freq)
{
	int nex = NEXOBS; /* number of extended obs data */

	if (sys == SYS_GPS) {
		if (strstr(opt, "-GL1P") && freq == 0) return code == CODE_L1P ? 0 : -1;
		if (strstr(opt, "-GL2X") && freq == 1) return code == CODE_L2X ? 1 : -1;
		if (code == CODE_L1P) return nex < 1 ? -1 : NFREQ;
		if (code == CODE_L2X) return nex < 2 ? -1 : NFREQ + 1;
	}
	else if (sys == SYS_GLO) {
		if (strstr(opt, "-RL2C") && freq == 1) return code == CODE_L2C ? 1 : -1;
		if (code == CODE_L2C) return nex < 1 ? -1 : NFREQ;
	}
	else if (sys == SYS_GAL) {
		if (strstr(opt, "-EL1B") && freq == 0) return code == CODE_L1B ? 0 : -1;
		if (code == CODE_L1B) return nex < 1 ? -1 : NFREQ;
		if (code == CODE_L7Q) return nex < 2 ? -1 : NFREQ + 1;
		if (code == CODE_L8Q) return nex < 3 ? -1 : NFREQ + 2;
	}
	return freq < NFREQ ? freq : -1;
}

const std::string novatel_gps_driver::RangecmpParser::MESSAGE_NAME = "RANGECMP";


/* extract unsigned/signed bits ------------------------------------------------
* extract unsigned/signed bits from byte data
* args   : unsigned char *buff I byte data
*          int    pos    I      bit position from start of data (bits)
*          int    len    I      bit length (bits) (len<=32)
* return : extracted unsigned/signed bits
*-----------------------------------------------------------------------------*/
extern unsigned int getbitu(const unsigned char *buff, int pos, int len)
{
	unsigned int bits = 0;
	int i;
	for (i = pos; i < pos + len; i++) bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
	return bits;
}
extern int getbits(const unsigned char *buff, int pos, int len)
{
	unsigned int bits = getbitu(buff, pos, len);
	if (len <= 0 || 32 <= len || !(bits&(1u << (len - 1)))) return (int)bits;
	return (int)(bits | (~0u << len)); /* extend sign */
}

/* get two component bits ----------------------------------------------------*/
static unsigned int getbitu2(const unsigned char *buff, int p1, int l1, int p2,
	int l2)
{
	return (getbitu(buff, p1, l1) << l2) + getbitu(buff, p2, l2);
}
static int getbits2(const unsigned char *buff, int p1, int l1, int p2, int l2)
{
	if (getbitu(buff, p1, 1))
		return (int)((getbits(buff, p1, l1) << l2) + getbitu(buff, p2, l2));
	else
		return (int)getbitu2(buff, p1, l1, p2, l2);
}
/* get three component bits --------------------------------------------------*/
static unsigned int getbitu3(const unsigned char *buff, int p1, int l1, int p2,
	int l2, int p3, int l3)
{
	return (getbitu(buff, p1, l1) << (l2 + l3)) + (getbitu(buff, p2, l2) << l3) +
		getbitu(buff, p3, l3);
}
static int getbits3(const unsigned char *buff, int p1, int l1, int p2, int l2,
	int p3, int l3)
{
	if (getbitu(buff, p1, 1))
		return (int)((getbits(buff, p1, l1) << (l2 + l3)) +
		(getbitu(buff, p2, l2) << l3) + getbitu(buff, p3, l3));
	else
		return (int)getbitu3(buff, p1, l1, p2, l2, p3, l3);
}

uint32_t novatel_gps_driver::RangecmpParser::GetMessageId() const
{
  return MESSAGE_ID;
}

const std::string novatel_gps_driver::RangecmpParser::GetMessageName() const
{
  return MESSAGE_NAME;
}

novatel_gps_msgs::RangecmpPtr
novatel_gps_driver::RangecmpParser::ParseBinary(const novatel_gps_driver::BinaryMessage& bin_msg) noexcept(false)
{
  uint32_t num_obs = ParseUInt32(&bin_msg.data_[0]);
  if (bin_msg.data_.size() != (BINARY_OBSERVATION_SIZE * num_obs) + 4)
  {
    std::stringstream error;
    error << "Unexpected Rangecmp message size: " << bin_msg.data_.size();
    throw ParseException(error.str());
  }
  novatel_gps_msgs::RangecmpPtr ros_msg = boost::make_shared<novatel_gps_msgs::Rangecmp>();
  HeaderParser h_parser;
  ros_msg->novatel_msg_header = h_parser.ParseBinary(bin_msg);
  ros_msg->novatel_msg_header.message_name = "RangecmpCMP";

  ros_msg->numb_of_observ = num_obs;
  ros_msg->info.reserve(num_obs);
  for(int i = 0; i < num_obs; i++)
  {
    size_t obs_offset = 4 + i * BINARY_OBSERVATION_SIZE;

    novatel_gps_msgs::RangeInformation info;

    info.prn_number = ParseUInt16(&bin_msg.data_[obs_offset]);
    info.glofreq = ParseUInt16(&bin_msg.data_[obs_offset+2]);
    info.psr = ParseDouble(&bin_msg.data_[obs_offset+4]);
    info.psr_std = ParseFloat(&bin_msg.data_[obs_offset+12]);
    info.adr = ParseDouble(&bin_msg.data_[obs_offset+16]);
    info.adr_std = ParseFloat(&bin_msg.data_[obs_offset+24]);
    info.dopp = ParseFloat(&bin_msg.data_[obs_offset+28]);
    info.noise_density_ratio = ParseFloat(&bin_msg.data_[obs_offset+32]);
    info.locktime = ParseFloat(&bin_msg.data_[obs_offset+36]);
    info.tracking_status = ParseUInt32(&bin_msg.data_[obs_offset+40]);

    ros_msg->info.push_back(info);
  }
  return ros_msg;
}

novatel_gps_msgs::RangecmpPtr
novatel_gps_driver::RangecmpParser::ParseAscii(const novatel_gps_driver::NovatelSentence& sentence) noexcept(false)
{
  novatel_gps_msgs::RangecmpPtr msg = boost::make_shared<novatel_gps_msgs::Rangecmp>();
  HeaderParser h_parser;
  msg->novatel_msg_header = h_parser.ParseAscii(sentence);
  if (!ParseInt32(sentence.body[0], msg->numb_of_observ, 10))
  {
    std::stringstream error;
    error << "Unable to parse number of observations in Rangecmp log.";
    throw ParseException(error.str());
  }
  uint32_t numb_of_observ = static_cast<uint32_t>(msg->numb_of_observ);
  if (sentence.body.size() != 1 + numb_of_observ )
  {
    std::stringstream error;
    error << "Did not find expected number of observations in Rangecmp log.";
    throw ParseException(error.str());
  }
  bool valid = true;
  valid &= ParseInt32(sentence.body[0], msg->numb_of_observ, 10);
  msg->info.reserve(numb_of_observ);
  //将十六进制明文转化成二进制数组
  uint8 buffer[30];

  for (int i = 0; i < numb_of_observ; i++)
  {
	  uint8* p = buffer;
	  const char *ptr = sentence.body[i+1].c_str();
	  while (sscanf(ptr, "%2x", p) )
	  {
		  ptr = ptr + 2;
		  p = p + 1;
	  }     //xu?

	  novatel_gps_msgs::RangeInformation info;
	  info.tracking_status = U4(buffer);
	  int freq;
	  if ((freq = decode_trackstat(info.tracking_status, &info.trackstat.sys, &info.trackstat.code, &info.trackstat.track, &info.trackstat.plock, &info.trackstat.clock,
		  &info.trackstat.parity, &info.trackstat.halfc)) < 0); //continue;

	  char opt[256];      /* receiver dependent options */

	 /* obs position */
	  int pos;
	  if ((pos = checkpri(opt, info.trackstat.sys, info.trackstat.code, freq)) < 0);//continue;

	  info.prn_number = U1(buffer + 17);
	  if (info.trackstat.sys == SYS_GLO) info.prn_number -= 37;

	  int sat;
	  if (!(sat = satno(info.trackstat.sys, info.prn_number))) {
		  //continue;
	  }
	  if (info.trackstat.sys == SYS_GLO && !info.trackstat.parity); //continue; /* invalid if GLO parity unknown */


	   info.dopp = exsign(U4(buffer + 4) & 0xFFFFFFF, 28) / 256.0;
	   info.psr = (U4(buffer + 7) >> 4) / 128.0 + U1(buffer + 11)*2097152.0;

	  uint8 psr_std = (*reinterpret_cast<uint8 *>(&buffer[16]) & 0x0F);
	  uint8 adr_std = *reinterpret_cast<uint8 *>(&buffer[16]) >> 4;

	  info.prn_number = STDDEV_PSR[psr_std];
	  info.adr_std = (adr_std + 1) / 512.00;

	  double wavelen;
	  if ((wavelen = satwavelen(sat, freq)) <= 0.0) {
		  if (info.trackstat.sys == SYS_GLO) wavelen = CLIGHT / (freq == 0 ? FREQ1_GLO : FREQ2_GLO);
		  else wavelen = lam_carr[freq];
	  }

	  double adr = I4(buffer + 12) / 256.0;
	  double adr_rolls = (info.psr / wavelen + adr) / MAXVAL;
	  info.adr = adr - MAXVAL * floor(adr_rolls + (adr_rolls <= 0 ? -0.5 : 0.5));

	  info.locktime = (U4(buffer + 18) & 0x1FFFFF) / 32.0; /* lock time */

	  info.noise_density_ratio = ((U2(buffer + 20) & 0x3FF) >> 5) + 20.0;

	 // info.glofreq 
	// int8 a	  = getbits(buffer, 170, 5);
	  msg->info.push_back(info);
  }
  /*
  if (!valid)
  {
    throw ParseException("Error parsing Rangecmp log.");
  }
  */
  return msg;
}
