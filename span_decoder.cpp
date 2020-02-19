// span_decoder.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#include "kml.h"

#include "minizipdll.h"
#pragma comment(lib,"MINIZIPDLL.lib")

#define MAXFIELD 100

#define P2_33       1.164153218269348E-10 /* 2^-33 */
#define P2_29       1.862645149230957E-09 /* 2^-29 */

#define SPAN_CPT7 0
#define SPAN_FLEX6 1
#define SPAN_ACEINNA 2
#define SPAN_ADI16488 3

#define	grav_WGS84 9.7803267714e0
#ifndef PI
#define	PI 3.14159265358979
#endif

typedef struct {        /* time struct */
	time_t time;        /* time (s) expressed by standard time_t */
	double sec;         /* fraction of second under 1 s */
} gtime_t;

extern gtime_t epoch2time(const double* ep)
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

static const double gpst0[] = { 1980,1, 6,0,0,0 }; /* gps time reference */

/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
extern double time2gpst(gtime_t t, int* week)
{
	gtime_t t0 = epoch2time(gpst0);
	time_t sec = t.time - t0.time;
	int w = (int)(sec / (86400 * 7));

	if (week) *week = w;
	return (double)(sec - (double)w * 86400 * 7) + t.sec;
}

#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */


/* transform geodetic to ecef position -----------------------------------------
* transform geodetic position to ecef position
* args   : double *pos      I   geodetic position {lat,lon,h} (rad,m)
*          double *r        O   ecef position {x,y,z} (m)
* return : none
* notes  : WGS84, ellipsoidal height
*-----------------------------------------------------------------------------*/
extern void pos2ecef(const double* pos, double* r)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);
	double e2 = FE_WGS84 * (2.0 - FE_WGS84), v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

	r[0] = (v + pos[2]) * cosp * cosl;
	r[1] = (v + pos[2]) * cosp * sinl;
	r[2] = (v * (1.0 - e2) + pos[2]) * sinp;
}

/* multiply matrix -----------------------------------------------------------*/
extern void matmul(const char* tr, int n, int k, int m, double alpha,
	const double* A, const double* B, double beta, double* C)
{
	double d;
	int i, j, x, f = tr[0] == 'N' ? (tr[1] == 'N' ? 1 : 2) : (tr[1] == 'N' ? 3 : 4);

	for (i = 0; i < n; i++) for (j = 0; j < k; j++) {
		d = 0.0;
		switch (f) {
		case 1: for (x = 0; x < m; x++) d += A[i + x * n] * B[x + j * m]; break;
		case 2: for (x = 0; x < m; x++) d += A[i + x * n] * B[j + x * k]; break;
		case 3: for (x = 0; x < m; x++) d += A[x + i * m] * B[x + j * m]; break;
		case 4: for (x = 0; x < m; x++) d += A[x + i * m] * B[j + x * k]; break;
		}
		if (beta == 0.0) C[i + j * n] = alpha * d; else C[i + j * n] = alpha * d + beta * C[i + j * n];
	}
}

/* ecef to local coordinate transfromation matrix ------------------------------
* compute ecef to local coordinate transfromation matrix
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *E        O   ecef to local coord transformation matrix (3x3)
* return : none
* notes  : matirix stored by column-major order (fortran convention)
*-----------------------------------------------------------------------------*/
extern void xyz2enu(const double* pos, double* E)
{
	double sinp = sin(pos[0]), cosp = cos(pos[0]), sinl = sin(pos[1]), cosl = cos(pos[1]);

	E[0] = -sinl;      E[3] = cosl;       E[6] = 0.0;
	E[1] = -sinp * cosl; E[4] = -sinp * sinl; E[7] = cosp;
	E[2] = cosp * cosl;  E[5] = cosp * sinl;  E[8] = sinp;
}
/* transform ecef vector to local tangental coordinate -------------------------
* transform ecef vector to local tangental coordinate
* args   : double *pos      I   geodetic position {lat,lon} (rad)
*          double *r        I   vector in ecef coordinate {x,y,z}
*          double *e        O   vector in local tangental coordinate {e,n,u}
* return : none
*-----------------------------------------------------------------------------*/
extern void ecef2enu(const double* pos, const double* r, double* e)
{
	double E[9];

	xyz2enu(pos, E);
	matmul("NN", 3, 1, 3, 1.0, E, r, 0.0, e);
}

typedef enum INS_STATUS
{
	INS_INACTIVE = 0,
	DETERMINING_ORIENTATION,
	INS_ALIGNING,
	INS_ALIGNMENT_COMPLETE,
	INS_SOLUTION_GOOD
};

typedef struct
{
	int week;
	double gps_tow;
	double lat; // deg
	double lon;
	float hgt;
	float vn; // m/s
	float ve;
	float vu;
	float roll; // deg
	float pitch;
	float azimuth;
} ins_pva;

#define SECONDS_IN_WEEK (604800)

static gtime_t timeadd(gtime_t t, double sec)
{
	double tt;

	t.sec += sec;
	tt = floor(t.sec);
	t.time += (int)tt;
	t.sec -= tt;
	return t;
}
static gtime_t gpst2utc(gtime_t t)
{
	return timeadd(t, -18.0);
}

static gtime_t gpst2time(int week, double sec)
{
	gtime_t t = epoch2time(gpst0);

	if (sec < -1E9 || 1E9 < sec)
		sec = 0.0;
	t.time += SECONDS_IN_WEEK * week + (int)sec;
	t.sec = sec - (int)sec;
	return t;
}

static void time2epoch(gtime_t t, double* ep)
{
	const int mday[] = {/* # of days in a month */
						31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
						31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
	int days, sec, mon, day;

	/* leap year if year%4==0 in 1901-2099 */
	days = (int)(t.time / 86400);
	sec = (int)(t.time - (time_t)days * 86400);
	for (day = days % 1461, mon = 0; mon < 48; mon++)
	{
		if (day >= mday[mon])
			day -= mday[mon];
		else
			break;
	}
	ep[0] = 1970 + days / 1461 * 4 + mon / 12;
	ep[1] = mon % 12 + 1;
	ep[2] = day + 1;
	ep[3] = sec / 3600;
	ep[4] = sec % 3600 / 60;
	ep[5] = sec % 60 + t.sec;
}

static void deg2dms(double deg, double* dms, int ndec)
{
	double sign = deg < 0.0 ? -1.0 : 1.0, a = fabs(deg);
	double unit = pow(0.1, ndec);
	dms[0] = floor(a);
	a = (a - dms[0]) * 60.0;
	dms[1] = floor(a);
	a = (a - dms[1]) * 60.0;
	dms[2] = floor(a / unit + 0.5) * unit;
	if (dms[2] >= 60.0)
	{
		dms[2] = 0.0;
		dms[1] += 1.0;
		if (dms[1] >= 60.0)
		{
			dms[1] = 0.0;
			dms[0] += 1.0;
		}
	}
	dms[0] *= sign;
}

static int print_nmea_gga(gtime_t time, double* xyz, int nsat, int type, double dop, double age, char* buff)
{
	double h, ep[6], pos[3], dms1[3], dms2[3];
	char* p = (char*)buff, * q, sum;

	if ((xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]) < 1.0)
	{
		p += sprintf(p, "$GPGGA,,,,,,,,,,,,,,");
		for (q = (char*)buff + 1, sum = 0; *q; q++)
			sum ^= *q;
		p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
	}
	else
	{
		time = gpst2utc(time);
		time2epoch(time, ep);
#if 0
		int sss = 0;
		if (ep[3] == 10.0 && ep[4] == 29.0 && fabs(ep[5] - 46.1) < 0.01)
			sss = 1;
#endif
		pos[0] = xyz[0]; pos[1] = xyz[1]; pos[2] = xyz[2];
		h = 0.0; //geoidh(pos);
		deg2dms(fabs(pos[0]), dms1, 7);
		deg2dms(fabs(pos[1]), dms2, 7);
		p += sprintf(p, "$GPGGA,%02.0f%02.0f%05.2f,%02.0f%010.7f,%s,%03.0f%010.7f,%s,%d,%02d,%.1f,%.3f,M,%.3f,M,%.1f,",
			ep[3], ep[4], ep[5], dms1[0], dms1[1] + dms1[2] / 60.0, pos[0] >= 0 ? "N" : "S",
			dms2[0], dms2[1] + dms2[2] / 60.0, pos[1] >= 0 ? "E" : "W", type,
			nsat, dop, pos[2] - h, h, age);
		for (q = (char*)buff + 1, sum = 0; *q; q++)
			sum ^= *q; /* check-sum */
		p += sprintf(p, "*%02X%c%c", sum, 0x0D, 0x0A);
	}
	return p - (char*)buff;
}

static void parse_fields(char* const buffer, char** val)
{
	char* p, * q;
	int n = 0;

	/* parse fields */
	for (p = buffer; *p && n < MAXFIELD; p = q + 1) {
		if ((q = strchr(p, ',')) || (q = strchr(p, '*'))) {
			val[n++] = p; *q = '\0';
		}
		else break;
	}

}

void decode_span(const char* fname, int sensor, double sampleRate, int isKMZ)
{
	FILE* fdat = NULL;
	FILE* fgga = NULL, *fgga_ins = NULL;
	FILE* fpos = NULL;
	FILE* fgps = NULL;
	FILE* fimu = NULL;
	FILE* fins = NULL;
	FILE* fkml = NULL;
	FILE* fhdg = NULL;

	char fileName[255] = { 0 };
	char outfilename[255] = { 0 };
	char out_kml_fpath[255] = { 0 };

	fdat = fopen(fname, "r"); if (fdat == NULL) return;

	strcpy(fileName, fname);
	char* result1 = strrchr(fileName, '.');
	if (result1 != NULL) result1[0] = '\0';

	sprintf(outfilename, "%s-gnss.gga", fileName); fgga = fopen(outfilename, "w");
	sprintf(outfilename, "%s-gps.bin", fileName); fgps = fopen(outfilename, "w");
	sprintf(outfilename, "%s-pos.csv", fileName); fpos = fopen(outfilename, "w");
	sprintf(outfilename, "%s-imu.csv", fileName); fimu = fopen(outfilename, "w");
	sprintf(outfilename, "%s-ins.csv", fileName); fins = fopen(outfilename, "w");
	sprintf(outfilename, "%s-ins.gga", fileName); fgga_ins = fopen(outfilename, "w");
	sprintf(outfilename, "%s-hdg.csv", fileName); fhdg = fopen(outfilename, "w");
	sprintf(out_kml_fpath, "%s.kml", fileName); fkml = fopen(out_kml_fpath, "w");

	if (fkml!=NULL) print_kml_heder(fkml);

	int type = 0;
	int wn = 0;

	char buffer[1024*8] = { 0 }, sol_status[56] = {0};

	char* p, * q, * val[MAXFIELD];

	double time_vel[4] = { 0.0 };

	while (!feof(fdat))
	{
		fgets(buffer, sizeof(buffer), fdat);
		if (strlen(buffer) <= 0) continue;
		char *temp = strchr(buffer, '\n');
		if (temp != NULL) temp[0] = '\0';

		parse_fields(buffer, val);

		if (strstr(val[0], "BESTGNSSPOSA") != NULL)
		{
			/*
#BESTGNSSPOSA,SPECIAL,0,45.0,COARSESTEERING,2075,507495.200,00400000,bede,14392;SOL_COMPUTED,SINGLE,37.38509346056,-122.09022710468,29.3987,-32.3000,WGS84,4.4743,3.1986,5.9788,"",0.000,0.000,5,5,5,0,00,02,00,01*44f57d2d
			*/
			double blh[3] = { atof(val[11]), atof(val[12]), atof(val[13])+ atof(val[14]) };
			double wn = atof(val[5]);
			double ws = atof(val[6]);
			double rms[3] = { atof(val[16]), atof(val[17]), atof(val[18]) };
			int type = 0;
			if (strstr(val[10], "SINGLE") != NULL) type = 1;
			else if (strstr(val[10], "NARROW_INT") != NULL) type = 4;
			else if (strstr(val[10], "NARROW_FLOAT") != NULL) type = 5;
			else if (strstr(val[10], "PSRDIFF") != NULL) type = 2;
			if (fpos != NULL) {
				fprintf(fpos, "%4.0f,%10.3f,1,%14.10f,%14.10f,%10.4f,%10.4f,%10.4f,%10.4f,%3i\n", wn, ws, blh[0], blh[1], blh[2], rms[0], rms[1], rms[2], type);
#ifdef GNSS_ONLY
				char solsts[28] = { "GNSS only" };
				print_kml_gga(fkml, blh[0], blh[1], blh[2], type, ws, 0.0, solsts);
#endif
			}

			gtime_t gt = gpst2time(wn, ws);
			char gga[256] = { 0 };
			print_nmea_gga(gt, blh, atof(val[23]), type, 1.0, atof(val[20]), gga);
			if (NULL != fgga) fprintf(fgga, "%s", gga);
			
			continue;
		}
		if (strstr(val[0], "RTKPOSA") != NULL)
		{
			/*
#RTKPOSA,USB1,0,70.8,FINESTEERING,2076,112663.000,00000000,000e,10558;SOL_COMPUTED,NARROW_INT,40.04225041311,116.28967046789,49.2162,-9.7964,WGS84,0.0268,0.0212,0.0552,"1597",1.000,0.000,32,32,24,19,00,02,30,33*d9f90b44
			*/
			double blh[3] = { atof(val[11]), atof(val[12]), atof(val[13]) + atof(val[14]) };
			double wn = atof(val[5]);
			double ws = atof(val[6]);
			double rms[3] = { atof(val[16]), atof(val[17]), atof(val[18]) };
			int type = 0;
			if (strstr(val[10], "SINGLE") != NULL) type = 1;
			else if (strstr(val[10], "NARROW_INT") != NULL) type = 4;
			else if (strstr(val[10], "NARROW_FLOAT") != NULL) type = 5;
			else if (strstr(val[10], "PSRDIFF") != NULL) type = 2;
			if (fpos != NULL) fprintf(fpos, "%4.0f,%10.3f,1,%14.10f,%14.10f,%10.4f,%10.4f,%10.4f,%10.4f,%3i\n", wn, ws, blh[0], blh[1], blh[2], rms[0], rms[1], rms[2], type);
			continue;
		}
		if (strstr(val[0], "RTKVELA") != NULL)
		{
			/*
#BESTVELA,USB1,0,73.3,FINESTEERING,2076,112838.000,00000000,000e,10558;SOL_COMPUTED,DOPPLER_VELOCITY,0.000,1.000,8.6437,77.431308,0.0275,0.0*d771d968
			/* TODO, output to fpos
			*/
			continue;
		}
		if (strstr(val[0], "HEADING2A") != NULL)
		{
			/*
#HEADING2A,SPECIAL,0,23.5,FINESTEERING,2078,516279.000,02004000,1684,15826;SOL_COMPUTED,NARROW_INT,-1.000000000,239.906387329,-0.616668701,0.0,0.206709728,0.369018108,"H64Z","H64Z",32,29,29,23,04,01,15,33*272e3cb4
			/* TODO, output to fpos
			*/
			double wn = atof(val[5]);
			double ws = atof(val[6]);
			float baseline_len = atof(val[10]);
			float heading_da = atof(val[11]); // dual antenna heading
			float pitch_da = atof(val[12]); // dual antenna pitch/roll, depend on antenna installation
			int type = 0;
			if (strstr(val[10], "SINGLE") != NULL) type = 1;
			else if (strstr(val[10], "NARROW_INT") != NULL) type = 4;
			else if (strstr(val[10], "NARROW_FLOAT") != NULL) type = 5;
			else if (strstr(val[10], "PSRDIFF") != NULL) type = 2;
			if (fhdg != NULL) fprintf(fhdg, "%4.0f,%10.3f,%.3f,%.3f,%.3f,%d\n", wn, ws, baseline_len, heading_da, pitch_da, type);
			continue;
		}
		if (strstr(val[0], "BESTGNSSVELA") != NULL)
		{
			/*
#BESTGNSSVELA,SPECIAL,0,45.0,COARSESTEERING,2075,507495.200,00400000,00b0,14392;SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.1949,334.029536,-0.0767,0.0*ed8abf8e
			*/
			double wn = atof(val[5]);
			double ws = atof(val[6]);
			float latency = atof(val[11]), vh = atof(val[13]), heading = atof(val[14])*PI/180.0, vu = atof(val[15]);
			float vn = vh * cos(heading);
			float ve = vh * sin(heading);
			//if (fpos != NULL) fprintf(fpos, "%4.0f,%10.3f,%.3f,%.3f,%.3f,%.3f\n", wn, ws, latency, ve, vn, vu);

			continue;
		}
		if (strstr(val[0], "RANGECMPA") != NULL)
		{
			int k = 0;
			/* decode the RAW GNSS measurements from main antenna and 2nd antenna, then store to GPS file */
			/* TODO */
		}
		if (strstr(val[0], "RAWIMUSXA") != NULL)
		{
			/*
%RAWIMUSXA,2075,507495.165;00,13,2075,507495.165000000,00000000,32068,-1612,-1320,256,1536,-255*dd88585c
			*/
			double wn = atof(val[4]);
			double ws = atof(val[5]);
			double wxyz_scale = 1.0;
			double fxyz_scale = 1.0;
			if (sensor == SPAN_CPT7)
			{
				fxyz_scale = P2_29;
				wxyz_scale = P2_33;
			}
			else if (sensor == SPAN_FLEX6)
			{

			}
			else if (sensor == SPAN_ACEINNA)
			{
				fxyz_scale = 2.5e-4*grav_WGS84;
				wxyz_scale = 5.0e-3*PI / 180.0;
			}
			if (sampleRate > 0.0)
			{
				fxyz_scale *= sampleRate;
				wxyz_scale *= sampleRate;
			}
			double fxyz[3] = { atof(val[9]) * fxyz_scale, -atof(val[8]) * fxyz_scale, atof(val[7]) * fxyz_scale };
			double wxyz[3] = { atof(val[12]) * wxyz_scale, -atof(val[11]) * wxyz_scale, atof(val[10]) * wxyz_scale };
			if (fimu != NULL) fprintf(fimu, "%4.0f,%14.7f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f,%14.10f\n", wn, ws, fxyz[0], fxyz[1], fxyz[2], wxyz[0], wxyz[1], wxyz[2]);
			continue;
		}
		if (strstr(val[0], "INSPVAXA") != NULL)
		{
			/*
#INSPVAXA,SPECIAL,0,63.5,FINESTEERING,2077,426091.650,02004800,471d,15826;WAITING_AZIMUTH,NARROW_INT,37.38118475784,-121.94136658119,9.0947,-32.1000,0.0094,0.0022,-0.0245,1.128617803,3.143108851,-0.000000000,0.0111,0.0055,0.0167,0.0431,0.0235,0.0673,3.0000,3.0000,180.0000,01000000,0*284f2874
			*/
			//if (!strstr(val[9], "INS_ALIGNMENT_COMPLETE") && !strstr(val[9], "INS_SOLUTION_GOOD")) continue;
			int wn = atoi(val[5]);
			double ws = atof(val[6]);
			double blh[3] = { atof(val[11]), atof(val[12]), atof(val[13])+ atof(val[14]) };
			if (blh[0]==0.0 && blh[1]==0.0 && blh[2] == 0.0) continue;
			double vel_NEU[3] = { atof(val[15]), atof(val[16]), -atof(val[17]) };
			double att[3] = { atof(val[18]), atof(val[19]), atof(val[20]) };
			double rms_PosNEU[3] = { atof(val[21]), atof(val[22]), atof(val[23]) };
			double rms_VelNEU[3] = { atof(val[24]), atof(val[24]), atof(val[26]) };
			double rms_att[3] = { atof(val[27]), atof(val[28]), atof(val[29]) };
			double timeLast = atof(val[31]);

			time_vel[0] = ws;
			time_vel[1] = vel_NEU[0]; // N
			time_vel[2] = vel_NEU[1]; // E
			time_vel[3] =-vel_NEU[2]; // D

			strcpy(sol_status, strchr(val[9], ';')+1);

			if (fins != NULL)
			{
				fprintf(fins, "%4i,%10.3f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n"
					, wn, ws
					, blh[0], blh[1], blh[2]
					, vel_NEU[0], vel_NEU[1], vel_NEU[2]
					, att[0], att[1], att[2]
					, rms_PosNEU[0], rms_PosNEU[1], rms_PosNEU[2]
					, rms_VelNEU[0], rms_VelNEU[1], rms_VelNEU[2]
					, rms_att[0], rms_att[1], rms_att[2]
				);
			}

			int solType = -1;
			if (strstr(val[10], "INS_PSRSP") != NULL) solType = 1;
			else if (strstr(val[10], "INS_PSRDIFF") != NULL) solType = 2;
			else if (strstr(val[10], "PSRDIFF") != NULL) solType = 2;
			else if (strstr(val[10], "PROPAGATED") != NULL) solType = 3;
			else if (strstr(val[10], "INS_RTKFLOAT") != NULL) solType = 5;
			else if (strstr(val[10], "INS_RTKFIXED") != NULL) solType = 4;
			else {
				solType = 1;
				//printf("not supported\n");
			}
			strcpy(sol_status+strlen(sol_status), ",");
			strcpy(sol_status+strlen(sol_status), val[10]);

			gtime_t gt = gpst2time(wn, ws);
			char gga[256] = { 0 };
			print_nmea_gga(gt, blh, 20, solType, 1.0, 1.0, gga);
			if (NULL != fgga_ins) fprintf(fgga_ins, "%s", gga);

			//printf("GPS TOW: %f\n", ws);
			float heading = atof(val[20]);
#ifndef GNSS_ONLY
			print_kml_gga(fkml, blh[0], blh[1], blh[2], solType, ws, heading, sol_status);
#endif			
			continue;
		}
		if (strstr(val[0], "INSPVASA") != NULL)
		{
			/*
%INSPVASA,2083,374524.200;2083,374524.200000000,31.51668826020,120.38636567293,13.9998,13.1095,-12.9404,-0.0070,2.093819658,2.854168081,316.626703726,INS_SOLUTION_GOOD*8fc5adb1
			*/
			//if (!strstr(val[9], "INS_ALIGNMENT_COMPLETE") && !strstr(val[9], "INS_SOLUTION_GOOD")) continue;
			int wn = atoi(val[1]);
			double ws = atof(val[3]);
			double blh[3] = { atof(val[4]), atof(val[5]), atof(val[6]) };
			if (blh[0] == 0.0 && blh[1] == 0.0 && blh[2] == 0.0) continue;
			double vel_NEU[3] = { atof(val[7]), atof(val[8]), -atof(val[9]) };
			double att[3] = { atof(val[10]), atof(val[11]), atof(val[12]) };

			if (fins != NULL)
			{
				fprintf(fins, "%4i,%10.3f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f\n"
					, wn, ws
					, blh[0], blh[1], blh[2]
					, vel_NEU[0], vel_NEU[1], vel_NEU[2]
					, att[0], att[1], att[2]
				);
			}

			//int solType = -1;
			//if (strstr(val[10], "INS_PSRSP") != NULL) solType = 1;
			//else if (strstr(val[10], "INS_PSRDIFF") != NULL) solType = 2;
			//else if (strstr(val[10], "PSRDIFF") != NULL) solType = 2;
			//else if (strstr(val[10], "PROPAGATED") != NULL) solType = 3;
			//else if (strstr(val[10], "INS_RTKFLOAT") != NULL) solType = 5;
			//else if (strstr(val[10], "INS_RTKFIXED") != NULL) solType = 4;
			//else {
			//	solType = 1;
			//	//printf("not supported\n");
			//}
			/*strcpy(sol_status + strlen(sol_status), ",");
			strcpy(sol_status + strlen(sol_status), val[10]);
*/
			float heading = 0.0f;
			strcpy(sol_status, val[13]);
#ifdef INSPVA_SHORT
			print_kml_gga(fkml, blh[0], blh[1], blh[2], solType, ws, heading, sol_status);
#endif			
			continue;
		}
	}

	if (fkml != NULL) print_kml_eof(fkml);

	if (fdat != NULL) fclose(fdat);
	if (fgga != NULL) fclose(fgga);
	if (fpos != NULL) fclose(fpos);
	if (fgps != NULL) fclose(fgps);
	if (fimu != NULL) fclose(fimu);
	if (fins != NULL) fclose(fins);
	if (fgga_ins != NULL) fclose(fgga_ins);
	if (fhdg != NULL) fclose(fhdg);
	if (fkml != NULL) {
		fclose(fkml);

		if (isKMZ == 1)
		{
			/* zip kml to get kmz */
			char** paras = new char*[3];
			for (int i = 0; i < 3; i++) {
				paras[i] = new char[255];
			}
			strcpy(paras[0], "./minizipdll");
			sprintf(paras[1], "%s.kmz", fileName);
			strcpy(paras[2], out_kml_fpath);

			minizip(3, paras);
		}
	}

	return;
}

/* use SPAN CPT7 solutio as reference */
bool diff_test(const char* fname_sol, const char* fname_span)
{
	FILE* fpsol = NULL, * fpspan = NULL, * fpdiff = NULL;

	char fileName[255] = { 0 };
	char outfilename[255] = { 0 };
	char buffer[1024] = { 0 };

	fpsol = fopen(fname_sol, "r");
	fpspan = fopen(fname_span, "r");

	if (fpsol == NULL || fpspan == NULL) return false;

	strncpy(fileName, fname_sol, strlen(fname_sol));
	char* result1 = strrchr(fileName, '.');
	if (result1 != NULL) result1[0] = '\0';

	sprintf(outfilename, "%s.diff", fileName);
	fpdiff = fopen(outfilename, "w");

	char* val[MAXFIELD];
	//char* valspan[MAXFIELD];
	double gpstow_sol = 0.0, gpstow_span = 0.0;
	ins_pva sol_pva = { 0 }, span_pva = { 0 };

	int idxpos = 0, idxvel = 0, idxatt = 0;

	double ep[6] = { 0.0 };
	ep[0] = 2019;
	ep[1] = 11;
	ep[2] = 8;
	double posdata[10] = { 0.0 };
	double posdata2[20] = { 0.0 };

	while (!feof(fpsol))
	{
		memset(buffer, 0, sizeof(buffer));
		//memset(val, 0, sizeof(val));*/

		fgets(buffer, sizeof(buffer), fpsol);
		if (strlen(buffer) <= 0||strstr(buffer, "$GPGGA")==NULL) continue;

		parse_fields(buffer, val);

		std::string temp = val[1];

		ep[3] = atof(temp.substr(0, 2).c_str());
		ep[4] = atof(temp.substr(2, 2).c_str());
		ep[5] = atof(temp.substr(4).c_str())+18.0;

		gtime_t time = epoch2time(ep);

		int wk = 0;

		posdata[0] = time2gpst(time, &wk);

		temp = val[2]; // lat

		ep[3] = atof(temp.substr(0, 2).c_str());
		ep[4] = atof(temp.substr(2).c_str());

		posdata[1] = (ep[3] + ep[4] / 60.0) * PI / 180.0;

		temp = val[3]; // N

		temp = val[4]; // lon

		ep[3] = atof(temp.substr(0, 3).c_str());
		ep[4] = atof(temp.substr(3).c_str());
		posdata[2] =-(ep[3] + ep[4] / 60.0) * PI / 180.0;

		temp = val[5]; // W

		posdata[4] = atof(val[6]); // type

		if (posdata[4] == 1.0) continue;

		posdata[3] = atof(val[9]); // ht

		while (!feof(fpspan))
		{
			if (posdata[0] < (posdata2[0] - 0.001)) break;
			if (fabs(posdata[0] - posdata2[0]) < 0.01)
			{
				break;
			}
			memset(buffer, 0, sizeof(buffer));

			fgets(buffer, sizeof(buffer), fpspan);
			if (strlen(buffer) <= 0) continue;

			int type = 0;
			int fixID = 0;
			int num = sscanf(buffer, "%i,%lf,%i,%lf,%lf,%lf,%lf,%lf,%lf,%i,%lf,%lf,%lf,%lf",
				&wk, posdata2 + 0, &type, posdata2 + 1, posdata2 + 2, posdata2 + 3
				, posdata2 + 4, posdata2 + 5, posdata2 + 6, &fixID
				, posdata2 + 7, posdata2 + 8, posdata2 + 9, posdata2 + 10);

			posdata2[1] *= PI / 180.0;
			posdata2[2] *= PI / 180.0;
			posdata2[11] = fixID;
		}

		if (fabs(posdata[0] - posdata2[0]) < 0.01 && posdata2[11]==4)
		{
			double xyz1[3] = { 0.0 };
			double xyz2[3] = { 0.0 };
			pos2ecef(posdata + 1, xyz1);
			pos2ecef(posdata2 + 1, xyz2);
			double diffxyz[3] = { xyz2[0] - xyz1[0], xyz2[1] - xyz1[1],xyz2[2] - xyz1[2] };
			double diffenu[3] = { 0.0 };
			ecef2enu(posdata2 + 1, diffxyz, diffenu);
			fprintf(fpdiff, "%10.3f,%10.3f,%10.3f,%10.3f,%3i,%3i\n", posdata[0], diffenu[0], diffenu[1], diffenu[2], int(posdata[4]), (int)posdata2[11]);
		}
	}

	if (fpsol != NULL) fclose(fpsol);
	if (fpspan != NULL) fclose(fpspan);
	if (fpdiff != NULL) fclose(fpdiff);

	return true;
}


/* use SPAN CPT7 solutio as reference */
bool diff_with_span(const char *fname_sol, const char *fname_span)
{
	FILE* fpsol = NULL, * fpspan = NULL, *fpdiff = NULL;

	char fileName[255] = { 0 };
	char outfilename[255] = { 0 };
	char buffer[1024] = { 0 };

	fpsol = fopen(fname_sol, "r");
	fpspan = fopen(fname_span, "r");

	if (fpsol == NULL || fpspan == NULL) return false;

	strncpy(fileName, fname_sol, strlen(fname_sol));
	char* result1 = strrchr(fileName, '.');
	if (result1 != NULL) result1[0] = '\0';

	sprintf(outfilename, "%s.diff", fileName); 
	fpdiff = fopen(outfilename, "w");

	char* val[MAXFIELD];
	//char* valspan[MAXFIELD];
	double gpstow_sol= 0.0, gpstow_span = 0.0;
	ins_pva sol_pva = { 0 }, span_pva = { 0 };

	int idxpos = 0, idxvel = 0, idxatt = 0;

	while (!feof(fpsol))
	{
		memset(buffer, 0, sizeof(buffer));
		//memset(val, 0, sizeof(val));*/

		fgets(buffer, sizeof(buffer), fpsol);
		if (strlen(buffer) <= 0) continue; 

		parse_fields(buffer, val);

		if (!strstr(val[0], "#INSPVAXA")) continue;

		if (!strstr(val[9], "INS_ALIGNMENT_COMPLETE") && !strstr(val[9], "INS_SOLUTION_GOOD")) continue;
		gpstow_sol = atof(val[6]);

		idxpos = 11, idxvel = 15, idxatt = 18;
		sol_pva.gps_tow = gpstow_sol;
		sol_pva.lat = atof(val[idxpos]); sol_pva.lon = atof(val[idxpos + 1]); sol_pva.hgt = atof(val[idxpos + 2]) + atof(val[idxpos + 3]);
		sol_pva.vn = atof(val[idxvel]); sol_pva.ve = atof(val[idxvel + 1]); sol_pva.vu = atof(val[idxvel + 2]);
		sol_pva.roll = atof(val[idxatt]); sol_pva.pitch = atof(val[idxatt + 1]); sol_pva.azimuth = atof(val[idxatt + 2]);


		if (gpstow_span < gpstow_sol) {

			do {
				memset(buffer, 0, sizeof(buffer));
				//memset(val, 0, sizeof(val));*/

				fgets(buffer, sizeof(buffer), fpspan);
				if (strlen(buffer) <= 0) continue;

				parse_fields(buffer, val);

				if (!strstr(val[9], "INS_ALIGNMENT_COMPLETE") && !strstr(val[9], "INS_SOLUTION_GOOD")) continue;
				gpstow_span = atof(val[6]);

				idxpos = 11; idxvel = 15; idxatt = 18;
				span_pva.gps_tow = gpstow_span;
				span_pva.lat = atof(val[idxpos]); span_pva.lon = atof(val[idxpos + 1]); span_pva.hgt = atof(val[idxpos + 2]) + atof(val[idxpos + 3]);
				span_pva.vn = atof(val[idxvel]); span_pva.ve = atof(val[idxvel + 1]); span_pva.vu = atof(val[idxvel + 2]);
				span_pva.roll = atof(val[idxatt]); span_pva.pitch = atof(val[idxatt + 1]); span_pva.azimuth = atof(val[idxatt + 2]);

				if (gpstow_span >= gpstow_sol) {
					break;
				}

			} while (!feof(fpspan));
		}
		
		if (fabs(gpstow_span - gpstow_sol) < 0.005) {
			printf("sol time: %f, span time: %f\n", gpstow_sol, gpstow_span);

			fprintf(fpdiff, "%.3f,%lf,%lf,%f,%f,%f,%f,%f,%f,%f,", gpstow_sol,
				sol_pva.lat, sol_pva.lon, sol_pva.hgt,
				sol_pva.vn, sol_pva.ve, sol_pva.vu,
				sol_pva.roll, sol_pva.pitch, sol_pva.azimuth);

			fprintf(fpdiff, "%lf,%lf,%f,%f,%f,%f,%f,%f,%f,",
				span_pva.lat, span_pva.lon, span_pva.hgt,
				span_pva.vn, span_pva.ve, span_pva.vu,
				span_pva.roll, span_pva.pitch, span_pva.azimuth);

			fprintf(fpdiff, "%.9f,%.9f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
				sol_pva.lat - span_pva.lat, sol_pva.lon - span_pva.lon, sol_pva.hgt- span_pva.hgt,
				sol_pva.vn - span_pva.vn, sol_pva.ve - span_pva.ve, sol_pva.vu - span_pva.vu,
				sol_pva.roll - span_pva.roll, sol_pva.pitch - span_pva.pitch, sol_pva.azimuth - span_pva.azimuth);
		}
	}

	if (fpsol != NULL) fclose(fpsol);
	if (fpspan != NULL) fclose(fpspan);
	if (fpdiff != NULL) fclose(fpdiff);

	return true;
}

int main()
{
	decode_span("C:\\Users\\da\\Documents\\049\\2\\novatel_CPT7-2020_02_18_17_19_51.ASC", SPAN_CPT7, 100.0, 0);

	//diff_test("C:\\aceinna\\span_decoder\\2019-11-08-15-53-17.log", "C:\\aceinna\\span_decoder\\novatel_CPT7-2019_11_08_15_48_34-pos.csv");
	//decode_span("C:\\Users\\da\\Documents\\290\\span\\halfmoon\\novatel_FLX6-2019_10_16_20_32_44.ASC");
	//decode_span("C:\\Users\\da\\Documents\\312\\openrtk\\CompNovA\\novatel_CPT7-2019_11_08_15_48_34.ASC", SPAN_CPT7, 100.0, 0);
	//decode_span("C:\\femtomes\\Rover.log", SPAN_ACEINNA, 1.0, 0);
	//diff_with_span("C:\\Users\\da\\Documents\\290\\span\\halfmoon\\novatel_FLX6-2019_10_16_20_32_44.ASC","C:\\Users\\da\\Documents\\290\\span\\halfmoon\\novatel_CPT7-2019_10_16_20_31_52.ASC");
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
