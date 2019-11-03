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
	FILE* fgga = NULL;
	FILE* fpos = NULL;
	FILE* fgps = NULL;
	FILE* fimu = NULL;
	FILE* fins = NULL;
	FILE* fkml = NULL;

	char fileName[255] = { 0 };
	char outfilename[255] = { 0 };
	char out_kml_fpath[255] = { 0 };

	fdat = fopen(fname, "r"); if (fdat == NULL) return;

	strcpy(fileName, fname);
	char* result1 = strrchr(fileName, '.');
	if (result1 != NULL) result1[0] = '\0';

	sprintf(outfilename, "%s.gga", fileName); fgga = fopen(outfilename, "w");
	sprintf(outfilename, "%s-gps.bin", fileName); fgps = fopen(outfilename, "w");
	sprintf(outfilename, "%s-pos.csv", fileName); fpos = fopen(outfilename, "w");
	sprintf(outfilename, "%s-imu.csv", fileName); fimu = fopen(outfilename, "w");
	sprintf(outfilename, "%s-ins.csv", fileName); fins = fopen(outfilename, "w");
	sprintf(out_kml_fpath, "%s.kml", fileName); fkml = fopen(out_kml_fpath, "w");

	if (fkml!=NULL) print_kml_heder(fkml);

	int type = 0;
	int wn = 0;

	char buffer[1024*8] = { 0 }, sol_status[56] = {0};

	char* p, * q, * val[MAXFIELD];

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
			if (fpos != NULL) fprintf(fpos, "%4.0f,%10.3f,1,%14.10f,%14.10f,%10.4f,%10.4f,%10.4f,%10.4f,%3i\n", wn, ws, blh[0], blh[1], blh[2], rms[0], rms[1], rms[2], type);
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
		if (strstr(val[0], "HEADINGA") != NULL)
		{
			/*
#HEADINGA,USB1,0,73.3,FINESTEERING,2076,112838.000,00000000,000e,10558;SOL_COMPUTED,INS_RTKFIXED,6.5060,79.0343,2.7232,0.0000,0.0000,0.0000,"",25,11,10,0,0,2,10,11*c8340b21
			/* TODO, output to fpos
			*/
			continue;
		}
		if (strstr(val[0], "BESTGNSSVELA") != NULL)
		{
			/*
#BESTGNSSVELA,SPECIAL,0,45.0,COARSESTEERING,2075,507495.200,00400000,00b0,14392;SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,0.1949,334.029536,-0.0767,0.0*ed8abf8e
			*/
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

			strcpy(sol_status, strchr(val[9], ';')+1);

			if (fins != NULL)
			{
				fprintf(fins, "%4i,%10.3f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%14.9f,%14.9f,%14.9f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f,%10.4f\n", wn, ws
					, blh[0], blh[1], blh[2], vel_NEU[0], vel_NEU[1], vel_NEU[2], att[0], att[1], att[2]
					, rms_PosNEU[0], rms_PosNEU[1], rms_PosNEU[2], rms_VelNEU[0], rms_VelNEU[1], rms_VelNEU[2], rms_att[0], rms_att[1], rms_att[2]
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

			double time = atof(val[6]);
			//printf("GPS TOW: %f\n", time);
			float heading = atof(val[20]);

			print_kml_gga(fkml, blh[0], blh[1], blh[2], solType, time, heading, sol_status);
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
	//decode_span("C:\\Users\\da\\Documents\\290\\span\\halfmoon\\novatel_FLX6-2019_10_16_20_32_44.ASC");
	//decode_span("C:\\304\\attitude\\novatel_CPT7-2019_10_31_15_27_41.ASC", SPAN_CPT7, 100.0, 0);
	decode_span("C:\\femtomes\\Rover.log", SPAN_ACEINNA, 1.0, 0);
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
