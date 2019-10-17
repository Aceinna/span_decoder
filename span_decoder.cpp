// span_decoder.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#include "kml.h"

#include "minizipdll.h"
#pragma comment(lib,"MINIZIPDLL.lib")

#define MAXFIELD 100

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

void decode_span(const char* fname)
{
	FILE* fdat = NULL;
	FILE* fgga = NULL;
	FILE* fgps = NULL;
	FILE* fimu = NULL;
	FILE* fkml = NULL;

	char fileName[255] = { 0 };
	char outfilename[255] = { 0 };
	char out_kml_fpath[255] = { 0 };

	fdat = fopen(fname, "r"); if (fdat == NULL) return;

	strncpy(fileName, fname, strlen(fname));
	char* result1 = strrchr(fileName, '.');
	if (result1 != NULL) result1[0] = '\0';

	sprintf(outfilename, "%s.gga", fileName); fgga = fopen(outfilename, "w");
	sprintf(outfilename, "%s.gps", fileName); fgps = fopen(outfilename, "w");
	sprintf(outfilename, "%s.imu", fileName); fimu = fopen(outfilename, "w");
	sprintf(out_kml_fpath, "%s.kml", fileName); fkml = fopen(out_kml_fpath, "w");

	if (fkml!=NULL) print_kml_heder(fkml);

	int type = 0;
	int wn = 0;

	char buffer[1024] = { 0 }, sol_status[56] = {0};

	char* p, * q, * val[MAXFIELD];

	while (!feof(fdat))
	{
		fgets(buffer, sizeof(buffer), fdat);
		if (strlen(buffer) <= 0) continue;

		parse_fields(buffer, val);

		if (strstr(val[0], "#BESTGNSSPOSA") != NULL)
		{
			continue;
		}
		if (strstr(val[0], "#INSPVAXA") != NULL)
		{
			//if (!strstr(val[9], "INS_ALIGNMENT_COMPLETE") && !strstr(val[9], "INS_SOLUTION_GOOD")) continue;

			double blh[3] = { atof(val[11]), atof(val[12]), atof(val[13]) };
			if (blh[0] * blh[1] * blh[2] == 0.0) continue;

			strcpy(sol_status, strchr(val[9], ';')+1);

			int solType = -1;
			if (strstr(val[10], "INS_PSRSP") != NULL) solType = 1;
			else if (strstr(val[10], "INS_PSRDIFF") != NULL) solType = 2;
			else if (strstr(val[10], "PSRDIFF") != NULL) solType = 2;
			else if (strstr(val[10], "PROPAGATED") != NULL) solType = 3;
			else if (strstr(val[10], "INS_RTKFLOAT") != NULL) solType = 5;
			else if (strstr(val[10], "INS_RTKFIXED") != NULL) solType = 4;
			else {
				solType = 1;
				printf("not supported\n");
			}
			strcpy(sol_status+strlen(sol_status), ",");
			strcpy(sol_status+strlen(sol_status), val[10]);

			double time = atof(val[6]);
			printf("GPS TOW: %f\n", time);
			float heading = atof(val[20]);

			print_kml_gga(fkml, blh[0], blh[1], blh[2], solType, time, heading, sol_status);
			continue;
		}
	}

	if (fkml != NULL) print_kml_eof(fkml);

	if (fdat != NULL) fclose(fdat);
	if (fgga != NULL) fclose(fgga);
	if (fgps != NULL) fclose(fgps);
	if (fimu != NULL) fclose(fimu);
	if (fkml != NULL) {
		fclose(fkml);

		/* zip kml to get kmz */
		char** paras = new char* [3];
		for (int i = 0; i < 3; i++) {
			paras[i] = new char[255];
		}
		strcpy(paras[0], "./minizipdll");
		sprintf(paras[1], "%s.kmz", fileName);
		strcpy(paras[2], out_kml_fpath);

		minizip(3, paras);
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
	bool res;
	decode_span("C:\\Users\\da\\Documents\\290\\span\\halfmoon\\novatel_FLX6-2019_10_16_20_32_44.ASC");
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
