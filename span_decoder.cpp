// span_decoder.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>

#include "kml.h"

#define MAXFIELD 100

void decode_span(const char* fname)
{
	FILE* fdat = NULL;
	FILE* fgga = NULL;
	FILE* fgps = NULL;
	FILE* fimu = NULL;
	FILE* fkml = NULL;

	char fileName[255] = { 0 };
	char outfilename[255] = { 0 };

	fdat = fopen(fname, "r"); if (fdat == NULL) return;

	strncpy(fileName, fname, strlen(fname));
	char* result1 = strrchr(fileName, '.');
	if (result1 != NULL) result1[0] = '\0';

	sprintf(outfilename, "%s.gga", fileName); fgga = fopen(outfilename, "w");
	sprintf(outfilename, "%s.gps", fileName); fgps = fopen(outfilename, "w");
	sprintf(outfilename, "%s.imu", fileName); fimu = fopen(outfilename, "w");
	sprintf(outfilename, "%s.kml", fileName); fkml = fopen(outfilename, "w");

	if (fkml!=NULL) print_kml_heder(fkml);

	int type = 0;
	int wn = 0;

	char buffer[1024] = { 0 };

	char* p, * q, * val[MAXFIELD];

	while (!feof(fdat))
	{
		fgets(buffer, sizeof(buffer), fdat);
		if (strlen(buffer) <= 0) continue;

		int n = 0;

		/* parse fields */
		for (p = buffer; *p && n < MAXFIELD; p = q + 1) {
			if ((q = strchr(p, ',')) || (q = strchr(p, '*'))) {
				val[n++] = p; *q = '\0';
			}
			else break;
		}

		if (strstr(val[0], "#BESTGNSSPOSA") != NULL)
		{
			continue;
		}
		if (strstr(val[0], "#INSPVAXA") != NULL)
		{
			double blh[3] = { atof(val[11]), atof(val[12]), atof(val[13]) };

			int solType = 1;
			if (strstr(val[10], "INS_RTKFLOAT") != NULL) solType = 5;
			else if (strstr(val[10], "INS_RTKFIXED") != NULL) solType = 4;

			double time = atof(val[6]);
			float heading = atof(val[20]);

			print_kml_gga(fkml, blh[0], blh[1], blh[2], solType, time, heading);
			continue;
		}
	}

	if (fkml != NULL) print_kml_eof(fkml);

	if (fdat != NULL) fclose(fdat);
	if (fkml != NULL) fclose(fkml);
	if (fgga != NULL) fclose(fgga);
	if (fgps != NULL) fclose(fgps);
	if (fimu != NULL) fclose(fimu);
	return;
}

int main()
{
	decode_span("C:\\Users\\da\\Documents\\288\\span\\novatel_CPT7-2019_10_14_13_46_37.ASC");
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
