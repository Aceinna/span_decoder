#pragma once
#include <iostream>

void print_kml_heder(FILE *fKML);
void print_kml_gga(FILE* fKML, double lat, double lon, double ht, int solType, double time);
void print_kml_eof(FILE *fKML);