#include "kml.h"

void print_kml_heder(FILE *fKML)
{
	// write header for KML 
	if (fKML) {
		fprintf(fKML, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		fprintf(fKML, "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n");
		fprintf(fKML, "<Document>\n");
		// fprintf(fKML, "<Placemark>\n");    
		// fprintf(fKML, "<name>extruded</name>\n");
		// fprintf(fKML, "<LineString>\n");
		// fprintf(fKML, "<extrude>1</extrude>\n");
		// fprintf(fKML, "<tessellate>1</tessellate>\n");
		// fprintf(fKML, "<altitudeMode>relativeToGround</altitudeMode>\n");
		// fprintf(fKML, "<coordinates>\n"); 
		fprintf(fKML, "<Style id=\"spp\">\n");
		fprintf(fKML, "<IconStyle>\n");
		fprintf(fKML, "<color>ffff00ff</color>\n");
		fprintf(fKML, "<scale>0.300</scale>\n");
		fprintf(fKML, "<Icon>\n");
		fprintf(fKML, "<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n");
		fprintf(fKML, "</Icon>\n");
		fprintf(fKML, "</IconStyle>\n");
		fprintf(fKML, "</Style>\n");
		fprintf(fKML, "<Style id=\"fix\">\n");
		fprintf(fKML, "<IconStyle>\n");
		fprintf(fKML, "<color>ff008800</color>\n");
		fprintf(fKML, "<scale>0.300</scale>\n");
		fprintf(fKML, "<Icon>\n");
		fprintf(fKML, "<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n");
		fprintf(fKML, "</Icon>\n");
		fprintf(fKML, "</IconStyle>\n");
		fprintf(fKML, "</Style>\n");
		fprintf(fKML, "<Style id=\"flt\">\n");
		fprintf(fKML, "<IconStyle>\n");
		fprintf(fKML, "<color>ff00aaff</color>\n");
		fprintf(fKML, "<scale>0.300</scale>\n");
		fprintf(fKML, "<Icon>\n");
		fprintf(fKML, "<href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href>\n");
		fprintf(fKML, "</Icon>\n");
		fprintf(fKML, "</IconStyle>\n");
		fprintf(fKML, "</Style>\n");
	}
	return;
}

void print_kml_gga(FILE *fKML, double lat, double lon, double ht, int solType, double time, float heading)
{
	int day = 0;
	int hh = 0;
	int mm = 0;
	if (fKML == NULL) return;
	if (fKML) {
		day = (int)floor(time / (3600 * 24));
		time -= day * 24 * 3600;
		hh = (int)floor(time / 3600);
		time -= hh * 3600;
		mm = (int)floor(time / 60);
		time -= mm * 60;
		fprintf(fKML, "<Placemark>\n");
		if (solType == 1)
		{
			fprintf(fKML, "<styleUrl>#spp</styleUrl>\n");
		}
		else if (solType == 4)
		{
			fprintf(fKML, "<styleUrl>#fix</styleUrl>\n");
		}
		else if (solType == 5)
		{
			fprintf(fKML, "<styleUrl>#flt</styleUrl>\n");
		}
		else
		{
			fprintf(fKML, "<styleUrl>#spp</styleUrl>\n");
		}
		fprintf(fKML, "<Style>\n");
		fprintf(fKML, "<IconStyle>\n");
		fprintf(fKML, "<heading>%f</heading>\n", heading);
		fprintf(fKML, "</IconStyle>\n");
		fprintf(fKML, "</Style>\n");
		fprintf(fKML, "<ExtendedData>\n");
		fprintf(fKML, "<Data name=\"time\">\n");
		fprintf(fKML, "<value>%02i:%02i:%5.2f</value>\n", hh, mm, time);
		fprintf(fKML, "</Data>\n");
		fprintf(fKML, "<Data name=\"heading\">\n");
		fprintf(fKML, "<value>%.2f</value>\n", heading);
		fprintf(fKML, "</Data>\n");
		fprintf(fKML, "</ExtendedData>\n");
		fprintf(fKML, "<Point>\n");
		fprintf(fKML, "<coordinates>%14.9f,%14.9f,%14.4f</coordinates>\n", lon, lat, ht);
		fprintf(fKML, "</Point>\n");
		fprintf(fKML, "</Placemark>\n");
	}
	return;
}

void print_kml_eof(FILE *fKML)
{
	if (fKML)
	{
		// fprintf(fKML, "</coordinates>\n");    
		// fprintf(fKML, "</LineString>\n");
		// fprintf(fKML, "</Placemark>\n");
		fprintf(fKML, "</Document>\n");
		fprintf(fKML, "</kml>\n");

	}
}
