#include "include/rtktime.h"

#pragma warning(disable:4996)

/* string to number ------------------------------------------------------------
* convert substring in string to number
* args   : char   *s        I   string ("... nnn.nnn ...")
*          int    i,n       I   substring position and width
* return : converted number (0.0:error)
*-----------------------------------------------------------------------------*/
extern double str2num(const char *s, int i, int n)
{
	double value;
	char str[256], *p = str;

	if (i < 0 || (int)strlen(s) < i || (int)sizeof(str) - 1 < n) return 0.0;
	for (s += i; *s&&--n >= 0; s++) *p++ = *s == 'd' || *s == 'D' ? 'E' : *s; *p = '\0';
	return sscanf(str, "%lf", &value) == 1 ? value : 0.0;
}
/* string to time --------------------------------------------------------------
* convert substring in string to gtime_t struct
* args   : char   *s        I   string ("... yyyy mm dd hh mm ss ...")
*          int    i,n       I   substring position and width
*          gtime_t *t       O   gtime_t struct
* return : status (0:ok,0>:error)
*-----------------------------------------------------------------------------*/
extern int str2time(const char *s, int i, int n, gtime_t *t)
{
	double ep[6];
	char str[256], *p = str;

	if (i < 0 || (int)strlen(s) < i || (int)sizeof(str) - 1 < i) return -1;
	for (s += i; *s&&--n >= 0;) *p++ = *s++; *p = '\0';
	if (sscanf(str, "%lf %lf %lf %lf %lf %lf", ep, ep + 1, ep + 2, ep + 3, ep + 4, ep + 5) < 6)
		return -1;
	if (ep[0] < 100.0) ep[0] += ep[0] < 80.0 ? 2000.0 : 1900.0;
	*t = epoch2time(ep);
	return 0;
}
/* convert calendar day/time to time -------------------------------------------
* convert calendar day/time to gtime_t struct
* args   : double *ep       I   day/time {year,month,day,hour,min,sec}
* return : gtime_t struct
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern gtime_t epoch2time(const double *ep)
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
/* time to calendar day/time ---------------------------------------------------
* convert gtime_t struct to calendar day/time
* args   : gtime_t t        I   gtime_t struct
*          double *ep       O   day/time {year,month,day,hour,min,sec}
* return : none
* notes  : proper in 1970-2037 or 1970-2099 (64bit time_t)
*-----------------------------------------------------------------------------*/
extern void time2epoch(gtime_t t, double *ep)
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
/* gps time to time ------------------------------------------------------------
* convert week and tow in gps time to gtime_t struct
* args   : int    week      I   week number in gps time
*          double sec       I   time of week in gps time (s)
* return : gtime_t struct
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2time(int week, double sec)
{
	gtime_t t = epoch2time(gpst0);

	if (sec < -1E9 || 1E9 < sec) sec = 0.0;
	t.time += 86400 * 7 * week + (int)sec;
	t.sec = sec - (int)sec;
	return t;
}
/* time to gps time ------------------------------------------------------------
* convert gtime_t struct to week and tow in gps time
* args   : gtime_t t        I   gtime_t struct
*          int    *week     IO  week number in gps time (NULL: no output)
* return : time of week in gps time (s)
*-----------------------------------------------------------------------------*/
extern double time2gpst(gtime_t t, int *week)
{
	gtime_t t0 = epoch2time(gpst0);
	time_t sec = t.time - t0.time;
	int w = (int)(sec / (86400 * 7));

	if (week) *week = w;
	return (double)(sec - w * 86400 * 7) + t.sec;
}
/* add time --------------------------------------------------------------------
* add time to gtime_t struct
* args   : gtime_t t        I   gtime_t struct
*          double sec       I   time to add (s)
* return : gtime_t struct (t+sec)
*-----------------------------------------------------------------------------*/
extern gtime_t timeadd(gtime_t t, double sec)
{
	double tt;

	t.sec += sec; tt = floor(t.sec); t.time += (int)tt; t.sec -= tt;
	return t;
}
/* time difference -------------------------------------------------------------
* difference between gtime_t structs
* args   : gtime_t t1,t2    I   gtime_t structs
* return : time difference (t1-t2) (s)
*-----------------------------------------------------------------------------*/
extern double timediff(gtime_t t1, gtime_t t2)
{
	return difftime(t1.time, t2.time) + t1.sec - t2.sec;
}
/* gpstime to utc --------------------------------------------------------------
* convert gpstime to utc considering leap seconds
* args   : gtime_t t        I   time expressed in gpstime
* return : time expressed in utc
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t gpst2utc(gtime_t t)
{
	gtime_t tu;
	int i;

	for (i = 0; leaps[i][0] > 0; i++) {
		tu = timeadd(t, leaps[i][6]);
		if (timediff(tu, epoch2time(leaps[i])) >= 0.0) return tu;
	}
	return t;
}
/* utc to gpstime --------------------------------------------------------------
* convert utc to gpstime considering leap seconds
* args   : gtime_t t        I   time expressed in utc
* return : time expressed in gpstime
* notes  : ignore slight time offset under 100 ns
*-----------------------------------------------------------------------------*/
extern gtime_t utc2gpst(gtime_t t)
{
	int i;

	for (i = 0; leaps[i][0] > 0; i++) {
		if (timediff(t, epoch2time(leaps[i])) >= 0.0) return timeadd(t, -leaps[i][6]);
	}
	return t;
}
/* time to day and sec -------------------------------------------------------*/
static double time2sec(gtime_t time, gtime_t *day)
{
	double ep[6], sec;
	time2epoch(time, ep);
	sec = ep[3] * 3600.0 + ep[4] * 60.0 + ep[5];
	ep[3] = ep[4] = ep[5] = 0.0;
	*day = epoch2time(ep);
	return sec;
}
/* time to string --------------------------------------------------------------
* convert gtime_t struct to string
* args   : gtime_t t        I   gtime_t struct
*          char   *s        O   string ("yyyy/mm/dd hh:mm:ss.ssss")
*          int    n         I   number of decimals
* return : none
*-----------------------------------------------------------------------------*/
extern void time2str(gtime_t t, char *s, int n)
{
	double ep[6];

	if (n < 0) n = 0; else if (n > 12) n = 12;
	if (1.0 - t.sec < 0.5 / pow(10.0, n)) { t.time++; t.sec = 0.0; };
	time2epoch(t, ep);
	sprintf(s, "%04.0f/%02.0f/%02.0f %02.0f:%02.0f:%0*.*f", ep[0], ep[1], ep[2],
		ep[3], ep[4], n <= 0 ? 2 : n + 3, n <= 0 ? 0 : n, ep[5]);
}
/* get time string -------------------------------------------------------------
* get time string
* args   : gtime_t t        I   gtime_t struct
*          int    n         I   number of decimals
* return : time string
* notes  : not reentrant, do not use multiple in a function
*-----------------------------------------------------------------------------*/
extern char *time_str(gtime_t t, int n)
{
	static char buff[64];
	time2str(t, buff, n);
	return buff;
}
/* time to day of year ---------------------------------------------------------
* convert time to day of year
* args   : gtime_t t        I   gtime_t struct
* return : day of year (days)
*-----------------------------------------------------------------------------*/
extern double time2doy(gtime_t t)
{
	double ep[6];

	time2epoch(t, ep);
	ep[1] = ep[2] = 1.0; ep[3] = ep[4] = ep[5] = 0.0;
	return timediff(t, epoch2time(ep)) / 86400.0 + 1.0;
}