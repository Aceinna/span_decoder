#include "rtklib.h"

static int default_glo_frq_table[30] = { 1, -4, 05, 06, 01, -4, 05, 06, -2, -7, 00, -1, -2, -7, 00, -1, 04, -3, 03, 02, 04, -3, 03, 02, 0, -5, -99, -99, -99, -99 };

const double lam_carr[] = {       /* carrier wave length (m) */
	CLIGHT / FREQ1,CLIGHT / FREQ2,CLIGHT / FREQ5,CLIGHT / FREQ6,CLIGHT / FREQ7,CLIGHT / FREQ8
};
/* satellite system+prn/slot number to satellite number ------------------------
* convert satellite system+prn/slot number to satellite number
* args   : int    sys       I   satellite system (SYS_GPS,SYS_GLO,...)
*          int    prn       I   satellite prn/slot number
* return : satellite number (0:error)
*-----------------------------------------------------------------------------*/
extern int satno(int sys, int prn)
{
	if (prn <= 0) return 0;
	switch (sys) {
	case SYS_GPS:
		if (prn < MINPRNGPS || MAXPRNGPS < prn) return 0;
		return prn - MINPRNGPS + 1;
	case SYS_GLO:
		if (prn < MINPRNGLO || MAXPRNGLO < prn) return 0;
		return NSATGPS + prn - MINPRNGLO + 1;
	case SYS_GAL:
		if (prn < MINPRNGAL || MAXPRNGAL < prn) return 0;
		return NSATGPS + NSATGLO + prn - MINPRNGAL + 1;
	case SYS_QZS:
		if (prn < MINPRNQZS || MAXPRNQZS < prn) return 0;
		return NSATGPS + NSATGLO + NSATGAL + prn - MINPRNQZS + 1;
	case SYS_CMP:
		if (prn < MINPRNCMP || MAXPRNCMP < prn) return 0;
		return NSATGPS + NSATGLO + NSATGAL + NSATQZS + prn - MINPRNCMP + 1;
	case SYS_LEO:
		if (prn < MINPRNLEO || MAXPRNLEO < prn) return 0;
		return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + prn - MINPRNLEO + 1;
	case SYS_SBS:
		if (prn < MINPRNSBS || MAXPRNSBS < prn) return 0;
		return NSATGPS + NSATGLO + NSATGAL + NSATQZS + NSATCMP + NSATLEO + prn - MINPRNSBS + 1;
	}
	return 0;
}

extern void set_glo_frq(unsigned char prn, int frq)
{
	int max_prn = sizeof(default_glo_frq_table) / sizeof(int);
	if (prn <= max_prn)
	{
		default_glo_frq_table[prn - 1] = frq;
	}
	return;
}
extern int get_glo_frq(unsigned char prn)
{
	int max_prn = sizeof(default_glo_frq_table) / sizeof(int);
	int frq = -99;
	if (prn <= max_prn)
	{
		frq = default_glo_frq_table[prn - 1];
	}
	return frq;
}
/* satellite number to satellite system ----------------------------------------
* convert satellite number to satellite system
* args   : int    sat       I   satellite number (1-MAXSAT)
*          int    *prn      IO  satellite prn/slot number (NULL: no output)
* return : satellite system (SYS_GPS,SYS_GLO,...)
*-----------------------------------------------------------------------------*/
extern int satsys(int sat, int *prn)
{
	int sys = SYS_NONE;
	if (sat <= 0 || MAXSAT < sat) sat = 0;
	else if (sat <= NSATGPS) {
		sys = SYS_GPS; sat += MINPRNGPS - 1;
	}
	else if ((sat -= NSATGPS) <= NSATGLO) {
		sys = SYS_GLO; sat += MINPRNGLO - 1;
	}
	else if ((sat -= NSATGLO) <= NSATGAL) {
		sys = SYS_GAL; sat += MINPRNGAL - 1;
	}
	else if ((sat -= NSATGAL) <= NSATQZS) {
		sys = SYS_QZS; sat += MINPRNQZS - 1;
	}
	else if ((sat -= NSATQZS) <= NSATCMP) {
		sys = SYS_CMP; sat += MINPRNCMP - 1;
	}
	else if ((sat -= NSATCMP) <= NSATLEO) {
		sys = SYS_LEO; sat += MINPRNLEO - 1;
	}
	else if ((sat -= NSATLEO) <= NSATSBS) {
		sys = SYS_SBS; sat += MINPRNSBS - 1;
	}
	else sat = 0;
	if (prn) *prn = sat;
	return sys;
}
extern double satwavelen(int sat, int frq)
{
	const double freq_glo[] = { FREQ1_GLO, FREQ2_GLO };
	const double dfrq_glo[] = { DFRQ1_GLO, DFRQ2_GLO };
	int prn = 0, sys = satsys(sat, &prn);
	int frqnum = get_glo_frq(prn);

	if (sys == SYS_GLO)
	{
		if (frqnum == -99)
			return 0.0;
		if (0 <= frq && frq <= 1)
		{
			return CLIGHT / (freq_glo[frq] + dfrq_glo[frq] * frqnum);
		}
		else if (frq == 2)
		{ /* L3 */
			return CLIGHT / FREQ3_GLO;
		}
	}
	else if (sys == SYS_CMP)
	{
		if (frq == 0)
			return CLIGHT / FREQ1_CMP; /* B1 */
		else if (frq == 1)
			return CLIGHT / FREQ3_CMP; /* B3 */
		else if (frq == 2)
			return CLIGHT / FREQ2_CMP; /* B2 */
	}
	else if (sys == SYS_GAL)
	{
		if (frq == 0)
			return CLIGHT / FREQ1; /* E1 */
		else if (frq == 1)
			return CLIGHT / FREQ7; /* E5b */
		else if (frq == 2)
			return CLIGHT / FREQ5; /* E5a */
		else if (frq == 3)
			return CLIGHT / FREQ8; /* E5a+b */
		else if (frq == 4)
			return CLIGHT / FREQ6; /* E6 */
	}
	else if (sys == SYS_QZS)
	{
		if (frq == 0)
			return CLIGHT / FREQ1; /* L1 */
		else if (frq == 1)
			return CLIGHT / FREQ2; /* L2 */
		else if (frq == 2)
			return CLIGHT / FREQ5; /* L5 */
		else if (frq == 3)
			return CLIGHT / FREQ6; /* LEX */
	}
	else if (sys == SYS_GPS)
	{
		if (frq == 0)
			return CLIGHT / FREQ1; /* L1 */
		else if (frq == 1)
			return CLIGHT / FREQ2; /* L2 */
		else if (frq == 2)
			return CLIGHT / FREQ5; /* L5 */
	}
	return 0.0;
}