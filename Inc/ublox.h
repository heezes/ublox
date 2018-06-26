#ifndef _UBLOX_H_
#define _UBLOX_H_

#include "stm32f4xx_hal.h"
#include "main.h"

/*USER DEFINE DATA STRUCTURE*/
struct ublox_nav_data
{
	unsigned long iTow; // (ms)
	unsigned short year; // (year)
	unsigned char month; // {1-12}
	unsigned char day; // {1-31}
	unsigned char hour; // {0-23}
	unsigned char min;  // {0-59}
	unsigned char sec; // {0-60}
/*	________________________
    |__|__|__|__|__|__|__|__|
    		 	 	FR VT Vd
    FR: fullyResolved (1 for True)
    VT: valid UTC Time (1 for True)
    VD: valid UTD Date (1 for True)*/
	uint8_t valid;
	unsigned long tAcc; // {Time accuracy -ns}
	signed long nano; // {Fraction of Second range(-1e9~1e9) ns}
	/*
	 * 0: NO fix
	 * 1: Dead Reckoning Only
	 * 2: 2D-fix
	 * 3: 3D-fix
	 * 4: GNSS+Dead Reckoning combined
	 * 5: Time Only fix
	 * */
	unsigned char fixType;
	/*	________________________
	    |__|__|__|__|__|__|__|__|
	    CPS   HVV PSM      DS gFO

	    gFO: GNSS Fix OK  (1 = valid fix)
	    DS: differential Solution (1 = differential correction applied)
	    PSM: (Power Save Mode)
	    0: PSM is not active
	    1: Enabled
	    2: Acquisition
	    3: Tracking
	    4: Power Optimized Tracking
	    5: Inactive
	    HVV: (1=heading of Vehicle is valid)
	    CPS: Carrier Phase range Solution
	    0: No carrier phase range solution
	    1: float solution
	    2: fixed solution
*/
	uint8_t flags;
	/*	________________________
	    |__|__|__|__|__|__|__|__|
	     CT CD CA
		CA: 1=Information abt UTC Date,Time of Day validity is available
		CD: 1=UTC Time of Day could be confirmed
		CT: 1=UTC Time of Day could be confirmed
*/
	uint8_t flags2;
	unsigned char numSv; // {No. of Satellites used in Navigation}
	signed long lon; // {Longitutde (deg) Scaling:1e-7}
	signed long lat; // {Latitude (deg) Scaling:1e-7}
	signed long height; // {Height Above Ellipsoid mm}
	signed long hMSL; // {Height Above mean sea Level mm}
	unsigned long hAcc; // {Horizontal accuracy estimate mm}
	unsigned long vAcc; // {vertical Accuracy Estimate mm}
	signed long velN; // {NED North velocity mm/s}
	signed long velE; // {NED east velocity mm/s}
	signed long velD; // {NED Down Velocity mm/s}
	signed long gSpeed; // {Ground Speed (2-D) mm/s}
	signed long headMot; // {Heading of Motion (2-D) deg}
	unsigned long sAcc; // {Speed Accuracy estimate mm/s}
	unsigned long headAcc; // {Heading accuraccy estimate deg}
	unsigned short pDOP; // {Position PDOP}
	signed long headVeh; // {Heading of Vehicle (2-D) deg}
};
struct ublox_pos_data
{
	unsigned long i;
	signed long longitude;
	signed long latitude;
	signed long height;
	signed long hmsl;
	unsigned long hacc;
	unsigned long vacc;
	float lat;
	float lon;
};
/* USER DEFINE */
#define UBX_CFG_CFG 0x06,0x09
/* USER DEFINE FUCNTIONS */
int read_packet(uint8_t* pkt, int len, int timeout);
int send_packet(char* pkt, int len, int timeout);
int disableNmea(void);
int set_rate(void);
int PERIODIC_UBX_NAV_PVT(void);
int POLL_UBX_NAV_POSLLH(struct ublox_pos_data* pos_data);
int POLL_UBX_NAV_PVT(struct ublox_nav_data *data);
int UBX_AID_INI(void);
int reset(void);
#endif
