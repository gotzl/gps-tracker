
#ifndef TEST

#include <Arduino.h>

#else

#include <stdlib.h>
#include <cstdio>
#include <math.h>

#define PI 3.1415926535897932384626433832795

typedef char uint8_t;
typedef short int16_t;
typedef unsigned short uint16_t;
typedef int int32_t;
typedef unsigned int uint32_t;

#endif

//------------------------------------------------------------

#define BEGIN_LFRAME 	0x80
#define BEGIN_TFRAME 	0xC0
#define BEGIN_GFRAME 	0xE0

struct tFrame {
	uint8_t seconds;    //!< 00-59
	uint8_t minutes;    //!< 00-59
	uint8_t hours;      //!< 00-23
	uint8_t day;        //!< 01-07 Day of Week
	uint8_t date;       //!< 01-31 Day of Month
	uint8_t month;      //!< 01-12
	uint8_t year;       //!< 00-99
};

struct gFrame {
	float speed;
	int32_t lon;
	int32_t lat;
	uint16_t bearing;
	uint8_t sats;
	uint8_t fix;
};

struct lFrame{
	float dist_start;
	uint32_t start_time;
	uint32_t finish_time;
	int32_t delta;
	uint32_t time;
	uint32_t pts;
	uint8_t lap;
	uint8_t stat;
};

//------------------------------------------------------------

struct _point
{
	uint32_t         time;
    int32_t       lat;  // degrees * 1e7, negative is South
    int32_t       lon;  // degrees * 1e7, negative is West
};

struct _track
{
	_point* track;
	uint32_t points;
};

//------------------------------------------------------------

int mod(int32_t a, int32_t b)
{
    int r = a % b;
    return r < 0 ? r + b : r;
}

//------------------------------------------------------------
float orientation(float heading, float bearing) {
	return mod(bearing - heading + 180, 360) - 180;
}

int32_t offsetCorrection(float heading, float distance, float speed, float bearing) {
	static int32_t delta, ori;

	// delta to travel from ref_loc to current loc
	delta = ((distance/speed)*60*60*1000);

	// orientation of the ref_loc with respect to direction of heading
	ori = orientation(heading, bearing);

	// projection of the delta on direction of heading
	return delta*cos(ori*2*PI/360);
}
