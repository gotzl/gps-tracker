
#if !defined(TEST) && !defined(PYTHON)
#include <Arduino.h>
#endif

#define BEGIN_LFRAME 	0x80
#define BEGIN_TFRAME 	0xC0
#define BEGIN_GFRAME 	0xE0

struct tFrame {
	uint16_t session;
	uint16_t lap;
	uint8_t stat;
	uint8_t seconds;    //!< 00-59
	uint8_t minutes;    //!< 00-59
	uint8_t hours;      //!< 00-23
	uint8_t day;        //!< 01-07 Day of Week
	uint8_t date;       //!< 01-31 Day of Month
	uint8_t month;      //!< 01-12
	uint8_t year;       //!< 00-99
};

struct gFrame {
	int32_t lon;
	int32_t lat;
	uint32_t speed;
	uint16_t bearing;
	uint8_t sats;
	uint8_t fix;
};

struct lFrame{
	int32_t delta;
	uint32_t start_time;
	uint32_t finish_time;
	uint32_t time;
	uint32_t pts;
	uint16_t dist_start;
};

//------------------------------------------------------------
