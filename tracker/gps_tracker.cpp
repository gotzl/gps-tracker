// map the pins used for Serial1, not sure if the define works, to be sure edit
// ~/.arduinocdt/packages/esp32/hardware/esp32/1.0.2/cores/esp32/HardwareSerial.cpp
#define TX1 4
#define RX1 0


//#define NMEAGPS_PARSE_SATELLITES
#include <NMEAGPS.h>

#include <Adafruit_GPS.h>
#define PMTK_SET_NAVSPEED_THRESHOLD_06 "$PMTK386,0.6*3B"
#define PMTK_SET_BAUD_115200 "$PMTK251,115200*1F"
//Adafruit_GPS GPS(&Serial1);


#include "gps_tracker.hpp"
#include "sd_helper.hpp"


//#define DEBUG
#define GPS_PORT_NAME "Serial"


#define DEBUG_PORT Serial
#define displayPort Serial1

// in debug mode, read NMEA sentences from DEBUG_PORT
#ifndef DEBUG
#define gpsPort Serial2
#else
#define gpsPort DEBUG_PORT
#endif

//------------------------------------------------------------

NMEAGPS  gps;  // This parses the GPS characters
gps_fix  fix;  // This holds the latest GPS fix

//------------------------------------------------------------

// stuff thats send to the display periodically, using a timer
lFrame lframe;
gFrame gframe;
tFrame tframe;
hw_timer_t * timer = NULL;
volatile int interruptCounter;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

//------------------------------------------------------------

// stuff needed for keeping track of past locations
const int32_t MAX_TRACK_POINTS = 8000;
_track track_ref;
_point* track;
int32_t ref_idx = -1, delta = 0,fix_time_millis = 0;
int16_t session = -1;
float dist[32], speed[32];

bool has_ref = false, valid_lap = false;
bool moving = false, approach_start_finish = false;

NeoGPS::Location_t start_finish(480079167,78534000);

//------------------------------------------------------------
//------------------------------------------------------------


// move forward in the ref track positions
int32_t nextIdx(NeoGPS::Location_t pnt, int32_t current, uint32_t limit) {
	float distance, closest;
	int32_t idx;

	for(uint32_t i=current; i<min(limit, track_ref.points); i++) {
		distance = pnt.DistanceKm(
				NeoGPS::Location_t(
						track_ref.track[i].lat,
						track_ref.track[i].lon) );
		if (i==0 || distance < closest) {
			closest = distance;
			idx = i;
		}
	}
	return idx;
}


// find closest point to given location in ref track
int32_t refIdx(NeoGPS::Location_t pnt) {
	return nextIdx(pnt, 0, track_ref.points/2);
}


// 250km/h -> 70m/s
// 5Hz -> 14m distance per tick
// approach start finish when
//	* distance decreases for X samples (while accepting Y outliers)
// 	* distance smaller than Z
//  * approach end when minimum exceeded, ie for K samples the dist increases (while accepting L outliers)
bool checkApproachStartFinish() {
	static const uint32_t samples_dec = 15, samples_inc = 4;
	static const uint32_t outliers_dec = 5, outliers_inc = 2;
	uint32_t cnt_dec = 0, cnt_inc = 0;
	bool close = false;

	if (lframe.pts < max(samples_dec,samples_inc)) return false;

	for(uint32_t i=0;i<samples_dec-1;i++) {
		cnt_dec += dist[(lframe.pts-samples_dec+i)%32] > dist[(lframe.pts-samples_dec+i+1)%32];
		close |= dist[(lframe.pts-samples_dec+i+1)%32] < 10;
	}

	for(uint32_t i=0;i<samples_inc-1;i++)
		cnt_inc += dist[(lframe.pts-samples_inc+i)%32] < dist[(lframe.pts-samples_inc+i+1)%32];

	return cnt_dec >= samples_dec-outliers_dec && close && cnt_inc >= samples_inc-outliers_inc;
}

bool checkMoving() {
	uint32_t samples,thresh,i,cnt=0;

	samples = min(lframe.pts, 20u);
	thresh = samples/2;

	i = lframe.pts - samples;
	for(;i<lframe.pts;i++)
		cnt += speed[i%32]>3;

	return cnt > thresh;
}

bool checkStartFinish() {
	// check if distance to start/finish is decreasing
	if ( checkApproachStartFinish() ) {
		if (!approach_start_finish)
			DEBUG_PORT.println( F("!! Approaching Start/Finish !!") );
		approach_start_finish = true;

	// distance is increasing
	} else {

		// if previously distance increased, do nothing
		if (!approach_start_finish)
			return false;

		// distance was decreasing before, consider start/finish
		// crossed when reasonably close to the start/finish position
		DEBUG_PORT.print( F("!! Stop approaching Start/Finish, dist ") );
		DEBUG_PORT.println(dist[lframe.pts%32]);
		approach_start_finish = false;
		return true;
	}
	return false;
}

void setAsReference() {
	DEBUG_PORT.println( F("!! Setting new ref !!") );
	memcpy(track_ref.track, track, sizeof(_point)*MAX_TRACK_POINTS);
	track_ref.points = lframe.pts;
}

int32_t offsetCorrection(NeoGPS::Location_t ref_loc) {
	return offsetCorrection(
			fix.heading(),
			fix.location.DistanceKm(ref_loc),
			fix.speed_kph(),
			fix.location.BearingToDegrees(ref_loc)
			);
}

void handleFixInfo() {
	dist[lframe.pts%32] = fix.location.DistanceKm(start_finish)*1000;
	speed[lframe.pts%32] = fix.speed_kph();

	fix_time_millis =  ((uint32_t)fix.dateTime)*1000 + (fix.dateTime_ms()%1000);

	if (valid_lap)
		lframe.time = fix_time_millis - lframe.start_time;

	track[lframe.pts].time = lframe.time;
	track[lframe.pts].lon =  fix.longitudeL();
	track[lframe.pts].lat =  fix.latitudeL();

}

void doSomeWork() {
//	DEBUG_PORT.println( F("doSomeWork") );
	static bool warned = false; // that we're waiting for a valid location
	static bool start_finish_crossing = false;

	// exceeded the max storage, reset counter and invalidate lap
	if (lframe.pts >= MAX_TRACK_POINTS) {
		valid_lap = false;
		lframe.pts = 0;
	}

	if (fix.valid.location && fix.valid.date && fix.valid.time) {

		handleFixInfo();

		if (!checkMoving()) {
			// we have stopped, what do we do? invalidate lap ?
			if (moving) {
			}
			moving = false;

		} else
			moving = true;

		start_finish_crossing = checkStartFinish();

		if (start_finish_crossing && moving) {
			DEBUG_PORT.println( F("!! Start/Finish crossing !!") );


			uint32_t offset = offsetCorrection(start_finish);

			if (valid_lap) {

				lframe.finish_time = fix_time_millis + offset;

				if (session < 0)
					session = createSessionDir(SD);

				char name[23];
				sprintf(name, "/session%03i/lap%03i.bin", session, lframe.lap);
				writeFile(SD, name, (uint8_t*) track, sizeof(_point)*lframe.pts);

				if (!has_ref/*or faster*/) {
					setAsReference();
					has_ref = true;
				}

				lframe.lap++;
			}

			valid_lap = true;

			// set last point of previous track as first point of this track
			dist[0] = dist[lframe.pts%32];
			speed[0] = speed[lframe.pts%32];
			track[0] = track[lframe.pts];

			lframe.pts = 0;
			lframe.time = -offset;
			lframe.start_time = fix_time_millis + offset;
			ref_idx = -1;
		}


		if (has_ref) {
			if (ref_idx==-1) {
				if (start_finish_crossing)
					ref_idx = refIdx(start_finish);
			} else
				ref_idx = nextIdx(fix.location,
						ref_idx, ref_idx+100);

			if (ref_idx>=0) {

				NeoGPS::Location_t ref_loc = NeoGPS::Location_t(
						track_ref.track[ref_idx].lat,
						track_ref.track[ref_idx].lon);

				delta = offsetCorrection(ref_loc);
				lframe.delta = lframe.time - (track_ref.track[ref_idx].time + delta);
			}
		}

		lframe.pts++;

	} else {
		if (!warned) {
			warned = true;
			DEBUG_PORT.print( F("Waiting for fix...") );
		} else {
			DEBUG_PORT.print( '.' );
		}
	}
}


void setup_gps() {
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  gpsPort.begin(9600);
  gpsPort.println(PMTK_SET_BAUD_57600);
  gpsPort.begin(57600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  gpsPort.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  // Set the update rate
  gpsPort.println(PMTK_SET_NMEA_UPDATE_10HZ); // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  gpsPort.println(PMTK_API_SET_FIX_CTL_5HZ);
  gpsPort.println(PMTK_SET_NAVSPEED_THRESHOLD_06);


  // Request updates on antenna status, comment out to keep quiet
  gpsPort.println(PGCMD_ANTENNA);

  delay(1000);

  // Ask for firmware version
  gpsPort.println(PMTK_Q_RELEASE);
}


void setup_tracker() {
	// allocate memory for the track recording and reference
	track = new _point[MAX_TRACK_POINTS];
	track_ref.track = new _point[MAX_TRACK_POINTS];

	DEBUG_PORT.printf("\n\navailable heap after allocating %i\n", ESP.getFreeHeap());
	DEBUG_PORT.printf("biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

	// set some defaults
	lframe.lap = 0;
	lframe.pts = 0;
	lframe.time = 0;
	lframe.delta = 0;
}

bool setup_sd() {
	uint8_t cardType;
	uint64_t cardSize;

//	pinMode(23,INPUT_PULLUP);
//	SPI.begin(18,19,23);

	if(!SD.begin()){
		DEBUG_PORT.println("Card Mount Failed");
		return false;
	}

	cardType = SD.cardType();
	if(cardType == CARD_NONE){
		DEBUG_PORT.println("No SD card attached");
		return false;
	}

	DEBUG_PORT.print("SD Card Type: ");
	if(cardType == CARD_MMC){
		DEBUG_PORT.println("MMC");
	} else if(cardType == CARD_SD){
		DEBUG_PORT.println("SDSC");
	} else if(cardType == CARD_SDHC){
		DEBUG_PORT.println("SDHC");
	} else {
		DEBUG_PORT.println("UNKNOWN");
	}

	cardSize = SD.cardSize() / (1024 * 1024);
	DEBUG_PORT.printf("SD Card Size: %lluMB\n", cardSize);

	listDir(SD, "/", 0);
	return true;
}


void IRAM_ATTR onTimer();
void setup() {
  DEBUG_PORT.begin(115200);
//  while (!DEBUG_PORT)
//    ;

  displayPort.begin(115200);
  while (!displayPort)
    ;

  DEBUG_PORT.printf("\n\navailable heap %i\n", ESP.getFreeHeap());
  DEBUG_PORT.printf("biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

  setup_tracker();
  if (!setup_sd())
	  return;

#ifndef DEBUG
  setup_gps();
#endif

  timer = timerBegin(1, 80, true); // count in microseconds
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 100000, true); // every tenth of a second
  timerAlarmEnable(timer);
}


static int incomingByte = 0;
void loop() {

//#ifndef DEBUG
	// handle gps data
	while (gps.available( gpsPort )) {
		fix = gps.read();
		doSomeWork();
	}
//#else
//	while (gpsPort.available() )
//		DEBUG_PORT.write(gpsPort.read());
//#endif

	// handle incoming commands
	while (displayPort.available())
		incomingByte = displayPort.read();

	if (incomingByte>0) {
		if (incomingByte == 1) {
			lframe.pts = 0;
			lframe.lap = 0;
			valid_lap = false;
			has_ref = false;

			start_finish = fix.location;
		}
		DEBUG_PORT.print("Received command ");
		DEBUG_PORT.println(incomingByte);
		incomingByte = 0;
	}

	// check if there was an interrupt
	if (interruptCounter > 0) {
		portENTER_CRITICAL(&timerMux);
		interruptCounter--;
		portEXIT_CRITICAL(&timerMux);
	}
}

static int8_t cnt=0;
void IRAM_ATTR onTimer() {
	portENTER_CRITICAL_ISR(&timerMux);
	interruptCounter++;

	if (cnt%2) {
		{
//			DEBUG_PORT.println( F("Lap frame") );
			lframe.stat = moving<<3 | valid_lap<<2 | has_ref<<1 | approach_start_finish;
			lframe.dist_start = 0;
			if (fix.valid.location)
				lframe.dist_start = fix.location.DistanceKm(start_finish)*1000;

			displayPort.write(BEGIN_LFRAME);
			displayPort.write((uint8_t*)&lframe, sizeof lframe);
		}
	}

	if (cnt==1) {
		{
//			DEBUG_PORT.println( F("Time frame") );
			tframe.seconds = fix.dateTime.seconds;
			tframe.minutes = fix.dateTime.minutes;
			tframe.hours = fix.dateTime.hours;
			tframe.day = fix.dateTime.day;
			tframe.date = fix.dateTime.date;
			tframe.month = fix.dateTime.month;
			tframe.year = fix.dateTime.year;

			displayPort.write(BEGIN_TFRAME);
			displayPort.write((uint8_t*)&tframe, sizeof tframe);
		}
//	}

//	if (cnt==1) {
		{
//			DEBUG_PORT.println( F("GPS frame") );
			gframe.sats = fix.satellites;
			gframe.fix = fix.valid.location;
			gframe.bearing = 0;
			if (lframe.pts>5) {
				NeoGPS::Location_t prev(track[lframe.pts-5].lat, track[lframe.pts-5].lon);
				gframe.bearing = fix.location.BearingTo( prev ) * NeoGPS::Location_t::DEG_PER_RAD;
			}
			gframe.lon = 0;
			gframe.lat = 0;
			gframe.speed = 0;
			if (fix.valid.location) {
				gframe.lon = fix.location._lon;
				gframe.lat = fix.location._lat;
				gframe.speed = fix.speed_kph();
			}
			displayPort.write(BEGIN_GFRAME);
			displayPort.write((uint8_t*)&gframe, sizeof gframe);
		}
	}

	cnt++;
	if (cnt>10)
		cnt = 0;

	if (cnt==1) {
		DEBUG_PORT.print( F("GPS location at ") );
		DEBUG_PORT.print( fix.dateTime.hours );
		DEBUG_PORT.print( ':' );
		DEBUG_PORT.print( fix.dateTime.minutes );
		DEBUG_PORT.print( ':' );
		DEBUG_PORT.print( fix.dateTime.seconds );
		DEBUG_PORT.print( '.' );
		DEBUG_PORT.print( fix.dateTime_cs );
		DEBUG_PORT.print( F(" from ") );
		DEBUG_PORT.print( fix.satellites );
		DEBUG_PORT.println( F(" satellites.") );

		DEBUG_PORT.print( F("Lap: ") );
		DEBUG_PORT.print( lframe.lap);
		DEBUG_PORT.print( F(" n_points: ") );
		DEBUG_PORT.print( lframe.pts );
		DEBUG_PORT.print( F(" time: ") );
//		DEBUG_PORT.print( lap_frame.time);
		uint16_t s = lframe.time/1000;
		uint16_t m = s/60;
		s %= 60;
		char value[128] = {};
		sprintf(value, "%3u:%02u.%1lu", m, s, lframe.time%1000);
		DEBUG_PORT.print(value);

		DEBUG_PORT.print( F(" moving? ") );
		DEBUG_PORT.print( moving );
		DEBUG_PORT.print( F(" valid? ") );
		DEBUG_PORT.print( valid_lap );
		DEBUG_PORT.print( F(" has ref? ") );
		DEBUG_PORT.println( has_ref );
		if (fix.valid.location) {
			DEBUG_PORT.print( F("Current loc / speed: ") );
			DEBUG_PORT.print( fix.latitudeL() );
			DEBUG_PORT.print( ',' );
			DEBUG_PORT.print( fix.longitudeL() );
			DEBUG_PORT.print( F("; ") );
			DEBUG_PORT.print( fix.speed_kph(), 2 );
			DEBUG_PORT.println( "km/h" );

			DEBUG_PORT.print( F("Dist start/finish: ") );
			DEBUG_PORT.print( fix.location.DistanceKm(start_finish)*1000, 2 );
			DEBUG_PORT.print( 'm' );
			DEBUG_PORT.print( F(" apprchng? ") );
			DEBUG_PORT.print( approach_start_finish );
			DEBUG_PORT.print( F(" ref idx: ") );
			DEBUG_PORT.println( ref_idx );
			if (ref_idx>=0) {
				NeoGPS::Location_t ref(track_ref.track[ref_idx].lat,
						track_ref.track[ref_idx].lon);
				DEBUG_PORT.print( F("Dist: ") );
				DEBUG_PORT.print(fix.location.DistanceKm(ref)*1000 );
				DEBUG_PORT.print( 'm' );
				DEBUG_PORT.print( F(" Orientation: ") );
				DEBUG_PORT.print( orientation(fix.heading(), fix.location.BearingToDegrees(ref)) );
				DEBUG_PORT.println( "deg" );
				DEBUG_PORT.print( F("Bearing: ") );
				DEBUG_PORT.print(fix.location.BearingToDegrees(ref));
				DEBUG_PORT.print( F(" Heading: ") );
				DEBUG_PORT.print(fix.heading());
				DEBUG_PORT.print( F(" delta: ") );
				DEBUG_PORT.print(lframe.delta);
				DEBUG_PORT.print( F(" dist_delta: ") );
				DEBUG_PORT.print(delta);
				DEBUG_PORT.println();
			}
		}
//		DEBUG_PORT.printf("available heap in loop %i", ESP.getFreeHeap());
//		DEBUG_PORT.println();
//		DEBUG_PORT.printf("biggest free block: %i", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
//		DEBUG_PORT.println();
		DEBUG_PORT.println();
	}
	portEXIT_CRITICAL_ISR(&timerMux);
}
