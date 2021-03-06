// map the pins used for Serial1, not sure if the define works, to be sure edit
// ~/.arduinocdt/packages/esp32/hardware/esp32/1.0.2/cores/esp32/HardwareSerial.cpp
#define TX1 4
#define RX1 0

#include "gps_tracker.hpp"
#include "protocol.hpp"

#include <WiFi.h>
#include <Update.h>
#include <EEPROM.h>

#include "sd_helper.hpp"
#include "webserver_helper.hpp"

//#define NMEAGPS_PARSE_SATELLITES
#include <NMEAGPS.h>

#include <Adafruit_GPS.h>

//------------------------------------------------------------
//------------------------------------------------------------


#define PMTK_SET_NAVSPEED_THRESHOLD_04 "$PMTK386,0.4*39"
#define PMTK_SET_NAVSPEED_THRESHOLD_06 "$PMTK386,0.6*3B"
#define PMTK_SET_BAUD_115200 "$PMTK251,115200*1F"

#define DEBUG
#define EEPROM_SIZE 8

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
//------------------------------------------------------------

NMEAGPS gps;  // This parses the GPS characters
gps_fix fix;  // This holds the latest GPS fix

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
#ifdef DEBUG
const uint16_t MAX_TRACK_POINTS = 100;
#else
const uint16_t MAX_TRACK_POINTS = 5000;
#endif
const char* tmp_lap = "/tmp/lap.bin";
_track track_ref;
_point* track;
int32_t ref_idx = -1, fix_time_millis = 0;
uint32_t speed[32]; // speed in m/h
uint16_t dist[32];  // distance to start/finish in m
int16_t ori[32];    // orientation to start/finish in degrees
uint16_t tmp_cnt = 0;

bool has_ref = false, valid_lap = false;
bool moving = false, has_sd = false;
NeoGPS::Location_t start_finish(480079500, 78542667);

//------------------------------------------------------------

// stuff needed for the wifi
const char* ssid = "tata";
const char* password = "123456abcd";
const char* host = "esp32sd";

IPAddress local_IP(192, 168, 178, 2);
IPAddress gateway(192, 168, 178, 1);
IPAddress subnet(255, 255, 255, 0);

bool has_wifi = false, ws_running = false;

//------------------------------------------------------------
//------------------------------------------------------------

// move forward in the ref track positions
int32_t nextIdx(NeoGPS::Location_t pnt, int32_t current, uint32_t limit) {
	float distance, closest;
	int32_t idx;

	for (uint32_t i = current; i < min(limit, track_ref.meta.points); i++) {
		// check if we have to fetch new points of the reference lap
		if (i>track_ref.points_loaded) {
			track_ref.points_offset = track_ref.points_loaded;
			track_ref.points_loaded = loadLapPoints(SD, track_ref, MAX_TRACK_POINTS);
		}
		distance = pnt.DistanceKm(track_ref.track[i-track_ref.points_offset].location);
		if (i == 0 || distance < closest) {
			closest = distance;
			idx = i;
		}
	}
	return idx;
}

// find closest point to given location in ref track
int32_t refIdx(NeoGPS::Location_t pnt) {
	return nextIdx(pnt, 0, track_ref.points_loaded / 2);
}

bool checkMoving() {
	uint32_t samples, thresh, i, cnt = 0;

	samples = min(lframe.pts, 20u);
	thresh = samples / 2;

	i = lframe.pts - samples;
	for (; i < lframe.pts; i++)
		cnt += speed[i % 32] > 3000;

	return cnt > thresh;
}

uint16_t orientationThreshold(uint16_t dist) {
	// orientation is good for far distances and bad for close distances (lever arm ...)
	return min(90, max(30, 90 - (dist - 20)));
}

bool startFinishCrossing() {
	if (lframe.pts < 1)
		return false;

	uint16_t a = orientationThreshold(dist[lframe.pts % 32]);
	uint16_t b = orientationThreshold(dist[(lframe.pts - 1) % 32]);

	// start/finish is front
	if (abs(ori[lframe.pts % 32]) < a)
		return false;

	// -> start/finish is behind

	// check if start/finish was previously in front
	if (abs(ori[(lframe.pts - 1) % 32]) <= b) {
		// if one of the two inspected points is reasonably close to
		// start/finish, we've crossed it
		uint16_t dist_thresh = min(
				uint16_t(50), max(uint16_t(15),
						(uint16_t) (speed[lframe.pts % 32] / (3600/2)) )); // distance covered in 0.5s in meters

		DEBUG_PORT.print("Ori flip ");
		DEBUG_PORT.print(ori[(lframe.pts) % 32]); DEBUG_PORT.print(", ");
		DEBUG_PORT.print(ori[(lframe.pts - 1) % 32]); DEBUG_PORT.print(", ");
		DEBUG_PORT.print(a); DEBUG_PORT.print(", ");
		DEBUG_PORT.print(b); DEBUG_PORT.print(", ");
		DEBUG_PORT.println(dist_thresh);
		return dist[lframe.pts % 32] < dist_thresh || dist[(lframe.pts - 1) % 32] < dist_thresh;
	}

	return false;
}

bool setAsReference(_track_meta &meta) {
	DEBUG_PORT.println(F("!! Setting new ref !!"));

	if (tmp_cnt>0) {
		snprintf(track_ref.path, sizeof track_ref.path,
				"/session%03hu/lap%03hu.bin",
				meta.session,
				meta.lap);

		if (!loadLapMeta(SD, track_ref))
			return false;

		track_ref.points_loaded = 0;
		track_ref.points_offset = 0;
		track_ref.points_loaded = loadLapPoints(SD, track_ref, MAX_TRACK_POINTS);

	} else {
		track_ref.meta = meta;
		track_ref.points_loaded = meta.points;
		track_ref.points_offset = 0;
		memcpy(track_ref.track, track,
			sizeof(_point) * MAX_TRACK_POINTS);
	}

	return true;
}

int32_t offsetCorrection(NeoGPS::Location_t ref_loc) {
	return offsetCorrection(
			fix.heading(),
			fix.location.DistanceKm(ref_loc),
			fix.speed_kph(),
			fix.location.BearingToDegrees(ref_loc));
}

void handleFixInfo() {
	ori[lframe.pts % 32] = orientation(fix.heading(), fix.location.BearingToDegrees(start_finish));
	dist[lframe.pts % 32] = fix.location.DistanceKm(start_finish) * 1000;
	speed[lframe.pts % 32] = fix.speed_kph() * 1000;

	fix_time_millis = ((uint32_t) fix.dateTime) * 1000 + (fix.dateTime_ms() % 1000);
	lframe.time = fix_time_millis;

	if (lframe.start_time==0)
		lframe.start_time = fix_time_millis;

	lframe.time -= lframe.start_time;

	track[lframe.pts].time = lframe.time;
	track[lframe.pts].speed = speed[lframe.pts % 32];
	track[lframe.pts].location = fix.location;
};

_track_meta createMeta() {
	_track_meta meta;
	DEBUG_PORT.println(F("Creating meta"));

	meta.points = lframe.pts + MAX_TRACK_POINTS*tmp_cnt;
	meta.time = lframe.time;
	meta.start_time = lframe.start_time;
	meta.finish_time = lframe.finish_time;
	meta.lap = tframe.lap;
	meta.session = tframe.session;
	meta.start_finish = start_finish;
	return meta;
}

void writeLap(_track_meta &meta) {
	char name[23];
	DEBUG_PORT.print(F("Creating file for lap: "));

	snprintf(name, sizeof name,
			"/session%03hu/lap%03hu.bin",
			meta.session,
			meta.lap);

	DEBUG_PORT.println(name);
	writeFile(SD, name,
			(uint8_t*) &meta,
			sizeof(_track_meta));

	DEBUG_PORT.println(F(" ... writing points"));
	// write points stored on SD to the lap file
	if (tmp_cnt>0)
		appendFile(SD, name, tmp_lap);

	// write points from RAM to the lap files
	appendFile(SD, name,
			(uint8_t*) track,
			sizeof(_point) * (lframe.pts));

	DEBUG_PORT.println(F(" ... Done."));
}


void doSomeWork() {
//	DEBUG_PORT.println( F("doSomeWork") );
	static bool warned = false; // that we're waiting for a valid location
	static bool start_finish_crossing;
	static int16_t offset, delta = 0;

	// exceeded the max storage, reset counter and invalidate lap
	if (lframe.pts >= MAX_TRACK_POINTS) {
		if (tframe.lap>0) {
			if (tmp_cnt == 0)
				writeFile(SD, tmp_lap,
						(uint8_t*) track,
						sizeof(_point) * (lframe.pts));
			else
				appendFile(SD, tmp_lap,
						(uint8_t*) track,
						sizeof(_point) * (lframe.pts));

			tmp_cnt += 1;
		}

		lframe.pts = 0;
	}

	if (fix.valid.location && fix.valid.date && fix.valid.time) {

		handleFixInfo();
		start_finish_crossing = startFinishCrossing();

//		if (!checkMoving()) {
//			// we have stopped, what do we do? invalidate lap ?
//			if (moving) {
//			}
//			moving = false;
//
//		} else
//			moving = true;

		if (start_finish_crossing && lframe.pts > 10) {
			DEBUG_PORT.println(F("!! Start/Finish crossing !!"));

			offset = offsetCorrection(start_finish);

			if (valid_lap) {

				lframe.finish_time = fix_time_millis + offset;

				_track_meta meta = createMeta();
				writeLap(meta);

				if (!has_ref/*or faster*/)
					has_ref = setAsReference(meta);

			}

			valid_lap = true;

			// set last point of previous track as first point of this track
			ori[0] = ori[lframe.pts % 32];
			dist[0] = dist[lframe.pts % 32];
			speed[0] = speed[lframe.pts % 32];
			track[0] = track[lframe.pts];


			if (tframe.session == 0)
				tframe.session = createSessionDir(SD);
			tframe.lap++;

			lframe.pts = 0;
			lframe.time = -offset;
			lframe.start_time = fix_time_millis + offset;
			ref_idx = -1;
			tmp_cnt = 0;
		}

		if (has_ref) {
			if (ref_idx == -1) {
				if (start_finish_crossing)
					ref_idx = refIdx(start_finish);
			} else
				ref_idx = nextIdx(fix.location, ref_idx, ref_idx + 100);

			if (ref_idx >= 0) {
				delta = offsetCorrection(track_ref.track[ref_idx - track_ref.points_offset].location);
				lframe.delta = lframe.time - track_ref.track[ref_idx - track_ref.points_offset].time + delta;
			}
		}

		lframe.pts++;

	} else {
		if (!warned) {
			warned = true;
			DEBUG_PORT.print(F("Waiting for fix..."));
		} else {
			DEBUG_PORT.print('.');
		}
	}
}

void setup_gps() {
	// 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
	gpsPort.begin(9600);
	gpsPort.println(PMTK_SET_BAUD_57600);
	gpsPort.end();

	delay(1000);

	gpsPort.begin(57600);
	gpsPort.setRxBufferSize(2048);

	gpsPort.println(PMTK_SET_NMEA_OUTPUT_RMCGGA);

	gpsPort.println(PMTK_SET_NMEA_UPDATE_10HZ);
	gpsPort.println(PMTK_API_SET_FIX_CTL_5HZ);

	gpsPort.println(PMTK_SET_NAVSPEED_THRESHOLD_04);

	// Request updates on antenna status, comment out to keep quiet
	gpsPort.println(PGCMD_ANTENNA);

	delay(1000);

	// Ask for firmware version
	gpsPort.println(PMTK_Q_RELEASE);
}

void setup_tracker() {

	// load previous start/finish from eeprom
	EEPROM.begin(EEPROM_SIZE);
	int32_t lat = EEPROM.readInt(0);
	int32_t lon = EEPROM.readInt(4);
	if (lat>0 && lon>0) {
		DEBUG_PORT.print( F("Loaded start/finish: ") );
		start_finish = NeoGPS::Location_t(lat, lon);
		DEBUG_PORT.print(start_finish.lat());
		DEBUG_PORT.print(',');
		DEBUG_PORT.println(start_finish.lon());
	}


	// allocate memory for the track recording and reference
	track = new _point[MAX_TRACK_POINTS];
	track_ref.track = new _point[MAX_TRACK_POINTS];

	DEBUG_PORT.printf("\n\navailable heap after allocating %i\n", ESP.getFreeHeap());
	DEBUG_PORT.printf("biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

	// set some defaults
	tframe.lap = 0;
	tframe.session = 0;

	lframe.pts = 0;
	lframe.time = 0;
	lframe.delta = 0;
	lframe.start_time = 0;
}

bool check_WiFi(uint16_t thresh) {
	uint16_t cnt=0;
	while (WiFi.status() != WL_CONNECTED) {
		if (cnt>thresh)
			return false;

		delay(500);
		DEBUG_PORT.print(".");
		cnt +=1;
	}

	DEBUG_PORT.println("");
	DEBUG_PORT.println("WiFi connected!");
	DEBUG_PORT.print("IP address: ");
	DEBUG_PORT.println(WiFi.localIP());
	DEBUG_PORT.print("ESP Mac Address: ");
	DEBUG_PORT.println(WiFi.macAddress());
	DEBUG_PORT.print("Subnet Mask: ");
	DEBUG_PORT.println(WiFi.subnetMask());
	DEBUG_PORT.print("Gateway IP: ");
	DEBUG_PORT.println(WiFi.gatewayIP());
	DEBUG_PORT.print("DNS: ");
	DEBUG_PORT.println(WiFi.dnsIP());

	return true;
}

bool setup_WiFi() {
	if (!WiFi.config(local_IP, gateway, subnet)) {
		DEBUG_PORT.println("STA Failed to configure");
	}

	DEBUG_PORT.print("Connecting to ");
	DEBUG_PORT.println(ssid);

	WiFi.begin(ssid, password);
	return check_WiFi(20);
}

void IRAM_ATTR onTimer();
void setup() {
	DEBUG_PORT.begin(115200);

	displayPort.begin(115200);
	while (!displayPort)
		;

#ifndef DEBUG
  setup_gps();
#endif

	DEBUG_PORT.printf("\n\navailable heap %i\n", ESP.getFreeHeap());
	DEBUG_PORT.printf("biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

	// do this first to allow reserving large chunks of connected memory
	setup_tracker();

	DEBUG_PORT.printf("\n\navailable heap %i\n", ESP.getFreeHeap());
	DEBUG_PORT.printf("biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

	pinMode(LED_BUILTIN, OUTPUT);
	has_wifi = setup_WiFi();
	ws_running = setup_webserver();

	DEBUG_PORT.printf("\n\navailable heap %i\n", ESP.getFreeHeap());
	DEBUG_PORT.printf("biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

	has_sd = setup_sd();
	if (!has_sd)
		return;
	createDir(SD, "/tmp");


	timer = timerBegin(1, 80, true); // count in microseconds
	timerAttachInterrupt(timer, &onTimer, true);
	timerAlarmWrite(timer, 1000000, true); // every second
	timerAlarmEnable(timer);

	DEBUG_PORT.printf("\n\navailable heap %i\n", ESP.getFreeHeap());
	DEBUG_PORT.printf("biggest free block: %i\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

	gps.overrun(false);
}


void loop() {
	static int incomingByte = 0;
	bool gps_valid = false;

	// handle gps data
	while (gps.available( gpsPort )) {
		fix = gps.read();
		doSomeWork();
		gps_valid = true;
	}

	if (gps_valid) {
		lframe.dist_start = 0;
		if (fix.valid.location)
			lframe.dist_start = dist[lframe.pts % 32];

		displayPort.write(BEGIN_LFRAME);
		displayPort.write((uint8_t*) &lframe, sizeof lframe);
	}

	if (ws_running && has_wifi) {
//		portENTER_CRITICAL_ISR(&timerMux);
		server.handleClient();
//		portEXIT_CRITICAL(&timerMux);
	}

	// handle incoming commands
	while (displayPort.available())
		incomingByte = displayPort.read();

	if (incomingByte > 0) {
		if (incomingByte == 1 && fix.valid.location) {
			start_finish = fix.location;

			DEBUG_PORT.print( F("Writing start/finish: ") );
			EEPROM.writeInt(0, start_finish.lat());
			EEPROM.writeInt(4, start_finish.lon());
			EEPROM.commit();
			DEBUG_PORT.print(start_finish.lat());
			DEBUG_PORT.print(',');
			DEBUG_PORT.println(start_finish.lon());

			// start a new, valid lap
			// set last point of previous track as first point of this track
			ori[0] = ori[lframe.pts % 32];
			dist[0] = dist[lframe.pts % 32];
			speed[0] = speed[lframe.pts % 32];
			track[0] = track[lframe.pts];


			if (tframe.session == 0)
				tframe.session = createSessionDir(SD);
			tframe.lap++;

			lframe.pts = 0;
			lframe.time = 0;
			lframe.start_time = fix_time_millis;
			ref_idx = -1;

			tmp_cnt = 0;
			valid_lap = true;
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

void IRAM_ATTR onTimer() {
	portENTER_CRITICAL_ISR(&timerMux);
	interruptCounter++;

//	if (cnt == 1) {
		{
//			DEBUG_PORT.println( F("Time frame") );
			tframe.seconds = fix.dateTime.seconds;
			tframe.minutes = fix.dateTime.minutes;
			tframe.hours = fix.dateTime.hours;
			tframe.day = fix.dateTime.day;
			tframe.date = fix.dateTime.date;
			tframe.month = fix.dateTime.month;
			tframe.year = fix.dateTime.year;

			tframe.stat = (tmp_cnt>0) << 5| valid_lap << 4| has_ref << 3| has_sd << 2| has_wifi << 1| ws_running;

			displayPort.write(BEGIN_TFRAME);
			displayPort.write((uint8_t*) &tframe, sizeof tframe);
		}
		{
//			DEBUG_PORT.println( F("GPS frame") );
			gframe.sats = fix.satellites;
			gframe.fix = fix.valid.location;
			gframe.bearing = 0;
			if (lframe.pts > 5) {
				gframe.bearing = fix.location.BearingTo(track[lframe.pts - 5].location) * NeoGPS::Location_t::DEG_PER_RAD;
			}
			gframe.lon = 0;
			gframe.lat = 0;
			gframe.speed = 0;
			if (fix.valid.location) {
				gframe.lon = fix.location._lon;
				gframe.lat = fix.location._lat;
				gframe.speed = fix.speed_kph()*1000;
			}
			displayPort.write(BEGIN_GFRAME);
			displayPort.write((uint8_t*) &gframe, sizeof gframe);
		}

		has_wifi = WiFi.status() == WL_CONNECTED;
		digitalWrite(LED_BUILTIN, (valid_lap && lframe.pts<25)?HIGH:LOW);

//	}
//
//	cnt++;
//	if (cnt > 10)
//		cnt = 0;
//
//	if (cnt == 1) {


	if (false) {

		DEBUG_PORT.print(F("GPS location at "));
		DEBUG_PORT.print(fix.dateTime.hours);
		DEBUG_PORT.print(':');
		DEBUG_PORT.print(fix.dateTime.minutes);
		DEBUG_PORT.print(':');
		DEBUG_PORT.print(fix.dateTime.seconds);
		DEBUG_PORT.print('.');
		DEBUG_PORT.print(fix.dateTime_cs);
		DEBUG_PORT.print(F(" from "));
		DEBUG_PORT.print(fix.satellites);
		DEBUG_PORT.println(F(" satellites."));

		DEBUG_PORT.print(F("Lap: "));
		DEBUG_PORT.print(tframe.lap);
		DEBUG_PORT.print(F(" n_points: "));
		DEBUG_PORT.print(lframe.pts);
		DEBUG_PORT.print(F(" time: "));
//		DEBUG_PORT.print( lap_frame.time);
		uint16_t s = lframe.time / 1000;
		uint16_t m = s / 60;
		s %= 60;
		char value[128] = { };
		sprintf(value, "%3u:%02u.%1lu", m, s, lframe.time % 1000);
		DEBUG_PORT.print(value);

		DEBUG_PORT.print(F(" moving? "));
		DEBUG_PORT.print(moving);
		DEBUG_PORT.print(F(" valid? "));
		DEBUG_PORT.print(valid_lap);
		DEBUG_PORT.print(F(" has ref? "));
		DEBUG_PORT.print(has_ref);
		DEBUG_PORT.print(F(" overrun? "));
		DEBUG_PORT.println(gps.overrun());
		gps.overrun(false);

		if (fix.valid.location) {
			DEBUG_PORT.print(F("Current loc / speed: "));
			DEBUG_PORT.print(fix.latitudeL());
			DEBUG_PORT.print(',');
			DEBUG_PORT.print(fix.longitudeL());
			DEBUG_PORT.print(F("; "));
			DEBUG_PORT.print(fix.speed_kph(), 2);
			DEBUG_PORT.println("km/h");

			DEBUG_PORT.print(F("Dist start/finish: "));
			DEBUG_PORT.print(dist[lframe.pts % 32], 2);
			DEBUG_PORT.print('m');
			DEBUG_PORT.print(F(" ref idx: "));
			DEBUG_PORT.println(ref_idx);

//			DEBUG_PORT.print(F("Orientation: "));
//			DEBUG_PORT.print(orientation(fix.heading(), fix.location.BearingToDegrees(start_finish)));
//			DEBUG_PORT.print("deg");
//			DEBUG_PORT.print(F(" Bearing: "));
//			DEBUG_PORT.print(fix.location.BearingToDegrees(start_finish));
//			DEBUG_PORT.print(F(" Heading: "));
//			DEBUG_PORT.println(fix.heading());

			if (ref_idx >= 0) {
				NeoGPS::Location_t &ref = track_ref.track[ref_idx].location;
				DEBUG_PORT.print(F("Dist: "));
				DEBUG_PORT.print(fix.location.DistanceKm(ref) * 1000);
				DEBUG_PORT.print('m');
				DEBUG_PORT.print(F(" Orientation: "));
				DEBUG_PORT.print(orientation(fix.heading(), fix.location.BearingToDegrees(ref)));
				DEBUG_PORT.println("deg");
				DEBUG_PORT.print(F("Bearing: "));
				DEBUG_PORT.print(fix.location.BearingToDegrees(ref));
				DEBUG_PORT.print(F(" delta: "));
				DEBUG_PORT.print(lframe.delta);
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
