#include <Adafruit_GFX.h>    // Core graphics library
#include <SPI.h>       // this is needed for display
#include "Adafruit_ILI9341.h"
#include <Wire.h>      // this is needed for FT6206
#include <Adafruit_FT6206.h>
#include <avr/interrupt.h>

#include "../tracker/gps_tracker.hpp"

// The FT6206 uses hardware I2C (SCL/SDA)
Adafruit_FT6206 ctp = Adafruit_FT6206();

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

// Size of the color selection boxes and the paintbrush size
#define PENRADIUS 3

//------------------------------------------------------------

static char value[128] = {};
static unsigned long time = 0;

//------------------------------------------------------------

static struct tFrame tframe;
static struct gFrame gframe;
static struct lFrame lframe;
static uint8_t buffer[max(
		max(sizeof(tFrame), sizeof(gFrame)),
		sizeof(lFrame))];
static bool gvalid = false, tvalid = false, lvalid = false;

//------------------------------------------------------------

static TS_Point p;

// position of various info on screen
#define SATS_X 0
#define SATS_Y 220
#define SATS_X_OFFSET 60
#define FIX_X 100
#define FIX_Y 220
#define FIX_X_OFFSET 60
#define SPD_X 120
#define SPD_Y 220
#define SPD_X_OFFSET 80
#define BEAR_X 180
#define BEAR_Y 0
#define BEAR_X_OFFSET 80

#define LAP_X 0
#define LAP_Y 20
#define LAP_X_OFFSET 50
#define TIME_X 100
#define TIME_Y 20
#define TIME_X_OFFSET 80
#define DELTA_X 100
#define DELTA_Y 100
#define DELTA_X_OFFSET 100
#define PNTS_X 00
#define PNTS_Y 40
#define PNTS_X_OFFSET 80
#define STAT_X 0
#define STAT_Y 180
#define STAT_X_OFFSET 260
#define DIST_X 0
#define DIST_Y 200
#define DIST_X_OFFSET 140

//------------------------------------------------------------

const char nCD  [] PROGMEM = "N";
const char nneCD[] PROGMEM = "NNE";
const char neCD [] PROGMEM = "NE";
const char eneCD[] PROGMEM = "ENE";
const char eCD  [] PROGMEM = "E";
const char eseCD[] PROGMEM = "ESE";
const char seCD [] PROGMEM = "SE";
const char sseCD[] PROGMEM = "SSE";
const char sCD  [] PROGMEM = "S";
const char sswCD[] PROGMEM = "SSW";
const char swCD [] PROGMEM = "SW";
const char wswCD[] PROGMEM = "WSW";
const char wCD  [] PROGMEM = "W";
const char wnwCD[] PROGMEM = "WNW";
const char nwCD [] PROGMEM = "NW";
const char nnwCD[] PROGMEM = "NNW";

const char * const dirStrings[] PROGMEM =
  { nCD, nneCD, neCD, eneCD, eCD, eseCD, seCD, sseCD,
    sCD, sswCD, swCD, wswCD, wCD, wnwCD, nwCD, nnwCD };

const __FlashStringHelper *compassDir( uint16_t bearing ) // degrees CW from N
{
  const int16_t directions    = sizeof(dirStrings)/sizeof(dirStrings[0]);
  const int16_t degreesPerDir = 360 / directions;
        int8_t  dir           = (bearing + degreesPerDir/2) / degreesPerDir;

  while (dir < 0)
    dir += directions;
  while (dir >= directions)
    dir -= directions;

  return (const __FlashStringHelper *) pgm_read_ptr( &dirStrings[ dir ] );

} // compassDir

//------------------------------------------------------------


void updateLapData() {
	static float max_delta = 500, min_delta = 500;
	tft.setRotation(1);

	tft.setCursor(LAP_X+LAP_X_OFFSET, LAP_Y);
	sprintf(value, "%02i", lframe.lap);
	tft.print(value);

	{
		tft.setCursor(TIME_X+TIME_X_OFFSET, TIME_Y);
		uint16_t s = lframe.time/1000;
		uint16_t m = s/60;
		s %= 60;
		sprintf(value, "%3u:%02u.%03lu", m, s, lframe.time%1000);
		tft.print(value);
	}

	{
		tft.setCursor(DELTA_X+DELTA_X_OFFSET, DELTA_Y);
		// get delta in seconds, preserve the sign
		float s = lframe.delta/1000.;
		char str_temp[4];
		// print only the seconds to the string
		dtostrf(s, 4, 0, str_temp);
		sprintf(value, "%s.%03li", str_temp, abs(lframe.delta%1000));
		tft.print(value);

		// adapt the width of the delta bar to current delta
		if (abs(lframe.delta) > max_delta)
			max_delta = abs(lframe.delta);
		// slow decay of the width if current delta significantly smaller
		else if (max_delta > 2*abs(lframe.delta))
			max_delta /= 2;

		// apply minimum width of the delta bar
		max_delta = max(max_delta, min_delta);

		// width of one side of the bar is 149, scale with ratio of current delta to max_delta
		int width = min(149 * (abs(lframe.delta)/max_delta), 149);

		if (lframe.delta>0) {
			// fill part of the bar green
			tft.fillRect(161, DELTA_Y+20, width, 10, ILI9341_GREEN);
			// .. and the rest black
			tft.fillRect(161+width, DELTA_Y+20, 149-width, 10, ILI9341_BLACK);
			// clear the other side
			tft.fillRect(10, DELTA_Y+20, 149, 10, ILI9341_BLACK);
		} else {
			// fill part of the bar red
			tft.fillRect(159-width, DELTA_Y+20, width, 10, ILI9341_RED);
			// .. and the rest black
			tft.fillRect(10, DELTA_Y+20, 149-width, 10, ILI9341_BLACK);
			// clear the other side
			tft.fillRect(161, DELTA_Y+20, 149, 10, ILI9341_BLACK);
		}

	}

	tft.setCursor(PNTS_X+PNTS_X_OFFSET, PNTS_Y);
	sprintf(value, "%05lu", lframe.pts);
	tft.print(value);

	tft.setCursor(DIST_X+DIST_X_OFFSET, DIST_Y);
	{
		char str_temp[6];
		dtostrf(lframe.dist_start, 5, 2, str_temp);
		sprintf(value, "%05sm", str_temp);
	}
	tft.print(value);

	tft.setCursor(STAT_X+STAT_X_OFFSET, STAT_Y+20);
	tft.print(lframe.stat>>3&0x1);
	tft.print(lframe.stat>>2&0x1);
	tft.print(lframe.stat>>1&0x1);
	tft.print(lframe.stat&0x1);

	lvalid = false;
}


void updateGpsData() {
	tft.setRotation(1);

	tft.setCursor(SATS_X+SATS_X_OFFSET, SATS_Y);
	sprintf(value, "%02i", gframe.sats);
	tft.print(value);

	tft.setCursor(FIX_X+FIX_X_OFFSET, FIX_Y);
	tft.print(gframe.fix);

	tft.setCursor(SPD_X+SPD_X_OFFSET, SPD_Y);
	{
		char str_temp[6];
		dtostrf(gframe.speed, 5, 2, str_temp);
		sprintf(value, "%05skm/h", str_temp);
	}
	tft.print(value);

	tft.setCursor(BEAR_X+BEAR_X_OFFSET, BEAR_Y);
	tft.print(compassDir(gframe.bearing));

	gvalid = false;
}

void updateTimeData() {
	tft.setCursor(0, 0);
	tft.setRotation(1);

	sprintf(value, "%02i-%02i-%02i %02i:%02i:%02i",
			tframe.date,tframe.month,tframe.year,
			tframe.hours,tframe.minutes,tframe.seconds);
	tft.print(value);

	tvalid = false;
}


void handleTouch() {
	// erase previous point
	tft.setRotation(0);
	tft.fillCircle(p.x, p.y, PENRADIUS, ILI9341_BLACK);

	// Retrieve a point
	p = ctp.getPoint();
	// flip it around to match the screen.
	p.x = map(p.x, 0, 240, 240, 0);
	p.y = map(p.y, 0, 320, 320, 0);

	// Print out the remapped (rotated) coordinates
	Serial.print("(");
	Serial.print(p.x);
	Serial.print(", ");
	Serial.print(p.y);
	Serial.println(")");

	tft.fillCircle(p.x, p.y, PENRADIUS, ILI9341_RED);

	if (p.x < 120) { // bottom
		if (p.y < 160) { // left
			Serial.println("left bottom");
		} else { // right
			Serial.println("right bottom");
			Serial1.write(1);
		}
	} else { // top
		if (p.y < 160) { // left
			Serial.println("left top");
		} else { // right
			Serial.println("right top");
		}
	}
}


void setup() {
	Serial.begin(115200);
//	while (!Serial)
//		;
	Serial1.begin(115200);
	Serial1.setTimeout(100);

	Serial.println( F("Hello there") );

	tft.begin();

	if (!ctp.begin(40)) {  // pass in 'sensitivity' coefficient
		Serial.println( F("Couldn't start FT6206 touchscreen controller") );
		while (1)
			;
	}

	// read diagnostics (optional but can help debug problems)
	uint8_t x = tft.readcommand8(ILI9341_RDMODE);
	Serial.print( F("Display Power Mode: 0x") );
	Serial.println(x, HEX);
	x = tft.readcommand8(ILI9341_RDMADCTL);
	Serial.print( F("MADCTL Mode: 0x") );
	Serial.println(x, HEX);
	x = tft.readcommand8(ILI9341_RDPIXFMT);
	Serial.print( F("Pixel Format: 0x") );
	Serial.println(x, HEX);
	x = tft.readcommand8(ILI9341_RDIMGFMT);
	Serial.print( F("Image Format: 0x") );
	Serial.println(x, HEX);
	x = tft.readcommand8(ILI9341_RDSELFDIAG);
	Serial.print( F("Self Diagnostic: 0x") );
	Serial.println(x, HEX);

	// setup the UI
	tft.fillScreen(ILI9341_BLACK);
	tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK); // set bg color to overwrite old text
	tft.setTextSize(2);
	tft.setRotation(1);

	tft.setCursor(LAP_X, LAP_Y);
	tft.print(F("Lap: "));
	tft.setCursor(TIME_X, TIME_Y);
	tft.print(F("Time: "));
	tft.setCursor(DELTA_X, DELTA_Y);
	tft.print(F("Delta: "));
	tft.setCursor(PNTS_X, PNTS_Y);
	tft.print(F("Pts: "));
	tft.setCursor(DIST_X, DIST_Y);
	tft.print(F("Dist start: "));
	tft.setCursor(STAT_X, STAT_Y);
	tft.print( F("stat(mvng,vld,ref,appr): "));


	tft.setCursor(SATS_X, SATS_Y);
	tft.print(F("Sats: "));
	tft.setCursor(FIX_X, FIX_Y);
	tft.print(F("Fix: "));
//	tft.setCursor(SPD_X, SPD_Y);
//	tft.print(F(", speed: "));

	tft.drawRect(9, DELTA_Y+19, 302, 12, ILI9341_WHITE);
	tft.drawRect(159, DELTA_Y+19, 2, 12, ILI9341_WHITE);
}


void loop() {
	// update display
	if (lvalid)
		updateLapData();
	if (gvalid)
		updateGpsData();
	if (tvalid)
		updateTimeData();

	// periodic things
	if (millis() - time>=2000) {
		time = millis();

	}

	// React to a touch
	if (ctp.touched())
		handleTouch();
}


// listen to data coming from tracker
void serialEvent1() {
	static char *target;
	static bool *valid;
	static uint8_t byte;
	static size_t size;
	static uint16_t myIndex;
	static bool inFrame = false;

	Serial.print( F("Got serial data ") );
	Serial.println(Serial1.available());

	while (Serial1.available()) {
		// Read a char
		byte = (uint8_t) Serial1.read();
//		Serial.println(inChar);

		// we are reading a frame
		if (inFrame) {

			if (myIndex < size) { // Add a byte
				*((char*)&buffer + myIndex) = byte;
				myIndex++;
			}

			// we're done reading the frame
			if (myIndex == size) {
				memcpy(target, &buffer, size);
				*valid = true;
				inFrame = false;
			}

		} else if (byte >> 7) { // Start New Frame
			myIndex = 0;
			inFrame = true;

			switch (byte) {
			case BEGIN_LFRAME:
				size = sizeof(lFrame);
				target = (char*) &lframe;
				valid = &lvalid;
				break;
			case BEGIN_GFRAME:
				size = sizeof(gFrame);
				target = (char*) &gframe;
				valid = &gvalid;
				break;
			case BEGIN_TFRAME:
				size = sizeof(tFrame);
				target = (char*) &tframe;
				valid = &tvalid;
				break;
			default:
				inFrame = false;
			}

			if (inFrame) {
				memset(&buffer, 0, sizeof buffer);
				*valid = false;
			}
		}
	}
}
