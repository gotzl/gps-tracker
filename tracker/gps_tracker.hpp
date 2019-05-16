
#if !defined(TEST) && !defined(PYTHON)
#include <Arduino.h>
#endif


#ifdef PYTHON
#include <pybind11/pybind11.h>
#endif


#ifdef TEST
#include <stdlib.h>
#include <cstdio>
#include <math.h>

typedef char uint8_t;
typedef short int16_t;
typedef unsigned short uint16_t;
typedef int int32_t;
typedef unsigned int uint32_t;
#endif


#if defined(TEST) || defined(PYTHON)
#define PI 3.1415926535897932384626433832795
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

struct _track_meta
{
	uint32_t points;
	uint32_t start_time;
	uint32_t finish_time;
	uint32_t time;
	int32_t start_finish_lat;
	int32_t start_finish_lon;
	int16_t lap;
	int16_t session;
};

struct _track
{
	_track_meta meta;
	_point* track;
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


//#define PYTHON
#ifdef PYTHON
#include <iostream>
#include <fstream>
#include <pybind11/stl.h>
using namespace std;

void readTrackMeta(string name, const _track_meta *meta) {
	ifstream file (name, ios::in|ios::binary|ios::ate);
	if (file.is_open()) {
		file.seekg (0, ios::beg);
		file.read ((char *) meta, sizeof(_track_meta));
		file.close();
	}
}

vector<_point> readTrack(string name, const _track_meta *meta) {
	vector<_point> track;
	ifstream file (name, ios::in|ios::binary|ios::ate);
	if (file.is_open()) {
		file.seekg (0, ios::beg);
		file.read ((char *) meta, sizeof(_track_meta));

		_point  track_buff[sizeof(_point)*meta->points];
		file.read ((char *) &track_buff, sizeof(_point)*meta->points);
		file.close();

		track = vector<_point>(track_buff, track_buff + meta->points);
	}
	return track;
}

#endif


#ifdef PYTHON
namespace py = pybind11;

PYBIND11_MODULE(example, m) {
    m.doc() = "pybind11 test";
    m.def("offsetCorrection",&offsetCorrection,"blablup");
    m.def("orientation",&orientation,"blablup");
    m.def("readTrackMeta",&readTrackMeta,"blablup");
    m.def("readTrack",&readTrack,"blablup");
    py::class_<_point>(m, "_point")
        .def(py::init<>())
		.def_readwrite("time", &_point::time)
		.def_readwrite("lat", &_point::lat)
		.def_readwrite("lon", &_point::lon);
    py::class_<_track_meta>(m, "_track_meta")
        .def(py::init<>())
		.def_readwrite("points", &_track_meta::points)
		.def_readwrite("start_time", &_track_meta::start_time)
		.def_readwrite("finish_time", &_track_meta::finish_time)
		.def_readwrite("time", &_track_meta::time)
		.def_readwrite("start_finish_lat", &_track_meta::start_finish_lat)
		.def_readwrite("start_finish_lon", &_track_meta::start_finish_lon)
		.def_readwrite("lap", &_track_meta::lap)
		.def_readwrite("session", &_track_meta::session);
}
#endif
