
#if !defined(TEST) && !defined(PYTHON)
#include <Arduino.h>
#include "Location.h"
typedef NeoGPS::Location_t _location;
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
struct _location {
    int32_t       _lat;  // degrees * 1e7, negative is South
    int32_t       _lon;  // degrees * 1e7, negative is West
};
#endif

//------------------------------------------------------------


struct _point
{
	uint32_t        time;
	_location		location;
};

struct _track_meta
{
	uint32_t points;
	uint32_t start_time;
	uint32_t finish_time;
	uint32_t time;
	_location start_finish;
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
int16_t orientation(int16_t heading, int16_t bearing) {
	return mod(bearing - heading + 180, 360) - 180;
}

int16_t offsetCorrection(float heading, float distance, float speed, float bearing) {
	static int16_t delta, ori;

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
    py::class_<_location>(m, "_location")
        .def(py::init<>())
		.def_readwrite("lon", &_location::_lon)
		.def_readwrite("lat", &_location::_lat);
    py::class_<_point>(m, "_point")
        .def(py::init<>())
		.def_readwrite("time", &_point::time)
		.def_readwrite("location", &_point::location);
    py::class_<_track_meta>(m, "_track_meta")
        .def(py::init<>())
		.def_readwrite("points", &_track_meta::points)
		.def_readwrite("start_time", &_track_meta::start_time)
		.def_readwrite("finish_time", &_track_meta::finish_time)
		.def_readwrite("time", &_track_meta::time)
		.def_readwrite("start_finish", &_track_meta::start_finish)
		.def_readwrite("lap", &_track_meta::lap)
		.def_readwrite("session", &_track_meta::session);
}
#endif
