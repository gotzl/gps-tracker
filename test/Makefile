test: test.C
	gcc -o test test.C -I. -lm

gps_tracker.o: ../tracker/gps_tracker.hpp
	g++ -DPYTHON -O3 -Wall -fPIC `python3 -m pybind11 --includes` -c ../tracker/gps_tracker.hpp -o gps_tracker.o

lib: gps_tracker.o
	g++ -shared -Wl,-soname,libTest.so -o libTest.so gps_tracker.o

python: gps_tracker.o
	g++ -DPYTHON -O3 -Wall -shared -std=c++11 -fPIC `python3 -m pybind11 --includes` python.cpp -o example`python3-config --extension-suffix`

