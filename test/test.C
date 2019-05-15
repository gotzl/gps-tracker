/*
 * test.C
 *
 *  Created on: May 13, 2019
 *      Author: gotzl
 */

#define TEST
#include "../gps_tracker.hpp"


int main() {
	for(int i=0;i<=360;i+=2)
		printf("%i %i\n",i, offsetCorrection(0,0.020,80,i));
	for(int i=0;i<=360;i+=2)
		printf("%i %i\n",i, offsetCorrection(i,0.020,80,0));

	for(int i=0;i<=360;i+=2)
		printf("%i %i\n",i, offsetCorrection(154,0.020,80,i));
	for(int i=0;i<=360;i+=2)
		printf("%i %i\n",i, offsetCorrection(i,0.020,80,248));
}
