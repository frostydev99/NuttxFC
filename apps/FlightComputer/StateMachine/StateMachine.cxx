#include <stdio.h>

#include "sensors/LPS25/LPS25.h"

LPS25 barometer = LPS25();

void initFlightComputer() {

	barometer.init();
}

extern "C" int FC_StateMachine_main(int argc, char *argv[])
{
	printf("Starting Flight Computer...\n");

	initFlightComputer();

  	return 0;
}