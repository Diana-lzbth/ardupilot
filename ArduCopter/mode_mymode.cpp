/*
 * mode_mymode.cpp
 *
 *  Created on: 18 feb. 2021
 *      Author: lizbt
 */


#include "copter.h"
//#include "UserVariables.h"

void ModeMymode::run(){

	g.a_roll=ahrs.roll;
	const AP_InertialSensor &ins = AP::ins();
	const Vector3f &gyroRAW = ins.get_gyro();

	hal.console->printf("%.2f\n",(float)gyroRAW[1]);

}

