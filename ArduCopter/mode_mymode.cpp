/*
 * mode_mymode.cpp
 *
 *  Created on: 18 feb. 2021
 *      Author: lizbt
 */


#include "copter.h"
#include "UserVariables.h"

#define kpT -4
#define kdT -.19738

#define kpA -2
#define kdA 4.2216
//#define kiA

void ModeMymode::run(){

	//angulos deseados
	Th_d=0;
	alabeo_d=0;

	g.a_roll=ahrs.roll;
	const AP_InertialSensor &ins = AP::ins();
	const Vector3f &gyroRAW = ins.get_gyro();

	//hal.console->printf("Gyro: %.2f\n",gyroRAW.z);

	//control de cabeceo
	PD_Th=kpT*(Th_d-ahrs.pitch)-kdT*gyroRAW.z;
	//control de alabeo delta_a pag 102
		//checar si si es y
		//omitiendo kiA
	PD_aleron=kpA*(alabeo_d-gyroRAW.y) - (kdA*gyroRAW.y);

	//control de guiñada


}

