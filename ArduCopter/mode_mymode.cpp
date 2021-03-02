/*
 * mode_mymode.cpp
 *
 *  Created on: 18 feb. 2021
 *      Author: lizbt
 */


#include "Copter.h"
#include "UserVariables.h"
#include "RC_Channel.h"

#define kpT -4
#define kdT -.19738

#define PI	3.14159

#define kpA -2
#define kdA 4.2216


//#define kiA

bool ModeMymode::init(bool ignore_checks)
{
    // initialise position and desired velocity
 /*   if (!pos_control->is_active_z()) {
        pos_control->set_alt_target_to_current_alt();
 //       pos_control->set_desired_velocity_z(inertial_nav.get_velocity_z());
    }*/

//	copter.enable_motor_output();

//	copter.default_dead_zones();
	hal.rcout->enable_ch(CH_1);
	hal.rcout->enable_ch(CH_2);
	hal.rcout->enable_ch(CH_3);
	hal.rcout->set_freq(CH_1, 50);
	hal.rcout->set_freq(CH_2, 50);
	hal.rcout->set_freq(CH_3, 490);

	copter.motors->armed(true);
	hal.rcout->force_safety_off();

    return true;
}

void ModeMymode::run(){

	//angulos deseados
//	Th_d=0;
//	alabeo_d=0;

//	g.a_roll=ahrs.roll;
//	const AP_InertialSensor &ins = AP::ins();
//	const Vector3f &gyroRAW = ins.get_gyro();

//	hal.console->printf("Gyro: %.2f\n",gyroRAW.z);

	//control de cabeceo
//	PD_Th=kpT*(Th_d-ahrs.pitch)-kdT*gyroRAW.z;
	//control de alabeo Mydelta_a pag 102
		//checar si si es y
		//omitiendo kiAS
//	PD_aleron=kpA*(alabeo_d-gyroRAW.y) - (kdA*gyroRAW.y);
//	MyRC.roll     = hal.rcin->read(0);		//Palanca derecha: izquierda a derecha de 1013 a 1998, 1498 nominal

// --------------- Lectura de canales de IMU y asignacion a variables globales ----------------------------
	g.IMU_deg_roll  = (ahrs.roll  *180) / PI;
	g.IMU_deg_pitch = (ahrs.pitch *180) / PI;
	g.IMU_deg_yaw   = (ahrs.yaw   *180) / PI;

// --------------- Lectura de canales RC y asignacion a variables globales ----------------------------

	g.RC_roll     = hal.rcin->read(0);		//Palanca derecha: izquierda a derecha de 1013 a 1998, 1498 nominal
	g.RC_pitch    = hal.rcin->read(1); 		//Palanca derecha: abajo y arriba de 1004 a 2000, 1500 nominal
	g.RC_throttle = hal.rcin->read(2);		//Palanca izquierda: abajo y arriba de 1004 a 2000, 1004 nominal
	g.RC_yaw      = hal.rcin->read(3);		//Palanca izuierda : derecha a izquierda de 1000 a 1980, 1500 nominal
	g.RC_aux_1    = hal.rcin->read(4);		// VRA -> regulador izquierdo de 999 a 2000
	g.RC_aux_2    = hal.rcin->read(5);		// VRB -> regulador derecho de 999 a 2000

	// --------------- Lectura de posicion en GPS y asignacion a variables globales ----------------------------
	const Vector3f &Position_GPS = inertial_nav.get_velocity();

	g.GPS_pos_x = Position_GPS[0];
	g.GPS_pos_y = Position_GPS[1];
	g.GPS_pos_z = Position_GPS[2];

	/*    for (uint8_t i=0; i < 14; i++) {
    	i = i + 1;
//        hal.rcout->write(i, Mypwm);
        Mypwm += Mydelta;
        if (Mydelta > 0 && Mypwm >= 2000) {
            Mydelta = -1;
 //           hal.console->printf("decreasing\n");
        } else if (Mydelta < 0 && Mypwm <= 1000) {
            Mydelta = 1;
//            hal.console->printf("increasing\n");
        }
    }*/
//	hal.rcout->write(0, MyRC.roll);
	if (g.Bandera_50Hz == 1){
		hal.rcout->write(CH_1, Mypwm);
		hal.rcout->write(CH_2, Mypwm);
//
		Mypwm += Mydelta;
		if (Mydelta > 0 && Mypwm >= 2000) {
			Mydelta = -1;
//			hal.console->printf(" reversing ");
		}
		else if (Mydelta < 0 && Mypwm <= 1000) {
			Mydelta = 1;
//			hal.console->printf(" normalizing ");
		}
		g.Bandera_50Hz = 0;
		hal.console->printf(" 1 ");
	}
	else{
		hal.console->printf(" 0 ");
	}
	hal.rcout->write(CH_3, Mypwm);
}

