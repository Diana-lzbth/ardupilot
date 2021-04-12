#include "Copter.h"
//#include <AP_GPS/AP_GPS.h>
//#include "UserVariables.h"
//#include "AP_BoardConfig/AP_BoardConfig.h"


//const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*********************************COMPASS***************************/

//static AP_BoardConfig board_config;

//class DummyVehicle {
//public:
    //AP_AHRS_DCM ahrs;  // Need since https://github.com/ArduPilot/ardupilot/pull/10890
  //  AP_Baro baro; // Compass tries to set magnetic model based on location.
//};

//static DummyVehicle vehicle;
// create compass object
//static Compass compass;

//uint32_t timer;


/*******************************************************************/


#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
//	hal.rcout->force_safety_off();
//	AC_PosControl->set_alt_target_to_current_alt();

    // put your initialisation code here
    // this will be called once at start-up

	// Initialize the UART for GPS system
//	serial_manager.init();
	//AP_BoardConfig BoardConfig;

//	gps.init(serial_manager);
//	BoardConfig.io_enabled();
//	hal.rcout->set_default_rate(490);

//	hal.rcout->init();
//	hal.rcout->enable_px4io_sbus_out(490);
/*	hal.rcout->enable_ch(0);
	hal.rcout->enable_ch(1);
	hal.rcout->enable_ch(CH_2);
	hal.rcout->enable_ch(CH_3);*/
	//hal.rcout->set_freq(CH_1, 50);

//    hal.rcin->init();
 //   hal.rcout->init();

//    uint8_t i = 0;

/*    hal.console->printf("Starting AP_HAL::RCOutput test\n");
 #if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
     BoardConfig.init();
 #endif

    for (uint8_t i = 0; i< 8; i++) {
        hal.rcout->enable_ch(i);
        hal.rcout->set_freq(i, 490);
    }
*/

/*	copter.motors->armed(true);
	hal.rcout->force_safety_off();
*/

// ****************************SETUP BRUJULA**********************************

	//hal.console->printf("Compass library test\n");
	//board_config.init();
	//vehicle.ahrs.init();
//	compass.init();

	// set offsets to account for surrounding interference
	//compass.set_and_save_offsets(0, Vector3f(0, 0, 0));
	// set local difference between magnetic north and true north
	//compass.set_declination(ToRad(0.0f));

	//timer = AP_HAL::micros();

// **************************************************************************



}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz codes here


 //	g.Bandera_50Hz = 1;
 /*   for (uint8_t i = 0; i < 8; i++) {
    	hal.rcout->write(i, Mypwm);
    	Mypwm += Mydelta;
    	if (Mydelta > 0 && Mypwm >= 2000) {
    		Mydelta = -1;
    		hal.console->printf("reversing\n");
    	} else if (Mydelta < 0 && Mypwm <= 1000) {
    		Mydelta = 1;
    		hal.console->printf("normalizing\n");
    	}
    }*/
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
//	valor_RC_8 = hal.rcin->num_channels();

//	MyRC.pitch    = hal.rcin->read(1); 		//Palanca derecha: abajo y arriba de 1004 a 2000, 1500 nominal


//	RC_Channel_aux::set_servo_out(CH_4,4500);

/*	if(MyRC.roll >= 2000){
		MyRC.roll = 2000;
	}
	else if(MyRC.roll <= 1000){
		MyRC.roll = 1000;

	}*/
/*	MyRC.pitch    = hal.rcin->read(1); 		//Palanca derecha: abajo y arriba de 1004 a 2000, 1500 nominal
	MyRC.throttle = hal.rcin->read(2);		//Palanca izquierda: abajo y arriba de 1004 a 2000, 1004 nominal
	MyRC.yaw      = hal.rcin->read(3);		//Palanca izuierda : derecha a izquierda de 1000 a 1980, 1500 nominal
	MyRC.aux_1    = hal.rcin->read(4);		// SW -> regulador izquierdo de 999 a 2000
	MyRC.aux_2    = hal.rcin->read(5);		// VRB -> regulador derecho de 999 a 2000


	PWM_Roll = MyRC.roll;
	if(MyRC.aux_1>=1800){

		hal.rcout->write(0,1900);
//		hal.console->printf(" 1800 ");
	}
	else{
		hal.rcout->write(0,1001);
//		hal.console->printf(" 1000 ");
	}*/

//	hal.rcout->cork();
//	hal.rcout->write(2,(uint16_t)MyRC.throttle);
	//hal.rcout->write(CH_4,(uint16_t)MyRC.throttle);
//	hal.rcout->push();

//	last_read =hal.rcout->read(CH_1);

//	const Vector3f &Position_GPS = inertial_nav.get_velocity();

//	hal.console->printf(" %d ",(int)copter.get_throttle_mid());
//	hal.console->printf(" %f ",(float)g.IMU_deg_yaw);
//	hal.scheduler->delay(500);

//	hal.rcout->write(0,g.RC_roll);
//	hal.rcout->write(1,g.RC_pitch);
//	hal.rcout->write(2,g.RC_throttle);
/*
	if (hal.util->get_soft_armed()) {

			hal.console->printf("Software armed") ;
			    	//return;
		}*/
// *********************************************COMPASS*****************************************************
/*	hal.console->printf("init done - %u compasses detected\n", compass.get_count());

	static const uint8_t compass_count = compass.get_count();
		    static float min[COMPASS_MAX_INSTANCES][3];
		    static float max[COMPASS_MAX_INSTANCES][3];
		    static float offset[COMPASS_MAX_INSTANCES][3];

		    // run read() at 10Hz
		   // if ((AP_HAL::micros() - timer) > 100000L) {
		     //   timer = AP_HAL::micros();
		        compass.read();
		        //const uint32_t read_time = AP_HAL::micros() - timer;

		        for (uint8_t i = 0; i < compass_count; i++) {
		            float heading;

		            hal.console->printf("Compass #%u: ", i);

		            if (!compass.healthy()) {
		                hal.console->printf("not healthy\n");
		                continue;
		            }

		            Matrix3f dcm_matrix;
		            // use roll = 0, pitch = 0 for this example
		            dcm_matrix.from_euler(0, 0, 0);
		            heading = compass.calculate_heading(dcm_matrix, i);

		            const Vector3f &mag = compass.get_field(i);

		            // capture min
		            min[i][0] = MIN(mag.x, min[i][0]);
		            min[i][1] = MIN(mag.y, min[i][1]);
		            min[i][2] = MIN(mag.z, min[i][2]);

		            // capture max
		            max[i][0] = MAX(mag.x, max[i][0]);
		            max[i][1] = MAX(mag.y, max[i][1]);
		            max[i][2] = MAX(mag.z, max[i][2]);

		            // calculate offsets
		            offset[i][0] = -(max[i][0] + min[i][0]) / 2;
		            offset[i][1] = -(max[i][1] + min[i][1]) / 2;
		            offset[i][2] = -(max[i][2] + min[i][2]) / 2;

		            // display all to user
		            hal.console->printf("Heading: %.2f (%3d, %3d, %3d)",
		                                (double)ToDeg(heading),
		                                (int)mag.x,
		                                (int)mag.y,
		                                (int)mag.z);

		            // display offsets
		            hal.console->printf(" offsets(%.2f, %.2f, %.2f)",
		                                (double)offset[i][0],
		                                (double)offset[i][1],
		                                (double)offset[i][2]);

		          //  hal.console->printf(" t=%u", (unsigned)read_time);

		            hal.console->printf("\n");
		        }
		    //}

*/
// *************************************************************************************************************

}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
    // put your 1Hz code here
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif
