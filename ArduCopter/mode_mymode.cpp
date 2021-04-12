/*
 * mode_mymode.cpp
 *
 *  Created on: 18 feb. 2021
 *      Author: Edson E.G.P.
 */


#include "Copter.h"
#include "UserVariables.h"


bool ModeMymode::init(bool ignore_checks){

	ii=0;	//contador arreglo circular
	hal.storage->init();
	g.offset=0;
	return true;
}


void ModeMymode::run(){

	const AP_InertialSensor &ins = AP::ins();
	const Vector3f &gyro = ins.get_gyro();

	// ---------> Lectura de canales de IMU y asignacion a variables globales <-----------
	g.IMU_deg_roll  = (ahrs.roll  * 180) / PI ;
	g.IMU_deg_pitch = (ahrs.pitch * 180) / PI ;
	g.IMU_deg_yaw   = (ahrs.yaw   * 180) / PI ;

	x_grad_s = (gyro.x * 180)/PI;
	y_grad_s = (gyro.y * 180)/PI;
	z_grad_s = (gyro.z * 180)/PI;

	x_gyro = gyro.x;
	y_gyro = gyro.y;
	z_gyro = gyro.z;


	// ---------> Leer switches <----------
	rd.aux_1       = rc().get_radio_in(4);
	rd.aux_2       = rc().get_radio_in(5);

	sw_aux_1    = rd.aux_1 ;
	sw_aux_2    = rd.aux_2 ;

	// ---------------------------- Modo manual -----------------------------------------
	if( sw_aux_2 < 1300 ){

		// ---------> Lectura y asignacion de canales RC <---------
		rd.radio_roll[ii]  = copter.channel_roll->get_radio_in();
		rd.radio_pitch[ii] = copter.channel_pitch->get_radio_in();
		rd.radio_th[ii]    = copter.channel_throttle->get_radio_in();
		rd.radio_yaw[ii]   = copter.channel_yaw->get_radio_in();

		// ---------> Manejo de arreglo circular RC <---------
		if(ii==40){ii=0;}
		else{ii++;}

		// ---------> Promediar lecturas RC <---------
		// Roll
		aux=0;
		for(jj=0;jj<40;jj++)
			aux=aux + rd.radio_roll[jj];
		g.RC_roll_b=aux/40;

		// Pitch
		aux=0;
		for(jj=0;jj<40;jj++)
			aux=aux + rd.radio_pitch[jj];
		g.RC_pitch=aux/40;

		//Yaw
		aux=0;
		for(jj=0;jj<40;jj++)
			aux=aux + rd.radio_yaw[jj];
		g.RC_yaw=aux/40;

		//Throttle
		aux=0;
		for(jj=0;jj<40;jj++)
			aux=aux + rd.radio_th[jj];
		g.RC_throttle=aux/40;
	}

	// -------------------------- Modo automático --------------------------------

	if( sw_aux_2 > 1800){

		PD_pitch = kpT * ( Th_d - g.IMU_deg_pitch) - (kdT * y_grad_s) ;
		PD_roll  = kpA * (alabeo_d - g.IMU_deg_roll) - (kdA * x_grad_s);
		PI_yaw   = -kp_b*g.IMU_deg_yaw - (ki_b * z_grad_s);

		g.RC_pitch     = (int)(1500.0 + (PD_pitch*4.5) ) ;
		g.RC_roll_b    = (int)(1500.0 +      (PD_roll) ) ;
		g.RC_yaw       = (int)(1500.0 +       (PI_yaw) ) ;
		g.RC_throttle  = copter.channel_throttle->get_radio_in();

		//hal.console->printf("%d\t",(int)g.RC_yaw);

		// ---------> Ajuste de límites del control <---------
		if( g.RC_pitch > 2000 )
			g.RC_pitch = 2000;

		if( g.RC_pitch < 1000 )
			g.RC_pitch = 1000;

		if( g.RC_roll_b > 2000 )
			g.RC_roll_b = 2000;

		if( g.RC_roll_b < 1000 )
			g.RC_roll_b = 1000;

		if( g.RC_yaw > 2000 )
			g.RC_yaw = 2000;

		if( g.RC_yaw < 1000 )
			g.RC_yaw = 1000;
	}
	if(g.Bandera_10Hz == 1){
	//HAL_STORAGE_SIZE=16384 detination source size

	//	 for(g.offset = 0; g.offset < 80; g.offset += 8)
	//	    {
	//	     hal.storage->write_block(dst, src, n)
	//		 	hal.storage->write_block((uint)g.bufff, (void *)g.bufff, sizeof(g.bufff));
	//		 	hal.storage->read_block((void *) g.bufff, g.offset, sizeof(g.bufff));
	//	        hal.console->printf(" Se tiene %d ", g.bufff[g.offset]);
	//	    }

	//	hal.storage->write_block(g.RC_pitch, src, sizeof(uint16_t));
	//	hal.storage->write_block(x_grad_s, &x_grad_s, sizeof(x_grad_s));
	//	hal.storage->read_block((void *)bufff,&x,8);
		hal.console->printf(" FIN\t");

		//	DataFlash.WriteBlock(&x_grad_s, sizeof(x_grad_s));
		g.Bandera_10Hz = 0;
	}
}
