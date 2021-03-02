// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables
#ifdef USERHOOK_VARIABLES

#define MAX_RC 2000
#define MIN_RC 00
#define SERVO_HZ 50

#define ENABLE ENABLED
#define DISABLE DISABLED


	float Th_d, PD_Th;// Theta_deseado, PD_Theta
	float PD_aleron,alabeo_d; //PD_alerones, angulo alabeo_deseado

	uint16_t PWM_Roll;

//	bool bandera_50Hz = false;

    uint16_t Mypwm = 1500;
    int8_t Mydelta = 1;

 //   uint8_t i =  0;



#endif  // USERHOOK_VARIABLES


