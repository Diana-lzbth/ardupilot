// user defined variables

// example variables used in Wii camera testing - replace with your own
// variables

// ----------- Ganancias originales ---------
//#define kpT -4
//#define kdT -.19738

// #define kpA -4
// #define kdA 2.2216

// #define kp_b -1.75
// #define ki_b -3.0872
// ------------------------------------------
#define	PI	3.14159

// ---------> Variables de control pitch <-----------
#define kpT -4
#define kdT -.8

// ---------> Variables de control roll <------------
#define kpA -4
#define kdA 4.2216

// ---------> Variables de control yaw <-------------
#define kp_b -4.75
#define ki_b -3.0872

// ---------> Variables auxiliares del control <-----
float PD_pitch , Th_d     = 0.0;  // PD_Theta, Theta deseado
float PD_roll  , alabeo_d = 0.0;  // PD_alerones, angulo alabeo deseado
float PI_yaw   , yaw_d = 0.0;     // PI_ruder, angulo yaw deseado

struct radio_rc{
	int radio_roll[40];
	int radio_pitch[40];
	int radio_yaw[40];
	int radio_th[40];
	int aux_1;
	int volatile aux_2;
}rd;


// Variables de gyro en grados/s
float x_grad_s, y_grad_s, z_grad_s,x_gyro,y_gyro,z_gyro;

// Variables de salida PWM
float  sw_aux_1, sw_aux_2, aux_roll;

//Variable manejo arreglo de lectura RC
int ii,jj,aux;

//Variables de Manejo de memoria
struct PACKED log_Pose{
    LOG_PACKET_HEADER;
    float    Roll;
    float    Pitch;
    float    Yaw;
    float    giroz;
    float    girox;
    float    giroy;
};


