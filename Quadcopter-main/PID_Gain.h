/*  PID Gain    */  

#include "mbed.h"

/*  For Attitude   */
float K_scale = 0.01;
float Kp_roll = 8.7, Kd_roll = 54.0;  //8.5-11.0    8.7- 11.0
float Kp_pitch = 8.5, Kd_pitch = 54.0;
float Kp_yaw = 1.0, Kd_yaw = 31.0;
//-----------------------------------------------------------//

/*  For Altitude    */

float alt_p = 1.0, alt_d = 0.1, alt_i = 0.01;
//-----------------------------------------------------------//
/*  For Controller  */
/*
// inner loop
float D_I[0] = 4.5; //0.5 * 65.536; //ROLL 45 //~3000/100
float D_I[1] = 4.5; //0.5 * 65.536;
float D_I[2] = 6; //0.5 * 65.536; //YAW damping 48
float A_I[0] = 4.5; A_I[1] = 4.5; A_I[2] = 4.5; //10 * 16.4

// outer loop Angle FB
float P_O[0] = 0;
float P_O[1] = 0;
float P_O[2] = 0;    
float P_O[3] = 15;//800; //6000/100 10 too small
//-----------------------------------------------------------//
*/