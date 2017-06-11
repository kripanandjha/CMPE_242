#ifndef __PID_CONT
#define __PID_CONT

/*
 * 1d_con.cpp contains source code the binary to support 1d convolution
 * 1d_con binary ask user which kind convolution they wants like this
 * 1. Forward difference
 * 2. Backward difference
 * 3. Central difference
 *
 * kripanand_pid_cont.cpp contains source to support pid control
 * kripanand_pid_cont binary gives the pid value and
 * and various Error and samplling output at each 30ms interval
 */
// define all macros here
/*
 *  Sum_pid = alpha_p * K_p * Error(n) + alpha_i * K_i * iDError(n) + alpha_d
 			* K_d * cDError(n);
 *
 */
#define pi			3.1415926
#define f_pwm		200 // 200 Hz
#define f_sample	30 // take lateral distance sample every 30 ms
#define SPDVHL		5000 // vehicle speed 5 KM per hour
#define PWMMOT		500 // pwm max frequency to drive the motor controller 500 hz
#define ANGSTR		pi/3 // max steering angle
//#define STPMOT		0.225 // 1/8 micro step of the steering motor 
#define STPMOT		1.8 // 1 step of the steering motor 
#define RADWHL		0.3 // radius of the wheel
#define ANGSMP		30 // angle sampling rate from i2c sensor





#define number_IError	4

// currently using only arbitrary value will change later
/// based on auto-tune or hit by trial
// this value is best till now 
#define K_p 	30
#define K_i 	20
#define K_d		5
#if 1
#define alpha_p		1
#define alpha_d		1
#define alpha_i		1
#endif
#if 0
#define alpha_p		0.33
#define alpha_d		0.33
#define alpha_i		0.33
#endif
#define minError	-1000.0
#define maxError	 1000.0
#if 0
Error[n]           // for error 
DError[n]          // derivative of error 
FDError[n]         // forward difference computation for the derivative of error
BDError[n]         // backward difference computation for the derivative of error
CDError[n]         // central difference computation for the derivative of error
IError[n]          // integral of error (squared each individual error) 
SumError[n]        // summation of pid errors 
#endif
//Note 
//(1) define Error[n] range for steering, e.g., clip it to the following range 
#define ErrorMin        -1000.0// min error limit 
#define ErrorMax        1000.0 // max error limit
//set any error value to its max or min error limit for error computed from the pid 
//controller.


#define setPoint 	1
#endif
