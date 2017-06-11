#include <vector>
#include <cstdint>
#include <climits>
#include <cmath>
#include <iostream>
#include <cstdio>
#include <cstring>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <cstdlib>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <linux/fs.h>
#include <fcntl.h>

#include "24cXX.h"
#include "pid_cont.hpp"

#define PWM_IOCTL_SET_FREQ		1
#define PWM_IOCTL_STOP			0
using namespace std;

int fd,fd3;
int fd_i2c;
float Pi = 3.14159;
double Error = 0;

double get_angle_lpcnod(void)
{
	double heading ;

	char c, data[10];
	int id = 0;
	//double comp_angl = 0;
	while(1) {
		read(fd3, &c, 1);
		if(c == 'x')
			break;

		data[id] = c;
		id++;
	}
	data[id] = '\0';

	sscanf(data, "%lf", &heading); // subtract this in later readings
	return heading;
}
int main()
{
	// pid controller calculate error
	// input data x(n) = 1 | for n = 0 .. 9
	// initially Contl(n) = 0 | Error(0) = x(0) - Contl(0) = 1
	// change input based on the error so that it should minimize the Error(n) ~ 0

	// currently took some arbitrary value of K_p, K_d, K_i 
	// may change in future of these values
	// compute cDError(n) by central difference

	// compute iDError(n) by summation of past values
	// for iDError(n-k) if n-k < 0, then use 0 value for Error(n-k) = 0 if n-k < 0


/*
 *  Sum_pid = alpha_p * K_p * Error(n) + alpha_i * K_i * iDError(n) + alpha_d
 			* K_d * cDError(n);
 *
 */
	vector<double> der_point;
	vector<double> Y(1000, 1); // this will be constant to value 1
	vector<double> Contl(1000, 0);
	double last_error[1000] = {0};
	// if Error is -ive than overshoot
	last_error[0] = Y[0] - Contl[0]; // boundary condition repitive

	int n = 0;

	double iDError = 0;
	double max = 0;
	double min = INT_MIN;
	//int F_pwm = f_pwm;
	double F_pwm = f_pwm;
	double pos_x[1000]={0};
	double pos_y[1000]={0};
	pos_y[0] = Contl[0];
	double Angl = 0;
	fd3 = open("/dev/ttySAC3", O_RDWR);
	if(fd3 < 0)
	{
		printf("unable to open uart 3\n");
		return -1;
	}
	char c, data[10];
	int id = 0;
	double comp_angl = 0;
	while(1) {
		read(fd3, &c, 1);
		if(c == 'x')
			break;

		data[id] = c;
		id++;
	}
	data[id] = '\0';
	sscanf(data, "%lf", &comp_angl); // subtract this in later readings
	memset(data,'\0', sizeof(data));
	
	uint8_t ij = 1;
	Error = 1;
	// open pwm device
	while(1) {
		ij =  Error*100;
		if(ij == 0)
			break;

		
		Error = Y[n] - Contl[n];
		//const double kp_gain = K_p * error;
		cout << "Error " << Error << endl;

		int x = n-1;

		if(n-1 < 0)
			x = 0;
		if(n+1 > 1000)
			x = 0;

		//double Angl_future	= (STPMOT * pi / 180) * 0.03 * (F_pwm); //
	//	double Angl_future	= angle_lsm303(); // initially angle should be zero
		//double cDError = K_d * (future_er - Contl[x])/2; // central difference
		double cDError = (Contl[n] - Contl[x]); // Backward difference
		//cout << "Error(n+1) " << 1-future_er << endl;
		cout << "cDError " << cDError << endl;

		int ii = n;
		int count = 0;
		last_error[n] = Error;
		double iDError1 = 0;

		while(count < 4){
			if (ii < 0)
				break;
			// expecting discrete area always +ve
			iDError1 += (last_error[ii] * last_error[ii]); // need to use only last 4 values

			count++;
			ii--;
		}
		iDError = iDError1; // only add last 4 values
		cout << "iDError " << iDError << endl;

		//double Sum_pid = alpha_p * K_p * Error + alpha_i * K_i * iDError + alpha_d * K_d * cDError;
		// if setput or input value is constant use this formula
		// it is called derivative on measurement
		double Sum_pid = alpha_p * K_p * Error + alpha_i * K_i * iDError + alpha_d * K_d * cDError;
		//double Sum_pid = alpha_p * K_p * Error ;
#if 0	
		if(Sum_pid > max)
			max = Sum_pid;
		if(Sum_pid < min)
			min = Sum_pid;
#endif
   // filtering unwanted noise out of window
		// http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup/
#if 1
		if(Sum_pid > 107)  Sum_pid = 107;

		else if(Sum_pid < 0) Sum_pid = 0;
		cout << "Sum_pid " << Sum_pid << endl;

#endif
		cout << "value of Sum_pid " << Sum_pid << endl;
		// got new Sum_pid value now what we have to do 

		// convert sum_pid into no. of pulse applied to plant
		// assuming Sum_pid rangs 0-115
		F_pwm = (float)f_pwm * (Sum_pid/108);
		if(n == 0)
			F_pwm = f_pwm;
#if 0
		if(F_pwm == 0)
			F_pwm = 1; // pwm
#endif
		cout << "F_pwm pulse " << F_pwm << endl;

		
		// update pwm fulse with this latest F_pwm values
		//set_motor_freq(F_pwm);
		snprintf(data,sizeof(data)-1, "%lf", F_pwm);
		// sent this pwm frequency to LPCNOD
		if(Angl < 0)
			data[strlen(data)] = 'x';
		
		else if(Error < 0)
			data[strlen(data)] = 'y';

		if((Angl >= 0) && (Error > 0))
			data[strlen(data)] = 'x';


		//data[strlen(data)+1] = '\0';
	printf("before\n");
		write(fd3, data, sizeof(data));
	printf("after\n");
	memset(data, '\0', sizeof(data));

		// give 30 ms delay
		//usleep(30000);

		// calculate angle movement after 30 ms from lsm303
	//	Angl = angle_lsm303();	 // in radian
		Angl = get_angle_lpcnod() ;
			//comp_angl;

		cout <<"data " << data << endl;

		cout << "lpc_angl " << Angl*180/Pi << " comp_angl " << comp_angl*180/Pi  << " Angl " << (Angl-comp_angl)*180/Pi<< endl;
		Angl =  Angl - comp_angl;

		// car velocity is 5 km /hr = 1.388 m/sec
		// converting 1.388 m into cm 138.8 cm per second
		// converting into 30 ms distance will be
		double dist_min = 1.388 * 0.03 * (sin(Angl));
		double dist_x = 1.388 * 0.03 * (cos(Angl)); //horizontal distance

		//double disVen = dist_min ;

		n++;
		Contl[n] = Contl[n-1] + dist_min;
		pos_y[n] = Contl[n];
		pos_x[n] = pos_x[n-1] + dist_x;
		cout << "dist_min " << dist_min << " Contl "<< Contl[n] << endl <<"-----" << endl ;
	}

	cout << "max sum_pid " << max << endl;
	cout << "min sum_pid " << min << endl;
#if 1
	
	for(int k = 0;k<1000;k++)
	{
		cout << pos_x[k] << endl;
	}
	cout << "position y " << endl;
	for(int k = 0;k<1000;k++)
	{
		cout << pos_y[k] << endl;
	}
#endif
	return 0;
}
