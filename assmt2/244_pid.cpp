#include <vector>
#include <climits>
#include <cmath>
#include <iostream>
#include "pid_cont.hpp"
using namespace std;
// include conv obj. file during compilation to add conv_valid support

vector<double> conv_valid(vector<double> const &f, vector<double> const &g, bool f_diff);

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
	vector<double> x(1000, 1); // this will be constant to value 1
	vector<double> Contl(1000, 0);
	double last_error[1000] = {0};

	last_error[0] = x[0] - Contl[0]; // boundary condition repitive

	int n = 0;

	double iDError = 0;
	double max = 0;
	double min = INT_MIN;
	double F_pwm = f_pwm;
	double pos_x[1000]={0};
	double pos_y[1000]={0};
	pos_y[0] = Contl[0];
	while(n < 1000) {
		double Error = x[n] - Contl[n];

		//const double kp_gain = K_p * error;
		cout << "Error " << Error << endl;

		int x = n-1;

		if(n-1 < 0)
			x = 0;
		if(n+1 > 1000)
			x = 0;


		double Angl_future	= (STPMOT * pi / 180) * 0.03 * (F_pwm); // in place of Sum_pid we can place pwm_pulse

		double dist_min1 = 1.388 * 0.03 * (sin(Angl_future));
		double future_er = Contl[n] + dist_min1;

//		double cDError = K_d * (Contl[n] - Contl[x]); // backward difference
		double cDError = K_d * (future_er - Contl[x])/2; // central difference
		cout << "Error(n+1) " << 1-future_er << endl;
		cout << "cDError " << cDError << endl;

		int ii = n;
		int count = 0;
		last_error[n] = Error;
		double iDError1 = 0;
			
		while(count < 4){
			if (ii < 0)
				break;
			// expecting discrete area always +ve
			iDError1 += K_i * (last_error[ii] * last_error[ii]); // need to use only last 4 values

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
		if(Sum_pid > max)
			max = Sum_pid;
		if(Sum_pid < min)
			min = Sum_pid;

		cout << "Sum_pid " << Sum_pid << endl;
#if 1   // filtering unwanted noise out of window
		// http://brettbeauregard.com/blog/2011/04/improving-the-beginner%E2%80%99s-pid-reset-windup/
#if 1
		if(Sum_pid > 115)  Sum_pid = 115;

		else if(Sum_pid < 0) Sum_pid = 0;
#endif
#endif
		cout << "value of Sum_pid " << Sum_pid << endl;
		// got new Sum_pid value now what we have to do 

//		double Angl	= steps_angle*pwm_pulse;
		// convert sum_pid into no. of pulse applied to plant
		// assuming Sum_pid rangs 0-4095
		F_pwm = (float)f_pwm * Sum_pid/115;
		if(n == 0)
			F_pwm = f_pwm;
		cout << "F_pwm pulse " << F_pwm << endl;
 		double Angl	= (STPMOT * pi / 180) * 0.03 * (F_pwm); // in place of Sum_pid we can place pwm_pulse
//		double Angl	= STPMOT * Sum_pid * ((float)f_pwm / 9); // in place of Sum_pid we can place pwm_pulse

		cout << "Angl in radian " << Angl << endl;

		// double distance = 2 * 3.14 * 0.3 * Angl;
		// car velocity is 5 km /hr = 1.388 m/sec
		// comverting 1.388 m into cm 138.8 cm per second
		// comverting into 30 ms distance will be
		double dist_min = 1.388 * 0.03 * (sin(Angl));
		double dist_x = 1.388 * 0.03 * (cos(Angl)); //horizontal distance

		double disVen = dist_min ;

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
