#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <linux/fs.h>
#include <errno.h>
#include <string.h>

#include <math.h>
struct Complex
{	double a;        //Real Part
	double b;        //Imaginary Part
}          X[5], U, W, T, Tmp;

void FFT(void)
{
	int M = 3; // change it to log n 
	int N = pow(2, M);

	int i = 1, j = 1, k = 1;
	int LE = 0, LE1 = 0;
	int IP = 0;

	for (k = 1; k <= M; k++) // totoal stages of n samples log (n)
	{
		LE = pow(2, M + 1 - k);
		LE1 = LE / 2;

		U.a = 1.0;
		U.b = 0.0;

		W.a = cos(M_PI / (double)LE1);
		W.b = -sin(M_PI/ (double)LE1);

		for (j = 1; j <= LE1; j++)
		{
			for (i = j; i <= N; i = i + LE)
			{
				IP = i + LE1;
				T.a = X[i].a + X[IP].a;
				T.b = X[i].b + X[IP].b;
				Tmp.a = X[i].a - X[IP].a;
				Tmp.b = X[i].b - X[IP].b;
				X[IP].a = (Tmp.a * U.a) - (Tmp.b * U.b);
				X[IP].b = (Tmp.a * U.b) + (Tmp.b * U.a);
				X[i].a = T.a;
				X[i].b = T.b;
			}
			Tmp.a = (U.a * W.a) - (U.b * W.b);
			Tmp.b = (U.a * W.b) + (U.b * W.a);
			U.a = Tmp.a;
			U.b = Tmp.b;
		}
	}

	int NV2 = N / 2;
	int NM1 = N - 1;
	int K = 0;

	j = 1;
	for (i = 1; i <= NM1; i++)
	{
		if (i >= j) goto TAG25;
		T.a = X[j].a;
		T.b = X[j].b;

		X[j].a = X[i].a;
		X[j].b = X[i].b;
		X[i].a = T.a;
		X[i].b = T.b;
TAG25:	K = NV2;
TAG26:	if (K >= j) goto TAG30;
		j = j - K;
		K = K / 2;
		goto TAG26;
TAG30:	j = j + K;
	}
}
int main(void)
{
	int i = 0;
	float arr[16] = {0};
	float power[16] = {0};
	char data[5]= {0};
	char c;

	int fd3 = open("/dev/ttySAC3", O_RDWR);
	if(fd3 < 0)
	{
		printf("unable to open uart 3\n");
		return -1;
	}

	while(i<10){
	
		int id = 0;
		while(1) {
			read(fd3, &c, 1);
			if(c == 'x')
				break;

			data[id] = c;
			id++;
		}
		data[id] = '\0';

		sscanf(data, "%f", &arr[i]);
		printf("%f resistance %f ohm\n", arr[i], arr[i]*12.21);
			i++;
	}
	close(fd3);


	for (i = 1; i <= 8; i++)
	{
		X[i].a = arr[i];
		X[i].b = 0.0;
	}

	printf ("*********Before*********\n");
	for (i = 1 ;i <= 8; i++)
		printf ("X[%d]:real == %f  imaginary == %f\n", i, X[i].a, X[i].b);
	FFT();

	printf ("\n\n**********After*********\n");
	for (i = 1; i <= 8; i++)
		printf ("X[%d]:real == %f  imaginary == %f\n", i, X[i].a, X[i].b);

	for (i = 1; i <= 8; i++)
	{
		X[i].a = X[i].a/8;
		X[i].b = X[i].b/8;
	}
	printf ("\n\n**********After*********\n");
	for (i = 1; i <= 8; i++)
		printf ("X[%d]:real == %f  imaginary == %f\n", i, X[i].a, X[i].b);
	for(i=1;i<=8; i++)
	{
		power[i]= sqrt(((X[i].a*X[i].a)+(X[i].b*X[i].b)));
		printf("power spectrum value of power[%d] is = %f \n",i,power[i]);
	}
	float mean=0;
	for(i=3;i<=4;i++)
	{
		mean= mean + power[i];
	}
	mean/=2;
	printf("mean is %f \n",mean);
	if(mean<(0.5*power[1]))
		printf("valid energy\n");
	else
		printf("not valid energy\n");

	return 0;
}
