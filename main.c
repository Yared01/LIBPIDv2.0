#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pid/libpid.h"

#define NUM_PNTS 100
#define NUM_CMND 2

pidc_t pid1;
par_t ppar;

int main(){
	/* variable for handling status of plant output */
	double yk,ykm1; 
	double ck, ckm1;
	/* Initialization */
	ykm1 = 0;
	ckm1 = 0;

	int k=0;


	/* System response is plotted using gnuplot tools*/
	char *cmd_gnuplot[]={"set title \"Unit tep - Response \"","plot'out1.dat' with points pointtype 1"};
	FILE * temp = fopen("out1.dat","w");
	FILE * gnuplotPipe = popen("gnuplot -persistent","w");

	/* setting the basic configuration for plant1 controller */
	set_input_range(&pid1,-4,4);
	set_actuator_limit(&pid1,-25,25);

	/* set the initial value of plant for controller pid1 */
	initialize(&pid1,0.0,0.0);

	/* Reference input */
	set_setpoint(&pid1, 1.0); //2*sin(50*k)
	for(k=0;k<NUM_PNTS;k++){

		if(k==0){
			set_feedback(&pid1,ykm1);

		}else {
			set_feedback(&pid1,yk);

		}


		ck=pid(&pid1,0.16409,11.1478,0,100,1,1,0.01, Trapezoidal, Backward);
		//ck=pid(&pid1,0.01674,0.15148,0.00076,100,1,1,0.01, Trapezoidal, Backward);

		float noise = 0.001 *( (rand() % 1) - 0.5);

		yk = 0.8187*ykm1 + 0.09365*ck + 0.08187*ckm1; 

		ykm1 = yk;  //update the variable
		ckm1 = ck;

		fprintf(temp,"%lf %lf \n",k*0.1,yk);
	}
	
	/*
	* Testing Autotune function
	*/
	

        auto_tune(&ppar,fback, spoint, 3000, 10);

	int i=0;
	for (i=0; i < NUM_CMND; i++){
		/*Send commands to gnu-plot one by one.*/
		fprintf(gnuplotPipe, "%s \n", cmd_gnuplot[i]); 
	}

	return 0;
}
