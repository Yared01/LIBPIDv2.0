/*
 *  
 * @author : Yared Tadesse
 *
 * @date : 2:42 PM Thursday, Sept-27,2018 GC
 *	   Kekenu 8:41, Hamus, Meskerem 17,2011 EC
 *         Addis Ababa ,Ethiopia
 * @reference:
 * 1. PID controller-Wikipedia, the free encyclopedia 
 *       https://en.wikipedia.org/wiki/PID_controller ..07/26/2016 02:06PM
 * 2. Karl Johan Astrom and Richard M. Murray ,
 *      "Feedback systems : an introduction for scientists and engineers 2009,
 *       Princeton University Press,Version v2.10
 * 3. I.Kar,"Digital Control,Module 1:Introduction to Digital Control Lecture Note
 * 4. Xin-lan Li,Jong-Gyu Park,& Hwi-Beom Shin,"Comparison and Evaluation of
 * 	    Anti-Windup PI Controllers",Journal of Power Electronics, Vol.11, No.1,
 * 		January 2011
 */

#include <math.h>
#include <float.h>
#include <stdint.h>
#include "libpid.h" 

/* change this with time function of mcu ide */
#include <time.h>
#include <stdio.h>
/* required to access the process time in micro-second */
clock_t clock(void);
/* local function prototype */

double proportional(pidc_t *pidp, double P, double b);
double derivative(pidc_t *pidp, double K, double Td, double Ts, double N, double c, char method);
double integral(pidc_t *pidp, double K, double Ti, double Tt, double Ts, char method);
double input_filter(pidc_t *pidp, double set_point);
double actuator_limit(pidc_t *pidp,double ckp);

void initialize(pidc_t *pidp,double _ckm1, double _ekm1)
{

	pidp->ckm1=_ckm1;

	pidp->eskm1=_ekm1;
	pidp->ekm1=_ekm1;

	pidp->esk = actuator_limit(pidp, 0.0);

}

void set_feedback(pidc_t *pidp, double value)
{

	pidp->feedback = value;

}

double get_feedback(pidc_t *pidp)
{

	return pidp->feedback;
}

/*PID Control Algorithm Source Code*/
void set_setpoint(pidc_t *pidp,double set_point)
{

	pidp->r = input_filter(pidp, set_point);

}

double get_setpoint(pidc_t *pidp)
{
	return pidp->r;
}

void set_input_range(pidc_t *pidp,double minima,double maxima)
{
	pidp->min_input = minima;
	pidp->max_input = maxima;
}

void set_actuator_limit(pidc_t *pidp,double minima,double maxima)
{
	pidp->min_output = minima;
	pidp->max_output = maxima;
}

double input_filter(pidc_t *pidp, double set_point)
{
	if (set_point > pidp->max_input){

		return pidp->max_input; 

	}else if(set_point < pidp->min_input){

		return pidp->min_input;
	}else{
		return  set_point;
	}
}

double actuator_limit(pidc_t *pidp,double ckp)
{

	if( ckp > pidp->max_output){
		/* return Negative error */
		return  (pidp->max_output - ckp);

	}else if (ckp < pidp->min_output){
		/* return Positive error */
		return (pidp->min_output - ckp);
	}else{

		return 0;
	}

}

double proportional(pidc_t *pidp, double P, double b){

	double epk = pidp->ek + (b - 1)*pidp->r;

	return	P * epk;
}

double derivative(pidc_t *pidp, double K, double Td, double Ts, double N, double 
c, char method)
{
	double cdk = 0.0;
	double edk = pidp->ek + (c - 1)*pidp->r;
	double derr = edk - pidp->ekm1;
	double dn = K * Td * N;
	double nts = N*Ts;

	switch (method)
	{

	case(Forward):

		cdk = (1- nts)* pidp->ckm1 + dn*derr;
		break;			
	case(Backward):

		cdk =  (pidp->ckm1 + dn*derr)/(nts+1);
		break;

	case(Trapezoidal):

		cdk = ((1-nts/2)*pidp->ckm1 + dn*derr)/(1+nts/2);
		break;

	};

	return cdk;			 
}

double integral(pidc_t *pidp, double K, double Ti, double Tt, double Ts, char 
method)
{

	double cik;
	double eik = pidp->ek;		

	switch(method){

	case(Forward):

		cik = pidp->ckm1 + Ts*((pidp->ekm1*K)/Ti  +  pidp->eskm1/Tt);

	case(Backward):

		cik = pidp->ckm1 + Ts*((eik*K)/Ti  +  pidp->esk/Tt);

	case(Trapezoidal):

		cik = pidp->ckm1 + 0.5*Ts*(K*(eik + pidp->ekm1)/Ti - (pidp->esk + pidp->eskm1)/Tt);

	};

	return cik;
}


/* Main controller algorithm */
double pid(pidc_t *pidp, double Kp, double Ki, double Kd, double N, double 
b,double c, double Ts, char integral_type, char filter_type)
{
	double Ti = Kp/Ki;
	double Td = Kd/Kp;
	double Tt = sqrt(fabs(Ti*Td));;
	/* Update Tt if and only if PI controller*/
	if(Kd == 0){
		Tt = Ti;
	}

	pidp->ek = pidp->r - get_feedback(pidp);

	double p_term = proportional(pidp, Kp, b);  

	double d_term = derivative(pidp, Kp, Td, Ts, N, c, filter_type);

	double i_term = integral(pidp, Kp, Ti, Tt, Ts, integral_type); 

	double ck = p_term + i_term + d_term;//

	/* updating controller parameters*/

	pidp->eskm1 = pidp->esk;	
	pidp->esk = actuator_limit(pidp, ck);

	pidp->ekm1 = pidp->ek;

	pidp->ckm1 = pidp->ck;
	pidp->ck = ck + pidp->esk;

	return pidp->ck;		
}

/*
 * Function: auto_tune
 * Argument: fbp            pointer to function reading feedback signal
 *			    simply put function name
 *           rtp             pointer to the function that set reference
 *           Tmax           maximum time of tuning in ms
 *           Ts             sampling time in ms
 * Return: void
 * ----------------------------------------------------------------------------
 * Function:    fbp
 * Argument:    void.
 * Return:      value read from sensor
 * ----------------------------------------------------------------------------
 * Function:    stp
 * Argument:    value of set_point
 * Return:      void
 * ----------------------------------------------------------------------------
 * usage:
 *  #include libpid.h
 *  par_t ppar;
 *  auto_tune(&ppar, feedback_func, setpoint_func, Tmax, Ts);
 *  
 *  The value resulted from autotune is store at ppar structure
 *  ppar.kp, ppar.ki, ppar.kd, ppar.n, ppar.b, ppar.c
 *  eg.
 *  double pid(pidc_t *pidp, ppar.kp, ppar.ki, ppar.kd, ppar.n,
 *              ppar.b,ppar.c,Ts, FORWARD, BACKWARD)
 */
 
void auto_tune(par_t *parp,
               double (*fbp)(void),
               void (*rtp)(double ref),
               double Tmax,
               double Ts )
{
	// Solving maximum number of sample
	unsigned int max_ns = (unsigned int) Tmax/Ts;
	double data[1000];
	// Set the reference signal to unit-step
	rtp(1.0);
	double y0 = fbp();
	double ykm1 = y0;
	double ysmax = 0.0, tsmax = 0.0;
	double yss = 0.0, yk = 0.0;
	double tss = Tmax;
	double T63;
	double smax = 0, s = 0;
	uint32_t i=0, j=0;
	unsigned long time = 0;

	for ( i = 1; i < max_ns; i++ )
	{	
		// The clock return time in micro-seconds
		time = clock();

		yk = fbp();
		data[i] = yk;
		s = (yk - ykm1) / Ts;

		if ( s > smax  )
		{
			smax = s;
			tsmax = Ts * (i - 1);
			ysmax = ykm1;			
		}

		// At steady-state,  ess < 2% or 5%
		if ( s*Ts <= 0.00002)
		{
			yss = yk;
			break; 
		}
		/* wait until the sampling time is reached */
		while((clock() - time) < Ts*1000);

	}

	/*
	 *  Searching for T63 from Yss to y-half-energy might speedup
	 *  the process by focusing on 37% of the data rather to 63%
	 */

	while ( data[i] > 0.63 * (yss - y0) )
		i--;
	T63 = i * Ts;

	double Kp = yss - y0;
	double L = tsmax - ysmax/smax;
	double T = T63 - L;

	// Calculate value of K, Ti, Td	
	double K = (0.2 + (0.45*T/L))/Kp;
	double Ti = (0.4*L + 0.8*T)*L/(L + 0.1*T);
	double Td = (0.5*L*T)/(0.3*L + T);

	// Set pid parameters
        parp->kp = Kp;
	parp->ki = Kp/Ti;
	parp->kd = Td*Kp;
        
        // The rest is set to default value of matlab
        parp->n = 100;
        parp->b = 1;
        parp->c = 1;

        
}

