#define Forward 1
#define Backward 2
#define Trapezoidal 3

typedef struct
{
	
	double r;
	double ek;
	double ekm1;
	
	double ck;
	double ckm1;
		
	double esk;
	double eskm1;
	
	
	double feedback;	/*process_variable; //feedBack variable value*/
	
	
	double max_output;
	double min_output;
	
	double min_input;
	double max_input;

} pidc_t;

typedef struct{
    double kp;
    double ki;
    double kd;
    double n;
    double b;
    double c;
} par_t;

void initialize(pidc_t *pidp,double _ckm1, double _ekm1);
void set_feedback(pidc_t *pidp, double value);
double get_feedback(pidc_t *pidp);
double pid(pidc_t *pidp, double Kp, double Ki, double Kd,
           double N, double b,double c, double Ts,
           char integral_type, char filter_type);
		 
void set_setpoint(pidc_t *pidp,double set_point);
double get_setpoint(pidc_t *pidp);
void set_actuator_limit(pidc_t *pidp,double minima,double maxima);
void set_input_range(pidc_t *pidp,double minima,double maxima);

void auto_tune(par_t *parp,
               double (*fbp)(void),
               void (*rtp)(double ref),
               double Tmax,
               double Ts );
