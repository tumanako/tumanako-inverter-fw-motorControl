/* This is part of the Tumanako Project LGPL3
 * Compilable pseudo code for Indirect Field Oriented Control
 * (c) Edward Cheeseman 2012
 */

//data types
struct Cartesian {
	Cartesian(){q=0;d=0;}
	float q;  //or x
	float d;  //or y
};

struct Polar {
	Polar(){mag=0;ang=0;}
	float mag;
	float ang;
};

//global variables
short g_icur1; //hardware adc to update global currents
short g_icur2; //hardware adc to update global currents
short g_icur3; //hardware adc to update global currents
short rotor_position; // encoder derived (timer)
Cartesian torque_flux_req; //controller input (maybe from user throttle input etc)
float dc_bus_voltage;

//constants
#define ROTOR_TIME_CONSTANT ((float)(43)) //motor dependant
#define IFOC_DDT ((float)(0.001)) // ifoc routine period d/dt (most likely matches PWM period)

//prototypes
float calculate_rotor_flux_angle(Cartesian, short, float, float); // determining the angle of stationary frame
Cartesian clark_park(short, short, short, float); //convert 120degree 3 axis to rotor flux referenced 90 degree 2 axis
Cartesian dq_pid_controller(Cartesian, Cartesian);
Polar inv_park(Cartesian, float);
void space_vector_machine(Polar, float);

//methods
void init_ifoc(){ //set up intial values etc

}

void ifoc(){
#ifdef SKiiP
   short ia = g_icur1; // get this from ADC/timer interrupt, that should have just finished
   short ib = g_icur2; // get this from ADC/timer interrupt, that should have just finished
   short ic = -ia -ib;
#else
   short ia = g_icur1;  // get this from ADC/timer interrupt, that should have just finished
   short ic = g_icur3; // get this from ADC/timer interrupt, that should have just finished
   short ib = -ia -ic;
#endif

   static Cartesian stator_current;

   float rotor_flux_angle;
   rotor_flux_angle = calculate_rotor_flux_angle(stator_current /*from last ifoc()*/,
				rotor_position, ROTOR_TIME_CONSTANT, IFOC_DDT );

   stator_current = clark_park(ia, ib, ic, rotor_flux_angle);

   Cartesian cartesian_output_voltage = dq_pid_controller(stator_current, torque_flux_req);

   Polar polar_output_voltage = inv_park(cartesian_output_voltage, rotor_flux_angle);

   space_vector_machine(polar_output_voltage, dc_bus_voltage /* to compensate for bus sag*/);
}
