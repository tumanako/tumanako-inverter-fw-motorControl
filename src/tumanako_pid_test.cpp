//------------------------------------------------------------------------------
//   tumanako_pid.cpp
//
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2010 Donovan Johnson <donovan.johnson@gmail.com>
//
//   This file is part of TumanakoVC.
//
//   TumanakoVC is free software: you can redistribute it and/or modify
//   it under the terms of the GNU Lesser General Public License as published
//   by the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   TumanakoVC is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public License
//   along with TumanakoVC.  If not, see <http://www.gnu.org/licenses/>.
//
// DESCRIPTION:
//   This file provides test functions for testing the tumanako PID header
//   file.
//
// HISTORY:
//   Donovan Johnson: 17th May 2010
//
// CREDITS:
//    Jack Klein -  pidloop.c formed the basis for this implementation and is
//                  released under the GNU.
//------------------------------------------------------------------------------

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "tpid_class.h"

// structure to hold test inputs
struct _test_data{
  float efficiency;   // Process efficiency
  float lag;          // Process lag for testing
  float accum;        // weighted accumulator for testing
  float delay;        // Delay time between cycles
  int hold;           // hold to test cycle hold
  int cycles;         // number of cycles to test
};

// Flesh out structure
struct _test_data testd, *test_data;
// variables for test use
float p_g, i_g, d_g, iv, ni, accel, min, max;

// Initialise test data
void init_test(struct _test_data *tdata)
{
	struct _test_data *test_data;
	test_data = tdata;
	test_data->efficiency = 1.0;
	test_data->lag = 0.7;
	test_data->hold = 0;
	test_data->cycles = 100;
	test_data->delay = 0.03;
}

 //------------------------------------------------------------------------
 // GET_PP
 // inputs: pointer to run structure
 // outputs: Slowed PV value
 // process:    Simulates the effect of the PID output on a device with
 //              lag provided by test_data->lag & test_data->efficiency.
 // usage: Used for testing PID loop
 //------------------------------------------------------------------------
float get_pp(float result, struct _test_data *test_data)
{
  // Equipment response to PID input
  float laggedOP;
	
  // Weighted accummulated PID ouput
  test_data->accum += result;
	
  // Accummulated output * lag ratio * efficiency
  laggedOP = test_data->accum * (1 - test_data->lag) * test_data->efficiency;

  // Reset accum to the base lag times the last accum to emulate
  // lag over last loop iteration.
  test_data->accum *= test_data->lag;
  return laggedOP;
}

 //------------------------------------------------------------------------
 // MAIN
 // inputs: command line
 // outputs: returns 0
 // process: Creates a pid structure, runs a loop to test functionality
 // usage: Run from command line or debugger
 //------------------------------------------------------------------------
int main(int argc, char **argv)
{
	
	test_data = &testd; // pointer to PID parameters
	_pid pclass;        // _pid classs
	int pv = 0;         // current process value
	int sp = -20;       // desired setpoint
	int i;              // loop counter
	char input;         // char input for command line
	bool runme = true;  // run flag
	
	p_g = 1.0;       // proportional gain
	i_g = 0.2;       // integral gain
	d_g = 0.01;      // derivitive gain
	accel = 0.0;     // Acceleration limit
	min = -100;      // Minimum PID output
	max = 100;       // Maximum PID output
	
	// setup the PID
	pclass.tune_pid(p_g,i_g,d_g);
	
	// initialise PID with PV and SP
	pclass.init_pid(pv,sp,min,max,accel);
	
	// initialise test variables
	init_test(&testd);
	
	/* if there is no acceleration limit set then set sps */
	if (0.0 == pclass.get_acceleration_limit()) {
		pclass.set_next_setpoint(pclass.get_setpoint());
		pclass.set_this_setpoint(pclass.get_setpoint());
	}
	
	// Set hold here if required
	// pid->hold=20;
	
	while(runme)
	{
		i = 0;
		// Loop for test_data->cycles
		while (i < test_data->cycles)
		{
			// If we're in a holding pattern
			if (test_data->hold > 0)
			{
				// reduce the hold
				--test_data->hold;
				// hold the PId output
				pclass.set_pid_result(pclass.get_last_pid_result());
				// hold the process variable
				pclass.set_process_point(pclass.get_last_process_point());
				// reset the integral before restarting
				if(test_data->hold == 0)
				{
					pclass.pid_set_integral(0.1);
				}
			}
			else if (pclass.get_pid_result() != pclass.get_last_pid_result())
			{
				// else if there was some kind of error or missed loop
				test_data->hold = (unsigned long)floor(test_data->hold);
				pclass.set_last_pid_result(pclass.get_pid_result());
				pclass.set_last_process_point(pclass.get_process_point());
				pclass.pid_set_integral(0.1);
			}
			else
			{
				// get pv and pid output
				pclass.set_last_process_point(pclass.get_process_point());
				pclass.set_process_point(get_pp(pclass.get_last_pid_result(),&testd));
				pclass.set_pid_result(pclass.calc_pid());
			}
			
			// Show this loops results
			printf("%5lu: SP: %7.2f PP: %7.2f Out: %7.2f \n",
				   ++i, pclass.get_this_setpoint(), pclass.get_process_point(),
				   pclass.get_pid_result());
			
			
			// Delay loop - NOTE this is NOT cross compiler friendly
			// Translate to TIME if required (limitation of second delays)
			if (test_data->delay > 0.0)
			{
				clock_t start_time;
				start_time = clock();
				while((clock() - start_time) < (test_data->delay * CLOCKS_PER_SEC));
			}
    }
		
		printf ("Enter a [S] for a new setpoint or [Q] to exit: ");
		scanf("%c",&input);
		getchar(); // clear stdin char
		
		if(input == 'Q' || input == 'q')
			break;
		else
		{
			printf ("Enter the new setpoint (integer): ");
			scanf("%d",&sp);
			pclass.init_pid(pv,sp,min,max,accel);
			getchar(); // clear stdin char
		}
		
	}
	return 0;
}

