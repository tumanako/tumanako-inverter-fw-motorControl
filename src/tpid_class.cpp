//------------------------------------------------------------------------------
//   tpid_class.cpp
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
//   Implementation of tpid_class.h, provides external access to class
//   variables and functions.
//   Contains functions to run PID control - refer to function descriptors
//   For detailed information regarding usage.
//
// HISTORY:
//   Donovan Johnson: 19th May 2010, converted to C++ and heaviliy modified.
//
// CREDITS:
//    Jack Klein -  pidloop.c formed the basis for this implementation and is
//                  released under the GNU.
//------------------------------------------------------------------------------

#include "tpid_class.h"

inline float _pid::getsp(){ return this->sp; }
inline void _pid::setsp(float sp){ this->sp = sp; }

inline float _pid::getthissp(){ return this->this_sp; }
inline void _pid::setthissp(float tsp){ this->this_sp = tsp; }

inline float _pid::getnextsp(){ return this->next_sp; }
inline void _pid::setnextsp(float nsp){ this->next_sp = nsp; }

inline float _pid::getlastsp(){ return this->last_sp; }
inline void _pid::setlastsp(float lsp){ this->last_sp = lsp; }

inline float _pid::getpp(){ return this->pp; }
inline void _pid::setpp(float pp){ this->pp = pp; }

inline float _pid::getlastpp(){ return this->last_pp; }
inline void _pid::setlastpp(float lpp){ this->last_pp = lpp; }

inline float _pid::getaccel(){ return this->accel; }
inline void _pid::setaccel(float acc){ this->accel = acc; }

inline float _pid::getmin(){ return this->min_val; }
inline void _pid::setmin(float min){ this->min_val = min; }

inline float _pid::getmax(){ return this->max_val; }
inline void _pid::setmax(float max){ this->max_val = max; }

inline float _pid::getresult(){ return this->result; }
inline void _pid::setresult(float sr){ this->result = sr; }

inline float _pid::getlastresult(){ return this->last_result; }
inline void _pid::setlastresult(float lr){ this->last_result = lr; }

inline float _pid::geterr(){ return this->err; }


/*------------------------------------------------------------------------
 // INIT_pid
 // inputs:  process point, setpoint, min value, max value, acceleration
 *           limit
 // outputs: none
 // process: sets up pid variables before starting to control
 //------------------------------------------------------------------------*/
inline void _pid::init_pid(int pp, int sp, float min, float max, float accel)
{
	this->last_err = 0.0;
	this->err = 0.0;
	this->min_val = min;
	this->max_val = max;
	this->pp = pp;
	this->last_pp = pp;
	this->last_result = pp;
	this->result = pp;
	this->sp = sp;
	this->next_sp = sp;
	this->this_sp = sp;
	this->last_sp = sp;
	this->accel = accel;
	this->i = 0.0;
}

/*------------------------------------------------------------------------
 // TUNE_PID
 // inputs: proportional gain, integral gain, derivitive gain
 // outputs: none
 // process: Updates variables in _pid class with new values, used for tuning
 //------------------------------------------------------------------------*/
inline void _pid::tune_pid(float p_g, float i_g, float d_g)
{
	this->pg = p_g;
	this->ig = i_g;
	this->dg = d_g;
}

/*------------------------------------------------------------------------
 // PID_SETINTEGRAL
 // inputs:  New integral value
 // outputs: none
 // process: Sets integral to input value, resets last error to zero
 // usage: Use to reset pid control when starting up
 //------------------------------------------------------------------------*/
inline void _pid::pid_setintegral(float ni)
{
	this->i = ni;
	this->last_err = 0.0;
}

/*------------------------------------------------------------------------
 // CALC_PID
 // inputs: None
 // outputs: Next PID point
 // process: Calculates PID point based on setpoint & process value
 // usage: Get instantaneous PID control point
 //------------------------------------------------------------------------*/
inline float _pid::calc_pid()
{
    // Derivitive
    float d;
	
    // Set the target for this loop
    this->this_sp = this->next_sp;
	
    // If acceleration limiting is on - note, need to move this to it's own
	// function, not needed in normal operation so we can cut the overhead out.
    if (this->accel > 0.0 && this->this_sp != this->sp){
        // If we're below the user sp
        if (this->this_sp < this->sp){
            this->next_sp += this->accel;
            if (this->next_sp > this->sp) {
			this->next_sp = this->sp; }
        }
        else {
            // we're above the user sp
            // next_target -= params.accel;
            this->next_sp -= this->accel;
            if (this->next_sp < this->sp) {
			this->next_sp = this->sp;}
        }
    }
    else {
        /* No acceleration limit so go for it */
	this->next_sp = this->sp; }
	
    // Rest error for this loop
    this->err = this->this_sp - this->pp;
    
    // derive current loops error rate
    d = this->err - this->last_err;
	
    // increment the integral by the error amount
    this->i += this->next_sp - this->pp;
	
    // Calculate the resulting PID output
    this->result = this->pg * this->err
	+ this->ig * this->i
	+ this->dg * d;
	
    // set the last error to the current error for next
    // Loop iteration
    this->last_err = this->err;
	
    /* Check whether we're in the process limits */
    if (this->result < this->min_val) {
		this->result = this->min_val; }
    else if (this->result > this->max_val){
		this->result = this->max_val; }
	
    // store the result for next iteration and return the
    // output value
    return this->last_result = this->result;
}


