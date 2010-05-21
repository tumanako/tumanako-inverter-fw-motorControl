//------------------------------------------------------------------------------
//   tpid_class.h
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
//   This file provides a class for controlling PID loops. Refer to
//   tpid_class.cpp for implementation of this class.
//
// HISTORY:
//   Donovan Johnson: 19th May 2010, converted to C++ and heaviliy modified.
//
// CREDITS:
//    Jack Klein -  pidloop.c formed the basis for this implementation and is
//                  released under the GNU.
//------------------------------------------------------------------------------

#ifndef _TPID_CLASS_H
#define	_TPID_CLASS_H

class _pid{
private:
	float pg;           // proportional gain
	float ig;           // integral gain
	float dg;           // derivitive gain
	float accel;        // Acceleration limit (rate of change)
	float sp;           // User setpoint
	float min_val;      // minimum setpoint
	float max_val;      // maximum setpoint
	float err;          // current error
	float last_err;     // previous error
	float i;            // integral
	float last_pp;      // last process point
	float pp;           // current process point
	float next_sp;      // next setpoint
	float this_sp;      // current setpoint
	float last_sp;      // last setpoint
	float result;       // current PId output
	float last_result;  // previous PID output
public:
	
    // Function constructors
    void init_pid(int, int, float, float, float);
    void tune_pid(float, float, float);
    void pid_setintegral(float);
    float calc_pid();
	
    // Getter setter for user setpoint
    void setsp(float);
    float getsp();
	
    // Getter setter for current target setpoint
    void setthissp(float);
    float getthissp();
	
    // Getter setter for next target setpoint
    void setnextsp(float);
    float getnextsp();
	
    // Getter setter for last target setpoint
    void setlastsp(float);
    float getlastsp();
	
    // Getter setter for current process point
    void setpp(float);
    float getpp();
	
    // Getter setter for last process point
    void setlastpp(float);
    float getlastpp();
	
    // Getter setter for acceleration limiting
    void setaccel(float);
    float getaccel();
	
    // Getter setter for pid output minimum
    void setmin(float);
    float getmin();
	
    // Getter setter for pid output maximum
    void setmax(float);
    float getmax();
	
    // Getter setter for current pid output result
    void setresult(float);
    float getresult();
	
    // Getter setter for last pid output result
    void setlastresult(float);
    float getlastresult();
	
    // Getter for current error
    float geterr();
};


#endif	/* _TPID_CLASS_H */

