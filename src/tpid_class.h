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
  float pg_;					// proportional gain
  float ig_;					// integral gain
  float dg_;					// derivitive gain
  float acceleration_limit_;	// Acceleration limit (rate of change)
  float setpoint_;				// User setpoint
  float minimum_pid_;			// minimum setpoint
  float maximum_pid_;			// maximum setpoint
  float current_err_;			// current error
  float last_err_;				// previous error
  float i_;						// integral
  float last_process_point_;    // last process point
  float process_point_;         // current process point
  float next_setpoint_;			// next setpoint
  float this_setpoint_;			// current setpoint
  float last_setpoint_;			// last setpoint
  float pid_result_;			// current PId output
  float last_pid_result_;		// previous PID output
 public:
  // Contstructor & Destructor
  _pid();
  ~_pid(){};	
	
  // Inline getters and setters
  inline const float get_setpoint(){ return setpoint_; }
  inline void set_setpoint(float sp){ setpoint_ = setpoint_; }
  inline const float get_this_setpoint(){ return this_setpoint_; }
  inline void set_this_setpoint(float tsp){ this_setpoint_ = tsp; }
  inline const float get_next_setpoint(){ return next_setpoint_; }
  inline void set_next_setpoint(float nsp){ next_setpoint_ = nsp; }
  inline const float get_last_setpoint(){ return last_setpoint_; }
  inline void set_last_setpoint(float lsp){ last_setpoint_ = lsp; }
  inline const float get_process_point(){ return process_point_; }
  inline void set_process_point(float pp){ process_point_ = pp; }
  inline const float get_last_process_point(){ return last_process_point_; }
  inline void set_last_process_point(float lpp){ last_process_point_ = lpp; }
  inline const float get_acceleration_limit(){ return acceleration_limit_; }
  inline void set_acceleration_limit(float acc){ acceleration_limit_ = acc; }
  inline const float get_minimum_pid(){ return minimum_pid_; }
  inline void set_minimum_pid(float min){ minimum_pid_ = min; }
  inline const float get_maximum_pid(){ return maximum_pid_; }
  inline void set_maximum_pid(float max){ maximum_pid_ = max; }
  inline const float get_pid_result(){ return pid_result_; }
  inline void set_pid_result(float sr){ pid_result_ = sr; }
  inline const float get_last_pid_result(){ return last_pid_result_; }
  inline void set_last_pid_result(float lr){ last_pid_result_ = lr; }
  inline const float get_current_error(){ return current_err_; }
	
	//---------------------------------------------------------------------------
  // TUNE_PID
  // inputs: proportional gain, integral gain, derivitive gain
  // outputs: none
  // process: Updates variables in _pid class with new values, used for tuning
  //---------------------------------------------------------------------------
	inline void tune_pid(float p_g, float i_g, float d_g)
	{
		pg_ = p_g;
		ig_ = i_g;
		dg_ = d_g;
	}
	
	//------------------------------------------------------------------------
  // PID_SET_INTEGRAL
  // inputs:  New integral value
  // outputs: none
  // process: Sets integral to input value, resets last error to zero
  // usage: Use to reset pid control when starting up
  //------------------------------------------------------------------------//
	inline void _pid::pid_set_integral(float ni)
	{
		i_ = ni;
		last_err_ = 0.0;
	}
	
	// Methods constructors
	void init_pid(int, int, float, float, float);
	float calc_pid();
};
#endif	/* _TPID_CLASS_H */

