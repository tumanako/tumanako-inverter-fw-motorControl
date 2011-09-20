//------------------------------------------------------------------------------
//   Tumanako - Electric Vehicle software
//   Copyright (C) 2011 Graham Osborne <gjoengineer@users.sourceforge.net>
//
//   This file is part of Tumanako.
//
//   Tumanako is free software: you can redistribute it and/or modify
//   it under the terms of the GNU Lesser General Public License as published
//   by the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   Tumanako is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public License
//   along with Tumanako.  If not, see <http://www.gnu.org/licenses/>.
//
// DESCRIPTION:
//   This file contains some simple C code for calculating slip angle from the torque and flux genartaing currents
//	 The rotor time constant and PWM period are also required.
//	 The method used is from Richard Valentines book "Motor Control Electronics Handbook"
// 	 It is described here - sourceforge.net/apps/mediawiki/tumanako/index.php?title=Field_Oriented_Control#Calculating_Slip_Angle
//
//	 Parameters required
//   iqs - torque generating current
//   ids - flux generating current
//   sPWM - PWM loop period
//   tRotor - Rotor Time Constant
//
//	 Variables
//   OmegaSlip - slip frequency (rad/sec)
//   ThetaSlip - flux slip angle (rad)
//
// HISTORY:
//   Graham Osborne 21/9/2011 - First Cut in C (needs refinement!)
//
//------------------------------------------------------------------------------


#include <stdio.h>

// Variables
double OmegaSlip[2], ThetaSlip[2], SlipAngle;
double Pi = 3.14159265358979;

// Prototypes
void SlipInit (void);
double SlipCalc(double iqs, double ids, double sPWM, double tRotor);


int main (void)
{
	// initialise slip frequency and angle
	SlipInit ();

	// call slip angle calculation each PWM loop period
	SlipAngle = SlipCalc (1,2,3,4);
}

void SlipInit (void)
{
	//  Initialisation of slip frequency and angle
	OmegaSlip[0] = 0;
	ThetaSlip[0] = 0;
}

double SlipAngle(double iqs, double ids, double sPWM, double tRotor)
{
	// Calculate current slip velocity
	OmegaSlip[1] = ((1 + tRotor*sPWM)/tRotor) + (iqs/ids);

	// Calculate accumulated slip angle using average value
	ThetaSlip[1] = (ThetaSlip[0] + ((OmegaSlip[0] +OmegaSlip[1])/2)*sPWM);

	// Restrict slip angle to a value between 0-Pi radians
	while (ThetaSlip[1] >= Pi)
	{
		ThetaSlip[1] = ThetaSlip[1] - Pi;
	}

	OmegaSlip[0] = OmegaSlip[1];
	ThetaSlip[0] = ThetaSlip[1];

	return ThetaSlip[1];
}
