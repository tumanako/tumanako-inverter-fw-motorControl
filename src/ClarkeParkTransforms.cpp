//------------------------------------------------------------------------------
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2010 Graeme Bell <graemeb@users.sourceforge.net>
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
//   This file provides standalone functions for implementing Clarke and Park
//   transforms. A highlevel implementation suitable for initial use & testing,
//   but may require subsequent optimisiation for the final hardware.
//
// HISTORY:
//   Graeme Bell 1/May/2001 - First Cut
//------------------------------------------------------------------------------

#ifndef MATHS_CLARKEPARKTRANSFORMS_H
#define MATHS_CLARKEPARKTRANSFORMS_H

//--- Includes -----------------------------------------------------------------
#include <cmath>

//--- Defines ------------------------------------------------------------------
#define CONST_SQRT3_  1.732050807568877293527446315059
#define CONST_1_SQRT3 tReal(1/CONST_SQRT3_)
#define CONST_2_SQRT3 tReal(2/CONST_SQRT3_)
#define CONST_SQRT3_2 tReal(CONST_SQRT3_/2)
    
namespace Maths {

    //-------------------------------------------------------------------------
    // 
    
    // typedef fixed< int32_t, 16 > tReal
    typedef float tReal;

    /** 
     * Simple structure for representing values in a three-phase system.
     */
    struct tThreePhase {
        tThreePhase( tReal ra, tReal rb, tReal rc ) : a(ra), b(rb), c(rc) {}
        
        tReal  a, b, c;
    };
    
    /**
     * Simple structure for representing values in a two-phase orthogonal
     * system.
     */
    struct tTwoPhase { 
        tTwoPhase( tReal ralpha, tReal rbeta ) : alpha(ralpha), beta(rbeta) {}
        
        tReal  alpha, beta;
    };
    
    /**
     * Simple structure for representing values in a two-phase system with a 
     * rotating frame of reference.
     */
    struct tTwoPhaseDQ {
        tTwoPhaseDQ( tReal rd, tReal rq ) : d(rd), q(rq) {}
        
        tReal  d, q;
    };
    
    /**
     * Simple structure for holding the sin & cos of an angle. This is to allow
     * the values to be calculated just once for the current rotor angle and
     * then be used in multiple transforms.
     */
    struct tCosSin {
        tCosSin( tReal radians ) : cos( std::cos(radians) ), sin( std::sin(radians) ) {}
        
        tReal  cos;
        tReal  sin;
    };

    //-------------------------------------------------------------------------
    // Park Transformations
    
    inline tTwoPhaseDQ Park( const tCosSin& angle, const tTwoPhase& in ) {
        return tTwoPhaseDQ( 
             in.alpha*angle.cos + in.beta*angle.sin, 
            -in.alpha*angle.sin + in.beta*angle.cos );
    }
    inline tTwoPhaseDQ Park( tReal angle, const tTwoPhase& in ) {
        return Park( tCosSin(angle), in );
    }
    
    inline tTwoPhase InvPark( const tCosSin& angle, const tTwoPhaseDQ& in ) {
        return tTwoPhase(
             in.d*angle.cos - in.q*angle.sin, 
             in.d*angle.sin - in.q*angle.cos );
    }
    inline tTwoPhase InvPark( tReal angle, const tTwoPhaseDQ& in ) {
        return InvPark( tCosSin(angle), in );
    }

    //-------------------------------------------------------------------------
    // Clarke Transformations
    
    inline tTwoPhase Clarke( const tThreePhase& in ) {
        // note: this assumes in.a+in.b+in.c == 0
        return tTwoPhase( in.a, in.a*CONST_1_SQRT3 + in.b*CONST_2_SQRT3 );
    }
    inline tThreePhase InvClarke( const tTwoPhase& in ) {
        tReal r1 = -in.alpha/2;
        tReal r2 = in.beta*CONST_SQRT3_2;
        return tThreePhase( in.alpha, r1 + r2, r1 - r2 );
    }

    //-------------------------------------------------------------------------
    // Combined Clarke & Park Transformations
    
    inline tTwoPhaseDQ ClarkePark( tReal angle, const tThreePhase& in ) {
        return Park( angle, Clarke( in ) );
    }
    inline tThreePhase InvClarkePark( tReal angle, const tTwoPhaseDQ& in ) {
        return InvClarke( InvPark( angle, in ) );
    }

//-----------------------------------------------------------------------------
    
}  // namespace Maths

#endif  // inclusion guard
