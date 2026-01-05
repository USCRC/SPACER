/*
 * park_XY.h
 *
 *  Created on: Dec 19, 2025
 *      Author: USSOAGO
 */

#ifndef INC_PARK_XY_H_
#define INC_PARK_XY_H_

typedef struct {  float  Alpha;  		// Input: stationary d-axis stator variable
				  float  Beta;	 	    // Input: stationary q-axis stator variable
				  float  Angle;           // Input: rotating angle (pu)
				  float  Ds;			// Output: rotating d-axis stator variable
				  float  Qs;			// Output: rotating q-axis stator variable
				  float  Sine;
				  float  Cosine;
		 	 	} PARK_XY;

/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/
#define PARK_XY_DEFAULTS {   0, \
                          0, \
                          0, \
                          0, \
                          0, \
						  0, \
                          0, \
              			  }

/*------------------------------------------------------------------------------
	PARK Transformation Macro Definition
------------------------------------------------------------------------------*/


#define PARK_XY_MACRO(v)											\
																\
	v.Ds = (v.Alpha*-v.Cosine) +  (v.Beta*v.Sine);	\
    v.Qs = (v.Alpha*v.Sine) +   (v.Beta*v.Cosine);


#endif /* INC_PARK_XY_H_ */
