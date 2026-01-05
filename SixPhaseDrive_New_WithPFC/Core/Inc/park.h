/*
 * park.h
 *
 *  Created on: Jul 25, 2023
 *      Author: saagoro
 */

#ifndef SRC_PARK_H_
#define SRC_PARK_H_

typedef struct {  float  Alpha;  		// Input: stationary d-axis stator variable
				  float  Beta;	 	// Input: stationary q-axis stator variable
				  float  Angle;		// Input: rotating angle (pu)
				  float  Ds;			// Output: rotating d-axis stator variable
				  float  Qs;			// Output: rotating q-axis stator variable
				  float  Sine;
				  float  Cosine;
		 	 	} PARK;

/*-----------------------------------------------------------------------------
Default initalizer for the PARK object.
-----------------------------------------------------------------------------*/
#define PARK_DEFAULTS {   0, \
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


#define PARK_MACRO(v)											\
																\
	v.Ds = (v.Alpha*v.Cosine) +  (v.Beta*v.Sine);	\
    v.Qs = (v.Beta*v.Cosine) -   (v.Alpha*v.Sine);


#endif /* SRC_PARK_H_ */
