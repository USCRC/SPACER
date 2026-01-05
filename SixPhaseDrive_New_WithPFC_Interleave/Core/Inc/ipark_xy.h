/*
 * ipark_xy.h
 *
 *  Created on: Dec 19, 2025
 *      Author: USSOAGO
 */

#ifndef INC_IPARK_XY_H_
#define INC_IPARK_XY_H_


typedef struct {  float  Alpha;  		// Output: stationary d-axis stator variable
				  float  Beta;		// Output: stationary q-axis stator variable
				  float  Angle;		// Input: rotating angle (pu)
				  float  Ds;			// Input: rotating d-axis stator variable
				  float  Qs;			// Input: rotating q-axis stator variable
				  float  Sine;		// Input: Sine term
				  float  Cosine;		// Input: Cosine term
		 	    } IPARK_XY;

/*-----------------------------------------------------------------------------
Default initalizer for the IPARK object.
-----------------------------------------------------------------------------*/
#define IPARK_XY_DEFAULTS {  0, \
                          0, \
                          0, \
                          0, \
                          0, \
						  0, \
                          0, \
              		   }

/*------------------------------------------------------------------------------
	Inverse PARK Transformation Macro Definition
------------------------------------------------------------------------------*/

#define IPARK_XY_MACRO(v)										\
															\
v.Alpha = (v.Ds*-v.Cosine) + (v.Qs*v.Sine);		            \
v.Beta  = (v.Ds*v.Sine) + (v.Qs*v.Cosine);                  \


#endif /* INC_IPARK_XY_H_ */
