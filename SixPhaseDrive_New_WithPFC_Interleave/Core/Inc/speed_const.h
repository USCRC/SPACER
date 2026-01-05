/*
 * speed_const.h
 *
 *  Created on: Mar 21, 2025
 *      Author: USSOAGO
 */

#ifndef INC_SPEED_CONST_H_
#define INC_SPEED_CONST_H_

typedef struct 	{ float  Rr;				// Input: Rotor resistance (ohm)
				  float  Lr;				// Input: Rotor inductance (H)
				  float  Tr;				// Variable: Rotor time constant
				  float  fb;              // Input: Base electrical frequency (Hz)
				  float  Wb;              // Variable: Base angular speed (rad/s)
                  float  fc;              // Input: Cut-off frequency of lowpass filter (Hz)
                  float  Tc;              // Variable: Time constant (sec)
				  float  Ts;				// Input: Sampling period in sec
			      float  K1;				// Output: constant using in rotor flux calculation
			      float  K2;				// Output: constant using in rotor flux calculation
			      float  K3;				// Output: constant using in rotor flux calculation
			      float  K4;				// Output: constant using in stator current calculation
				} SPEED_CONST;

/*-----------------------------------------------------------------------------
	Default initalizer for the ACISE_CONST object.
-----------------------------------------------------------------------------*/
#define SPEED_CONST_DEFAULTS {0,0,0,0,	\
                              0,0,0,0, 	\
 		          	          0,0,0,0, 	\
                             }

/*------------------------------------------------------------------------------
	ACI_SE_CONST macro definition
------------------------------------------------------------------------------*/


#define PI 3.14159265358979

#define SPEED_CONST_MACRO(v)				\
											\
/* Rotor time constant (sec) */				\
	v.Tr = v.Lr/v.Rr;						\
											\
/* Lowpass filter time constant (sec) */	\
	v.Tc = 1/(2*PI*v.fc);					\
											\
	v.Wb = 2*PI*v.fb;						\
	v.K1 = 1/(v.Wb*v.Tr);					\
	v.K2 = 1/(v.fb*v.Ts);					\
	v.K3 = v.Tc/(v.Tc+v.Ts);				\
	v.K4 = v.Ts/(v.Tc+v.Ts);


#endif /* INC_SPEED_CONST_H_ */
