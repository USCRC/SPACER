/*
 * fluxObs_const.h
 *
 *  Created on: Mar 20, 2025
 *      Author: USSOAGO
 */

#ifndef INC_FLUXOBS_CONST_H_
#define INC_FLUXOBS_CONST_H_

typedef struct 	{ float  Rs; 				// Input: Stator resistance (ohm)
				  float  Rr;				// Input: Rotor resistance (ohm)
			      float  Ls;				// Input: Stator inductance (H)
				  float  Lr;				// Input: Rotor inductance (H)
				  float  Lm;				// Input: Magnetizing inductance (H)
				  float  Ib; 				// Input: Base phase current (amp)
				  float  Vb;				// Input: Base phase voltage (volt)
				  float  Ts;				// Input: Sampling period in sec
				  float  Tr;				// Parameter: Rotor time constant
			      float  K1;				// Output: constant using in rotor flux calculation
			      float  K2;				// Output: constant using in rotor flux calculation
			      float  K3;				// Output: constant using in rotor flux calculation
			      float  K4;				// Output: constant using in stator current calculation
			      float  K5;				// Output: constant using in stator current calculation
			      float  K6;				// Output: constant using in stator current calculation
			      float  K7;				// Output: constant using in stator current calculation
			      float  K8;				// Output: constant using in torque calculation
				} FLUXOBS_CONST;

/*-----------------------------------------------------------------------------
	Default initalizer for the ACIFE_CONST object.
-----------------------------------------------------------------------------*/
#define FLUXOBS_CONST_DEFAULTS {0,0,0,0, 		\
                              0,0,0,0,		\
                              0,0,0,0,		\
 		          	          0,0,0,0,	 	\
                             }

/*------------------------------------------------------------------------------
	ACIFE_CONST MACRO Definition
------------------------------------------------------------------------------*/


#define FLUXOBS_CONST_MACRO(v)					\
												\
/* Rotor time constant (sec)*/					\
   v.Tr = v.Lr/v.Rr;							\
   												\
   v.K1 = v.Tr/(v.Tr+v.Ts);						\
   v.K2 = v.Ts/(v.Tr+v.Ts);						\
   v.K3 = v.Lm/v.Lr;							\
   v.K4 = (v.Ls*v.Lr-v.Lm*v.Lm)/(v.Lr*v.Lm);	\
   v.K5 = v.Ib*v.Rs/v.Vb;						\
   v.K6 = v.Vb*v.Ts/(v.Lm*v.Ib);				\
   v.K7 = v.Lr/v.Lm;							\
   v.K8 = (v.Ls*v.Lr-v.Lm*v.Lm)/(v.Lm*v.Lm);


#endif /* INC_FLUXOBS_CONST_H_ */
