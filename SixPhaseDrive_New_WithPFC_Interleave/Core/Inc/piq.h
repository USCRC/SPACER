/*
 * pi.h
 *
 *  Created on: Aug 9, 2023
 *      Author: saagoro
 */

#ifndef SRC_PIQ_H_
#define SRC_PIQ_H_
typedef struct {  float  Ref;   			// Input: reference set-point
				  float  Fbk;   			// Input: feedback
				  float  Out;   			// Output: controller output
				  float  Kp;				// Parameter: proportional loop gain
				  float  Ki;			    // Parameter: integral gain
				  float  Umax;			    // Parameter: upper saturation limit
				  float  Umin;			    // Parameter: lower saturation limit
				  float  up;				// Data: proportional term
				  float  ui;				// Data: integral term
				  float  v1;				// Data: pre-saturated controller output
				  float  i1;				// Data: integrator storage: ui(k-1)
				  float  w1;				// Data: saturation record: [u(k-1) - v(k-1)]
				  float  We;                // Input
				  float  L;                 // Parameter
				  float  Phi;               // Parameter
				} PIQ_CONTROLLER;


/*-----------------------------------------------------------------------------
Default initalisation values for the PI_GRANDO objects  Q-axis
-----------------------------------------------------------------------------*/

#define PIQ_CONTROLLER_DEFAULTS {		\
						   0, 			\
                           0, 			\
						   0, 			\
                           1.0,	       \
                           0.0,           \
                           1.0,	        \
                           -1.0, 	    \
                           0.0,      	\
                           0.0, 	    \
                           0.0,	       \
                           0.0,	       \
                           1.0, 	    \
						   0.0,          \
						   0.0,          \
						   0.0,          \
              			  }


/*------------------------------------------------------------------------------
 	PID_GRANDO Macro Definition (D-axis)
------------------------------------------------------------------------------*/

#define PIQ_MACRO(v)												\
																\
	/* proportional term */ 									\
	v.up = v.Ref - v.Fbk;										\
																\
	/* integral term */ 										\
	/*v.ui = (v.Out == v.v1)?((v.Ki* v.up)+ v.i1) : v.i1;	 */       \
	if (v.Out == v.v1)\
		v.ui = v.Ki*v.up + v.i1;\
	else\
		v.ui = v.i1;        \
	v.i1 = v.ui;												\
																\
	/* control output */ 										\
	/*v.v1 = v.Kp* (v.up + v.ui) + v.We*(v.L*v.Fbk + v.Phi)        ;*/							        \
	v.v1 = v.Kp* (v.up + v.ui) ;   \
	if (v.v1 > v.Umax)													\
		v.Out = v.Umax;									\
	else if (v.v1 < v.Umin)								\
		v.Out = v.Umin;							\
	else 										\
		v.Out = v.v1;                         \
							             \

#endif /* SRC_PIQ_H_ */
