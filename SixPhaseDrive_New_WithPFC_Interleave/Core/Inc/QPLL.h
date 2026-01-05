/*
 * QPLL.h
 *
 *  Created on: Feb 15, 2024
 *      Author: USSOAGO
 */

#ifndef INC_QPLL_H_
#define INC_QPLL_H_
typedef struct {  float  err;   			// Input: reference set-point
				  float  Ealpha;   			// Input: feedback
				  float  Ebeta;   			// Output: controller output
				  float  E;				// Parameter: proportional loop gain
				  float  Theta;			    // Parameter: integral gain
				  float  oldTheta;			    // Parameter: upper saturation limit
				  float  speed;
				  float  Kp;			    // Parameter: lower saturation limit
				  float  Ki;				// Data: proportional term
				  float  integral;				// Data: integral term
				  float  oldIntegral;				// Data: pre-saturated controller output
				  float  Ts;
				} Q_PLL;

#define twoPI 6.283185307

/*-----------------------------------------------------------------------------
Default initalisation values for the PI_GRANDO objects
-----------------------------------------------------------------------------*/

#define Q_PLL_DEFAULTS {		\
						   0.0, 			\
                           0.0, 			\
						   0.0, 			\
                           0.0,	          \
                           0.0,           \
                           0.0,	        \
						   0.0,          \
                           1.0, 	    \
                           0.0,      	\
                           0.0, 	    \
                           0.0,	       \
						   0.0001, \
              			  }
/*------------------------------------------------------------------------------
				 	PLL_GRANDO Macro Definition
------------------------------------------------------------------------------*/

#define PLL_MACRO(v)	   \
   v.E = sqrt((v.Ealpha*v.Ealpha) + (v.Ebeta*v.Ebeta));   \
   \
	if (v.E <= 0)                                  \
		v.E = 0.00001;                              \
	if (v.E > 1)                                    \
		v.E = 1;                                    \
												  \
	v.Ealpha = v.Ealpha*cos(v.Theta);     				\
	v.Ebeta = v.Ebeta*sin(v.Theta);       				\
													\
	v.err = (- v.Ealpha - v.Ebeta)/ v.E;   					 \
													\
	v.integral = v.oldIntegral + v.err*v.Ts;  				\
													\
	v.oldIntegral = v.integral;							\
													\
	v.speed = v.Kp*v.err + v.Ki*v.integral;					\
													\
	v.Theta = v.oldTheta + v.speed*v.Ts;					\
													\
	v.oldTheta = v.Theta;								\
	\
	while (v.Theta >= twoPI)     \
	{                            \
		v.Theta -= twoPI;          \
	}                             \
	while (v.Theta < 0)                 \
	{                                       \
		v.Theta += twoPI;                         \
	}                                      \
	 \
	v.oldTheta = v.Theta;                 \
\

#endif /* INC_QPLL_H_ */
