/*
 * rampctrl.h
 *
 *  Created on: Jul 25, 2023
 *      Author: saagoro
 */

#ifndef SRC_RAMPCTRL_H_
#define SRC_RAMPCTRL_H_

typedef struct { float    TargetValue; 	    // Input: Target input (pu)
				 uint32_t RampDelayMax;	    // Parameter: Maximum delay rate (Q0) - independently with global Q
		 	 	 float    RampLowLimit;	    // Parameter: Minimum limit (pu)
				 float    RampHighLimit;	// Parameter: Maximum limit (pu)
				 uint32_t RampDelayCount;     // Variable: Incremental delay (Q0) - independently with global Q
				 float    SetpointValue;	// Output: Target output (pu)
				 uint32_t EqualFlag;		    // Output: Flag output (Q0) - independently with global Q
				 float	  Tmp;			    // Variable: Temp variable
				 float    Tstep;
		  	   } RMPCNTL;


/*-----------------------------------------------------------------------------
Default initalizer for the RMPCNTL object.
-----------------------------------------------------------------------------*/
#define RMPCNTL_DEFAULTS {  0, 		 \
                            5,		 \
                           -1,  \
                            1,   \
                            0,       \
                          	0,       \
                          	0,       \
                          	0,       \
							0.0,     \
                   		  }

/*------------------------------------------------------------------------------
 	RAMP Controller Macro Definition
------------------------------------------------------------------------------*/

#define RC_MACRO(v)																	\
	v.Tmp = v.TargetValue - v.SetpointValue;										\
/*  0.0000305 is resolution of Q15 */												\
if (fabs(v.Tmp) >= v.Tstep)				    							        \
{																					\
	v.RampDelayCount++	;															\
		if (v.RampDelayCount >= v.RampDelayMax)										\
		{																			\
			if (v.TargetValue >= v.SetpointValue)									\
				v.SetpointValue += (v.Tstep);									    \
			else																	\
				v.SetpointValue -= (v.Tstep);									    \
																					\
			if (v.SetpointValue >= v.RampHighLimit)                                  \
			{                                                                       \
		  		 v.SetpointValue = v.RampHighLimit;                                 \
			}                                                                       \
			else if (v.SetpointValue <= v.RampLowLimit)                             \
			{                                                                       \
				v.SetpointValue = v.RampLowLimit;                                   \
			}                                                                       \
			else                                                                    \
				v.SetpointValue = v.SetpointValue;                                  \
			v.RampDelayCount = 0;													\
																					\
		}																			\
}																					\
else v.EqualFlag = 0x7FFFFFFF;

#endif /* SRC_RAMPCTRL_H_ */
