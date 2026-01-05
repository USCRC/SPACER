/*
 * PhaseVoltage.h
 *
 *  Created on: Aug 9, 2023
 *      Author: saagoro
 */

#ifndef SRC_PHASEVOLTAGE_H_
#define SRC_PHASEVOLTAGE_H_

typedef struct 	{ float  DcBusVolt;		    // Input: DC-bus voltage (pu)
			  	  float  MfuncV1;  		     // Input: Modulation voltage phase A (pu)
		  	  	  float  MfuncV2;			// Input: Modulation voltage phase B (pu)
		   	  	  float  MfuncV3;			// Input: Modulation voltage phase C (pu)
                  uint16_t  OutOfPhase; 	    // Parameter: Out of Phase adjustment (0 or 1) (Q0) - independently with global Q
	  	  	  	  float  VphaseA;			// Output: Phase voltage phase A (pu)
		   	  	  float  VphaseB;			// Output: Phase voltage phase B (pu)
		  	  	  float  VphaseC;			// Output: Phase voltage phase C (pu)
		  	  	  float  Valpha;			// Output: Stationary d-axis phase voltage (pu)
		  	  	  float  Vbeta;  			// Output: Stationary q-axis phase voltage (pu)
		  	  	  float  temp;				// Variable: temp variable
		  	  	} PHASEVOLTAGE;


/*
OutOfPhase = 1 for the out of phase correction if
* MfuncV1 is out of phase with PWM1,
* MfuncV2 is out of phase with PWM3,
* MfuncV3 is out of phase with PWM5,
otherwise, set 0 if their phases are correct.
*/

/*-----------------------------------------------------------------------------
Default initalizer for the PHASEVOLTAGE object.
-----------------------------------------------------------------------------*/
#define PHASEVOLTAGE_DEFAULTS { 0, \
                          		0, \
                          		0, \
                          		0, \
                          		0, \
                          		0, \
                          		0, \
                          		0, \
		         				0, \
 		          				0, \
                  				}

#define ONE_THIRD  0.33333333333333
#define TWO_THIRD  0.66666666666667
#define INV_SQRT3  0.57735026918963
/*------------------------------------------------------------------------------
 	Phase Voltage Calculation Macro Definition
------------------------------------------------------------------------------*/


#define PHASEVOLT_MACRO(v)														\
																				\
																				\
/* Scale the incomming Modulation functions with the DC bus voltage value*/		\
/* and calculate the 3 Phase voltages */										\
  v.temp 	  = v.DcBusVolt*ONE_THIRD ;								     	\
  v.VphaseA   = (v.temp*((v.MfuncV1*2)-v.MfuncV2-v.MfuncV3));		\
  v.VphaseB   = (v.temp*((v.MfuncV2*2)-v.MfuncV1-v.MfuncV3));		\
\
/* Voltage transformation (a,b,c)  ->  (Alpha,Beta)	*/							\
  v.Valpha = v.VphaseA;															\
  v.Vbeta = (v.VphaseA + (v.VphaseB*2))*INV_SQRT3;



#endif /* SRC_PHASEVOLTAGE_H_ */
