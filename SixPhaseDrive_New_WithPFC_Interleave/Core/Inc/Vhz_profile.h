/*
 * Vhz_profile.h
 *
 *  Created on: Nov 11, 2024
 *      Author: USSOAGO
 */

#ifndef INC_VHZ_PROFILE_H_
#define INC_VHZ_PROFILE_H_

typedef struct 	{ float  Freq; 		    // Input: Input Frequency (pu)
				float  VoltOut;			// Output: Output voltage (pu)
				float  LowFreq;			// Parameter: Low Frequency (pu)
				float  HighFreq;			// Parameter: High Frequency at rated voltage (pu)
				float  FreqMax; 			// Parameter: Maximum Frequency (pu)
				float  VoltMax;			// Parameter: Rated voltage (pu)
			    float  VoltMin;	 		// Parameter: Voltage at low Frequency range (pu)
			    float  VfSlope;			// Variable
			    float  AbsFreq;			// Variable

				} VHZPROF;


/*-----------------------------------------------------------------------------
Default initalizer for the VHZPROF object.
-----------------------------------------------------------------------------*/
#define VHZPROF_DEFAULTS { 0,0, 		\
                           0,0,0,0,0, 	\
                  		 }

/*------------------------------------------------------------------------------
	 VHZ_PROF Macro Definitions
------------------------------------------------------------------------------*/


#define VHZ_PROF_MACRO(v)															\
/* Take absolute frequency to allow the operation of both rotational directions	*/	\
    v.AbsFreq = v.Freq;														\
	if (v.AbsFreq <= v.LowFreq)   													\
	        /* Compute output voltage in profile #1	*/								\
        	v.VoltOut = v.VoltMin;													\
	else if ((v.AbsFreq > v.LowFreq)&&(v.AbsFreq <= v.HighFreq))      				\
       {																			\
        	/* Compute slope of V/f profile	*/										\
        	/*v.VfSlope = ((v.VoltMax - v.VoltMin)/(v.HighFreq - v.LowFreq));*/	\
			v.VfSlope = v.VoltMax;  \
        	/* Compute output voltage in profile #2	*/								\
        	/*v.VoltOut = v.VoltMin + (v.VfSlope*(v.AbsFreq-v.LowFreq));	*/	\
			v.VoltOut = v.VoltMin + v.VfSlope*v.AbsFreq;   \
       }																			\
    else if ((v.AbsFreq > v.HighFreq)&&(v.AbsFreq < v.FreqMax))      				\
        	/* Compute output voltage in profile #3	*/								\
        	v.VoltOut = v.VoltMax;

#endif /* INC_VHZ_PROFILE_H_ */
