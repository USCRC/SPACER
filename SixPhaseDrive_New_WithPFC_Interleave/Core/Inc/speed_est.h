/*
 * speed_est.h
 *
 *  Created on: Mar 20, 2025
 *      Author: USSOAGO
 */

#ifndef INC_SPEED_EST_H_
#define INC_SPEED_EST_H_

typedef struct {  float  	IQsS;  			// Input: Stationary q-axis stator current
				  float  	PsiDrS;  		// Input: Stationary d-axis rotor flux
				  float  	IDsS;			// Input: Stationary d-axis stator current
				  float  	PsiQrS;			// Input: Stationary q-axis rotor flux
		 	 	  float  	K1;				// Parameter: Constant using in speed computation
                  float  	SquaredPsi; 	// Variable: Squared rotor flux
    			  float  	ThetaFlux;  	// Input: Rotor flux angle
		 	 	  float     K2;				// Parameter: Constant using in differentiator (Q21) - independently with global Q
    			  float  	OldThetaFlux; 	// Variable: Previous rotor flux angle
		 	 	  float  	K3;				// Parameter: Constant using in low-pass filter
		 	 	  float     WPsi;			// Variable: Synchronous rotor flux speed (Q21) - independently with global Q
		 	 	  float  	K4;				// Parameter: Constant using in low-pass filter
		 	 	  float  	WrHat;			// Output: Estimated speed in per unit
				  uint32_t     BaseRpm; 		    // Parameter: Base rpm speed (Q0) - independently with global Q
		 	 	  int32_t     WrHatRpm;		// Output: Estimated speed in rpm (Q0) - independently with global Q
		 	 	  float     WSlip;			// Variable: Slip
		 	 	  float	    WSyn;			// Variable: Synchronous speed
				 } SPEEDEST;


/*-----------------------------------------------------------------------------
Default initalizer for the ACISE object.
----------------------------------------------------------------------------- */
#define SPEEDEST_DEFAULTS {  0, 			\
                          0, 			\
                          0, 			\
                          0, 			\
                          0.1, 	\
                          0, 			\
                          0, 			\
                          0.1, 	\
                          0, 			\
                          0.1, 	\
                          0, 			\
                          0.1, 	\
                          0, 			\
                          0, 		\
              			  0, 			\
              			  0, 			\
              			  0, 			\
              			}

#define DIFF_MAX_LIMIT  0.80
#define DIFF_MIN_LIMIT  0.20

/*------------------------------------------------------------------------------
	ACI Speed Estimator MACRO Definition
------------------------------------------------------------------------------ */


#define SPEEDEST_MACRO(v)															\
																				\
/*  Slip computation */															\
	v.SquaredPsi = (v.PsiDrS*v.PsiDrS)+ (v.PsiQrS*v.PsiQrS);			\
																				\
	v.WSlip= (v.K1*((v.PsiDrS*v.IQsS) - (v.PsiQrS*v.IDsS)));	                \
	v.WSlip= (v.WSlip/v.SquaredPsi);										\
																				\
/*	Synchronous speed computation	*/											\
	if ((v.ThetaFlux < DIFF_MAX_LIMIT)&(v.ThetaFlux > DIFF_MIN_LIMIT))			\
/* 	Q21 = Q21*(GLOBAL_Q-GLOBAL_Q)*/												\
		  v.WSyn = (v.K2*(v.ThetaFlux - v.OldThetaFlux));					\
	else  v.WSyn = v.WPsi;														\
																				\
/* low-pass filter, */						\
	v.WPsi = (v.K3*v.WPsi) + (v.K4*v.WSyn);							\
																									\
	v.OldThetaFlux = v.ThetaFlux;												\
	v.WrHat = v.WPsi - v.WSlip;										\
																				\
/* Limit the estimated speed between -1 and 1 per-unit */						\
	if (v.WrHat > 1)                                             \
	v.WrHat = 1;                                                     \
	else if (v.WrHat < -1)                                                \
	v.WrHat = -1;                                                 \
	else                  \
	v.WrHat = v.WrHat;												\
																				\
	v.WrHatRpm = (v.BaseRpm*v.WrHat);


#endif /* INC_SPEED_EST_H_ */
