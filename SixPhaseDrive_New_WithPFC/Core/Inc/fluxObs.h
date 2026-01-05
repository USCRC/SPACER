/*
 * fluxObs.h
 *
 *  Created on: Mar 21, 2025
 *      Author: USSOAGO
 */

#ifndef INC_FLUXOBS_H_
#define INC_FLUXOBS_H_
typedef struct {  float  ThetaFlux;      	// Output: Rotor flux angle
	              float  IQsS;		    // Input: Stationary q-axis stator current
	              float  IDsS;         	// Input: Stationary d-axis stator current
	              float  IDsE;			// Variable: Measured current in sync. reference frame
	              float  K1;           	// Parameter: Constant using in current model
	              float  FluxDrE;        	// Variable: Rotating d-axis rotor flux (current model)
	              float  K2;           	// Parameter: Constant using in current model
	              float  FluxQrS;        	// Variable: Stationary q-axis rotor flux (current model)
	              float  FluxDrS;        	// Variable: Stationary d-axis rotor flux (current model)
	              float  K3;           	// Parameter: Constant using in stator flux computation
	              float  K4;           	// Parameter: Constant using in stator flux computation
	              float  FluxDsS;        	// Variable: Stationary d-axis stator flux (current model)
	              float  FluxQsS;        	// Variable: Stationary q-axis stator flux (current model)
				  float  PsiDsS;			// Variable: Stationary d-axis stator flux (voltage model)
	              float  Kp;           	// Parameter: PI proportionnal gain
	              float  Error;			// Parameter: Error term
	              float  UiDsS;           // Variable: Stationary d-axis integral term
	              float  UCompDsS;        // Variable: Stationary d-axis compensated voltage
	              float  Ki;           	// Parameter: PI integral gain
	              float  PsiQsS;       	// Variable: Stationary q-axis stator flux (voltage model)
	              float  UiQsS;           // Variable: Stationary q-axis integral term
	              float  UCompQsS;        // Variable: Stationary q-axis compensated voltage
	              float  EmfDsS;          // Variable: Stationary d-axis back emf
	              float  UDsS;         	// Input: Stationary d-axis stator voltage
	              float  K5;           	// Parameter: Constant using in back emf computation
	              float  K6;           	// Parameter: Constant using in back emf computation
	              float  EmfQsS;          // Variable: Stationary q-axis back emf
	              float  UQsS;         	// Input: Stationary q-axis stator voltage
	              float  K8;           	// Parameter: Constant using in rotor flux computation
	              float  K7;           	// Parameter: Constant using in rotor flux computation
				  float  PsiDrS;			// Output: Stationary d-axis estimated rotor flux
				  float  PsiQrS;			// Output: Stationary q-axis estimated rotor flux
				  float  OldEmf;		    // Variable: Old back-emf term
				  float  Sine;			// Variable: Sine term
				  float  Cosine;			// Variable: Cosine term
				 } FLUX_OBS;

/*-----------------------------------------------------------------------------
	Default initalizer for the ACIFE object.
-----------------------------------------------------------------------------*/
#define FLUXOBS_DEFAULTS {  0,    /*  ThetaFlux  */  	\
	                      0,    /*  IDsS  */     	\
	                      0,    /*  IQsS  */     	\
	                      0,    /*  IQsE  */     	\
	                      0,    /*  K1 */       	\
	                      0,    /*  FluxDrE  */    	\
	                      0,    /*  K2  */       	\
	                      0,    /*  FluxDrS  */    	\
	                      0,    /*  FluxQrS  */    	\
	                      0,    /*  K3  */       	\
	                      0,    /*  K4  */       	\
	                      0,    /*  FluxDsS  */    	\
	                      0,    /*  FluxQsS  */    	\
	 		              0,    /*  PsiDsS  */   	\
	                      0,    /*  Kp  */       	\
	                      0,    /*  Error  */      	\
	                      0,    /*  UiDsS  */    	\
	                      0,    /*  UCompDsS  */    \
	                      0,    /*  Ki  */ 			\
	                      0,    /*  PsiQsS  */   	\
 	                      0,    /*  UiQsS  */    	\
	                      0,    /*  UCompQsS  */    \
	                      0,    /*  EmfDsS  */      \
                          0,    /*  UDsS  */     	\
	                      0,    /*  K5  */       	\
	                      0,    /*  K6  */       	\
	                      0,    /*  EmfQsS  */      \
                          0,    /*  UQsS  */     	\
	                      0,    /*  K8  */       	\
	                      0,    /*  K7  */       	\
		                  0,    /*  PsiDrS  */   	\
		                  0,	/*  PsiQrS  */   	\
		                  0,	/*  OldEmf  */   	\
		                  0,	/*  Sine  */	   	\
		                  0,	/*  Cosine  */   	\
						}

#define PI 3.14159265358979323846
/*------------------------------------------------------------------------------
	ACI Flux Estimator MACRO Definition
------------------------------------------------------------------------------*/
#define FLUX_OBS_MACRO(v)															\
																				\
/* Calculate Sine and Cosine terms for Park/IPark transformations	*/			\
	v.Sine 	 = sin(v.ThetaFlux);											\
	v.Cosine = cos(v.ThetaFlux);											\
																				\
/* Park transformation on the measured stator current*/							\
	v.IDsE = (v.IQsS*v.Sine);												\
	v.IDsE += (v.IDsS*v.Cosine);											\
																				\
/* The current model section (Classical Rotor Flux Vector Control Equation)*/	\
	v.FluxDrE = (v.K1*v.FluxDrE) + (v.K2*v.IDsE);					\
																				\
/* Inverse park transformation on the rotor flux from the current model*/		\
	v.FluxDrS = (v.FluxDrE*v.Cosine);										\
	v.FluxQrS = (v.FluxDrE*v.Sine);										\
																				\
/* Compute the stator flux based on the rotor flux from current model*/			\
	v.FluxDsS = (v.K3*v.FluxDrS) + (v.K4*v.IDsS);					\
	v.FluxQsS = (v.K3*v.FluxQrS) + (v.K4*v.IQsS);					\
																				\
/* Conventional PI controller section */										\
	v.Error =  v.PsiDsS - v.FluxDsS;											\
	v.UCompDsS = (v.Kp*v.Error) + v.UiDsS;								    \
	v.UiDsS = (v.Kp*(v.Ki*v.Error)) + v.UiDsS;						          \
																				\
	v.Error =  v.PsiQsS - v.FluxQsS;											\
	v.UCompQsS = (v.Kp*v.Error) + v.UiQsS;								\
	v.UiQsS = (v.Kp*(v.Ki*v.Error)) + v.UiQsS;						\
																				\
/* Compute the estimated stator flux based on the integral of back emf*/		\
	v.OldEmf = v.EmfDsS;														\
	v.EmfDsS = v.UDsS - v.UCompDsS - (v.K5*v.IDsS);						\
	v.PsiDsS = v.PsiDsS + 0.5*((v.K6*(v.EmfDsS + v.OldEmf)));			\
																				\
	v.OldEmf = v.EmfQsS;														\
	v.EmfQsS = v.UQsS - v.UCompQsS - (v.K5*v.IQsS);						\
	v.PsiQsS = v.PsiQsS + 0.5*((v.K6*(v.EmfQsS + v.OldEmf)));			\
																				\
/* Estimate the rotor flux based on stator flux from the integral of back emf*/	\
																				\
	v.PsiDrS = (v.K7*v.PsiDsS) - (v.K8*v.IDsS);						\
	v.PsiQrS = (v.K7*v.PsiQsS) - (v.K8*v.IQsS);						\
																				\
/* Compute the rotor flux angle*/												\
	v.ThetaFlux = atan2(v.PsiQrS,v.PsiDrS);    \
	if (v.ThetaFlux < 0)	                               \
		v.ThetaFlux = v.ThetaFlux + 2*PI;

#endif /* INC_FLUXOBS_H_ */
