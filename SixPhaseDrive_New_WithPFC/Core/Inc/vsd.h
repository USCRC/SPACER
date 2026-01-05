/*
 * vsd.h
 *
 *  Created on: Mar 6, 2025
 *      Author: USSOAGO
 */

#ifndef INC_VSD_H_
#define INC_VSD_H_

typedef struct {  float  As1;  		    // Input: phase-a1 stator variable
				  float  Bs1;			// Input: phase-b1 stator variable
				  float  Cs1;			// Input: phase-c1 stator variable
				  float  As2;  		    // Input: phase-a1 stator variable
				  float  Bs2;			// Input: phase-b1 stator variable
				  float  Cs2;			// Input: phase-c1 stator variable
				  float  Alpha;		// Output: stationary d-axis stator variable
				  float  Beta;		// Output: stationary q-axis stator variable
				  float  X;
				  float  Y;
		 	 	} VSD;

#define VSD_DEFAULTS {    0, \
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

#define  Const1   0.33333333
#define  Const2   0.5
#define  Const3   0.86602540

#define VSD_MACRO(v)	\
v.Alpha = 	Const1*(v.As1 - Const2*(v.Bs1 + v.Cs1) + Const3*(v.As2 - v.Bs2)); \
v.Beta  = 	Const1*(Const3*(v.Bs1 - v.Cs1) + Const2*(v.As2 + v.Bs2) - v.Cs2); \
v.X =       Const1*(v.As1 - Const2*(v.Bs1 + v.Cs1) - Const3*(v.As2 - v.Bs2)); \
v.Y =       Const1*(-Const3*(v.Bs1 - v.Cs1) + Const2*(v.As2 + v.Bs2) - v.Cs2); \

#endif /* INC_VSD_H_ */
