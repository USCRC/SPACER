/*
 * ivsd.h
 *
 *  Created on: Mar 7, 2025
 *      Author: USSOAGO
 */

#ifndef INC_IVSD_H_
#define INC_IVSD_H_

typedef struct {  float  Alpha;  		// Input: Alpha variable
				  float  Beta;			// Input: Beta variable
				  float  X;			    // Input: X  variable
				  float  Y;  		    // Input: Y  variable
				  float  As1;			// Output: phase-A1 stator variable
				  float  Bs1;			// Output: phase-B1 stator variable
				  float  Cs1;		    // Output: phase-C1 stator variable
				  float  As2;		    // Output: phase-A2 stator variable
				  float  Bs2;           // Output: phase-B2 stator variable
				  float  Cs2;           // Output: phase-Const2 stator variable
		 	 	} IVSD;

#define IVSD_DEFAULTS {   0, \
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

#define  Const2   0.5
#define  Const3   0.86602540

#define IVSD_MACRO(v)	\
v.As1 = v.Alpha + v.X;                      \
v.Bs1 = -Const2*(v.Alpha + v.X) + Const3*(v.Beta  - v.Y); \
v.Cs1 = -Const2*(v.Alpha + v.X) - Const3*(v.Beta  - v.Y); \
v.As2 =  Const3*(v.Alpha - v.X) + Const2*(v.Beta  + v.Y);  \
v.Bs2 = -Const3*(v.Alpha - v.X) + Const2*(v.Beta  + v.Y); \
v.Cs2 = -v.Beta - v.Y; \

#endif /* INC_IVSD_H_ */
