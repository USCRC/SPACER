/*
 * PFC_control.h
 *
 *  Created on: Mar 30, 2023
 *      Author: Goran Mandic
 */

#ifndef INC_PFC_CONTROL_H_
#define INC_PFC_CONTROL_H_


typedef volatile struct df22 {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float x1;
    float x2;
} DF22;



float RunFilterDF22(DF22 *filter, float input);
void InitFilterDF22(DF22 *filter, float b0, float b1, float b2, float a1, float a2);
void ResetFilterDF22(DF22 *filter);


#endif /* INC_PFC_CONTROL_H_ */
