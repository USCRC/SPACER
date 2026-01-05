/*
 * PFC_control.c
 *
 *  Created on: Mar 30, 2023
 *      Author: Goran Mandic
 */
#include "PFC_control.h"

float RunFilterDF22(DF22 *filter, float input){
	float y=filter->b0*input+filter->x1;
	filter->x1=filter->b1*input+filter->x2-filter->a1*y;
	filter->x2=filter->b2*input-filter->a2*y;
	return y;
}

void InitFilterDF22(DF22 *filter, float b0, float b1, float b2, float a1, float a2){
	filter->b0=b0;
	filter->b1=b1;
	filter->b2=b2;
	filter->a1=a1;
	filter->a2=a2;
	filter->x1=0.0f;
	filter->x2=0.0f;
}

void ResetFilterDF22(DF22 *filter){
	filter->x1=0.0f;
	filter->x2=0.0f;
}


