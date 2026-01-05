/*
 * DataLogger.h
 *
 *  Created on: Feb 28, 2023
 *      Author: Goran Mandic
 */

#ifndef INC_DATALOGGER_H_
#define INC_DATALOGGER_H_

typedef volatile struct{
	float *inputDataPointer1;
	float *inputDataPointer2;
	float *inputDataPointer3;
	float *inputDataPointer4;
	//float *inputDataPointer5;
	float *outputDataPointer1;
	float *outputDataPointer2;
	float *outputDataPointer3;
	float *outputDataPointer4;
	//float *outputDataPointer5;
	float previousValue;
	float triggerValue;
	int16_t status;
	int16_t preScalar;
	int16_t skipCount;
	int16_t size;
	int16_t count;
} DATA_LOG_4CH_FLOAT;

typedef volatile struct{
	int16_t *inputDataPointer1;
	int16_t *inputDataPointer2;
	int16_t *inputDataPointer3;
	int16_t *inputDataPointer4;
	//int16_t *inputDataPointer5;
	int16_t *outputDataPointer1;
	int16_t *outputDataPointer2;
	int16_t *outputDataPointer3;
	int16_t *outputDataPointer4;
	//int16_t *outputDataPointer5;
	int16_t previousValue;
	int16_t triggerValue;
	int16_t status;
	int16_t preScalar;
	int16_t skipCount;
	int16_t size;
	int16_t count;
} DATA_LOG_4CH_INT;


static inline void DATA_LOG_4CH_FLOAT_reset(DATA_LOG_4CH_FLOAT *d)
{
	d->previousValue=0;
	d->triggerValue=0;
	d->status=0;
	d->skipCount=0;
	d->count=0;
}


static inline void DATA_LOG_4CH_INT_reset(DATA_LOG_4CH_INT *d)
{
	d->previousValue=0;
	d->triggerValue=0;
	d->status=0;
	d->skipCount=0;
	d->count=0;
}


static inline void DATA_LOG_4CH_FLOAT_config(DATA_LOG_4CH_FLOAT *d,
                                  float *inputDataPointer1,
                                  float *inputDataPointer2,
                                  float *inputDataPointer3,
                                  float *inputDataPointer4,
								  //float *inputDataPointer5,
                                  float *outputDataPointer1,
                                  float *outputDataPointer2,
                                  float *outputDataPointer3,
                                  float *outputDataPointer4,
								  //float *outputDataPointer5,
                                  int16_t sz,
                                  float tV,
								  int16_t pS
                                  )
{
    d->inputDataPointer1 = inputDataPointer1;
    d->inputDataPointer2 = inputDataPointer2;
    d->inputDataPointer3 = inputDataPointer3;
    d->inputDataPointer4 = inputDataPointer4;
    //d->inputDataPointer5 = inputDataPointer5;
    d->outputDataPointer1 = outputDataPointer1;
    d->outputDataPointer2 = outputDataPointer2;
    d->outputDataPointer3 = outputDataPointer3;
    d->outputDataPointer4 = outputDataPointer4;
    //d->outputDataPointer5 = outputDataPointer5;
    d->size = sz;
    d->triggerValue = tV;
    d->preScalar = pS;
    d->status = 0;
    d->skipCount = 0;
    d->count = 0;
    d->previousValue = 0.0f;
}



static inline void DATA_LOG_4CH_INT_config(DATA_LOG_4CH_INT *d,
                                  int16_t *inputDataPointer1,
								  int16_t *inputDataPointer2,
								  int16_t *inputDataPointer3,
								  int16_t *inputDataPointer4,
								  //int16_t *inputDataPointer5,
								  int16_t *outputDataPointer1,
								  int16_t *outputDataPointer2,
								  int16_t *outputDataPointer3,
								  int16_t *outputDataPointer4,
								  //int16_t *outputDataPointer5,
                                  int16_t sz,
								  int16_t tV,
								  int16_t pS
                                  )
{
    d->inputDataPointer1 = inputDataPointer1;
    d->inputDataPointer2 = inputDataPointer2;
    d->inputDataPointer3 = inputDataPointer3;
    d->inputDataPointer4 = inputDataPointer4;
    //d->inputDataPointer5 = inputDataPointer5;
    d->outputDataPointer1 = outputDataPointer1;
    d->outputDataPointer2 = outputDataPointer2;
    d->outputDataPointer3 = outputDataPointer3;
    d->outputDataPointer4 = outputDataPointer4;
    //d->outputDataPointer5 = outputDataPointer5;
    d->size = sz;
    d->triggerValue = tV;
    d->preScalar = pS;
    d->status = 0;
    d->skipCount = 0;
    d->count = 0;
    d->previousValue = 0;
}


static inline void DATA_LOG_4CH_FLOAT_run(DATA_LOG_4CH_FLOAT *d)
{
	switch(d->status)
	{
		case 0:
			break;
        //
        // wait for trigger
        //
        case 1:
            if(*d->inputDataPointer1>d->triggerValue && d->previousValue<d->triggerValue)
            {
                //
                // rising edge detected start logging data
                //
                d->status=2;
            }
            break;
        case 2:
            d->skipCount++;
            if(d->skipCount==d->preScalar)
            {
                d->skipCount=0;
                d->outputDataPointer1[d->count]=*d->inputDataPointer1;
                d->outputDataPointer2[d->count]=*d->inputDataPointer2;
                d->outputDataPointer3[d->count]=*d->inputDataPointer3;
                d->outputDataPointer4[d->count]=*d->inputDataPointer4;
                //d->outputDataPointer5[d->count]=*d->inputDataPointer5;
                d->count++;
                if(d->count==d->size)
                {
                    d->count=0;
                    d->status=1;
                }
            }
            break;
	}
	d->previousValue=*d->inputDataPointer1;
}

static inline void DATA_LOG_4CH_INT_run(DATA_LOG_4CH_INT *d)
{
	switch(d->status)
	{
		case 0:
			break;
        //
        // wait for trigger
        //
        case 1:
            if(*d->inputDataPointer1>d->triggerValue && d->previousValue<d->triggerValue)
            {
                //
                // rising edge detected start logging data
                //
                d->status=2;
            }
            break;
        case 2:
            d->skipCount++;
            if(d->skipCount==d->preScalar)
            {
                d->skipCount=0;
                d->outputDataPointer1[d->count]=*d->inputDataPointer1;
                d->outputDataPointer2[d->count]=*d->inputDataPointer2;
                d->outputDataPointer3[d->count]=*d->inputDataPointer3;
                d->outputDataPointer4[d->count]=*d->inputDataPointer4;
                //d->outputDataPointer5[d->count]=*d->inputDataPointer5;
                d->count++;
                if(d->count==d->size)
                {
                    d->count=0;
                    d->status=1;
                }
            }
            break;
	}
	d->previousValue=*d->inputDataPointer1;
}


#endif /* INC_DATALOGGER_H_ */
