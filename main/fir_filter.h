/**********************************************************************/
/* Freeware Demo SW from the www.dsp-weimich.com                      */
/* Version 1.0                                                        */
/* fir_filter.h                                                       */
/* FIR Filter                                                         */
/**********************************************************************/
#ifndef INC_FIR_FILTER_H_
#define INC_FIR_FILTER_H_
#include "stdint.h"
/* Export defines */
#define uint8    unsigned char 
#define sint8    signed char
#define uint16   unsigned int
#define sint16   signed int
#define uint32   unsigned long
#define sint32   signed long

/* FIR Filter Order */
#define FIR_ORDER             10
/* FIR coefficient factor: 2^17=131072. Only for integer variant */
#define COEFF_SHIFT_FACTOR    10
 #define NUM_SAMPLE_UI 210
#define U_FIR_OUTPUT_GAIN         360000
#define I_FIR_OUTPUT_GAIN         360000

typedef struct fir_history {
    uint16 fir_history_position;
    sint16 fir_history_buffer[FIR_ORDER-1];
} fir_history_s;

extern fir_history_s fir_history_U_data;
extern fir_history_s fir_history_I_data;
/* Export function */
 //extern const sint16 input_test_data[];
extern void fir_filter_initialization(void);
void fir_filters_intit(fir_history_s *p_fir_data);
extern sint16 fir_filter_float(sint16);
extern int16_t  fir_filter_integer(int16_t);
int16_t fir_filters_interger(fir_history_s *fir_his_data, int16_t x_input, int16_t *coeff_int_data, int32_t GAIN_i);
void FIR_Filter_transform_U(int16_t *U_input); //, int16_t *U_output);
void FIR_Filter_transform_I(int16_t *I_input); //, int16_t *I_output);

int16_t fir(int16_t NewSample);

#endif /* INC_FIR_FILTER_H_ */
