/**********************************************************************/
/* Freeware Demo SW from the www.dsp-weimich.com                      */
/* Version 1.0                                                        */
/* fir_filter.c                                                       */
/* FIR Filter                                                         */
/* y(n) = x(n)h(0)+x(n-1)h(1)+...+x(n-N+1)h(N-1)                      */
/**********************************************************************/

#include "fir_filter.h"



#if 0 // Test_Data
const sint16 input_test_data[]=
{
    1727, 1716, 1795, 1655, 1821, 1793, 1696, 1837, 1869, 1887,
    1880, 1899, 1919, 1928, 1953, 2036, 1890, 2015, 2023, 2070,    
    1893, 2092, 2137, 2059, 2168, 2192, 2202, 2195, 2203, 2249,
    2140, 2291, 2301, 2339, 2348, 2358, 2383, 2401, 2425, 2418,
    2426, 2451, 2465, 2497, 2504, 2511, 2520, 2533, 2619, 2571,
    2604, 2615, 2583, 2582, 2625, 2651, 2613, 2621, 2667, 2674,
    2628, 2646, 2651, 2640, 2660, 2685, 2657, 2591, 2631, 2602,
    2639, 2559, 2595, 2558, 2559, 2579, 2495, 2543, 2529, 2517,
    2527, 2531, 2505, 2487, 2529, 2507, 2500, 2476, 2465, 2443,
    2425, 2403, 2368, 2389, 2368, 2365, 2327, 2320 ,2318 ,2301,
    2257, 2251, 2234, 2227, 2190, 2054, 2156, 2164, 2128, 2109, 
    2090, 2069, 2036, 2016, 1992, 1956, 1957, 1831, 1936, 1894,
    1877, 1838, 1840, 1813, 1826, 1821, 1776, 1771, 1754, 1741, 
    1704, 1730, 1715, 1673, 1680, 1635, 1644, 1621, 1617, 1612, 
    1599, 1631, 1634, 1597, 1573, 1547, 1580, 1607, 1546, 1557, 
    1319, 1532, 1548, 1585, 1520, 1483, 1464, 1560, 1532, 1501, 
    1407, 1532, 1471, 1487, 1492, 1519, 1473, 1520, 1511, 1522, 
    1536, 1522, 1537, 1568, 1567, 1605, 1623, 1643, 1638, 1664, 
    1682, 1686, 1685, 1727, 1752, 1751, 1766, 1765, 1805, 1811, 
    1799, 1845, 1877, 1874, 1908, 1935, 1995, 1960, 2000, 2075,
	2027, 2059, 2121, 2171, 2133, 2137, 2155, 2157, 2192, 2237	
};
#endif // test_data



#define Ntap 10

#define DCgain 262144

int16_t fir(int16_t NewSample) {
    int16_t FIRCoef[Ntap] = { 
        25615,
        26169,
        26569,
        26812,
        26893,
        26812,
        26569,
        26169,
        25615,
        24916
    };

    static int16_t x[Ntap]; //input samples
    int32_t y=0;            //output sample
    int n;

    //shift the old samples
    for(n=Ntap-1; n>0; n--)
       x[n] = x[n-1];

    //Calculate the new output
    x[0] = NewSample;
    for(n=0; n<Ntap; n++)
        y += FIRCoef[n] * x[n];
    
    return y / DCgain;
}
/* Float FIR coefficients */
const float fir_coeff_float[17] = 
{
    -0.012200, -0.021014, -0.024591, -0.011148,  0.025974, 
     0.083952,  0.148888,  0.200253,  0.219802,  0.200253, 
     0.148888,  0.083952,  0.025974, -0.011148, -0.024591, 
    -0.021014, -0.012200
};

/* Integer FIR coefficients with the factor 2^COEFF_SHIFT_FACTOR=2^17=131072 */
// const sint16 fir_coeff_integer[FIR_ORDER] = 
// {
//     -1599,  -2754,  -3223,  -1461,   3404,  11004,  19515,  26248,  28810,  26248, 
//     19515,  11004,   3404,  -1461,  -3223,  -2754,  -1599
// };

  int16_t fir_coeff_integer[FIR_ORDER] = 
{
    25615, 26169, 26569, 26812, 26893, 26812, 26569, 26169, 25615, 24916
};
/* FIR History */
fir_history_s fir_history_data;

fir_history_s fir_history_U_data;

fir_history_s fir_history_I_data;
/*
 FIR Filter Initialization
 Input:  void
 Return: void
*/
void fir_filter_initialization(void)
{
    uint16 index;
    fir_history_data.fir_history_position = 0;
    for(index=0; index < (FIR_ORDER-1); index++)
    {
        fir_history_data.fir_history_buffer[index]=0;
    }
}

/*
 FIR Filter Initialization
 use it if you want to use many FIR filters
 Input:  poiter of fir_data 
 Return: void
*/

void fir_filters_intit(fir_history_s *p_fir_data)
{
    uint16 index;
    p_fir_data->fir_history_position = 0;
    for(index=0; index < (FIR_ORDER-1); index++)
    {
        p_fir_data->fir_history_buffer[index] = 0;
    }
}
/*
 Float FIR Filter: coefficients have a float format
 Input
 sint16 x_input - filter input x(n)
 Return
 sint16 - filter output y(n)
*/
sint16 fir_filter_float(sint16 x_input)
{
    uint16 index;
	float y_output;

    /* Calculate the output value */
	y_output = ((float)x_input)*fir_coeff_float[0];
	for(index=0; index < (FIR_ORDER-1); index++)
	{
		y_output += ((float)fir_history_data.fir_history_buffer[(index+fir_history_data.fir_history_position)%((FIR_ORDER-1))])*fir_coeff_float[index+1];
	}
	/* New history position */
	if(0 == fir_history_data.fir_history_position)
	{
		fir_history_data.fir_history_position = FIR_ORDER-2;
	}
	else
	{
		fir_history_data.fir_history_position--;
	}
	/* Save the history value */
	fir_history_data.fir_history_buffer[fir_history_data.fir_history_position]=x_input;

	return (sint16)(y_output);
}




/*
 Integer FIR Filter
 Input
 sint16 x_input - filter input x(n)
 Return
 sint16 - filter output y(n)
*/
int16_t fir_filter_integer(int16_t x_input)
{
	
    uint16 index;
	int32_t y_output;

    /* Calculate the output value */
	y_output = ((sint32)x_input)*((sint32)fir_coeff_integer[0]);
	for(index=0; index < (FIR_ORDER-1); index++)
	{
		y_output += ((sint32)fir_history_data.fir_history_buffer[ (index+fir_history_data.fir_history_position) % ( (FIR_ORDER-1) )])*((sint32)fir_coeff_integer[index+1]);
	}
	/* New history position */
	if(0 == fir_history_data.fir_history_position)
	{
		fir_history_data.fir_history_position = FIR_ORDER-4;
	}
	else
	{
		fir_history_data.fir_history_position--;
	}
	/* Save the history value */
	fir_history_data.fir_history_buffer[fir_history_data.fir_history_position] = x_input;

	//return (sint16)(y_output>>(10));
	return  (y_output/U_FIR_OUTPUT_GAIN);
}

int16_t fir_filters_interger(fir_history_s *fir_his_data, int16_t x_input, int16_t *coeff_int_data, int32_t GAIN_i)
{ 
    uint16_t index;
    int32_t  y_out_put;

    /* Calculate the output value */
    y_out_put = ((int32_t) x_input) * ((int32_t) (*coeff_int_data)); 

    for(index =0; index < (FIR_ORDER-1); index++)
    {
       y_out_put += ((int32_t) (fir_his_data->fir_history_buffer[(index + fir_his_data->fir_history_position)%(FIR_ORDER-1)]) ) * ( (int32_t) (*(coeff_int_data + (index + 1))) );   
    }

    /* New history position */
     if( 0 == fir_his_data->fir_history_position) 
    {
      fir_his_data->fir_history_position = FIR_ORDER - 4; 
       
    }
    else
    {
         fir_his_data->fir_history_position--; 
    }
    /* Save the history value */
     fir_his_data->fir_history_buffer[fir_his_data->fir_history_position] = x_input; 
 
     return (y_out_put/GAIN_i); 

	
}
void FIR_Filter_transform_U(int16_t *U_input ) //, int16_t *U_output)
{
	fir_filters_intit(&fir_history_U_data);
	
    uint16_t ADC_0V_Calib = 2834; // RD_Cal_0v_ADC(U_input); RD_Edit: 

    for(int i=0; i< NUM_SAMPLE_UI; i++)
    { 

       // int16_t output_data = fir_filters_interger(&fir_history_U_data, *(U_input + i) - CALIB_U0V_ADC,  &fir_coeff_integer[0], U_FIR_OUTPUT_GAIN);
        int16_t output_data = fir_filters_interger(&fir_history_U_data, *(U_input + i) - ADC_0V_Calib,  &fir_coeff_integer[0], U_FIR_OUTPUT_GAIN);
        
        if(i>=10)
        { 
					*(U_input + i - 10) = output_data;
        }
    }
}

void FIR_Filter_transform_I(int16_t *I_input) //, int16_t *I_output)
{
	fir_filters_intit(&fir_history_I_data);
	
		#if 0 // print I
		uartCsend_DEBUG(" B_FIR_ampe");
		for(int i=0; i<210; i++)
		{
			uint8_t Buff[64];
			sprintf((char *) Buff, "%d \n", *(I_input+i) ); 
			uartCsend_DEBUG((char *) Buff);
		}
		#endif 
	uint16_t ADC_0V_Calib = 2834;//RD_Cal_0v_ADC(I_input);
	
    for(int i=0; i< NUM_SAMPLE_UI; i++)
    {
        //int16_t output_data = fir_filters_interger(&fir_history_I_data, *(I_input + i) - CALIB_I0A_ADC, &fir_coeff_integer[0], I_FIR_OUTPUT_GAIN);

		int16_t output_data = fir_filters_interger(&fir_history_I_data, *(I_input + i) - ADC_0V_Calib, &fir_coeff_integer[0], I_FIR_OUTPUT_GAIN);
		
        if(i>=10)
        {
          //  *(I_output + i - 10) = output_data;
			    *(I_input + i - 10) = output_data;
        }
    }
		
		#if 0 // print I
		uartCsend_DEBUG(" A_FIR_ampe");
		for(int i=0; i<200; i++)
		{
			uint8_t Buff[64];
			sprintf((char *) Buff, "%d \n", *(I_input+i) ); 
			uartCsend_DEBUG((char *) Buff);
		}
		#endif 
}
/* Only Test */
#if 0
int main_test(void)
{
    uint16 index;
    /* Step */
	#if 0
    const sint16 input_test_data[]=
    {
        1727, 1716, 1795, 1655, 1821, 1793, 1696, 1837, 1869, 1887,
        1880, 1899, 1919, 1928, 1953, 2036, 1890, 2015, 2023, 2070,    
        1893, 2092, 2137, 2059, 2168, 2192, 2202, 2195, 2203, 2249,
        2140, 2291, 2301, 2339, 2348, 2358, 2383, 2401, 2425, 2418,
        2426, 2451, 2465, 2497, 2504, 2511, 2520, 2533, 2619, 2571,
        2604, 2615, 2583, 2582, 2625, 2651, 2613, 2621, 2667, 2674,
        2628, 2646, 2651, 2640, 2660, 2685, 2657, 2591, 2631, 2602,
        2639, 2559, 2595, 2558, 2559, 2579, 2495, 2543, 2529, 2517,
        2527, 2531, 2505, 2487, 2529, 2507, 2500, 2476, 2465, 2443,
        2425, 2403, 2368, 2389, 2368, 2365, 2327, 2320 ,2318 ,2301,
        2257, 2251, 2234, 2227, 2190, 2054, 2156, 2164, 2128, 2109, 
        2090, 2069, 2036, 2016, 1992, 1956, 1957, 1831, 1936, 1894,
        1877, 1838, 1840, 1813, 1826, 1821, 1776, 1771, 1754, 1741, 
        1704, 1730, 1715, 1673, 1680, 1635, 1644, 1621, 1617, 1612, 
        1599, 1631, 1634, 1597, 1573, 1547, 1580, 1607, 1546, 1557, 
        1319, 1532, 1548, 1585, 1520, 1483, 1464, 1560, 1532, 1501, 
        1407, 1532, 1471, 1487, 1492, 1519, 1473, 1520, 1511, 1522, 
        1536, 1522, 1537, 1568, 1567, 1605, 1623, 1643, 1638, 1664, 
        1682, 1686, 1685, 1727, 1752, 1751, 1766, 1765, 1805, 1811, 
        1799, 1845, 1877, 1874, 1908, 1935, 1995, 1960, 2000, 2075 
	};
	#endif

    /* Sinus, Amlitude=16383, Frequency 500Hz, Length 100 samples, Sampling Frequency 12kHz */
/*
    const sint16 input_test_data[]=
    {
             0,   4240,   8191,  11585,  14188,  15825,  16383,  15825,  14188,  11585, 
          8192,   4240,      0,  -4240,  -8191, -11585, -14188, -15825, -16383, -15825, 
        -14188, -11585,  -8192,  -4240,      0,   4240,   8191,  11585,  14188,  15825, 
         16383,  15825,  14188,  11585,   8192,   4240,      0,  -4240,  -8191, -11585, 
        -14188, -15825, -16383, -15825, -14188, -11585,  -8192,  -4240,      0,   4240, 
          8191,  11585,  14188,  15825,  16383,  15825,  14188,  11585,   8192,   4240, 
             0,  -4240,  -8191, -11585, -14188, -15825, -16383, -15825, -14188, -11585, 
         -8192,  -4240,      0,   4240,   8191,  11585,  14188,  15825,  16383,  15825, 
         14188,  11585,   8192,   4240,      0,  -4240,  -8191, -11585, -14188, -15825, 
        -16383, -15825, -14188, -11585,  -8192,  -4240,      0,   4240,   8191,  11585
    };
*/
    /* Sinus, Amlitude=16383, Frequency 2000Hz, Length 100 samples, Sampling Frequency 12kHz */
/*
    const sint16 input_test_data[]=
    {
             0,  14188,  14188,      0, -14188, -14188,      0,  14188,  14188,      0, 
        -14188, -14188,      0,  14188,  14188,      0, -14188, -14188,      0,  14188, 
         14188,      0, -14188, -14188,      0,  14188,  14188,      0, -14188, -14188, 
             0,  14188,  14188,      0, -14188, -14188,      0,  14188,  14188,      0, 
        -14188, -14188,      0,  14188,  14188,      0, -14188, -14188,      0,  14188, 
         14188,      0, -14188, -14188,      0,  14188,  14188,      0, -14188, -14188, 
             0,  14188,  14188,      0, -14188, -14188,      0,  14188,  14188,      0, 
        -14188, -14188,      0,  14188,  14188,      0, -14188, -14188,      0,  14188, 
         14188,      0, -14188, -14188,      0,  14188,  14188,      0, -14188, -14188, 
             0,  14188,  14188,      0, -14188, -14188,      0,  14188,  14188,      0
    };
*/
    /* Frequency 500Hz + 2000Hz, Length 100 samples, Sampling Frequency 12kHz */
/*
    const sint16 input_test_data[]=
    {
             0,  18428,  22379,  11585,      0,   1637,  16383,  30013,  28376,  11585, 
         -5996,  -9948,      0,   9948,   5997, -11585, -28376, -30013, -16383,  -1637, 
             0, -11585, -22380, -18428,      0,  18428,  22379,  11585,      0,   1637, 
         16383,  30013,  28376,  11585,  -5996,  -9948,      0,   9948,   5997, -11585,
        -28376, -30013, -16383,  -1637,      0, -11585, -22380, -18428,      0,  18428,
         22379,  11585,      0,   1637,  16383,  30013,  28376,  11585,  -5996,  -9948, 
             0,   9948,   5997, -11585, -28376, -30013, -16383,  -1637,      0, -11585, 
        -22380, -18428,      0,  18428,  22379,  11585,      0,   1637,  16383,  30013, 
         28376,  11585,  -5996,  -9948,      0,   9948,   5997, -11585, -28376, -30013, 
        -16383,  -1637,      0, -11585, -22380, -18428,      0,  18428,  22379,  11585
    };
*/
    sint16 output_fir_data_integer[sizeof(input_test_data)/sizeof(sint16)];
    sint16 output_fir_data_float[sizeof(input_test_data)/sizeof(sint16)];
	
    /* Integer Variant */
	fir_filter_initialization();
	
	for (index=0; index < sizeof(input_test_data)/sizeof(sint16); index++)
	{
		output_fir_data_integer[index]=fir_filter_integer(input_test_data[index]);
	}

    /* Float Variant */
	fir_filter_initialization();
	
	for (index=0; index < sizeof(input_test_data)/sizeof(sint16); index++)
	{
		output_fir_data_float[index]=fir_filter_float(input_test_data[index]);
	}

    return 0;
}
#endif
