
#include <string.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_continuous.h"
#include "fir_filter.h"
#include "math.h"
#define EXAMPLE_READ_LEN   840 // 210 sample/cycle/channel 
#define GET_UNIT(x)        ((x>>3) & 0x1)

#define ADC_CONV_MODE       ADC_CONV_SINGLE_UNIT_1  //ESP32 only supports ADC1 DMA mode
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE1

static adc_channel_t channel[2] = {ADC_CHANNEL_7 , ADC_CHANNEL_6};
//static adc_channel_t channel[1] = {ADC_CHANNEL_7};
adc_continuous_handle_t handle1 = NULL;
static TaskHandle_t s_task_handle;
static const char *TAG = "EXAMPLE";

int16_t vol_sensor_data[210];
const float    U_Zero_Band		= 7.0; 

static float RD_Integral_Rms_U(int16_t Voltage_Data_In[])
{
	float Rms_Voltage;
	uint32_t Vol_Total_Buff =0;
	for(int i=0; i<(NUM_SAMPLE_UI - FIR_ORDER); i++)
	{
		int Vol_i_Buff = 0; 
		
		Vol_i_Buff = Voltage_Data_In[i];
		//Voltage_Data_In[i] =0; 
		Vol_Total_Buff = Vol_Total_Buff + Vol_i_Buff*Vol_i_Buff; 
		
	}
	
	Vol_Total_Buff = Vol_Total_Buff/(NUM_SAMPLE_UI - FIR_ORDER);
	
	Rms_Voltage	= sqrt(Vol_Total_Buff);
	Rms_Voltage = (Rms_Voltage < U_Zero_Band) ? 1:Rms_Voltage; 
	
  
	#if 0 // TEST pt v2
	/************* Rms_Voltage = Rms_Voltage * a + b ********************/
	const float Voltage_Phase1 = 134;
	const float Voltage_Phase3 = 197;
	const float Voltage_Phase4 = 234; 

	  float a = 1.1; //0.8;
	  float b = 2.6;  //1.2;

	if(Rms_Voltage <= Voltage_Phase1) 		{ a = 0.91;  	b = -24.5;}
	else if(Rms_Voltage <= Voltage_Phase3)  { a = 0.8;  	b = 1.2;}
	else if(Rms_Voltage <= Voltage_Phase4)  { a = 0.833;    b = -8.33;} 
	else    								{ a = 0.81; 	b = -1.3;}
	
	Rms_Voltage = Rms_Voltage * a + b;
	#endif 
	float a = 1.1; //0.8;
	float b = 2.6;  //1.2;
	Rms_Voltage = Rms_Voltage * a + b;
	if(Rms_Voltage <= 0.1) Rms_Voltage =0.1;
	//Rms_Voltage = Rms_Voltage  ;
	

	return Rms_Voltage;
	
}
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
   //ESP_ERROR_CHECK(adc_continuous_stop(handle));
    return (mustYield == pdTRUE);
}

static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = 840, //1024,
        .conv_frame_size = EXAMPLE_READ_LEN,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = 24 * 1000  ,
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_OUTPUT_TYPE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        uint8_t unit = GET_UNIT(channel[i]);
        uint8_t ch = channel[i] & 0x7;
        adc_pattern[i].atten = ADC_ATTEN_DB_11;
        adc_pattern[i].channel = ch;
        adc_pattern[i].unit = unit;
        adc_pattern[i].bit_width = SOC_ADC_DIGI_MAX_BITWIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%x", i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%x", i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%x", i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}


static bool check_valid_data(const adc_digi_output_data_t *data)
{
    const unsigned int unit = data->type2.unit;
    if (unit > 2) return false;
    if (data->type2.channel >= SOC_ADC_CHANNEL_NUM(unit)) return false;

    return true;
}
void app_main(void)
{
   esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[EXAMPLE_READ_LEN] = {0};
    memset(result, 0xcc, EXAMPLE_READ_LEN);

   s_task_handle = xTaskGetCurrentTaskHandle();


    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle1);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = s_conv_done_cb,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle1, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(handle1));

        while(1) {

        /**
         * This is to show you the way to use the ADC continuous mode driver event callback.
         * This `ulTaskNotifyTake` will block when the data processing in the task is fast.
         * However in this example, the data processing (print) is slow, so you barely block here.
         *
         * Without using this event callback (to notify this task), you can still just call
         * `adc_continuous_read()` here in a loop, with/without a certain block timeout.
         */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (1) {
            ret = adc_continuous_read(handle1, result, EXAMPLE_READ_LEN, &ret_num, 0);
            if (ret == ESP_OK) {
               // ESP_ERROR_CHECK(adc_continuous_stop(handle1));
                ESP_LOGI("TASK", "ret is %x, ret_num is %"PRIu32, ret, ret_num);
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES) {
                    adc_digi_output_data_t *p = (void*)&result[i];
        #if CONFIG_IDF_TARGET_ESP32
                    if(p->type1.channel == 6)
                    {
                        vol_sensor_data[i/4] = p->type1.data;
                      //  ESP_LOGI(TAG, "Unit: %d, Channel: %d, Value: %d, vol_sensor: %d", 1, p->type1.channel, p->type1.data, vol_sensor_data[i/4]);
                    }
        #else
        #endif
                }
                FIR_Filter_transform_U(&vol_sensor_data[0]);
                for(int i=0; i<210; i++)
                {
                   //  ESP_LOGI(TAG, "FIR Ch6: %d",vol_sensor_data[i]);
                }
                float vol_rms = RD_Integral_Rms_U(vol_sensor_data);
                vol_rms = vol_rms/1.7;
                ESP_LOGI(TAG, "Voltage RMS: %0.2f V",vol_rms);
                /**
                 * Because printing is slow, so every time you call `ulTaskNotifyTake`, it will immediately return.
                 * To avoid a task watchdog timeout, add a delay here. When you replace the way you process the data,
                 * usually you don't need this delay (as this task will block for a while).
                 */
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
            vTaskDelay(1000);
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle1));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle1));
}
