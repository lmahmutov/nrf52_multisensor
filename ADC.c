
#include "ADC.h"

// Input range of internal Vdd measurement = (0.6 V)/(1/6) = 3.6 V
// 3.0 volts -> 14486 ADC counts with 14-bit sampling: 4828.8 counts per volt
#define ADC12_COUNTS_PER_VOLT 4551
//uint16_t bat_volt;
uint8_t bat_volt1;
uint8_t bat_volt2;

/**
 * @brief Function for 14-bit adc init in polled mode
 */
void Adc12bitPolledInitialise(void)
{
    nrf_saadc_channel_config_t myConfig =
    {
        .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
        .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
        .gain       = NRF_SAADC_GAIN1_6,
        .reference  = NRF_SAADC_REFERENCE_INTERNAL,
        .acq_time   = NRF_SAADC_ACQTIME_40US,
        .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
        .burst      = NRF_SAADC_BURST_ENABLED,
        .pin_p      = NRF_SAADC_INPUT_VDD,
        .pin_n      = NRF_SAADC_INPUT_DISABLED
    };

    nrf_saadc_resolution_set((nrf_saadc_resolution_t) 3);   // 3 is 14-bit
    nrf_saadc_oversample_set((nrf_saadc_oversample_t) 2);   // 2 is 4x, about 150uSecs total
    nrf_saadc_int_disable(NRF_SAADC_INT_ALL);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
    nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
    nrf_saadc_enable();

    NRF_SAADC->CH[1].CONFIG =
              ((myConfig.resistor_p << SAADC_CH_CONFIG_RESP_Pos)   & SAADC_CH_CONFIG_RESP_Msk)
            | ((myConfig.resistor_n << SAADC_CH_CONFIG_RESN_Pos)   & SAADC_CH_CONFIG_RESN_Msk)
            | ((myConfig.gain       << SAADC_CH_CONFIG_GAIN_Pos)   & SAADC_CH_CONFIG_GAIN_Msk)
            | ((myConfig.reference  << SAADC_CH_CONFIG_REFSEL_Pos) & SAADC_CH_CONFIG_REFSEL_Msk)
            | ((myConfig.acq_time   << SAADC_CH_CONFIG_TACQ_Pos)   & SAADC_CH_CONFIG_TACQ_Msk)
            | ((myConfig.mode       << SAADC_CH_CONFIG_MODE_Pos)   & SAADC_CH_CONFIG_MODE_Msk)
            | ((myConfig.burst      << SAADC_CH_CONFIG_BURST_Pos)  & SAADC_CH_CONFIG_BURST_Msk);

    NRF_SAADC->CH[1].PSELN = myConfig.pin_n;
    NRF_SAADC->CH[1].PSELP = myConfig.pin_p;
}

/**
 * @brief Function for 14-bit adc battery voltage by direct blocking reading
 */
int16_t GetBatteryVoltage1(void)
{			
		Adc12bitPolledInitialise();
		uint16_t result = 9999;         // Some recognisable dummy value
		uint32_t timeout = 10000;       // Trial and error
		volatile int16_t buffer[8];
		// Enable command
		nrf_saadc_enable();
		NRF_SAADC->RESULT.PTR = (uint32_t)buffer;
		NRF_SAADC->RESULT.MAXCNT = 1;
		nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
		nrf_saadc_task_trigger(NRF_SAADC_TASK_START);
		nrf_saadc_task_trigger(NRF_SAADC_TASK_SAMPLE);

		while (0 == nrf_saadc_event_check(NRF_SAADC_EVENT_END) && timeout > 0)
		{
			timeout--;
		}
		nrf_saadc_task_trigger(NRF_SAADC_TASK_STOP);
		nrf_saadc_event_clear(NRF_SAADC_EVENT_STARTED);
		nrf_saadc_event_clear(NRF_SAADC_EVENT_END);
		// Disable command to reduce power consumption
		nrf_saadc_disable();
		if (timeout != 0)
		{		
			result = (((buffer[0] * 1000L)+(ADC12_COUNTS_PER_VOLT/2)) / ADC12_COUNTS_PER_VOLT);
			#if DEBUG_MODE
			SEGGER_RTT_printf(0, "battery_voltage: %d\n", result);
			#endif
		}    
		return result;
}
