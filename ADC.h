
#ifndef MA_ADC_H__
#define MA_ADC_H__

#ifdef __cplusplus
extern "C" {
#endif
    
    #include "nrf_drv_saadc.h"
		#ifdef DEBUG_MODE
		#include "SEGGER_RTT.h"
		#endif
		void Adc12bitPolledInitialise(void);
    int16_t GetBatteryVoltage1( void );
    
#ifdef __cplusplus
}
#endif

#endif
