#ifndef ZIGBEE_LACKING_H__
#define ZIGBEE_LACKING_H__

#include "zb_zcl_power_config.h"
#include "zb_zcl_rel_humidity_measurement.h"

/**@brief power Measurement cluster attributes according to ZCL Specification 4.5.2.1.1. */
typedef struct
{
    zb_uint8_t battery_voltage;
    zb_uint8_t battery_remaining_percentage;
} battery_simplified_attr_t;


#define ZB_ZCL_DECLARE_POWER_CONFIG_SIMPLIFIED_ATTRIB_LIST(attr_list,voltage, remaining)  \
  ZB_ZCL_START_DECLARE_ATTRIB_LIST(attr_list)                                             \
  ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID((voltage),),         \
  ZB_SET_ATTR_DESCR_WITH_ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_REMAINING_ID((remaining), ),    \
  ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST

/**@brief Humidity Measurement cluster attributes according to ZCL Spec 4.4.2.1.1. */
typedef struct
{
    zb_int16_t  measure_value;
    zb_int16_t  min_measure_value;
    zb_int16_t  max_measure_value;
} zb_zcl_humm_measurement_attrs_t;

#ifdef __cplusplus
}
#endif
#endif // ZB_MULTI_SENSOR_H__