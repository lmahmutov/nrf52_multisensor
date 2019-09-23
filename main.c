/**
 * Copyright (c) 2018 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup zigbee_examples_multi_sensor main.c
 * @{
 * @ingroup zigbee_examples
 * @brief Zigbee Pressure and Temperature sensor
 */

#include "zboss_api.h"
#include "zb_mem_config_med.h"
#include "zb_error_handler.h"
#include "zigbee_helpers.h"
#include "app_timer.h"
#include "bsp.h"
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "zb_multi_sensor.h"
/* I2c library */
#include "I2C.h"
#include "BME280.h"
#include "ADC.h"

#define IEEE_CHANNEL_MASK                  ZB_TRANSCEIVER_ALL_CHANNELS_MASK     /**< Scan all channel to find the coordinator. */
#define ERASE_PERSISTENT_CONFIG            ZB_FALSE                             /**< Do not erase NVRAM to save the network parameters after device reboot or power-off. */

#define ZIGBEE_NETWORK_STATE_LED           BSP_BOARD_LED_2                      /**< LED indicating that light switch successfully joind ZigBee network. */
#define LEAVE_JOIN_BUTTON                  BSP_BOARD_BUTTON_0                  /**< Button ID used to join to network and leave from network*/
#define LEAVE_JOIN_BUTTON_THRESHOLD        ZB_TIME_ONE_SECOND*4                      /**< Number of beacon intervals the button should be pressed to dimm the light bulb. */
#define LEAVE_JOIN_BUTTON_SHORT_POLL_TMO   ZB_MILLISECONDS_TO_BEACON_INTERVAL(50)  /**< Delay between button state checks used in order to detect button long press. */
#define LEAVE_JOIN_BUTTON_LONG_POLL_TMO    ZB_MILLISECONDS_TO_BEACON_INTERVAL(300) /**< Time after which the button state is checked again to detect button hold - the dimm command is sent again. */

static zb_bool_t in_progress;
static zb_bool_t joined;
static zb_time_t timestamp;

/* --------- BME280--------*/
static int32_t  resultPTH[3];
/* ----------------------- */
#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE to compile End Device source code.
#endif

static sensor_device_ctx_t m_dev_ctx;

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(identify_attr_list, &m_dev_ctx.identify_attr.identify_time);

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(basic_attr_list,
                                     &m_dev_ctx.basic_attr.zcl_version,
                                     &m_dev_ctx.basic_attr.app_version,
                                     &m_dev_ctx.basic_attr.stack_version,
                                     &m_dev_ctx.basic_attr.hw_version,
                                     m_dev_ctx.basic_attr.mf_name,
                                     m_dev_ctx.basic_attr.model_id,
                                     m_dev_ctx.basic_attr.date_code,
                                     &m_dev_ctx.basic_attr.power_source,
                                     m_dev_ctx.basic_attr.location_id,
                                     &m_dev_ctx.basic_attr.ph_env,
                                     m_dev_ctx.basic_attr.sw_ver);


ZB_ZCL_DECLARE_TEMP_MEASUREMENT_ATTRIB_LIST(temperature_attr_list, 
                                            &m_dev_ctx.temp_attr.measure_value,
                                            &m_dev_ctx.temp_attr.min_measure_value, 
                                            &m_dev_ctx.temp_attr.max_measure_value, 
                                            &m_dev_ctx.temp_attr.tolerance);

ZB_ZCL_DECLARE_REL_HUMIDITY_MEASUREMENT_ATTRIB_LIST(humydity_attr_list, 
                                            &m_dev_ctx.humm_attr.measure_value,
                                            &m_dev_ctx.humm_attr.min_measure_value, 
                                            &m_dev_ctx.humm_attr.max_measure_value);

ZB_ZCL_DECLARE_PRES_MEASUREMENT_ATTRIB_LIST(pressure_attr_list, 
                                            &m_dev_ctx.pres_attr.measure_value, 
                                            &m_dev_ctx.pres_attr.min_measure_value, 
                                            &m_dev_ctx.pres_attr.max_measure_value, 
                                            &m_dev_ctx.pres_attr.tolerance);

ZB_ZCL_DECLARE_POWER_CONFIG_SIMPLIFIED_ATTRIB_LIST(battery_simplified_attr_list, 
                                            &m_dev_ctx.power_attr.battery_voltage,
                                            &m_dev_ctx.power_attr.battery_remaining_percentage,
                                            &m_dev_ctx.power_attr.alarm_state);

ZB_DECLARE_MULTI_SENSOR_CLUSTER_LIST(multi_sensor_clusters,
                                     basic_attr_list,
                                     identify_attr_list,
                                     temperature_attr_list,
                                     humydity_attr_list,
                                     pressure_attr_list,
                                     battery_simplified_attr_list);

ZB_ZCL_DECLARE_MULTI_SENSOR_EP(multi_sensor_ep,
                               MULTI_SENSOR_ENDPOINT,
                               multi_sensor_clusters);

ZBOSS_DECLARE_DEVICE_CTX_1_EP(multi_sensor_ctx, multi_sensor_ep);

APP_TIMER_DEF(zb_app_timer);


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing all clusters attributes.
 */
static void multi_sensor_clusters_attr_init(void)
{
    /* Basic cluster attributes data */
    m_dev_ctx.basic_attr.zcl_version   = ZB_ZCL_VERSION;
    m_dev_ctx.basic_attr.app_version   = SENSOR_INIT_BASIC_APP_VERSION;
    m_dev_ctx.basic_attr.stack_version = SENSOR_INIT_BASIC_STACK_VERSION;
    m_dev_ctx.basic_attr.hw_version    = SENSOR_INIT_BASIC_HW_VERSION;

    /* Use ZB_ZCL_SET_STRING_VAL to set strings, because the first byte should
     * contain string length without trailing zero.
     *
     * For example "test" string wil be encoded as:
     *   [(0x4), 't', 'e', 's', 't']
     */
    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.mf_name,
                          SENSOR_INIT_BASIC_MANUF_NAME,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_MANUF_NAME));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.model_id,
                          SENSOR_INIT_BASIC_MODEL_ID,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_MODEL_ID));

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.date_code,
                          SENSOR_INIT_BASIC_DATE_CODE,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_DATE_CODE));

    m_dev_ctx.basic_attr.power_source = SENSOR_INIT_BASIC_POWER_SOURCE;

    ZB_ZCL_SET_STRING_VAL(m_dev_ctx.basic_attr.location_id,
                          SENSOR_INIT_BASIC_LOCATION_DESC,
                          ZB_ZCL_STRING_CONST_SIZE(SENSOR_INIT_BASIC_LOCATION_DESC));


    m_dev_ctx.basic_attr.ph_env = SENSOR_INIT_BASIC_PH_ENV;

    /* Identify cluster attributes data */
    m_dev_ctx.identify_attr.identify_time        = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

     /* Temperature measurement cluster attributes data */
    m_dev_ctx.temp_attr.measure_value            = ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_UNKNOWN;
    m_dev_ctx.temp_attr.min_measure_value        = ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_MIN_VALUE;
    m_dev_ctx.temp_attr.max_measure_value        = ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_MAX_VALUE;
    m_dev_ctx.temp_attr.tolerance                = ZB_ZCL_ATTR_TEMP_MEASUREMENT_TOLERANCE_MAX_VALUE;

    /* Pressure measurement cluster attributes data */
    m_dev_ctx.pres_attr.measure_value            = ZB_ZCL_ATTR_PRES_MEASUREMENT_VALUE_UNKNOWN;
    m_dev_ctx.pres_attr.min_measure_value        = ZB_ZCL_ATTR_PRES_MEASUREMENT_MIN_VALUE_MIN_VALUE;
    m_dev_ctx.pres_attr.max_measure_value        = ZB_ZCL_ATTR_PRES_MEASUREMENT_MAX_VALUE_MAX_VALUE;
    m_dev_ctx.pres_attr.tolerance                = ZB_ZCL_ATTR_PRES_MEASUREMENT_TOLERANCE_MAX_VALUE;
    
    /* humidity measurement cluster attributes data */
    m_dev_ctx.humm_attr.measure_value            = ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_UNKNOWN;
    m_dev_ctx.humm_attr.min_measure_value        = ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_MIN_VALUE;
    m_dev_ctx.humm_attr.max_measure_value        = ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_MIN_VALUE_MAX_VALUE;


    /* Voltage measurement cluster attributes data */
    m_dev_ctx.power_attr.battery_voltage          = ZB_ZCL_POWER_CONFIG_BATTERY_VOLTAGE_INVALID;
    m_dev_ctx.power_attr.battery_remaining_percentage        = ZB_ZCL_POWER_CONFIG_BATTERY_REMAINING_UNKNOWN;
    m_dev_ctx.power_attr.alarm_state              = ZB_ZCL_POWER_CONFIG_BATTERY_ALARM_STATE_DEFAULT_VALUE;
}

/**@brief Callback for detecting button press duration.
 *
 * @param[in]   button   BSP Button that was pressed.
 */
static zb_void_t leave_join_button_handler(zb_uint8_t button)
{
    zb_time_t current_time;
    zb_bool_t short_expired;
    zb_bool_t  ret;
    zb_ret_t zb_err_code;

    current_time = ZB_TIMER_GET();

    if (ZB_TIME_SUBTRACT(current_time, timestamp) > LEAVE_JOIN_BUTTON_THRESHOLD)
    {
        short_expired = ZB_TRUE;
    }
    else
    {
        short_expired = ZB_FALSE;
    }

    /* Check if button was released during LEAVE_JOIN_BUTTON_SHORT_POLL_TMO. */
    if (!bsp_button_is_pressed(button))
    {
        if (!short_expired)
        {
            /* Check joined or not!!!. */
            if (joined)
            {
              NRF_LOG_INFO("Short press. Joined, asking from network for new data");
              zb_zdo_pim_start_turbo_poll_packets(10);
            }
            else
            {
              NRF_LOG_INFO("Short press. Not joined, starting join attempt");
              ret = bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
              ZB_COMM_STATUS_CHECK(ret);
            }            
        }

        /* Button released - wait for accept next event. */
        in_progress = ZB_FALSE;
    }
    else
    {
        if (short_expired)
        {
           if (joined)
            {
              NRF_LOG_INFO("Long press. Leaving from network - and soft reset");
              zb_bdb_reset_via_local_action(0);
              NVIC_SystemReset();
            }
            else
            {
              NRF_LOG_INFO("Long press. Not joined");
            }
            /* Long press detected - wait for accept next event. */
            in_progress = ZB_FALSE;
        }
        else
        {
            /* Wait another LEAVE_JOIN_BUTTON_SHORT_POLL_TMO, until LEAVE_JOIN_BUTTON_THRESHOLD will be reached. */
            zb_err_code = ZB_SCHEDULE_ALARM(leave_join_button_handler, button, LEAVE_JOIN_BUTTON_SHORT_POLL_TMO);
            ZB_ERROR_CHECK(zb_err_code);
        }
    }
}


/**@brief Callback for button events.
 *
 * @param[in]   evt      Incoming event from the BSP subsystem.
 */
static void buttons_handler(bsp_event_t evt)
{
    zb_ret_t zb_err_code;
    zb_uint32_t button;

    switch(evt)
    {
        case BSP_EVENT_KEY_0:
            button = LEAVE_JOIN_BUTTON;
            break;

        default:
            NRF_LOG_INFO("Unhandled BSP Event received: %d", evt);
            return;
    }

    if (!in_progress)
    {
        in_progress = ZB_TRUE;
        timestamp = ZB_TIMER_GET();

        zb_err_code = ZB_SCHEDULE_ALARM(leave_join_button_handler, button, LEAVE_JOIN_BUTTON_SHORT_POLL_TMO);
        ZB_ERROR_CHECK(zb_err_code);
    }
}

/**@brief Function for initializing LEDs.
 */
static zb_void_t leds_init(void)
{
    ret_code_t error_code;

     /* Initialize LEDs and buttons - use BSP to control them. */
    error_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, buttons_handler);
    APP_ERROR_CHECK(error_code);

    bsp_board_leds_off();
}



/**@brief Function for handling nrf app timer.
 * 
 * @param[IN]   context   Void pointer to context function is called with.
 * 
 * @details Function is called with pointer to sensor_device_ep_ctx_t as argument.
 */
static void zb_app_timer_handler(void * context)
{
   zb_zcl_status_t zcl_status;
    static zb_int16_t new_temp_value, new_humm_value, new_pres_value;
    static zb_int8_t new_voltage_value;
    
    /* Get data from bme280 */
    BME280_Get_Data( resultPTH );
    /* Get battery voltage                     */
    int16_t VBAT = GetBatteryVoltage1();
    NRF_LOG_INFO("Battery voltage %d.", VBAT);

    /* Get new temperature measured value */
     new_temp_value = (zb_int16_t)resultPTH[1];
    zcl_status = zb_zcl_set_attr_val(MULTI_SENSOR_ENDPOINT, 
                                     ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, 
                                     ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                     ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, 
                                     (zb_uint8_t *)&new_temp_value, 
                                     ZB_FALSE);
    if(zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Set temperature value fail. zcl_status: %d", zcl_status);
    }

    /* Get new humm measured value */
    new_humm_value = (zb_int16_t)(resultPTH[2] / 10);
    zcl_status = zb_zcl_set_attr_val(MULTI_SENSOR_ENDPOINT,
                                     ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, 
                                     ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                     ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, 
                                     (zb_uint8_t *)&new_humm_value, 
                                     ZB_FALSE);
    if(zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Set humm value fail. zcl_status: %d", zcl_status);
    }
    
    /* Get new pressure measured value */
    new_pres_value = (zb_int16_t)(resultPTH[0] / 100);
    zcl_status = zb_zcl_set_attr_val(MULTI_SENSOR_ENDPOINT,
                                     ZB_ZCL_CLUSTER_ID_PRESSURE_MEASUREMENT, 
                                     ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                     ZB_ZCL_ATTR_PRES_MEASUREMENT_VALUE_ID, 
                                     (zb_uint8_t *)&new_pres_value, 
                                     ZB_FALSE);
                                         if(zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Set pressure value fail. zcl_status: %d", zcl_status);
    }

    /* Get new voltage measured value */
    new_voltage_value =   (zb_int8_t)(VBAT / 100);
    zb_zcl_set_attr_val(MULTI_SENSOR_ENDPOINT,
                                     ZB_ZCL_CLUSTER_ID_POWER_CONFIG, 
                                     ZB_ZCL_CLUSTER_SERVER_ROLE, 
                                     ZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, 
                                     (zb_uint8_t *)&new_voltage_value, 
                                     ZB_FALSE);
    if(zcl_status != ZB_ZCL_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("Set voltage value fail. zcl_status: %d", zcl_status);
    }
}



/**@brief ZigBee stack event handler.
 *
 * @param[in]   param   Reference to ZigBee stack buffer used to pass arguments (signal).
 */
void zboss_signal_handler(zb_uint8_t param)
{
    zb_zdo_app_signal_hdr_t  * p_sg_p      = NULL;
    zb_zdo_app_signal_type_t   sig         = zb_get_app_signal(param, &p_sg_p);
    zb_ret_t                   status      = ZB_GET_APP_SIGNAL_STATUS(param);
    zb_bool_t                  comm_status;

    switch (sig)
    {
        case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (status == RET_OK)
            {
                NRF_LOG_INFO("Joined network successfully");
                bsp_board_led_on(ZIGBEE_NETWORK_STATE_LED);
                joined = ZB_TRUE;
                /* timeout for receiving data from sensor and voltage from battery */
                ret_code_t err_code = app_timer_start(zb_app_timer, APP_TIMER_TICKS(30000), NULL);
                APP_ERROR_CHECK(err_code);
                /* change data request timeout */
                zb_zdo_pim_set_long_poll_interval(60000);
            }
            else
            {
                    NRF_LOG_ERROR("Failed to join network. Status: %d", status);
                    joined = ZB_FALSE;
                    bsp_board_led_off(ZIGBEE_NETWORK_STATE_LED);
            }
            break;

        case ZB_ZDO_SIGNAL_LEAVE:
            if (status == RET_OK)
            {
                bsp_board_led_off(ZIGBEE_NETWORK_STATE_LED);
                joined = ZB_FALSE;
                ret_code_t err_code = app_timer_stop(zb_app_timer);
                APP_ERROR_CHECK(err_code);
                zb_zdo_signal_leave_params_t *p_leave_params = ZB_ZDO_SIGNAL_GET_PARAMS(p_sg_p, zb_zdo_signal_leave_params_t);
                NRF_LOG_INFO("Network left. Leave type: %d", p_leave_params->leave_type);
            }
            else
            {
                NRF_LOG_ERROR("Unable to leave network. Status: %d", status);
            }
            break;


        case ZB_COMMON_SIGNAL_CAN_SLEEP:
            zb_sleep_now();
            break;

        case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
            if (status != RET_OK)
            {
                NRF_LOG_WARNING("Production config is not present or invalid");
            }
            break;

        default:
            /* Unhandled signal. For more information see: zb_zdo_app_signal_type_e and zb_ret_e */
            NRF_LOG_INFO("Unhandled signal %d. Status: %d", sig, status);
            break;
    }

    if (param)
    {
        ZB_FREE_BUF_BY_REF(param);
    }
}

/**@brief Function for application main entry.
 */
int main(void)
{
    zb_ret_t       zb_err_code;
    ret_code_t     err_code;
    zb_ieee_addr_t ieee_addr;

    /* Initialize loging system and GPIOs. */
    timers_init();
    log_init();
    
    /* I2C Init */
    I2C_init();
    BME280_Turn_On();
    /* Create Timer for reporting attribute */
    err_code = app_timer_create(&zb_app_timer, APP_TIMER_MODE_REPEATED, zb_app_timer_handler);
    APP_ERROR_CHECK(err_code);

    /* Set ZigBee stack logging level and traffic dump subsystem. */
    ZB_SET_TRACE_LEVEL(ZIGBEE_TRACE_LEVEL);
    ZB_SET_TRACE_MASK(ZIGBEE_TRACE_MASK);
    ZB_SET_TRAF_DUMP_OFF();

    /* Initialize ZigBee stack. */
    ZB_INIT("multi_sensor");

    /* Set device address to the value read from FICR registers. */
    zb_osif_get_ieee_eui64(ieee_addr);
    zb_set_long_address(ieee_addr);

    /* Set static long IEEE address. */
    zb_set_network_ed_role(IEEE_CHANNEL_MASK);
    zigbee_erase_persistent_storage(ERASE_PERSISTENT_CONFIG);

    leds_init();

    zb_set_ed_timeout(ED_AGING_TIMEOUT_64MIN);
    zb_set_keepalive_timeout(ZB_MILLISECONDS_TO_BEACON_INTERVAL(60000));
    zb_set_rx_on_when_idle(ZB_FALSE);

    /* Initialize application context structure. */
    UNUSED_RETURN_VALUE(ZB_MEMSET(&m_dev_ctx, 0, sizeof(m_dev_ctx)));

    /* Register temperature sensor device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(&multi_sensor_ctx);

    /* Initialize sensor device attibutes */
    multi_sensor_clusters_attr_init();

    /** Start Zigbee Stack. */
    zb_err_code = zboss_start();
    ZB_ERROR_CHECK(zb_err_code);

    while(1)
    {
        zboss_main_loop_iteration();
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    }
}


/**
 * @}
 */
