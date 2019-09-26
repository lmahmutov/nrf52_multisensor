
#include "I2C.h"
#include "nrf_drv_twi.h"

// I2C instance.
#define TWI_INSTANCE_ID 1

static const nrf_drv_twi_t i2c = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

// Indicates if operation on TWI has ended.
static volatile bool m_xfer_done = false;


// @brief UART initialization.
void I2C_init(void)
{        
    ret_code_t err_code;

    const nrf_drv_twi_config_t i2c_config = 
    {
       .scl                = 26,
       .sda                = 27,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };
		
    //last one is some kind of context - no idea what that is....
    //documentation is vague/nonexistant
    err_code = nrf_drv_twi_init(&i2c, &i2c_config, I2C_handler, NULL);
    APP_ERROR_CHECK(err_code);
    nrf_drv_twi_enable(&i2c);
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    uint8_t temp[2];
    
    temp[0] = subAddress;
    temp[1] = data;
    
    ret_code_t err_code;
    m_xfer_done = false;
    err_code = nrf_drv_twi_tx(&i2c, address, &temp[0], 2, true);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false); //wait until end of transfer
}

 
uint8_t readByte(uint8_t address, uint8_t subAddress)
{
    ret_code_t err_code = 0;
    
    uint8_t value;
    
    m_xfer_done = false;
 
    err_code = nrf_drv_twi_tx(&i2c, address, &subAddress, 1, true);
    
    APP_ERROR_CHECK(err_code);
    
    while (m_xfer_done == false); //wait until end of transfer
    
    if (err_code == NRF_SUCCESS)
    {
        m_xfer_done = false;
       
        err_code = nrf_drv_twi_rx(&i2c, address, &value, 1);
        APP_ERROR_CHECK(err_code);
        
        while (m_xfer_done == false);
    };

    return value;
}

// @brief TWI events handler.
void I2C_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            
            //todo -difference between read and write???
            m_xfer_done = true;
            
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
            }
            
            break;
        default:
            break;
    }
}


void readBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t n_bytes )
{
    ret_code_t err_code = 0;
    
    m_xfer_done = false;
    
    err_code = nrf_drv_twi_tx(&i2c, address, &subAddress, 1, true);
    
    while (m_xfer_done == false) {};
    
    if (err_code == NRF_SUCCESS)
    {
        m_xfer_done = false;
        err_code = nrf_drv_twi_rx(&i2c, address, dest, n_bytes);

        while (m_xfer_done == false) {};
    };

}

