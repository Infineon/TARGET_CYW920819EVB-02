/*
 * Copyright 2019, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 * This file contains the re-definitions of the FW2 functions that are to be tapped for WPL.
 * The redefinition is wrapper with the WPL tap function called.
 */

#include "wiced.h"
#include "wiced_result.h"
#include "wiced_bt_stack.h"
#include "wiced_power_logger.h"
#include "wpl_platform_api.h"
#include "wiced_transport.h"
#include "wiced_sleep.h"
#include "wiced_timer.h"
#include "wiced_hal_puart.h"

#define CYPE_SLEEP_DELAY_TIMEOUT_IN_SECONDS    10  /* Seconds to delay the sleep so that CyPE tool can connect */

/* Prototype of FW2 functions used here */
int wiced_va_printf(char * buffer, int len, va_list va);

/* Functions to serialize BT Traces */
void bt_trace_serialize_lock(void);
void bt_trace_serialize_unlock(void);

/******************************************************
 *               Variables Definitions
 ******************************************************/
/* WPL CFG settings overwrite the app's settings */
wiced_bt_cfg_settings_t     wpl_bt_cfg_settings;

UINT32 wpl_baudrate;
parity_t wpl_parity;
stop_bit_t wpl_stop_bit;

wiced_timer_t cype_sleep_delay_timer;
uint8_t       cype_sleep_allowed;
wiced_sleep_allow_check_callback         app_low_power_sleep_handler;
void __real_wiced_hal_i2c_init(void);
uint8_t __real_wiced_hal_i2c_write(uint8_t* data, uint16_t length, uint8_t slave);
uint8_t __real_wiced_hal_i2c_combined_read(uint8_t* rx_data, uint8_t rx_data_len, uint8_t* tx_data, uint16_t tx_data_len, uint8_t slave);
wiced_result_t __real_wiced_sleep_configure(wiced_sleep_config_t *p_sleep_config );
void __real_wiced_hal_puart_configuration(UINT32 baudrate, parity_t parity, stop_bit_t stop_bit);
UINT8 __real_wiced_hal_i2c_read_rom(UINT8* data, UINT16 length, UINT8 slave);
wiced_result_t __real_wiced_bt_stack_init(wiced_bt_management_cback_t *p_bt_management_cback,const wiced_bt_cfg_settings_t *p_bt_cfg_settings,const wiced_bt_cfg_buf_pool_t wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS]);
wiced_result_t __real_wiced_transport_init( const wiced_transport_cfg_t* p_cfg );
wiced_result_t __real_wiced_transport_send_data ( uint16_t code, uint8_t* p_data, uint16_t length );
wiced_result_t __real_wiced_transport_send_buffer( uint16_t code, uint8_t* p_buf, uint16_t length );

/******************************************************
 *               Function Definitions
 ******************************************************/

void __wrap_wiced_hal_i2c_init(void)
{
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_I2C, EVENT_DESC_I2C_IDLE );
    __real_wiced_hal_i2c_init();
}

UINT8 __wrap_wiced_hal_i2c_read(UINT8* data, UINT16 length, UINT8 slave)
{
    UINT8 result;
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_I2C, EVENT_DESC_I2C_RX );
    result = __real_wiced_hal_i2c_read_rom(data, length, slave);
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_I2C, EVENT_DESC_I2C_IDLE );
    return result;
}

UINT8 __wrap_wiced_hal_i2c_write(UINT8* data, UINT16 length, UINT8 slave)
{
    UINT8 result;
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_I2C, EVENT_DESC_I2C_TX );
    result = __real_wiced_hal_i2c_write(data, length, slave);
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_I2C, EVENT_DESC_I2C_IDLE );
    return result;
}


UINT8 __wrap_wiced_hal_i2c_combined_read(UINT8* tx_data, UINT8 tx_data_len, UINT8* rx_data, UINT16 rx_data_len, UINT8 slave)
{
    UINT8 result;
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_I2C, EVENT_DESC_I2C_RX );
    result = __real_wiced_hal_i2c_combined_read(tx_data, tx_data_len, rx_data, rx_data_len,  slave);
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_I2C, EVENT_DESC_I2C_IDLE );
    return result;
}


/* WPL needs PUART as debug uart to receive the tapped information.
 * When WPL is enabled, do not allow to change debug uart
 */
void __wrap_wiced_set_debug_uart ( wiced_debug_uart_types_t uart )
{
    return;
}

void cype_sleep_delay_timeout( uint32_t count )
{
	cype_sleep_allowed = 1;
	wiced_stop_timer(&cype_sleep_delay_timer);
}

uint32_t cype_low_power_sleep_handler(wiced_sleep_poll_type_t type )
{
	uint32_t ret;
	ret = app_low_power_sleep_handler(type);
	if( !cype_sleep_allowed )
	{
		return WICED_SLEEP_NOT_ALLOWED;
	}
	return ret;
}

/* It is wrapped to delay the sleep when CyPE is enabled. It avoids changes in the app */
wiced_result_t __wrap_wiced_sleep_configure(wiced_sleep_config_t *p_sleep_config )
{
	if(p_sleep_config->sleep_permit_handler)
	{
		app_low_power_sleep_handler = p_sleep_config->sleep_permit_handler;
		p_sleep_config->sleep_permit_handler = cype_low_power_sleep_handler;
	}
    WPRINT_LIB_INFO(("CyPE: Delay sleep for CyPE host to connect\n"));
	return __real_wiced_sleep_configure(p_sleep_config);
}

/* WPL module can be started after BT stack is initialized. So this function needs to
 * be wrapped to avoid modification in the app to start WPL
 *
 */
wiced_result_t __wrap_wiced_bt_stack_init(wiced_bt_management_cback_t *p_bt_management_cback,
                                    const wiced_bt_cfg_settings_t     *p_bt_cfg_settings,
                                    const wiced_bt_cfg_buf_pool_t     wiced_bt_cfg_buf_pools[WICED_BT_CFG_NUM_BUF_POOLS])
{
    UINT8 result;

    /* app may use const type for wiced_bt_cfg_settings, so we make a local copy*/
    memcpy(&wpl_bt_cfg_settings, p_bt_cfg_settings,sizeof(wiced_bt_cfg_settings_t));
    /*we need extra for wpl buffer*/
    wpl_bt_cfg_settings.max_number_of_buffer_pools += 5;

    result = __real_wiced_bt_stack_init(p_bt_management_cback, (const wiced_bt_cfg_settings_t*)&wpl_bt_cfg_settings, wiced_bt_cfg_buf_pools);

    if( result == WICED_SUCCESS )
        wpl_start();

    /* Starting the app timers , seconds timer and the ms timer  */
    if (wiced_init_timer(&cype_sleep_delay_timer, cype_sleep_delay_timeout, 0, WICED_SECONDS_TIMER) == WICED_SUCCESS)
    {
        wiced_start_timer( &cype_sleep_delay_timer, CYPE_SLEEP_DELAY_TIMEOUT_IN_SECONDS );
    }

    return result;
}


wiced_result_t __wrap_wiced_transport_init( const wiced_transport_cfg_t* p_cfg )
{
    wiced_result_t result;

    result = __real_wiced_transport_init(p_cfg);
    if(result == WICED_SUCCESS)
    {
#ifdef WPL_PROFILE_SPI_TRANSPORT
        WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_SPI_TRANSPORT, EVENT_DESC_SPI_IDLE );
#else
        WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART_TRANSPORT, EVENT_DESC_UART_IDLE );
#endif
    }
    return result;
}

wiced_result_t __wrap_wiced_transport_send_data ( uint16_t code, uint8_t* p_data, uint16_t length )
{
    wiced_result_t result;
#ifdef WPL_PROFILE_SPI_TRANSPORT
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_SPI_TRANSPORT, EVENT_DESC_SPI_WRITE );
#else
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART_TRANSPORT, EVENT_DESC_UART_TX );
#endif
    result = __real_wiced_transport_send_data(code, p_data, length);

#ifdef WPL_PROFILE_SPI_TRANSPORT
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_SPI_TRANSPORT, EVENT_DESC_SPI_IDLE );
#else
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART_TRANSPORT, EVENT_DESC_UART_IDLE );
#endif
    return result;
}

wiced_result_t __wrap_wiced_transport_send_buffer( uint16_t code, uint8_t* p_buf, uint16_t length )
{
	wiced_result_t result;
#ifdef WPL_PROFILE_SPI_TRANSPORT
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_SPI_TRANSPORT, EVENT_DESC_SPI_WRITE );
#else
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART_TRANSPORT, EVENT_DESC_UART_TX );
#endif
    result = __real_wiced_transport_send_buffer(code, p_buf, length);

#ifdef WPL_PROFILE_SPI_TRANSPORT
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_SPI_TRANSPORT, EVENT_DESC_SPI_IDLE );
#else
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART_TRANSPORT, EVENT_DESC_UART_IDLE );
#endif
    return result;
}


/* Redefine wiced_printf to serialize the tx on PUART. This is not wrapper, it replaces ROM's wiced_printf */
int __wrap_wiced_printf(char * buffer, int len, ...)
{
    int used = 0;
    va_list va;
    bt_trace_serialize_lock();
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART, EVENT_DESC_UART_TX );
    va_start(va,len);
    used = wiced_va_printf(buffer, len, va);
    va_end(va);
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART, EVENT_DESC_UART_IDLE );
    bt_trace_serialize_unlock();
    return used;
}

/* Redefine to capture the baudrate set by application */
void __wrap_wiced_hal_puart_configuration(UINT32 baudrate, parity_t parity, stop_bit_t stop_bit)
{
    wpl_baudrate = baudrate;
    wpl_parity = parity;
    wpl_stop_bit = stop_bit;
    __real_wiced_hal_puart_configuration(baudrate, parity, stop_bit);
}
