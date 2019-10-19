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
 *
 */
#include "wiced.h"
#include "wiced_bt_dev.h"
#include "bt_types.h"
#include <wiced_sleep.h>
#include <wiced_hal_gpio.h>
#include "brcm_fw_types.h"
#include "clock_timer.h"
#include "wiced_hal_lpm.h"
#include "wiced_rtos.h"
#include "wiced_hal_gpio.h"
#include "wpl_platform_api.h"


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
wiced_result_t platform_uart_init( void );
BOOL32 pmu_sleep_isSleepEnabled();
wiced_result_t platform_uart_transmit_bytes( const uint8_t* data_out, uint32_t size );
wiced_result_t platform_uart_receive_bytes( uint8_t* data_in, uint32_t* expected_data_size, uint32_t timeout_ms );
wiced_result_t __real_wiced_sleep_configure(wiced_sleep_config_t *p_sleep_config );

// FW2 functions
BOOL32 wpl_register_for_low_power_queries(uint32_t (*fn)(wiced_bt_lpm_poll_type_t, uint32_t), UINT32 context);
void cype_sleep_handler_init(void);

/* Functions using FW2's hooks */
void get_bt_profile_data(uint32_t* tx_value, uint32_t* rx_value);
void reset_bt_profile_data(void);
void set_mcu_profile_data(UINT8 set);
void get_mcu_profile_data(uint32_t* sleepTime, uint32_t* idleTime);

/*  FW2 Functions */
wiced_result_t wiced_rtos_deinit_semaphore( wiced_semaphore_t* semaphore );
wiced_result_t wiced_rtos_deinit_mutex( wiced_mutex_t* mutex );

/******************************************************
 *               Variables Definitions
 ******************************************************/
static uint32_t prev_bt_profile_timestamp;

uint32_t WPL_DEEP_SLEEP_SAVED_VAR( deep_sleep_enter_timestamp );
uint16_t WPL_DEEP_SLEEP_SAVED_VAR( bt_duration_percentage_before_deep_sleep_poll );
static uint32_t time_spent_in_deepsleep;

wiced_sleep_config_t    wpl_sleep_config;

#define WPL_POOL_XXL_BUFFER_COUNT 2
#define WPL_POOL_XXL_BUFFER_SIZE 312

#define WPL_POOL_XL_BUFFER_COUNT 1
#define WPL_POOL_XL_BUFFER_SIZE 200

#define WPL_POOL_L_BUFFER_COUNT 1
#define WPL_POOL_L_BUFFER_SIZE 100

#define WPL_POOL_M_BUFFER_COUNT 2
#define WPL_POOL_M_BUFFER_SIZE 70

#define WPL_POOL_S_BUFFER_COUNT 6
#define WPL_POOL_S_BUFFER_SIZE 30

#define WPL_POOL_XS_BUFFER_COUNT 5
#define WPL_POOL_XS_BUFFER_SIZE 10

/* Pool for storing the wpl buffers */
wiced_bt_buffer_pool_t* wpl_m_buff_pool;
wiced_bt_buffer_pool_t* wpl_s_buff_pool;
wiced_bt_buffer_pool_t* wpl_xs_buff_pool;

uint8_t xl_buffer[WPL_POOL_XL_BUFFER_SIZE];
uint8_t l_buffer[WPL_POOL_L_BUFFER_SIZE];

/******************************************************
 *               Function Definitions
 ******************************************************/

/**
 * Gets time in milliseconds
 *
 * @Note: Platform specific, need to be ported according to the platform.
 *
 *
 * @returns Time in milliseconds
 */

uint32_t platform_wpl_get_time_stamp( void )
{
    return (uint32_t)( clock_SystemTimeMicroseconds64( ) / 1000 );
}

uint8_t platform_wpl_get_mcu_power_state_before_warm_boot( void )
{
    return EVENT_DESC_POWER_DEEPSLEEP;
}

uint8_t platform_wpl_get_first_mcu_power_state( void )
{
    return EVENT_DESC_POWER_ACTIVE1;
}

wiced_bool_t mcu_powersave_is_warmboot( void )
{
    return WICED_FALSE;
}

/* Retrieves the MCU and BT profiling information and updates in WPL data structures */
void bcs_profile_data_update( void )
{
     uint8_t status = 0xFF, *p;
     uint32_t curr_timestamp = 0;
     uint32_t profile_duration = 0;
     uint32_t tx_value, rx_value, bt_idle_value;
     uint32_t fw2_active_time = 0;
     uint32_t sleepTime, idleTime;
     uint32_t bt_time_during_deepsleep = 0;


    // Retrieve MCU Profile data
    get_mcu_profile_data(&sleepTime, &idleTime);

    /* Calculate profiling duration based on the stored timestamps */
    curr_timestamp = platform_wpl_get_time_stamp( );
    profile_duration = curr_timestamp - prev_bt_profile_timestamp;
 //   WICED_BT_TRACE("bcs_profile_data_update: duration:%d mcu idle:%d sleep:%d\n", profile_duration, idleTime, sleepTime);

    // Reset pre timestamp
    prev_bt_profile_timestamp = curr_timestamp;

    // Reset MCU data

    // Update Idle time
    WICED_POWER_LOGGER_DATA(EVENT_PROC_ID_MCU, EVENT_ID_POWERSTATE, EVENT_DESC_POWER_ACTIVE1, idleTime);
    // Get Active Time
    fw2_active_time = profile_duration - idleTime - sleepTime;

    WICED_POWER_LOGGER_DATA(EVENT_PROC_ID_MCU, EVENT_ID_POWERSTATE, EVENT_DESC_POWER_ACTIVE2, fw2_active_time);

    WICED_POWER_LOGGER_DATA(EVENT_PROC_ID_MCU, EVENT_ID_POWERSTATE, EVENT_DESC_POWER_PDS, sleepTime);
//    WPRINT_LIB_INFO(("FW2 Info: Duration:%d Sleep:%d Idle:%d , Active:%u CPU:%d  \n", (int)profile_duration, (int)sleepTime, (int)idleTime , fw2_active_time, (int)((fw2_active_time*100)/profile_duration)));

    // Update BT profiling
    get_bt_profile_data(&tx_value, &rx_value);

//    WICED_BT_TRACE("bcs_profile_data_update: tx:%d rx:%d\n", tx_value, rx_value);
    if(sleepTime )
    {
        WICED_POWER_LOGGER_DATA(EVENT_PROC_ID_BT, EVENT_ID_BT_DATA, EVENT_DESC_BT_POWER_TX_PDS, tx_value );

        WICED_POWER_LOGGER_DATA(EVENT_PROC_ID_BT, EVENT_ID_BT_DATA, EVENT_DESC_BT_POWER_RX_PDS, rx_value  );
    }
    else
    {
        WICED_POWER_LOGGER_DATA(EVENT_PROC_ID_BT, EVENT_ID_BT_DATA, EVENT_DESC_BT_POWER_TX, tx_value );
        WICED_POWER_LOGGER_DATA( EVENT_PROC_ID_BT, EVENT_ID_BT_DATA, EVENT_DESC_BT_POWER_RX, rx_value );
    }

    if ( time_spent_in_deepsleep )
    {
        /* Estimate the time spent in BT during deep sleep based on the time spent just before deep sleep */
        bt_time_during_deepsleep = ( bt_duration_percentage_before_deep_sleep_poll * time_spent_in_deepsleep ) / (100*1000);
        WICED_POWER_LOGGER_DATA( EVENT_PROC_ID_MCU, EVENT_ID_POWERSTATE, EVENT_DESC_POWER_DEEPSLEEP, time_spent_in_deepsleep );
        WICED_POWER_LOGGER_DATA( EVENT_PROC_ID_BT, EVENT_ID_BT_DATA, EVENT_DESC_BT_POWER_DEEP_SLEEP, bt_time_during_deepsleep );
    }
    else
    {
        WICED_POWER_LOGGER_DATA( EVENT_PROC_ID_MCU, EVENT_ID_POWERSTATE, EVENT_DESC_POWER_DEEPSLEEP, 0 );
        WICED_POWER_LOGGER_DATA( EVENT_PROC_ID_BT, EVENT_ID_BT_DATA, EVENT_DESC_BT_POWER_DEEP_SLEEP, 0 );
    }

    bt_idle_value = (profile_duration + time_spent_in_deepsleep) - (tx_value + rx_value + bt_time_during_deepsleep);

    /* For BT we are interested only in Tx and Rx, take rest as idle */
    WICED_POWER_LOGGER_DATA( EVENT_PROC_ID_BT, EVENT_ID_BT_DATA, EVENT_DESC_BT_POWER_IDLE, bt_idle_value );

    time_spent_in_deepsleep = 0;
    /* Keep track of the time spent in BT during the poll interval just before deep sleep.
     * We are dealing with very small time intervals, keep percentage*1000
     * */
    bt_duration_percentage_before_deep_sleep_poll = ( (tx_value + rx_value) * 100 * 1000) /  profile_duration;

}

void update_bt_power_data( void )
{
    wiced_bt_dev_status_t status;

    bcs_profile_data_update();
}

void platform_wpl_reset_power_data(cpl_procid_t processor_id)
{
    return;
}

void platform_wpl_update_power_data(cpl_procid_t processor_id)
{
    if(processor_id == EVENT_PROC_ID_BT)
        update_bt_power_data();
    return;
}


wiced_result_t handle_pre_sleep( void )
{
    /* During PDS mode, device will miss the log request messages from host. Give WPL a chance to send the
     * power log data to host.
     */
    if ( wpl_send_power_data_to_host( ) == WICED_SUCCESS )
    {
	return WICED_SUCCESS;
    }
    return WICED_ERROR;
}

/* Register lpm query method, to keep track of sleep in */
uint32_t lpm_queriable_method(wiced_bt_lpm_poll_type_t type, uint32_t context)
{
    uint32_t ret = WICED_BT_PMU_SLEEP_NOT_ALLOWED;

    switch( type )
    {
        case WICED_BT_LOW_POWER_MODE_POLL_TYPE_SLEEP:
            ret = WICED_SLEEP_MAX_TIME_TO_SLEEP;
            break;

        case WICED_BT_LOW_POWER_MODE_POLL_TYPE_SDS:
            if ( handle_pre_sleep() != WICED_SUCCESS )
                ret = WICED_BT_PMU_SLEEP_SDS_ALLOWED;
            break;
    }

    return ret;
}


/* Register dummy sleep handler. This is registered only when app has not configured the sleep.
 * This dummy handler always refuses sleep */
uint32_t dummy_sleep_handler_wpl(wiced_sleep_poll_type_t type )
{
    return WICED_SLEEP_NOT_ALLOWED;
}

wiced_result_t platform_wpl_sleep_register(void)
{
    wiced_result_t result;

    wpl_sleep_config.sleep_mode             = WICED_SLEEP_MODE_NO_TRANSPORT;

    wpl_sleep_config.device_wake_gpio_num   = WPL_WAKE_GPIO_NUM;
    wpl_sleep_config.device_wake_mode       = WICED_SLEEP_WAKE_ACTIVE_HIGH;
    wpl_sleep_config.device_wake_source     = WICED_SLEEP_WAKE_SOURCE_GPIO;
    wpl_sleep_config.host_wake_mode         = WICED_SLEEP_WAKE_ACTIVE_HIGH;
    wpl_sleep_config.sleep_permit_handler   = dummy_sleep_handler_wpl;
    wpl_sleep_config.post_sleep_cback_handler   = NULL;

    result = __real_wiced_sleep_configure( &wpl_sleep_config );
    WPRINT_LIB_DEBUG(("platform_wpl_sleep_register: result:%d\n", result));
    return result;
}

void platform_wpl_enable_logging( wiced_bool_t enable )
{
    wiced_bt_dev_status_t status;

    /* Reset BT profiling information and UART logging */
    if ( enable )
    {
        uint32_t sleepTime, idleTime;
        /* MCU profiling needs sleep to be configured */
        if(pmu_sleep_isSleepEnabled())
        {
            /* Application has already configured sleep. Register only for lpm queries and
             * pre shutdown sleep to keep track of the sleep */
            WPRINT_LIB_DEBUG(("CyPE: Register for lpm queries\n"));
            wpl_register_for_low_power_queries(lpm_queriable_method, 0);

        }
        else
        {
            /* Application has not registered the sleep. Configure sleep and register a dummy sleep handler */
            WPRINT_LIB_DEBUG(("CyPE: platform_wpl_enable_logging: Sleep NOT configured by app, configure in WPL\n"));
    //        bttransport_stop_polling_sense();
            platform_wpl_sleep_register();
        }

        /* Reset BT Profiling data in FW2 */
        reset_bt_profile_data();
        set_mcu_profile_data(WICED_TRUE);

        /* Retrieve the MCU profile to reset in FW2 */
        get_mcu_profile_data(&sleepTime, &idleTime);
        prev_bt_profile_timestamp = platform_wpl_get_time_stamp( );
        if ( !WPL_DEEP_SLEEP_IS_WARMBOOT( ) )
        {
            // Reset UART event
            WICED_POWER_LOGGER_DATA( EVENT_PROC_ID_MCU, EVENT_ID_UART, EVENT_DESC_UART_IDLE, 0 );
            // Start Logging
            WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART, EVENT_DESC_UART_IDLE );

#ifdef WPL_PROFILE_SPI_TRANSPORT
            WICED_POWER_LOGGER_DATA( EVENT_PROC_ID_MCU, EVENT_ID_SPI_TRANSPORT, EVENT_DESC_SPI_IDLE, 0 );
            WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_SPI_TRANSPORT, EVENT_DESC_SPI_IDLE );
#else
            WICED_POWER_LOGGER_DATA( EVENT_PROC_ID_MCU, EVENT_ID_UART_TRANSPORT, EVENT_DESC_UART_IDLE, 0 );
            WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART_TRANSPORT, EVENT_DESC_UART_IDLE );
#endif
        }
    }
    else
    {
        prev_bt_profile_timestamp = 0;
        set_mcu_profile_data(WICED_FALSE);
    }
}


/* Initializes sleep, UART */
wiced_result_t platform_wpl_init(void)
{

    if( WPL_DEEP_SLEEP_IS_WARMBOOT() )
    {
        // 20819 does not support deep sleep
    }

    // Creating buffer pools of multiple buffer sizes to optimize the memory
    wpl_m_buff_pool = (wiced_bt_buffer_pool_t*) wiced_bt_create_pool(WPL_POOL_M_BUFFER_SIZE, WPL_POOL_M_BUFFER_COUNT);
    if (wpl_m_buff_pool == NULL)
    {
        WPRINT_LIB_ERROR(("platform_wpl_init: wiced_bt_create_pool failed for M buffers\n"));
        return WICED_ERROR;
    }

    wpl_s_buff_pool = (wiced_bt_buffer_pool_t*) wiced_bt_create_pool(WPL_POOL_S_BUFFER_SIZE, WPL_POOL_S_BUFFER_COUNT);
    if (wpl_s_buff_pool == NULL)
    {
        WPRINT_LIB_ERROR(("platform_wpl_init: wiced_bt_create_pool failed for S buffers\n"));
        return WICED_ERROR;
    }

    wpl_xs_buff_pool = (wiced_bt_buffer_pool_t*) wiced_bt_create_pool(WPL_POOL_XS_BUFFER_SIZE, WPL_POOL_XS_BUFFER_COUNT);
    if (wpl_xs_buff_pool == NULL)
    {
        WPRINT_LIB_ERROR(("platform_wpl_init: wiced_bt_create_pool failed for XS buffers\n"));
        return WICED_ERROR;
    }

    if( platform_uart_init() != WICED_SUCCESS )
    {
        WPRINT_LIB_ERROR(("platform_wpl_init: Failed to initialize UART\n"));
        return WICED_ERROR;
    }

    // Register sleep handlers
    cype_sleep_handler_init();
    WPRINT_LIB_INFO(("CyPE - platform_wpl_init: done\n"));
    return WICED_SUCCESS;
}

void wpl_free_buffer( void* p_buf )
{
    if( (xl_buffer == p_buf) || (l_buffer == p_buf))
    {
       return;
    }
   wiced_bt_free_buffer(p_buf);
}

void* get_buffer_for_wpl( uint32_t buffer_size )
{
    void *p=0;

    if(!buffer_size)
        return NULL;

//    WPRINT_LIB_ERROR(("wpl_get_buffer: buffer allocation size: %d available memory: %d\n", buffer_size, wiced_memory_get_free_bytes()));

    /* Get the buffer from the pool based on the requested size */
    if( !p && buffer_size <= WPL_POOL_XS_BUFFER_SIZE)
    {
        p = wiced_bt_get_buffer_from_pool( wpl_xs_buff_pool ) ;
    }

    if( !p && buffer_size <= WPL_POOL_S_BUFFER_SIZE)
    {
        p = wiced_bt_get_buffer_from_pool( wpl_s_buff_pool ) ;
    }

    if( !p && buffer_size <= WPL_POOL_M_BUFFER_SIZE)
    {
        p = wiced_bt_get_buffer_from_pool( wpl_m_buff_pool ) ;
    }

    if( !p && buffer_size <= WPL_POOL_L_BUFFER_SIZE)
    {
        p = l_buffer;
    }

    if( !p && buffer_size <= WPL_POOL_XL_BUFFER_SIZE)
    {
        p = xl_buffer;
    }

    if(!p)
    {
        WPRINT_LIB_ERROR(("wpl_get_buffer: buffer allocation failed size: %d\n", buffer_size));
    }
    return p;
}

/* WPL RTOS related APIs */
wiced_result_t platform_wpl_rtos_init_semaphore( wpl_semaphore_t* semaphore )
{
    semaphore->semaphore = wiced_rtos_create_semaphore( );
    if ( semaphore->semaphore == NULL )
        return WICED_ERROR;
    return wiced_rtos_init_semaphore( semaphore->semaphore );
}

wiced_result_t platform_wpl_rtos_set_semaphore( wpl_semaphore_t* semaphore )
{
    return wiced_rtos_set_semaphore( semaphore->semaphore );
}

wiced_result_t platform_wpl_rtos_get_semaphore( wpl_semaphore_t* semaphore, uint32_t timeout_ms )
{
    wiced_result_t status;

    status = wiced_rtos_get_semaphore( semaphore->semaphore, timeout_ms );

    if ( status == TX_SUCCESS )
    {
        return WICED_SUCCESS;
    }
    else if ( status == TX_NO_INSTANCE )
    {
        return WICED_TIMEOUT;
    }
    else if ( status == TX_WAIT_ABORTED )
    {
        return WICED_ABORTED;
    }
    else
    {
        return WICED_ERROR;
    }
}

wiced_result_t platform_wpl_rtos_deinit_semaphore( wpl_semaphore_t* semaphore )
{
    return wiced_rtos_deinit_semaphore( semaphore->semaphore );
}

wiced_result_t platform_wpl_rtos_init_mutex( wpl_mutex_t* mutex )
{
    mutex->mutex = wiced_rtos_create_mutex( );
    if ( mutex->mutex == NULL )
        return WICED_ERROR;
    return wiced_rtos_init_mutex( mutex->mutex );
}

wiced_result_t platform_wpl_rtos_lock_mutex( wpl_mutex_t* mutex )
{
    return wiced_rtos_lock_mutex( mutex->mutex );
}

wiced_result_t platform_wpl_rtos_unlock_mutex( wpl_mutex_t* mutex )
{
    return wiced_rtos_unlock_mutex( mutex->mutex );
}

wiced_result_t platform_wpl_rtos_deinit_mutex( wpl_mutex_t* mutex )
{
    return wiced_rtos_deinit_mutex( mutex->mutex );
}

wiced_result_t platform_wpl_delay_milliseconds( uint32_t milliseconds )
{
    return wiced_rtos_delay_milliseconds(milliseconds, ALLOW_THREAD_TO_SLEEP);
}

wiced_result_t platform_wpl_rtos_create_thread( wpl_thread_t* thread, uint8_t priority, const char* name, wiced_thread_function_t function, uint32_t stack_size, void* arg )
{
    thread->thread = wiced_rtos_create_thread( );
    if ( thread->thread == NULL )
        return WICED_ERROR;
    /*2039 FW limits priority to 0(lowest) to 7(highest)*/
    if ( priority > 6 )
        priority = 1; /* priority 0 is reserved for IDLE thread in 20739 FW */
    else
        priority = (uint8_t) ( 7 - (uint32_t) priority );
    return wiced_rtos_init_thread( thread->thread, priority, name, function, stack_size, arg );
}

wiced_result_t platform_wpl_rtos_delete_thread( wpl_thread_t* thread )
{
    //return wiced_rtos_delete_thread( (wiced_thread_t*)thread->thread );
    return WICED_SUCCESS;
}

wiced_result_t platform_wpl_uart_receive_bytes( uint8_t* data_in, uint32_t* expected_data_size, uint32_t timeout_ms )
{
    return platform_uart_receive_bytes( data_in, expected_data_size, timeout_ms );
}

wiced_result_t platform_wpl_uart_transmit_bytes( const uint8_t* data_out, uint32_t size )
{
    return platform_uart_transmit_bytes( data_out, size );
}
