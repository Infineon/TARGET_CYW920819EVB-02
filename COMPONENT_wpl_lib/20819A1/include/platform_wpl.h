/*
 * Copyright 2020, Cypress Semiconductor Corporation or a subsidiary of
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
#pragma once

#include <stdint.h>
#include <string.h>

#include "wiced_rtos.h"
#include "stdlib.h"
#include "wiced_platform.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_memory.h"
#include "wiced_rtos.h"
#include "wiced_result.h"
#include "wiced_power_logger.h"

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef PLATFORM_WPL_HEADER_INCLUDED
#error "Platform WPL header file must not be included directly, Please use wpl_platform_api.h instead."
#endif

typedef uint32_t  wiced_time_t;        /**< Time value in milliseconds */

/******************************************************
 *                      WPL-Macros
 ******************************************************/

#define WPL_DEFAULT_WORKER_PRIORITY 5
#define WPL_UART_STACK_SIZE      500
#define WPL_TIME_TO_WAIT_FOR_CONSOLE        0 /* msec */

/* Use this macro to save WPL specific AON variables.  */
#define PLACE_DATA_IN_AON_RAM    __attribute__ ((section(".data_in_retention_ram")))

#define WPL_DEEP_SLEEP_SAVED_VAR(var)      PLACE_DATA_IN_AON_RAM var
#define WPL_DEEP_SLEEP_IS_WARMBOOT() mcu_powersave_is_warmboot()
#define WPL_DEEP_SLEEP_IS_WARMBOOT_HANDLE() mcu_powersave_is_warmboot()

/* Print Macros */
#define WPRINT_APP_DEBUG( args )
#define WPRINT_APP_INFO( args ) WICED_BT_TRACE args
#define WPRINT_LIB_DEBUG( args )
#define WPRINT_LIB_INFO( args ) WICED_BT_TRACE args
#define WPRINT_LIB_ERROR( args ) WICED_BT_TRACE args
#define WPRINT_PLATFORM_ERROR( args ) WICED_BT_TRACE args
#define UNUSED_PARAMETER(x) (void) x
#define NEVER_TIMEOUT ((uint32_t) 0xFFFFFFFF)

#define malloc get_buffer_for_wpl
#define free wpl_free_buffer

#define WICED_END_OF_CURRENT_THREAD()
#define setvbuf(...)

/******************************************************
 *                      Macros
 ******************************************************/

#define WPL_WAKE_GPIO_NUM WICED_P12

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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/
wiced_bool_t mcu_powersave_is_warmboot( void );
void *get_buffer_for_wpl( uint32_t buffer_size );
void wpl_free_buffer( void* p_buf );

// Inform WPL core that mandatory Platform MACROs and data structures are defined
#define PLATFORM_WPL_MACROS_DEFINED


#ifdef __cplusplus
} /* extern "C" */
#endif
