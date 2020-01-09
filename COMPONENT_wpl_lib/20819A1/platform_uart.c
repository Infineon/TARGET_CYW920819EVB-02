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
 * Debug UART Tx/Rx platform APIs for WPL on CYW920719B1
 */
#include <stdint.h>
#include <string.h>
#include "wiced_rtos.h"
#include "wiced_bt_trace.h"
#include "wiced_hal_puart.h"
#include "wiced_power_logger.h"
#include "ring_buffer.h"
#include "wpl_platform_api.h"
/******************************************************
 *                      Macros
 ******************************************************/
#define PLAT_UART_RX_BUFSIZE ((uint32_t)64)
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
void __real_wiced_hal_puart_configuration(UINT32 baudrate, parity_t parity, stop_bit_t stop_bit);
void __real_wiced_set_debug_uart ( wiced_debug_uart_types_t uart );

/******************************************************
 *               Static Function Declarations
 ******************************************************/
/*no prototype in wiced_hal_puart.h*/
extern void wiced_hal_puart_register_interrupt(void (*puart_rx_cbk)(void*));
extern void wiced_hal_puart_reset_puart_interrupt(void);

/******************************************************
 *               Variable Definitions
 ******************************************************/
//static uint8_t uart_rdata;
static wiced_bool_t puart_init=WICED_FALSE;

static wpl_semaphore_t uart_r;
static wpl_semaphore_t uart_t;
static wiced_ring_buffer_t *rx_ring_buf = NULL;

extern UINT32 wpl_baudrate;
extern parity_t wpl_parity;
extern stop_bit_t wpl_stop_bit;
/******************************************************
 *               Function Definitions
 ******************************************************/
/*RX callback handler
 * No user argument in the callback, it will be NULL */
void platfrom_puart_rx_intr_callback(void *arg)
{
    UNUSED_PARAMETER(arg);
    uint8_t uart_rdata;

    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART, EVENT_DESC_UART_RX );
    if(wiced_hal_puart_read(&uart_rdata) && rx_ring_buf)
    {
        ring_buffer_write( rx_ring_buf, &uart_rdata, sizeof(uart_rdata));
        /*if ring buf was empty, set semaphore, reader could be waiting for data*/
        if(ring_buffer_used_space(rx_ring_buf)==1)
          platform_wpl_rtos_set_semaphore(&uart_r);
    }
    wiced_hal_puart_reset_puart_interrupt( );
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART, EVENT_DESC_UART_IDLE );
}

wiced_result_t platform_uart_init( void )
{

    wiced_result_t result = WICED_SUCCESS;
    uint8_t *rx_buf;

    if(puart_init == WICED_TRUE)
        return WICED_SUCCESS;

    platform_wpl_rtos_init_semaphore( &uart_r );

    platform_wpl_rtos_init_semaphore( &uart_t );


    /*we allocate ring buffer*/
    rx_ring_buf = (wiced_ring_buffer_t*)malloc(sizeof(wiced_ring_buffer_t));
    if(rx_ring_buf == NULL)
    {
        WPRINT_LIB_ERROR(("platform_uart_init++ rx_ring_buf failed\n"));
        return WICED_ERROR;
    }
    rx_buf = (uint8_t*)malloc(PLAT_UART_RX_BUFSIZE);
    if(rx_buf == NULL)
    {
        WPRINT_LIB_ERROR(("platform_uart_init++ rx_buf failed\n"));
        free(rx_ring_buf);
        return WICED_ERROR;
    }

    if (WICED_SUCCESS != ring_buffer_init(rx_ring_buf, rx_buf, PLAT_UART_RX_BUFSIZE))
    {
        free(rx_buf);
        free(rx_ring_buf);
        WPRINT_LIB_ERROR(("platform_uart_init++ ring_buffer_init failed\n"));
        return WICED_ERROR;
    }

    __real_wiced_set_debug_uart( WICED_ROUTE_DEBUG_TO_PUART );
    wiced_hal_puart_init();
    wiced_hal_puart_flow_off();

    if(wpl_baudrate)
         __real_wiced_hal_puart_configuration(wpl_baudrate, wpl_parity, wpl_stop_bit);
     else
         __real_wiced_hal_puart_configuration(115200, PARITY_NONE, STOP_BIT_1);

    wiced_hal_puart_register_interrupt(platfrom_puart_rx_intr_callback);
    /* set water mark level to 1 to receive interrupt up on receiving each byte */
    wiced_hal_puart_set_watermark_level(1);
    wiced_hal_puart_enable_tx();
    wiced_hal_puart_enable_rx();

    puart_init = WICED_TRUE;
    // Set semaphore for Tx
    platform_wpl_rtos_set_semaphore( &uart_t );
    return result;
}

wiced_result_t platform_uart_deinit(  )
{
    wiced_result_t result;
    puart_init = WICED_FALSE;
    wiced_hal_puart_disable_tx();
    platform_wpl_rtos_deinit_semaphore(&uart_r);
    platform_wpl_rtos_deinit_semaphore(&uart_t);

    if(rx_ring_buf->buffer)
        free(rx_ring_buf->buffer);
    free(rx_ring_buf);
    rx_ring_buf = NULL;
    return WICED_SUCCESS;
}

wiced_result_t platform_uart_transmit_bytes( const uint8_t* data_out, uint32_t size )
{
    wiced_result_t result;


    result = platform_wpl_rtos_get_semaphore( &uart_t, NEVER_TIMEOUT );
    if ( result != WICED_SUCCESS )
    {
        WPRINT_PLATFORM_ERROR( ("platform_uart_transmit_bytes: Not able to transmit on UART, error: %d\n", result) );
        return WICED_ERROR;
    }
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART, EVENT_DESC_UART_TX );
    while(size)
    {
        wiced_hal_puart_write(*data_out);
        data_out++;
        size--;
    }
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART, EVENT_DESC_UART_IDLE );
    platform_wpl_rtos_set_semaphore(&uart_t);
    return WICED_SUCCESS;
}

wiced_result_t platform_uart_receive_bytes( uint8_t* data_in, uint32_t* expected_data_size, uint32_t timeout_ms )
{
    uint32_t    bytes_left   = 0;
    wiced_result_t result = WICED_SUCCESS;

    bytes_left = *expected_data_size;

    if( rx_ring_buf == NULL )
    {
        result = WICED_ERROR;
    }
    else
    {
        while ( 0 != bytes_left )
        {
            uint32_t bytes_available = 0;
            uint8_t* available_data = NULL;
            /* Get available bytes and pointer to available data in the ring buffer */
            ring_buffer_get_data( rx_ring_buf, &available_data, &bytes_available );

            bytes_available = MIN( bytes_available, bytes_left );

            /*if no rx data, wait for data until timeout*/
            if ( bytes_available == 0 )
            {
                result = platform_wpl_rtos_get_semaphore( &uart_r, timeout_ms );
                if ( WICED_SUCCESS != result )
                {
                    rx_ring_buf->head = 0;
                    rx_ring_buf->tail = 0;
                    break;
                }
                continue;
            }
            memcpy( data_in, available_data, bytes_available );
            data_in       += bytes_available;
            bytes_left    -= bytes_available;
            ring_buffer_consume( rx_ring_buf, bytes_available );
        }
    }

    /* Return actual bytes read */
    *expected_data_size -= bytes_left;
    return result;
}

void bt_trace_serialize_lock(void)
{
    if(!puart_init)
        return;
    platform_wpl_rtos_get_semaphore( &uart_t, NEVER_TIMEOUT );
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART, EVENT_DESC_UART_TX );
}

void bt_trace_serialize_unlock(void)
{
    if(!puart_init)
        return;
    WICED_POWER_LOGGER( EVENT_PROC_ID_MCU, EVENT_ID_UART, EVENT_DESC_UART_IDLE );
    platform_wpl_rtos_set_semaphore(&uart_t);
}
