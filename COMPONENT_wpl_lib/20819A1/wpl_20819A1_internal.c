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
#include <wiced_hal_gpio.h>
#include "brcm_fw_types.h"
#include "clock_timer.h"
#include "mcu_profile.h"
#include <wiced_bt_trace.h>

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

/******************************************************
 *               Variables Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/
void retrieve_bcs_profile_data(uint32_t* tx_value, uint32_t* rx_value);
void bcs_profilingReset(void);

void get_bt_profile_data(uint32_t* tx_value, uint32_t* rx_value)
{
    retrieve_bcs_profile_data(tx_value, rx_value);

 //   WICED_BT_TRACE("get_bt_profile_data: tx : %d  rx: %d\n", *tx_value, *rx_value);
    bcs_profilingReset();
}

void reset_bt_profile_data(void)
{
    bcs_profilingReset();
}

void set_mcu_profile_data(UINT8 set)
{
    mcu_profiling_set(set);
}

void get_mcu_profile_data(uint32_t* sleepTime, uint32_t* idleTime)
{
    MCU_PROFILING_DATA profileData;
    memset((void*)&profileData, 0, sizeof(MCU_PROFILING_DATA));
    mcu_profiling_retrieve_data(&profileData);
    *sleepTime = profileData.sleepTime;
    *idleTime = profileData.idleTime;
}
