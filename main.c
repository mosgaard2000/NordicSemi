/**
 * Copyright (c) 2014 - 2018, Nordic Semiconductor ASA
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
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


#include <stdint.h>
#include <string.h>
//#include "nordic_common.h"
#include "nrf.h"
#include "app_timer.h"
#include "nrfx_rtc.h"
#include "nrfx_timer.h"
#include "nrf_drv_power.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "nrfx_common.h"
#include "nrf_sdh.h"
#include "app_usbd.h"

#include "nrfx_gpiote.h"
#include "boards.h"
#include "app_error.h"
#include "app_sdcard.h"
#include "nrf_pwr_mgmt.h"
#include "bsp.h"
#include "nrf_drv_gpiote.h"//51
#include "led_softblink.h"
#include "app_scheduler.h"

#include "pca10056_C3_B12.h"
#include "BLE_Driver.h"
#include "Accel_Driver.h"
#include "ADS1293_Driver.h"
#include "RTC_Driver.h"
#include "Data_Handling.h"
#include "SDcard_Driver.h"
#include "NFC_Driver.h"
#include "registers.h"
#include "Battery_Driver.h"
#include "USB_Driver.h"
#include "LED_Driver.h"
#include "StateMachine_Typedef.h"
#include "ADC_Driver.h"
#include "Button_Driver.h"
#include "Watchdog.h"
#include "UICR.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//#define BOOTLOADER_DFU_START 0xB1



bool HouseKeeping_Event = false;
bool Delay_Event = false;


uint8_t LineTestActive = 0x00;
uint8_t BootloaderActive = 0x00;



main_statemachine_event_t Main_SM_Event;     //Defined in StateMachine_Typedef.h


APP_TIMER_DEF(m_housekeeping_timer_id);      /**< Handler for repeated timer used for housekeeping. */
APP_TIMER_DEF(m_delay_timer_id);             /**< Handler for repeated timer used for our own low power delay. */



//Here all other initialization used by your project can be.



/** @brief Function starting the internal LFCLK XTAL oscillator.
 */
static void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}




/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{

#if NRF_DFU_IN_APP
    app_sched_execute();
#endif //DFU_IN_APP

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

#if NRF_DFU_IN_APP
#include "nrf_dfu_utils.h"
#include "nrf_bootloader_info.h"
#include "nrf_dfu_validation.h"
#include "nrf_dfu.h"
#include "app_scheduler.h"
#include "nrf_dfu_settings.h"
#include "nrf_dfu_serial.h"
#include "slip.h"
#include "nrf_dfu_serial_uart.h"

#define SCHED_QUEUE_SIZE      70//64                            /**< Maximum number of events in the scheduler queue. */
#define SCHED_EVENT_DATA_SIZE NRF_DFU_SCHED_EVENT_DATA_SIZE /**< Maximum app_scheduler event size. */

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
//I always return true... I have shut down everything before going to bootloader and reinitializes all on reset.
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_RESET:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
                //uint32_t err_code;
                //err_code = sd_softdevice_disable();
                //APP_ERROR_CHECK(err_code);
                //err_code = app_timer_stop_all();
                //APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);

/**@brief Function for initializing the event scheduler.
 */
static void scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Weak implemenation of nrf_dfu_init
 *
 * @note   This function will be overridden if nrf_dfu.c is
 *         compiled and linked with the project
 */
 #if (__LINT__ != 1)
__WEAK uint32_t nrf_dfu_init(nrf_dfu_observer_t observer)
{
    NRF_LOG_DEBUG("in weak nrf_dfu_init");
    return NRF_SUCCESS;
}
#endif

void m_user_observer(nrf_dfu_evt_type_t notification)
{
    switch(notification)
    {
         case NRF_DFU_EVT_DFU_INITIALIZED:
              NRF_LOG_INFO("NRF_DFU_EVT_DFU_INITIALIZED");
              break;
         case NRF_DFU_EVT_TRANSPORT_ACTIVATED: 
              NRF_LOG_INFO("NRF_DFU_EVT_TRANSPORT_ACTIVATED");
              break;
         case NRF_DFU_EVT_TRANSPORT_DEACTIVATED:
              NRF_LOG_INFO("NRF_DFU_EVT_TRANSPORT_DEACTIVATED");
              break;
         case NRF_DFU_EVT_DFU_STARTED:
              NRF_LOG_INFO("NRF_DFU_EVT_DFU_STARTED");
              /*Stop advertising so it doesn't interfere with DFU*/
              (void)sd_ble_gap_adv_stop(0);
              break;
         case NRF_DFU_EVT_OBJECT_RECEIVED:
              NRF_LOG_INFO("NRF_DFU_EVT_OBJECT_RECEIVED");
              break;
         case NRF_DFU_EVT_DFU_FAILED:
              NRF_LOG_INFO("NRF_DFU_EVT_DFU_FAILED");
              break;
         case NRF_DFU_EVT_DFU_COMPLETED:
              NRF_LOG_INFO("NRF_DFU_EVT_DFU_COMPLETED");
              /*Perform a reset to active new image*/
              nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_RESET);
              break;
         case NRF_DFU_EVT_DFU_ABORTED:
              NRF_LOG_INFO("NRF_DFU_EVT_DFU_ABORTED");
              break;
         default:
              break;
    }
} //<! Observer callback set by the user.

void dfu_init(void)
{
    uint32_t err_code;
    uint32_t bootloader_addr_actual   = BOOTLOADER_ADDRESS;
    uint32_t bootloader_addr_expected = BOOTLOADER_START_ADDR;

    /* Check if bootloader start address is consistent with UICR/MBR register contents. */
    if ((bootloader_addr_expected != bootloader_addr_actual) && (bootloader_addr_actual != 0xFFFFFFFF))
    {
        NRF_LOG_ERROR("Incorrect bootloader start address. Set BOOTLOADER_START_ADDR to 0x%x", BOOTLOADER_ADDRESS);
        NRF_LOG_FINAL_FLUSH();
        ASSERT(0);
    }
    else if (bootloader_addr_actual == 0xFFFFFFFF)
    {
        NRF_LOG_ERROR("Bootloader start address is not set");
        NRF_LOG_FINAL_FLUSH();
        ASSERT(0)
    }

    scheduler_init();

    nrf_dfu_settings_init(true);

    err_code = nrf_dfu_init(m_user_observer);
    APP_ERROR_CHECK(err_code);
}

#endif //NRF_DFU_IN_APP





/**@brief Application main function.
 */
int main(void)
{
	uint32_t Reset_Status = NRF_POWER->RESETREAS; //Read 'Reset Reason register'
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS;  //Reset 'Reset Reason register'

	bool SD_initialized = false;

	uint8_t DoLineTest = 0x00;
	uint8_t DoBootloader = 0x00;

	sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

    if(NRF_POWER->GPREGRET == 0x02)         //Bootloader
    {
		Main_SM_Event = MAIN_SM_HW_INIT;
        BootloaderActive = 0x01;
        NRF_POWER->GPREGRET2 = 0x00;
    }
    else									//Normal run mode
    {
		Main_SM_Event = MAIN_SM_HW_INIT;
		BootloaderActive = 0x00;
		NRF_POWER->GPREGRET2 = 0x00;
	}
	
	NRF_POWER->GPREGRET = 0x00;   //Reset state we come from
	
	while(1)
	{
		switch (Main_SM_Event)
		{
			case MAIN_SM_OFF:
			{
				break;
			}
			
			case MAIN_SM_HW_INIT:
			{
				//Initialization of the system - here I initializes the SPI3 to interface to a SD-card.
				//The code for the SD-card is not included in this project, but I uses FatFS to read a file from the SD-card.
				nrf_delay_ms(100);
				leds_config();
				nrf_delay_ms(20);
				lfclk_config();
				nrf_delay_ms(20);
				uint32_t err_code = app_timer_init(); 
				APP_ERROR_CHECK(err_code);
				nrf_delay_ms(20);
				Watchdog_Init();
				Watchdog_Start();
				nrf_delay_ms(20);
				Setup_SD_HW();	 //Initialize control signals (default everything is off)
				nrf_delay_ms(20);
				Config_SD_HW(SD_nRF_DISABLE, SD_DIR_nRF, SD_VDDM_DISABLE, SD_3V3);
				nrf_delay_ms(20);
				Init_SDcard_HW();	 //Initializes SPIM3 to be used with SD-card
				nrf_delay_ms(20);
				
				if(BootloaderActive == 0x01)
				{
					Main_SM_Event = MAIN_SM_BOOTLOADER;
				}
				else
				{
					Main_SM_Event = MAIN_SM_RUN;
				}
				
				break;
			}
			
			case MAIN_SM_RUN:
			{
				//Here the normal program was executed
				break;
			}
			
			case MAIN_SM_BOOTLOADER:
			{
				Update_LED_State(0x00, false, false, false);
				
				log_init();
				nrf_delay_ms(20);
				NRF_LOG_INFO("Entered MAIN_SM_BOOTLOADER.");
				NRF_LOG_PROCESS();
				
				//Enable 3.3V for SD-card (custom function that just sets some output pins to enable my SD-card)
				Config_SD_HW(SD_nRF_ENABLE, SD_DIR_nRF, SD_VDDM_ENABLE, SD_3V3);
				nrf_delay_ms(20);
				
				Init_SDcard_FatFS();        //Initializes the SD-card
				
				
				#if NRF_DFU_IN_APP
					dfu_init();
					NRF_LOG_INFO("Application version %d", s_dfu_settings.app_version);
				#endif //NRF_DFU_IN_APP
				
				nrf_delay_ms(1000);
				
				Watchdog_Reload();	//If we uses a watchdog we need to reload it
				
				uint8_t Start_Bootloader = Open_Upgrade_Image();	//Opens the image file - this is located on the SD-card, but could be a memory location or something else.
				nrf_delay_ms(20);
				
				if(Start_Bootloader == true)
				{
					bool done = false;
					uint8_t temp[1];
					
					while(done == false)					//Loop that reads entire file 1 byte at a time
					{
						Watchdog_Reload();					//If we uses a watchdog we need to reload it
						
						app_sched_execute();
						NRF_LOG_PROCESS();
						
						temp[0] = Read_Upgrade_Image();		//Reads 1 byte from file (implmented in SDcard_Driver.c)
						
						if(Check_Upgrade_EOF() == 0x00)		//Just a check to see if we reached end-of-file (if file is correct we will never reach EOF because the DFU triggers a reset before that happens).
							send_some_chars(temp, 1);		//Passes 1 byte to the UART DFU bootloader (implemented in nrf_dfu_serial_uart.c)
						else
							done = true;
						
						nrf_delay_us(10);
					}
				}
			
			break;
		}
		
		default:
		{
			break;
		}
	}
}


/**
 * @}
 */
