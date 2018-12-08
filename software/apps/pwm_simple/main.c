/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
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
 * @defgroup pwm_example_main main.c
 * @{
 * @ingroup pwm_example
 *
 * @brief PWM Example Application main file.
 *
 * This file contains the source code for a sample application using PWM.
 */

#include <stdio.h>
#include <string.h>
#include "nrf_drv_pwm.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "boards.h"
#include "bsp.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "buckler.h"


#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "app_pwm.h"



APP_PWM_INSTANCE(PWM1,0);                   // Create the instance "PWM1" using TIMER1.
void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    //printf("READY!\n");
}

#define OUTPUT_PIN BUCKLER_LED0

static nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);

// Declare variables holding PWM sequence values. In this example only one channel is used 
nrf_pwm_values_individual_t seq_values[] = {0, 0, 0, 0};
nrf_pwm_sequence_t const seq =
{
    .values.p_individual = seq_values,
    .length          = NRF_PWM_VALUES_LENGTH(seq_values),
    .repeats         = 0,
    .end_delay       = 0
};


// Set duty cycle between 0 and 100%
void pwm_update_duty_cycle(uint8_t duty_cycle)
{
    
    // Check if value is outside of range. If so, set to 100%
    if(duty_cycle >= 100)
    {
        seq_values->channel_0 = 100;
    }
    else
    {
        seq_values->channel_0 = duty_cycle;
    }
    
    nrf_drv_pwm_simple_playback(&m_pwm0, &seq, 1, NRF_DRV_PWM_FLAG_LOOP);
}

static void pwm_init(void)
{
    nrf_drv_pwm_config_t const config0 =
    {
        .output_pins =
        {
            OUTPUT_PIN, // channel 0
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 1
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 2
            NRF_DRV_PWM_PIN_NOT_USED,             // channel 3
        },
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .base_clock   = NRF_PWM_CLK_1MHz,
        .count_mode   = NRF_PWM_MODE_UP,
        .top_value    = 100,
        .load_mode    = NRF_PWM_LOAD_INDIVIDUAL,
        .step_mode    = NRF_PWM_STEP_AUTO
    };
    // Init PWM without error handler
    APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config0, NULL));
    
}


int main(void)
{

    // Start clock for accurate frequencies
    /*NRF_CLOCK->TASKS_HFCLKSTART = 1; 
    // Wait for clock to start
    while(NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) 
        ;
    
    pwm_init();

    for (;;)
    {
        for(int i = 0; i <= 100; i++)
        {
            nrf_delay_ms(10);
            pwm_update_duty_cycle(i);
        }
    }*/
    ret_code_t err_code;
    /* 2-channel PWM, 200 Hz, output on DK LED pins. */
    app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_1CH(10000L, BUCKLER_GROVE_D0);
    nrf_gpio_pin_dir_set(BUCKLER_LED0, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(BUCKLER_LED1, NRF_GPIO_PIN_DIR_OUTPUT);
    nrf_gpio_pin_dir_set(BUCKLER_GROVE_D0, NRF_GPIO_PIN_DIR_OUTPUT);
    /* Switch the polarity of the second channel. */
    pwm1_cfg.pin_polarity[0] = APP_PWM_POLARITY_ACTIVE_HIGH;
    /* Initialize and enable PWM. */
    err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
    APP_ERROR_CHECK(err_code);
    app_pwm_enable(&PWM1);
            
            /* Se the duty cycle - keep trying until PWM is ready. */
    while(true)
        while (app_pwm_channel_duty_set(&PWM1, 0, 15) == NRF_ERROR_BUSY);

}


/** @} */
