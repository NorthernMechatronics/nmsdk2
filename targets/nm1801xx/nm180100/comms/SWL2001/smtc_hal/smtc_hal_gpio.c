/*!
 * \file      smtc_hal_gpio.c
 *
 * \brief     GPIO Hardware Abstraction Layer implementation
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */
#include <stdint.h>   // C99 types
#include <stdbool.h>  // bool type

#include <am_mcu_apollo.h>
#include "smtc_hal_gpio.h"

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE MACROS-----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE CONSTANTS -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE TYPES -----------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE VARIABLES -------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PRIVATE FUNCTIONS DECLARATION -------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS DEFINITION ---------------------------------------------
 */

//
// MCU input pin Handling
//

void hal_gpio_init_in( const hal_gpio_pin_names_t pin, const hal_gpio_pull_mode_t pull_mode,
                       const hal_gpio_irq_mode_t irq_mode, hal_gpio_irq_t* irq )
{
    am_hal_gpio_pincfg_t pincfg = {0};


    pincfg.uFuncSel  = 3;
    pincfg.eGPInput  = AM_HAL_GPIO_PIN_INPUT_ENABLE;

    switch (irq_mode)
    {
    case BSP_GPIO_IRQ_MODE_RISING:
        pincfg.eIntDir = AM_HAL_GPIO_PIN_INTDIR_LO2HI;
        break;

    case BSP_GPIO_IRQ_MODE_FALLING:
        pincfg.eIntDir = AM_HAL_GPIO_PIN_INTDIR_HI2LO;
        break;

    case BSP_GPIO_IRQ_MODE_RISING_FALLING:
        pincfg.eIntDir = AM_HAL_GPIO_PIN_INTDIR_BOTH;
        break;

    case BSP_GPIO_IRQ_MODE_OFF:
    default:
        // pincfg.eIntDir = AM_HAL_GPIO_PIN_INTDIR_NONE;  //ll: always return zero on read (RDZERO).
        break;
    }

    am_hal_gpio_pinconfig(pin, pincfg);
}

void hal_gpio_init_out( const hal_gpio_pin_names_t pin, const uint32_t value )
{
    am_hal_gpio_pinconfig(pin, g_AM_HAL_GPIO_OUTPUT);
    am_hal_gpio_state_write(pin, value);
}

void hal_gpio_irq_attach( const hal_gpio_irq_t* irq )
{
    if (irq != NULL)
    {
        am_hal_gpio_interrupt_register_adv(irq->pin, irq->callback, irq->context);
        AM_HAL_GPIO_MASKCREATE(GpioPin);
        AM_HAL_GPIO_MASKBIT(pGpioPin, irq->pin);
        am_hal_gpio_interrupt_clear(pGpioPin);
        am_hal_gpio_interrupt_enable(pGpioPin);
        NVIC_EnableIRQ(GPIO_IRQn);
    }
}

void hal_gpio_irq_deatach( const hal_gpio_irq_t* irq )
{
    if (irq != NULL)
    {
        AM_HAL_GPIO_MASKCREATE(GpioPin);
        AM_HAL_GPIO_MASKBIT(pGpioPin, irq->pin);
        am_hal_gpio_interrupt_disable(pGpioPin);
        am_hal_gpio_interrupt_clear(pGpioPin);
        am_hal_gpio_interrupt_register(irq->pin, NULL);
    }
}

void hal_gpio_irq_enable( void )
{
    NVIC_EnableIRQ(GPIO_IRQn);
}

void hal_gpio_irq_disable( void )
{
    NVIC_DisableIRQ(GPIO_IRQn);
}

//
// MCU pin state control
//

void hal_gpio_set_value( const hal_gpio_pin_names_t pin, const uint32_t value )
{
    am_hal_gpio_state_write(pin, value ? AM_HAL_GPIO_OUTPUT_SET : AM_HAL_GPIO_OUTPUT_CLEAR);
}

uint32_t hal_gpio_get_value( const hal_gpio_pin_names_t pin )
{
    uint32_t state;
    am_hal_gpio_state_read(pin, AM_HAL_GPIO_INPUT_READ, &state);

    return state;
}


void hal_gpio_clear_pending_irq( const hal_gpio_pin_names_t pin )
{
    AM_HAL_GPIO_MASKCREATE(GpioPin);
    AM_HAL_GPIO_MASKBIT(pGpioPin, pin);
    am_hal_gpio_interrupt_clear(pGpioPin);
}

/* --- EOF ------------------------------------------------------------------ */
