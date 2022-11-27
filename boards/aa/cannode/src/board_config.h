/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file board_config.h
 *
 * board internal definitions
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/* Boot config */
#define GPIO_BOOT_CONFIG      /* PB6 */ (GPIO_INPUT|GPIO_PORTB|GPIO_PIN6|GPIO_EXTI)

//#define GPIO_CAN1_SILENT_S0  /* PB5  */ (GPIO_OUTPUT|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN5)

/* LEDs */
//#define GPIO_LED_BLUE         /* PA4  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN4)

//#define GPIO_BTN_SAFETY              (GPIO_INPUT|GPIO_PORTC|GPIO_PIN13)

#define FLASH_BASED_PARAMS
#define CONFIG_I2C 1

#define GPIO_USART2_RX_GPIO     (GPIO_INPUT|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN3)
#define GPIO_USART2_TX_GPIO     (GPIO_OUTPUT|GPIO_SPEED_50MHz|GPIO_PORTA|GPIO_PIN2)

#define GPIO_USART3_RX_GPIO     (GPIO_INPUT|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN11)
#define GPIO_USART3_TX_GPIO     (GPIO_OUTPUT|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN10)


#define GPIO_UART4_RX_GPIO     (GPIO_INPUT|GPIO_SPEED_50MHz|GPIO_PORTC|GPIO_PIN11)
#define GPIO_UART4_TX_GPIO     (GPIO_OUTPUT|GPIO_SPEED_50MHz|GPIO_PORTC|GPIO_PIN10)

#define GPIO_UART5_RX_GPIO     (GPIO_INPUT|GPIO_SPEED_50MHz|GPIO_PORTD|GPIO_PIN2)
#define GPIO_UART5_TX_GPIO     (GPIO_OUTPUT|GPIO_SPEED_50MHz|GPIO_PORTC|GPIO_PIN12)

/* High-resolution timer */
#define HRT_TIMER                    3  /* use timer 3 for the HRT */
#define HRT_TIMER_CHANNEL            4  /* use capture/compare channel 4 */

#define PX4_GPIO_INIT_LIST { \
		GPIO_CAN1_TX,                     \
		GPIO_CAN1_RX,                     \
	}

__BEGIN_DECLS

#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */
/****************************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to configure USB IO.
 *
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

extern void stm32_usbinitialize(void);

//extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
