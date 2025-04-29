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
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTAGPIO_CNF_OUTOD BILITY AND
 *FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
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

#include <nuttx/compiler.h>
#include <px4_platform_common/px4_config.h>
#include <stdint.h>

/* Boot config */
#define GPIO_BOOT_CONFIG /* PB6 */                                             \
	(GPIO_INPUT | GPIO_PORTB | GPIO_PIN6 | GPIO_EXTI)

/* LEDs */
#define GPIO_nLED_BLUE /* PA10  */                                             \
	(GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_2MHz | GPIO_OUTPUT_SET |           \
	 GPIO_PORTA | GPIO_PIN10 | GPIO_OUTPUT_CLEAR)
#define GPIO_nLED_GREEN /* PA9  */                                             \
	(GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_2MHz | GPIO_OUTPUT_SET |           \
	 GPIO_PORTA | GPIO_PIN9 | GPIO_OUTPUT_CLEAR)
#define GPIO_nLED_RED /* PA8  */                                               \
	(GPIO_OUTPUT | GPIO_CNF_OUTPP | GPIO_MODE_2MHz | GPIO_OUTPUT_SET |           \
	 GPIO_PORTA | GPIO_PIN8 | GPIO_OUTPUT_CLEAR)

#define FLASH_BASED_PARAMS
#define CONFIG_I2C 1

#define GPIO_USART2_RX_GPIO                                                    \
	(GPIO_INPUT | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN3)
#define GPIO_USART2_TX_GPIO                                                    \
	(GPIO_OUTPUT | GPIO_SPEED_50MHz | GPIO_PORTA | GPIO_PIN2)

#define GPIO_USART3_RX_GPIO                                                    \
	(GPIO_INPUT |GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN11)
#define GPIO_USART3_TX_GPIO                                                    \
	(GPIO_OUTPUT|GPIO_SPEED_50MHz|GPIO_PORTB|GPIO_PIN10)

#define GPIO_UART4_RX_GPIO                                                     \
	(GPIO_INPUT | GPIO_SPEED_50MHz | GPIO_PORTC | GPIO_PIN11)
#define GPIO_UART4_TX_GPIO                                                     \
	(GPIO_OUTPUT | GPIO_SPEED_50MHz | GPIO_PORTC | GPIO_PIN10)

/* High-resolution timer */
#define HRT_TIMER 3         /* use timer 3 for the HRT */
#define HRT_TIMER_CHANNEL 4 /* use capture/compare channel 4 */

#define BOARD_HAS_CONTROL_STATUS_LEDS 1

#define PX4_GPIO_INIT_LIST                                                     \
	{ GPIO_CAN1_TX, GPIO_CAN1_RX, }

__BEGIN_DECLS

#ifndef __ASSEMBLY__

extern void stm32_spiinitialize(void);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
