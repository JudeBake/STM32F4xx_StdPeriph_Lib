/*
 * SerialPinAssignment.h
 *
 *  Created on: Apr 28, 2013
 *      Author: julien
 */

#ifndef SERIALPINASSIGNMENT_H_
#define SERIALPINASSIGNMENT_H_

#include "stm32f4xx.h"

//USART pin assignment for STM32F407 LPQF100
#ifdef __STM32F407_100__

//USART port 1
#define USART1_GPIO_PORT			GPIOA
#define USART1_RX					GPIO_Pin_10
#define USART1_RX_SOURCE			GPIO_PinSource10
#define USART1_TX					GPIO_Pin_9
#define USART1_TX_SOURCE			GPIO_PinSource9
#define USART1_CLK					GPIO_Pin_8
#define USART1_CLK_SOURCE			GPIO_PinSource8
#define USART1_RTS					GPIO_Pin_12
#define USART1_RTS_SOURCE			GPIO_PinSource12
#define USART1_CTS					GPIO_Pin_11
#define USART1_CTS_SOURCE			GPIO_PinSource11

//USART port 2
#define USART2_GPIO_PORT			GPIO
#define USART2_RX
#define USART2_RX_SOURCE
#define USART2_TX
#define USART2_TX_SOURCE
#define USART2_CLK
#define USART2_CLK_SOURCE
#define USART2_RST
#define USART2_RST_SOURCE
#define USART2_CTS
#define USART2_CST_SOURCE

//USART port 3
#define USART3_GPIO_PORT			GPIOB
#define USART3_RX					GPIO_Pin_11
#define USART3_RX_SOURCE			GPIO_PinSource11
#define USART3_TX					GPIO_Pin_10
#define USART3_TX_SOURCE			GPIO_PinSource10
#define USART3_CLK					GPIO_Pin_12
#define USART3_CLK_SOURCE			GPIO_PinSource12
#define USART3_RST					GPIO_Pin_14
#define USART3_RST_SOURCE			GPIO_PinSource14
#define USART3_CTS					GPIO_Pin_13
#define USART3_CST_SOURCE			GPIO_PinSource13

//UART port 4
#define UART4_GPIO_PORT				GPIO
#define UART4_RX
#define UART4_RX_SOURCE
#define UART4_TX
#define UART4_TX_SOURCE

//UART port 5
#define UART5_GPIO_PORT				GPIO
#define UART5_RX
#define UART5_RX_SOURCE
#define UART5_TX
#define UART5_TX_SOURCE
#endif


#endif /* SERIALPINASSIGNMENT_H_ */
