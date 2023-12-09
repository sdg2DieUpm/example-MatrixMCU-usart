/**
 ******************************************************************************
 * @file           : port_system.h
 * @brief          : Header for port_system.c file.
 ******************************************************************************
 * @attention
 *
 * Josue Pagán Ortiz (j.pagan@upm.es)
 * Sistemas Digitales II
 *
 ******************************************************************************
 */

#ifndef PORT_SYSTEM_H_
#define PORT_SYSTEM_H_

/* Includes del sistema */
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define HSI_VALUE ((uint32_t)16000000) /*!< Value of the Internal oscillator in Hz */

/* Microcontroller STM32F446RE */
/* Timer configuration */
#define RCC_HSI_CALIBRATION_DEFAULT 0x10U            /*!< Default HSI calibration trimming value */
#define TICK_FREQ_1KHZ 1U                            /*!< Freqency in kHz of the System tick */
#define NVIC_PRIORITY_GROUP_0 ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority, \
                                                         4 bits for subpriority */
#define NVIC_PRIORITY_GROUP_4 ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority, \
                                                         0 bit  for subpriority */

/* Power */
#define POWER_REGULATOR_VOLTAGE_SCALE3 0x01 /*!< Scale 3 mode: the maximum value of fHCLK is 120 MHz. */


/* GPIOs */
#define HIGH true /*!< Logic 1 */
#define LOW false /*!< Logic 0 */

#define GPIO_MODE_IN 0x00        /*!< GPIO as input */
#define GPIO_MODE_OUT 0x01       /*!< GPIO as output */
#define GPIO_MODE_ALTERNATE 0x02 /*!< GPIO as alternate function */
#define GPIO_MODE_ANALOG 0x03    /*!< GPIO as analog */

#define GPIO_PUPDR_NOPULL 0x00 /*!< GPIO no pull up or down */
#define GPIO_PUPDR_PUP 0x01    /*!< GPIO no pull up */
#define GPIO_PUPDR_PDOWN 0x02  /*!< GPIO no pull down */

size_t port_system_init(void);
uint32_t port_system_get_millis(void);
void port_system_delay_ms(uint32_t ms);
void port_system_delay_until_ms(uint32_t *t, uint32_t ms);

#endif /* PORT_SYSTEM_H_ */
