/**
 * @file port_usart.h
 * @brief Header for port_usart.c file.
 * @author Josué Pagán (j.pagan@upm.es)
 * @date 2023-12-01
 */

#ifndef PORT_USART_H_
#define PORT_USART_H_

/* INCLUDES */
/* System includes */
#include <stdint.h>

/* DEFINES */
#define LINEA_9 9   /* Para el TX */
#define LINEA_10 10 /* Para el RX */

#define MODE_LINEA_9_MASK (0x03 << (2 * LINEA_9))   /* Desplaza los bits 0b11 a la izquierda 2*9=18 posiciones */
#define MODE_LINEA_10_MASK (0x03 << (2 * LINEA_10)) /* Desplaza los bits 0b11 a la izquierda 2*10=20 posiciones */

#define PUPDR_LINEA_9_MASK (0x03 << (2 * LINEA_9))   /* Desplaza los bits 0b11 a la izquierda 2*9=18 posiciones */
#define PUPDR_LINEA_10_MASK (0x03 << (2 * LINEA_10)) /* Desplaza los bits 0b11 a la izquierda 2*10=20 posiciones */

#define MODE_ALTERNATE 0x02U /* Modo alternativo */
#define MODE_PULL_UP 0x01U   /* Modo pull-up */

#define ALT_FUNC_UART1_TX 0x07U /* AF7 para funcion alternativa UART1 TX segun datasheet */
#define ALT_FUNC_UART1_RX 0x07U /* AF7 para funcion alternativa UART1 RX segun datasheet */

#define AFRH_LINEA_9_MASK (0x0F << (4 * (LINEA_9 & 0x7U)))   /* Desplaza los bits 0b1111 a la izquierda 4*(0x9 & 0x7)=4 posiciones */
#define AFRH_LINEA_10_MASK (0x0F << (4 * (LINEA_10 & 0x7U))) /* Desplaza los bits 0b1111 a la izquierda 4*(0xA & 0x7)=8 posiciones */


/* FUNCTIONS */
/**
 * @brief  Configures the GPIO port connected to the USART 1.
 */
void port_usart1_gpio_setup(void);

/**
 * @brief  Configures the registers of USART 1.
 */
void port_usart1_config(void);

/**
 * @brief  Writes a string to the USART 1 using polling.
 *
 * @param p_data Pointer to the string to be written.
 * @param nBytes Number of bytes to be written.
 */
void port_usart1_write(char *p_data, uint32_t nBytes);

/**
 * @brief  Reads a string from the USART 1.
 *
 */
void port_usart1_read();

/////////////////////////////////////////////////////
//void port_uart3_config();
//void port_usart3_read();

#endif // PORT_USART_H_
