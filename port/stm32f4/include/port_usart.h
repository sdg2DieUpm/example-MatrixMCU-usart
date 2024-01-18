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
#define LINEA_9 9   /*!< Pin for the TX */
#define LINEA_10 10 /*!< Pin for the RX */

#define MODE_LINEA_9_MASK (0x03 << (2 * LINEA_9))   /*!< Shifts 0b11 to the left 2*9=18 positions in the mode register */
#define MODE_LINEA_10_MASK (0x03 << (2 * LINEA_10)) /*!< Shifts 0b11 to the left 2*10=20 positions in the mode register */

#define PUPDR_LINEA_9_MASK (0x03 << (2 * LINEA_9))   /*!< Shifts 0b11 to the left 2*9=18 positions in the pull-up/pull-down register */
#define PUPDR_LINEA_10_MASK (0x03 << (2 * LINEA_10)) /*!< Shifts 0b11 to the left 2*10=20 positions in the pull-up/pull-down register */

#define MODE_ALTERNATE 0x02U /*!< Alternate function mode */
#define MODE_PULL_UP 0x01U   /*!< Pull-up mode */

#define ALT_FUNC_UART1_TX 0x07U /*!< AF7 for alternate function UART1 TX according to datasheet */
#define ALT_FUNC_UART1_RX 0x07U /*!< AF7 for alternate function UART1 RX according to datasheet */

#define AFRH_LINEA_9_MASK (0x0F << (4 * (LINEA_9 & 0x7U)))   /*!< Shifts 0b1111 to the left 4*(0x9 & 0x7)=8 positions in the alternate function register */
#define AFRH_LINEA_10_MASK (0x0F << (4 * (LINEA_10 & 0x7U))) /*!< Shifts 0b1111 to the left 4*(0xA & 0x7)=8 positions in the alternate function register */

/* GLOBAL VARIABLES */
extern char *g_p_tx_data;        /*!< Global pointer to the data to be transmitted. (g_ for global and p_ for pointer is a common naming convention only) */
extern uint32_t g_bytes_tx_data; /*!< Global variable to store the number of bytes to be transmitted. */

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
 * @brief Writes the global string to the USART 1. 
 * 
 * It comes here from the 1st call of the user in main.c and then from the interrupt handler (ISR) after each byte is transmitted.
 * 
 */
void port_usart1_write();

/**
 * @brief  Reads a string from the USART 1.
 *
 */
void port_usart1_read();

#endif // PORT_USART_H_
