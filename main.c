/**
 * @file main.c
 * @author Josué Pagán (j.pagan@upm.es)
 * @brief This code is a simple example of how to use the USART port for STM32F446RE.
 *
 * The code configures the USART 1 to work at 1200 bauds, 8 bits, no parity and 1 stop bit. The code also configures the GPIO port connected to the USART 1.
 *
 * The code sends a string to the USART 1 and waits until the transmission is complete. Then, the code waits until a character is received and prints it on the console. The transmission and the reception are done using interrupts.
 *
 * The microcontroller says hello to the world and then waits for a character to be received. When a character is received, the microcontroller prints it on the console (debug only).
 *
 * @version 0.1
 * @date 2023-12-07
 *
 * @copyright Copyright (c) 2023
 *
 */

/* INCLUDES */
/* System includes */
#include <string.h>

/* HW includes */
#include "port_system.h"
#include "port_usart.h"

/* Global variables */
char *g_p_tx_data;
uint32_t g_bytes_tx_data;

int main()
{
    /* Init board */
    port_system_init(); // Initialize the system (clocks, peripherals, etc.)

    port_usart1_gpio_setup(); // Configure the GPIO ports connected to the USART 1
    port_usart1_config();     // Configure the USART 1

    // Say hello to the world
    // Set value for the global pointer to the data to be transmitted
    g_p_tx_data = "Hello world!\n";
    // Set value for the global variable to store the number of bytes to be transmitted
    g_bytes_tx_data = strlen(g_p_tx_data);

    port_usart1_write();

    while (1)
    { // Do nothing. Wait for interrupt to receive a character
    }

    return 0;
}
