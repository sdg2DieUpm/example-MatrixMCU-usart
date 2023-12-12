/**
 * @file main.c
 * @author Josué Pagán (j.pagan@upm.es)
 * @brief This code is a simple example of how to use the USART port for STM32F446RE.
 *
 * The code configures the USART 1 to work at 1200 bauds, 8 bits, no parity and 1 stop bit. The code also configures the GPIO port connected to the USART 1.
 *
 * The code sends a string to the USART 1 and waits until the transmission is complete. Then, the code waits until a character is received and prints it on the console. The transmission is done using polling. The reception is done using interrupts.
 *
 * The microcontroller says hello to the world and then waits for a character to be received. When a character is received, the microcontroller prints it on the console (debug).
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

int main()
{
    /* Init board */
    port_system_init(); // Initialize the system (clocks, peripherals, etc.)

    port_usart1_gpio_setup(); // Configure the GPIO ports connected to the USART 1
    port_usart1_config();     // Configure the USART 1

    // Say hello to the world
    char *hello_str = "Hello world!\n";
    port_usart1_write(hello_str, strlen(hello_str));

    while (1)
    { // Do nothing. Wait for interrupt to receive a character
    }

    return 0;
}
