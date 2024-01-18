/**
 * @file port_usart.c
 * @brief This file contains the implementation of USART port for STM32F446RE.
 * @author Josué Pagán (j.pagan@upm.es)
 * @date 2023-12-01
 */

/* System includes */
#include <stdio.h>

/* HW dependent includes */
#include "stm32f4xx.h"
#include "port_system.h"
#include "port_usart.h"

/* FUNCTIONS */
void port_usart1_gpio_setup(void)
{
    /* Configure PA9 (TX) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable the clock for GPIOA (using CMSIS macro)

    /* Set alternate mode */
    GPIOA->MODER &= ~MODE_LINEA_9_MASK;                // Ensure the bits are clean
    GPIOA->MODER |= (MODE_ALTERNATE << (2 * LINEA_9)); // Set alternate mode: 0b10<<2*9=0x0008 0000

    /* Set to pull-up */
    GPIOA->PUPDR &= ~PUPDR_LINEA_9_MASK;             // Ensure the bits are clean
    GPIOA->PUPDR |= (MODE_PULL_UP << (2 * LINEA_9)); // Set pull-up: 0b01<<2*9=0x0004 0000

    /* Configure the alternate function to work as USART1 TX */
    /* As it is pin 9, we work with the AFRH (high) register */
    /* Look at Table 11 of the Datasheet to see that USART1 TX is alternate function 7 (AF7) */
    GPIOA->AFR[1] &= ~AFRH_LINEA_9_MASK;                          // Ensure the bits are clean
    GPIOA->AFR[1] |= (ALT_FUNC_UART1_TX << 4 * (LINEA_9 & 0x7U)); // Set AF7: 0x07<<(4*(0b1001 & 0b0111))=0x0070

    /* Configure PA10 (RX) */
    /* We need to enable the clock for GPIOA (but it is already enabled) */
    /* Set alternate mode */
    GPIOA->MODER &= ~MODE_LINEA_10_MASK;              // Ensure the bits are clean
    GPIOA->MODER |= (MODE_ALTERNATE << 2 * LINEA_10); // Set alternate mode: 0b10<<2*10=0x00100000

    /* Set to pull-up */
    GPIOA->PUPDR &= ~PUPDR_LINEA_10_MASK;           // Ensure the bits are clean
    GPIOA->PUPDR |= (MODE_PULL_UP << 2 * LINEA_10); // Set pull-up: 0b01<<2*10=0x00100000

    /* Configure the alternate function to work as USART1 RX */
    /* As it is pin 10, we work with the AFRH (high) register */
    /* Look at Table 11 of the Datasheet to see that USART1 RX is alternate function 7 (AF7) */
    GPIOA->AFR[1] &= ~AFRH_LINEA_10_MASK;                          // Ensure the bits are clean
    GPIOA->AFR[1] |= (ALT_FUNC_UART1_RX << 4 * (LINEA_10 & 0x7U)); // Set AF7: 0x07<<(4*(0b1010 & 0b0111))=0x0700
}

void port_usart1_config()
{
    /* First, we enable the peripheral clock for USART-1 */
        RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

        /* We disable USART-1 to touch the registers */
        USART1->CR1 &= ~USART_CR1_UE; // USART_CR1_UE is from CMSIS (see stm32f446xx.h)

        /* We configure the data length to 8 bits */
        USART1->CR1 &= ~USART_CR1_M; // We clear the M bit (8 bits)

        /* We configure the stop bit to 1 bit */
        USART1->CR2 &= ~USART_CR2_STOP; // We clear the STOP bits (1 bit)

        /* We configure the parity bit to no parity */
        USART1->CR1 &= ~USART_CR1_PCE; // We clear the PCE bit (no parity)

        /* We ensure oversampling of 16 (by default) */
        USART1->CR1 &= ~USART_CR1_OVER8; // We clear the OVER8 bit (oversampling of 16)

        /* We configure the baud rate (see calculation example) */
        USART1->BRR = 0x3415; // 1200 baud

        /* We enable transmission and reception */
        USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;

        /* We enable the reception interrupt */
        USART1->CR1 |= USART_CR1_RXNEIE; // We enable the reception interrupt

        /* We generally do not enable transmission interrupts at the beginning, but only when necessary */
        USART1->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE); // We disable the empty buffer and complete transmission interrupts, for now

        /* We enable the USART-1 interrupts globally */
        NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0)); /* Priority 1, sub-priority 0, for example */
        NVIC_EnableIRQ(USART1_IRQn);                                                          // We enable the USART-1 interrupt

        /* Finally, we enable USART-1 */
        USART1->CR1 |= USART_CR1_UE;
}

void port_usart1_write()
{
    // Static variable to store the number of bytes transmitted
    static uint32_t i = 0;

    // Before the transmission of the first data, the TE bit must be set. Enable transmission interrupt (TXEIE) and wait
    if (i == 0)
    {
        USART1->CR1 |= USART_CR1_TXEIE;
    }

    // Always wait until TXE = 1 (TXE is set when the TXDR register is empty)
    while (!(USART1->SR & USART_SR_TXE))
        ;

    // Transmit the i-th byte of the data of the global variable g_p_tx_data
    USART1->DR = g_p_tx_data[i++]; // Write data to DR. Writing to the DR clears the TXE flag

    // After transmission of the last data, restore the number of bytes transmitted to 0
    if (i >= g_bytes_tx_data)
    {
        // Disable transmission interrupt (TXEIE)
        USART1->CR1 &= ~USART_CR1_TXEIE;

        // Reset the number of bytes transmitted
        i = 0;

        // Uncomment the following lines to wait until TC = 1 (TC is set when the transmission of the last frame is complete) if TCIE is enabled
        // Wait until TC bit is set. This indicates that the transmission of the last frame is complete
        // while (!(USART1->SR & USART_SR_TC));
    }
}

void port_usart1_read()
{
    // Read data from DR
    char char_read = USART1->DR; // Reading the DR clears the RXNE flag
    printf("Char received: %c\n", char_read);
}