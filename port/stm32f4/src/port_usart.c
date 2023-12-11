/**
 * @file port_usart.c
 * @brief This file contains the implementation of USART port for STM32F446RE.
 * @author Josué Pagán (j.pagan@upm.es) *
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
    /* Configura PA9 (TX) */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Habilita el reloj de la GPIOA (uso macro de CMSIS)

    /* Poner modo alternativo */
    GPIOA->MODER &= ~MODE_LINEA_9_MASK;                // Asegura que los bits estan limpios
    GPIOA->MODER |= (MODE_ALTERNATE << (2 * LINEA_9)); // Pon modo alternativo: 0b10<<2*9=0x0008 0000

    /* Poner a pull-up */
    GPIOA->PUPDR &= ~PUPDR_LINEA_9_MASK;             // Asegura que los bits estan limpios
    GPIOA->PUPDR |= (MODE_PULL_UP << (2 * LINEA_9)); // Pon pull-up: 0b01<<2*9=0x0004 0000

    /* Configura la function alternativa para trabajar como USART1 TX */
    /* Como es el pin 9, trabajamos con el registro AFRH (high) */
    /* Miramos Table 11 del Datasheet para see que USART1 TX es la funcion alternativa 7 (AF7) */
    GPIOA->AFR[1] &= ~AFRH_LINEA_9_MASK;                          // Asegura que los bits estan limpios
    GPIOA->AFR[1] |= (ALT_FUNC_UART1_TX << 4 * (LINEA_9 & 0x7U)); // Pon AF7: 0x07<<(4*(0b1001 & 0b0111))=0x0070

    /* Configure PA10 (RX) */
    /* Hay que habilitar el reloj de la GPIOA (pero ya esta activado) */
    /* Poner modo alternativo */
    GPIOA->MODER &= ~MODE_LINEA_10_MASK;              // Asegura que los bits estan limpios
    GPIOA->MODER |= (MODE_ALTERNATE << 2 * LINEA_10); // Pon modo alternativo: 0b10<<2*10=0x00100000

    /* Poner a pull-up */
    GPIOA->PUPDR &= ~PUPDR_LINEA_10_MASK;           // Asegura que los bits estan limpios
    GPIOA->PUPDR |= (MODE_PULL_UP << 2 * LINEA_10); // Pon pull-up: 0b01<<2*10=0x00100000

    /* Configura la function alternativa para trabajar como USART1 RX */
    /* Como es el pin 10, trabajamos con el registro AFRH (high) */
    /* Miramos Table 11 del Datasheet para see que USART1 RX es la funcion alternativa 7 (AF7) */
    GPIOA->AFR[1] &= ~AFRH_LINEA_10_MASK;                          // Asegura que los bits estan limpios
    GPIOA->AFR[1] |= (ALT_FUNC_UART1_RX << 4 * (LINEA_10 & 0x7U)); // Pon AF7: 0x07<<(4*(0b1010 & 0b0111))=0x0700
}

void port_usart1_config()
{
    /* Primero habilitamos el reloj de perifericos de la USART-1 */
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* Deshabilitamos la USART-1 para tocar los registros */
    USART1->CR1 &= ~USART_CR1_UE; // USART_CR1_UE es de CMSIS (ver stm32f446xx.h)

    /* Configuramos la longitud de los datos a 8 bits */
    USART1->CR1 &= ~USART_CR1_M; // Limpiamos el bit M (8 bits)

    /* Configuramos el bit de parada a 1 bit */
    USART1->CR2 &= ~USART_CR2_STOP; // Limpiamos los bits STOP (1 bit)

    /* Configuramos el bit de paridad a no paridad */
    USART1->CR1 &= ~USART_CR1_PCE; // Limpiamos el bit PCE (no paridad)

    /* Aseguramos oversampling de 16 (por defecto) */
    USART1->CR1 &= ~USART_CR1_OVER8; // Limpiamos el bit OVER8 (oversampling de 16)

    /* Configuramos el baud rate (ver ejemplo de calculo) */
    USART1->BRR = 0x3415; // 1200 baudios

    /* Habilitamos la transmision y la recepcion */
    USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;

    /* Habilitamos las interrupcion de recepcion */
    USART1->CR1 |= USART_CR1_RXNEIE; // Habilitamos la interrupcion de recepcion

    /* No habilitamos las interrupciones de transmision generalmente al inicio, sino solo cuando es necesario */
    USART1->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE); // Deshabilitamos las interrupciones de buffer vacio y transmision completa, por ahora

    /* Habilitamos las interrupciones de la USART-1 globalmente */
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 1, 0)); /* Prioridad 1, sub-prioridad 0, por ejemplo */
    NVIC_EnableIRQ(USART1_IRQn);                                                          // Habilitamos la interrupcion de la USART-1

    /* Por ultimo, habilitamos la USART-1 */
    USART1->CR1 |= USART_CR1_UE;
}

void port_usart1_write(char *p_data, uint32_t nBytes)
{
    for (uint32_t i = 0; i < nBytes; i++)
    {
        // Wait until TXE = 1 (TXE is set when the TXDR register is empty)
        while (!(USART1->SR & USART_SR_TXE))
            ;

        // Write data to DR
        USART1->DR = p_data[i]; // Writing to the DR clears the TXE flag
    }

    // Wait intl TC bit is set. This indicates that the transmission of the last frame is complete
    while (!(USART1->SR & USART_SR_TC))
        ;
}

void port_usart1_read()
{
    // Read data from DR
    char char_read = USART1->DR; // Reading the DR clears the RXNE flag
    printf("Char received: %c\n", char_read);
}
