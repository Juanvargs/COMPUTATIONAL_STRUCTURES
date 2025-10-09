#include <stdint.h>
#include "nvic.h"    // EXTI, NVIC
#include "uart.h"    // define USART2 (igual que en main.c)
#include "room_control.h"

// #include "gpio.h" // solo si haces toggle de LED dentro de la ISR

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR1 & (1U << 13)) {
        EXTI->PR1 = (1U << 13);     // limpiar pendiente
        // GPIOA->ODR ^= (1U << 5);  // opcional: toggle LED si PA5 está configurado
        // uart_send_string("IRQ EXTI\r\n"); // opcional: mensaje para ver la IRQ
    }
}

void USART2_IRQHandler(void)
{
    if (USART2->ISR & (1U << 5)) {      // RXNE
        char b = (char)USART2->RDR;     // leer limpia RXNE
        room_control_on_rx_char(b);     // 👉 entregar el byte al parser
    }
}
