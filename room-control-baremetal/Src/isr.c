#include <stdint.h>
#include "nvic.h"          // EXTI, NVIC registros
#include "uart.h"          // USART2
#include "room_control.h"  // handlers de la guía 9

void EXTI15_10_IRQHandler(void)
{
    // PC13 está en EXTI13
    if (EXTI->PR1 & (1U << 13)) {
        EXTI->PR1 = (1U << 13);          // limpiar pendiente
        room_control_on_button_press();  // avisar a la app
    }
}

void USART2_IRQHandler(void)
{
    uint32_t isr = USART2->ISR;

    // Limpia SOLO lo necesario y no drenes RDR salvo en ORE
    if (isr & (1U << 3)) {                 // ORE
        USART2->ICR = (1U << 3);
        (void)USART2->RDR;                 // leer para drenar el overrun
    }
    if (isr & (1U << 0)) { USART2->ICR = (1U << 0); } // PE
    if (isr & (1U << 1)) { USART2->ICR = (1U << 1); } // FE
    if (isr & (1U << 2)) { USART2->ICR = (1U << 2); } // NE

    // Atiende TODOS los bytes que ya estén en el FIFO de recepción
    while (USART2->ISR & (1U << 5)) {      // RXNE
        char b = (char)USART2->RDR;        // leer limpia RXNE
        room_control_on_uart_receive(b);
    }
}
