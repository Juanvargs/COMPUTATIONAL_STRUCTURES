#include <stdint.h>
#include "nvic.h"          // EXTI, NVIC registros
#include "uart.h"          // USART2
#include "room_control.h"  // prototipos de la app

extern volatile uint32_t ms_counter;   // definido en main.c

// === Eventos globales (los consume main) ===
extern volatile uint8_t button_event;
extern volatile char    uart_event_char;

// ---- Debounce (timestamp del último flanco válido) ----
static uint32_t last_btn_ms = 0;

void EXTI15_10_IRQHandler(void)
{
    if (EXTI->PR1 & (1U << 13)) {
        EXTI->PR1 = (1U << 13);           // limpiar pendiente

        // Debounce: ignora flancos dentro de 50 ms
        uint32_t now = ms_counter;
        if ((uint32_t)(now - last_btn_ms) < 50U) {
            return; // rebote, no procesar
        }
        last_btn_ms = now;

        button_event = 1;                 // ← evento para main (guía 10)
    }
}

void USART2_IRQHandler(void)
{
    uint32_t isr = USART2->ISR;

    // Limpia SOLO lo necesario y no drenes RDR salvo en ORE
    if (isr & (1U << 3)) {                 // ORE
        USART2->ICR = (1U << 3);
        (void)USART2->RDR;                 // drenar overrun
    }
    if (isr & (1U << 0)) { USART2->ICR = (1U << 0); } // PE
    if (isr & (1U << 1)) { USART2->ICR = (1U << 1); } // FE
    if (isr & (1U << 2)) { USART2->ICR = (1U << 2); } // NE

    // Atiende TODOS los bytes pendientes
    while (USART2->ISR & (1U << 5)) {      // RXNE
        char b = (char)USART2->RDR;        // leer limpia RXNE
        uart_event_char = b;               // ← lo procesará main (guía 10)
    }
}
