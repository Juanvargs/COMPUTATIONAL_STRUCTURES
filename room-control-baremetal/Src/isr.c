#include <stdint.h>
#include "nvic.h"   // Registros EXTI/USART/NVIC, tipos
#include "uart.h"   // Definición de USART2 (registros)

// ms_counter viene de main.c (usado para debounce del botón)
extern volatile uint32_t ms_counter;

// === Eventos globales (los consume main en el bucle) ===
extern volatile uint8_t button_event;    // set por EXTI15_10_IRQHandler
extern volatile char    uart_event_char; // último byte RX (set por USART2_IRQHandler)

// ---- Debounce: timestamp del último flanco válido en PC13 ----
static uint32_t last_btn_ms = 0;

void EXTI15_10_IRQHandler(void)
{
    // ¿Pendiente en línea 13? (botón azul PC13)
    if (EXTI->PR1 & (1U << 13)) {

        // Limpiar la bandera de pendiente escribiendo 1
        EXTI->PR1 = (1U << 13);

        // Debounce: ignora flancos a menos de 50 ms
        uint32_t now = ms_counter;
        if ((uint32_t)(now - last_btn_ms) < 50U) {
            return; // rebote: no generar evento
        }
        last_btn_ms = now;

        // Notificar al bucle principal (Guía 10: estilo basado en eventos)
        button_event = 1;
    }
}

void USART2_IRQHandler(void)
{
    uint32_t isr = USART2->ISR;

    // --- Manejo básico de errores (limpieza mínima) ---
    if (isr & (1U << 3)) {                // ORE: Overrun Error
        USART2->ICR = (1U << 3);
        (void)USART2->RDR;                // leer para drenar el overrun
    }
    if (isr & (1U << 0)) { USART2->ICR = (1U << 0); } // PE: Parity Error clear
    if (isr & (1U << 1)) { USART2->ICR = (1U << 1); } // FE: Framing Error clear
    if (isr & (1U << 2)) { USART2->ICR = (1U << 2); } // NE: Noise Error clear

    // --- Consumir todos los bytes pendientes en RX ---
    while (USART2->ISR & (1U << 5)) {     // RXNE: dato disponible
        char b = (char)USART2->RDR;       // leer limpia RXNE
        // Modelo simple de evento: nos quedamos con el último byte recibido
        uart_event_char = b;
    }
}
