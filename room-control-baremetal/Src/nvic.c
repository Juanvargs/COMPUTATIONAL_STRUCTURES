#include "nvic.h"
#include "rcc.h"  // Para rcc_syscfg_clock_enable
#include "uart.h"

static void nvic_enable_irq(uint32_t IRQn)
{
    NVIC->ISER[IRQn / 32U] |= (1UL << (IRQn % 32U));
}

void nvic_exti_pc13_button_enable(void) {
    // 1. Habilitar el reloj para SYSCFG
    rcc_syscfg_clock_enable();

    // 2. Configurar la línea EXTI13 (SYSCFG_EXTICR)
    SYSCFG->EXTICR[3] &= ~(0x000FU << 4);  // Limpiar campo EXTI13
    SYSCFG->EXTICR[3] |=  (0x0002U << 4);  // Conectar EXTI13 a PC13

    // 3. Configurar la línea EXTI13 para interrupción
    EXTI->IMR1 |= (1U << 13);

    // 4. Configurar el trigger de flanco de bajada
    EXTI->FTSR1 |= (1U << 13);
    EXTI->RTSR1 &= ~(1U << 13);

    // 5. Habilitar la interrupción EXTI15_10 en el NVIC
    nvic_enable_irq(EXTI15_10_IRQn);
}

void nvic_usart2_irq_enable(void) {
    // Habilitar interrupción de recepción en USART2
    USART2->CR1 |= (1U << 5);  // RXNEIE
    nvic_enable_irq(USART2_IRQn);
}