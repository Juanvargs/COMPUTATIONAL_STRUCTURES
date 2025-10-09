#include "rcc.h"

void rcc_init(void)
{
    // Habilitar reloj de GPIOA (bit 0 del AHB2ENR)
    RCC->AHB2ENR |= (1U << 0);

    // Habilitar reloj de GPIOC (bit 2 del AHB2ENR)
    RCC->AHB2ENR |= (1U << 2);
}

void rcc_syscfg_clock_enable(void)
{
    // Habilitar reloj del SYSCFG (bit 0 del APB2ENR)
    RCC->APB2ENR |= (1U << 0);
}
void rcc_tim3_clock_enable(void)
{
    // TIM3 está en APB1ENR1, bit 1 en STM32L476
    RCC->APB1ENR1 |= (1U << 1);
}
