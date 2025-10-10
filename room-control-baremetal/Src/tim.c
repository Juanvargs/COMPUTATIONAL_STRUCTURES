#include "tim.h"
#include "rcc.h"
#include "gpio.h"

void tim3_ch1_pwm_init(uint32_t pwm_freq_hz)
{
    // 1) PA6 como función alternativa AF2 (TIM3_CH1)
    
    // Firma de tu API: init_gpio(PORT, pin, MODER, OTYPER, OSPEEDR, PUPDR, AFR)
    // MODER=0x02 (AF), OTYPER=0x00 (push-pull), OSPEEDR=0x01 (rápida),
    // PUPDR=0x00 (sin pull), AFR=0x02 (AF2 = TIM3_CH1 en PA6)
    
    init_gpio(GPIOA, 6, 0x02, 0x00, 0x01, 0x00, 0x02);

    // 2) Reloj de TIM3
    rcc_tim3_clock_enable();

    // 3) TIM3 a PWM
    TIM3->PSC = 100 - 1; // (4 MHz / 100 = 40 kHz)
    TIM3->ARR = (TIM_PCLK_FREQ_HZ / 100 / pwm_freq_hz) - 1; // 40kHz / pwm_freq_hz

    // Modo PWM1 en CH1 y habilitar salida
    TIM3->CCMR1 = (6U << 4);        // OC1M = 110 (PWM1)
    TIM3->CCER  |= (1U << 0);       // CC1E = 1

    // Encender contador
    TIM3->CR1 |= (1U << 0);
}

void tim3_ch1_pwm_set_duty_cycle(uint8_t duty_cycle_percent)
{
    if (duty_cycle_percent > 100) duty_cycle_percent = 100;

    uint16_t arr = TIM3->ARR;
    uint32_t ccr = (((uint32_t)arr + 1U) * duty_cycle_percent) / 100U;

    TIM3->CCR1 = ccr;
}
