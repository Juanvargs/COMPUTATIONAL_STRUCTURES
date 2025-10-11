#include "tim.h"
#include "rcc.h"
#include "gpio.h"

void tim3_ch1_pwm_init(uint32_t pwm_freq_hz)
{
    // 1) PA6 como función alternativa AF2 (TIM3_CH1)
    // Usa tu helper y además fuerza los registros para evitar dudas.
    init_gpio(GPIOA, 6, 0x02, 0x00, 0x01, 0x00, 0x00); // MODER=AF, push-pull, speed=fast, no pull

    // Forzar AF2 explícitamente en PA6 (bits AFRL[27:24] = 0b0010)
    GPIOA->AFRL &= ~(0xFU << (6U * 4U));
    GPIOA->AFRL |=  (0x2U << (6U * 4U));  // AF2 = TIM3_CH1

    // Opcional: asegurar push-pull, alta velocidad, sin pull
    GPIOA->TYPER  &= ~(1U << 6);
    GPIOA->SPEEDR &= ~(3U << (6U * 2U));
    GPIOA->SPEEDR |=  (2U << (6U * 2U));  // alta
    GPIOA->PUPDR  &= ~(3U << (6U * 2U));

    // 2) Reloj de TIM3
    rcc_tim3_clock_enable();

    // 3) Config TIM3 a PWM
    // Suponiendo TIM_PCLK = 4 MHz (tu define), prescaler a 100 => 40 kHz base
    TIM3->PSC = 100U - 1U; // (4 MHz / 100 = 40 kHz)
    TIM3->ARR = (TIM_PCLK_FREQ_HZ / 100U / pwm_freq_hz) - 1U; // 40kHz / pwm_freq_hz

    // Modo PWM1 en CH1, con preload del comparador (OC1PE=1)
    // OC1M = 110 (PWM1) en bits [6:4], OC1PE en bit 3
    TIM3->CCMR1 &= ~((7U << 4) | (1U << 3));
    TIM3->CCMR1 |=  ((6U << 4) | (1U << 3));  // PWM1 + preload

    // Habilitar CH1 salida activa-alta
    TIM3->CCER &= ~(1U << 1);   // CC1P=0 (activo alto)
    TIM3->CCER |=  (1U << 0);   // CC1E=1 (enable)

    // Preload en ARR y generar evento de actualización para cargar registros
    TIM3->CR1  |=  (1U << 7);   // ARPE=1
    TIM3->EGR   =  (1U << 0);   // UG=1

    // Iniciar contador
    TIM3->CR1  |=  (1U << 0);   // CEN=1
}

void tim3_ch1_pwm_set_duty_cycle(uint8_t duty_cycle_percent)
{
    if (duty_cycle_percent > 100U) duty_cycle_percent = 100U;

    uint32_t arr_plus1 = (uint32_t)TIM3->ARR + 1U;
    uint32_t ccr = (arr_plus1 * (uint32_t)duty_cycle_percent) / 100U;

    TIM3->CCR1 = (uint16_t)ccr;
    // Si usas preload en CCR1, puedes forzar un UG para aplicar inmediato:
    TIM3->EGR = (1U << 0); // UG
}
