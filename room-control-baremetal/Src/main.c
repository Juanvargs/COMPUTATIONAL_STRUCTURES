#include "gpio.h"
#include "systick.h"
#include "rcc.h"
#include "uart.h"
#include "nvic.h"
#include "tim.h"
#include "room_control.h"

static volatile uint32_t ms_counter = 17;
static char rx_buffer[256];           // (se deja, aunque no lo usamos con ISR)
static uint8_t rx_index = 0;          // (ídem)
static uint8_t  pwm_dc = 0;           // duty actual (0..100)
static int8_t   pwm_dir = 1;          // +1 sube, -1 baja
static uint32_t t_pwm = 0;            // timestamp en ms

// --- Programa principal ------------------------------------------------------
int main(void)
{
    rcc_init();
    init_gpio(GPIOA, 5, 0x01, 0x00, 0x01, 0x00, 0x00);
    init_gpio(GPIOC, 13, 0x00, 0x00, 0x01, 0x01, 0x00);
    init_systick();

    init_gpio_uart();   // GPIO para UART
    init_uart();        // UART

    nvic_exti_pc13_button_enable();   // EXTI13 para botón PC13
    nvic_usart2_irq_enable();         // IRQ de USART2 (RX)
    NVIC->IP[EXTI15_10_IRQn] = (1U << 4);   // Botón: prioridad alta
    NVIC->IP[USART2_IRQn]    = (3U << 4);   // UART: prioridad baja

    // --- PWM TIM3_CH1 (PA6) ---
    tim3_ch1_pwm_init(PWM_FREQUENCY);
    // Arrancamos el fade desde 0% (se sobreescribe el 50% de prueba)
    pwm_dc = 0;
    tim3_ch1_pwm_set_duty_cycle(pwm_dc);
    t_pwm = ms_counter;

    uart_send_string("Sistema Inicializado!\r\n");

    // --- ROOM CONTROL (parser de comandos por UART) ---
    room_control_init();
    uart_send_string("Comando: PWM <0..100>\r\n");

    while (1) {
        // Lógica del botón (del profe): enciende PA5 por 3s
        if (read_gpio(GPIOC, 13) != 0) {   // Botón presionado (según tu read_gpio)
            ms_counter = 0;                // reiniciar el contador de milisegundos
            set_gpio(GPIOA, 5);            // Encender LED
        }
        if (ms_counter >= 3000) {          // 3 segundos
            clear_gpio(GPIOA, 5);          // Apagar LED
        }

        // Barrido suave 0% -> 100% -> 0%, actualización cada 10 ms
        // if ((uint32_t)(ms_counter - t_pwm) >= 10) {
            // t_pwm = ms_counter;

            // int16_t next = (int16_t)pwm_dc + pwm_dir;
            // if (next >= 100) { next = 100; pwm_dir = -1; }
            // if (next <=   0) { next =   0; pwm_dir = +1; }

            // pwm_dc = (uint8_t)next;
            // tim3_ch1_pwm_set_duty_cycle(pwm_dc);
        // }

        // --- Polling UART (desactivado; ahora usamos la ISR + room_control) ---
        // if (USART2->ISR & (1 << 5)) { /* ... */ }
    }
}

// --- Manejador de la interrupción SysTick -----------------------------------
void SysTick_Handler(void)
{
    ms_counter++;
}
