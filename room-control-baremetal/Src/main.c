#include "gpio.h"
#include "systick.h"
#include "rcc.h"
#include "uart.h"
#include "nvic.h"
#include "tim.h"
#include "room_control.h"


// --- Heartbeat y timeout del LED PA5 ---
static volatile uint8_t  led_forced_on = 0;   // 1 mientras el LED está forzado encendido (botón)
static volatile uint32_t led_on_time   = 0;   // ms cuando se encendió por última vez
static volatile uint32_t hb_prev_ms    = 0;   // timestamp último parpadeo
static volatile uint8_t  hb_state      = 0;   // estado actual del heartbeat (0/1)

volatile uint32_t ms_counter = 17;
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


    // --- ROOM CONTROL (parser de comandos por UART) ---
    room_control_init();
    uart_send_string("Comando: PWM <0..100>\r\n");

    room_control_app_init();


    while (1) {
        
        room_control_update();

        // Lógica del botón (del profe): enciende PA5 por 3s
        if (read_gpio(GPIOC, 13) != 0) {   // Botón presionado (según tu read_gpio)
            ms_counter = 0;                // reiniciar el contador de milisegundos
            set_gpio(GPIOA, 5);            // Encender LED
            // NUEVO: marcar que está forzado y guardar el tiempo de encendido
            led_forced_on = 1;
            led_on_time   = ms_counter;
        }

    }
}

void SysTick_Handler(void)
{
    ms_counter++;

    // --- Heartbeat: parpadeo de LD2 cada 500 ms cuando NO está forzado ---
    if (!led_forced_on && (uint32_t)(ms_counter - hb_prev_ms) >= 500) {
        hb_prev_ms = ms_counter;
        hb_state ^= 1u;                 // alterna 0/1
        if (hb_state) set_gpio(GPIOA, 5);
        else          clear_gpio(GPIOA, 5);
    }

    // --- Timeout: si el LED está forzado, apaga a los 3 s y vuelve al heartbeat ---
    if (led_forced_on && (uint32_t)(ms_counter - led_on_time) >= 3000) {
        led_forced_on = 0;              // libera el control para que el heartbeat retome
        clear_gpio(GPIOA, 5);
        // (el heartbeat retomará en el siguiente tick de 500 ms)
    }
}
