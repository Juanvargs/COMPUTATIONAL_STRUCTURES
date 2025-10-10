#include "gpio.h"
#include "systick.h"
#include "rcc.h"
#include "uart.h"
#include "nvic.h"
#include "tim.h"
#include "room_control.h"

// === Eventos (estilo guía 10) ===
volatile uint8_t button_event = 0;     // 1 cuando hay un press válido (lo pone la ISR)
volatile char    uart_event_char = 0;  // último byte recibido por IRQ (lo pone la ISR)

// --- Heartbeat y timeout del LED PA5 ---
static volatile uint8_t  led_forced_on = 0;   // 1 mientras el LED está forzado encendido (botón)
static volatile uint32_t led_on_time   = 0;   // ms cuando se encendió por última vez
static volatile uint32_t hb_prev_ms    = 0;   // timestamp último parpadeo
static volatile uint8_t  hb_state      = 0;   // estado actual del heartbeat (0/1)

volatile uint32_t ms_counter = 17;

// (Se mantienen aunque no se usen con la ISR, para no romper nada del profe)
static char rx_buffer[256];
static uint8_t rx_index = 0;
static uint8_t  pwm_dc = 0;           // duty actual (0..100)
static int8_t   pwm_dir = 1;          // +1 sube, -1 baja
static uint32_t t_pwm = 0;            // timestamp en ms

// ---- Inicialización de periféricos (adaptado a tus funciones) ----
static void peripherals_init(void)
{
    rcc_init();

    // GPIOs
    init_gpio(GPIOA, 5, 0x01, 0x00, 0x01, 0x00, 0x00);  // LED PA5
    init_gpio(GPIOC, 13, 0x00, 0x00, 0x01, 0x01, 0x00); // Botón PC13

    // Systick + UART
    init_systick();
    init_gpio_uart();
    init_uart();

    // NVIC/EXTI
    nvic_exti_pc13_button_enable();
    nvic_usart2_irq_enable();
    NVIC->IP[EXTI15_10_IRQn] = (1U << 4);  // Botón: prioridad alta
    NVIC->IP[USART2_IRQn]    = (3U << 4);  // UART: prioridad baja

    // PWM TIM3_CH1 (PA6)
    tim3_ch1_pwm_init(PWM_FREQUENCY);
}

// --- Programa principal ------------------------------------------------------
int main(void)
{
    peripherals_init();

    // PWM arranca en 0%
    pwm_dc = 0;
    tim3_ch1_pwm_set_duty_cycle(pwm_dc);
    t_pwm = ms_counter;

    // ROOM CONTROL (parser y estados)
    room_control_init();
    uart_send_string("Comando: PWM <0..100>\r\n");
    room_control_app_init();
    uart_send_string("Sistema de Control de Sala Inicializado!\r\n");

    while (1) {

        // === Procesa eventos (estilo guía 10) ===
        if (button_event) {
            button_event = 0;

            // Heartbeat: forzar LED fijo 3 s al presionar el botón
            led_forced_on = 1;
            led_on_time   = ms_counter;
            set_gpio(GPIOA, 5);

            // Notificar a la app (máquina de estados del profe)
            room_control_on_button_press();
        }

        if (uart_event_char) {
            char c = uart_event_char;
            uart_event_char = 0;
            room_control_on_uart_receive(c);
        }

        // Tareas periódicas de la app (timeout a IDLE, etc.)
        room_control_update();

        // (Opcional) aquí podrías poner otras tareas de fondo
    }
}

// --- Manejador de la interrupción SysTick -----------------------------------
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
        led_forced_on = 0;              // libera control; heartbeat retoma solo
        clear_gpio(GPIOA, 5);
    }
}
