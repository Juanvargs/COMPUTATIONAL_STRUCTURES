#include "room_control.h"
#include "gpio.h"     // opcional si usas LEDs por GPIO aparte del PWM
#include "systick.h"
#include "uart.h"
#include "tim.h"
#include <stdint.h>

/* === Toma del tick global (de main.c) === */
extern volatile uint32_t ms_counter;

/* === Marca de tiempo del último evento === */
static uint32_t last_activity_ms = 0;

/* ========= GUÍA 9: estados (del profe) ========= */
typedef enum {
    ROOM_IDLE,
    ROOM_OCCUPIED
} room_state_t;

static room_state_t current_state = ROOM_IDLE;

/* ========= GUÍA 8: parser de línea "PWM <n>" ========= */
static char cmd[32];
static uint8_t idx;

/* ========= Utilidades internas ========= */
static inline void apply_pwm_percent(uint32_t pct) {
    if (pct > 100u) pct = 100u;
    tim3_ch1_pwm_set_duty_cycle((uint8_t)pct);
}

/* ========= API guía 8 (compatibilidad) ========= */
void room_control_init(void) {
    idx = 0;
    uart_send_string("Comando: PWM <0..100>\r\n");
}

static void apply_command(const char* s) {
    // Saltar espacios iniciales
    while (*s == ' ' || *s == '\t') s++;

    // Aceptar "PWM" en mayúsc/minúsc y con espacios variables: "PWM 25", "pwm    75"
    if ((s[0]=='P' || s[0]=='p') &&
        (s[1]=='W' || s[1]=='w') &&
        (s[2]=='M' || s[2]=='m')) {

        // Saltar hasta el primer dígito
        int i = 3;
        while (s[i] == ' ' || s[i] == '\t') i++;

        int val = -1;
        if (s[i] >= '0' && s[i] <= '9') {
            val = 0;
            for (; s[i]>='0' && s[i]<='9'; i++) {
                val = val*10 + (s[i]-'0');
            }
        }

        if (val >= 0 && val <= 100) {
            apply_pwm_percent((uint32_t)val);
            uart_send_string("OK PWM\r\n");
            last_activity_ms = ms_counter;
        } else {
            uart_send_string("ERR rango (0..100)\r\n");
        }
        return;
    }

    uart_send_string("CMD?\r\n");
}


void room_control_on_rx_char(char c) {
    // Parser de línea (guía 8). Lo dejamos por compatibilidad.
    if (c=='\r' || c=='\n') {
        if (idx>0) { cmd[idx]=0; apply_command(cmd); idx=0; }
        return;
    }
    if (idx < sizeof(cmd)-1) cmd[idx++] = c; else idx = 0;
}

/* ========= API guía 9 (del profe) ========= */

void room_control_app_init(void)
{
    current_state = ROOM_IDLE;
    apply_pwm_percent(0u);
    uart_send_string("ROOM: IDLE\r\n");
    last_activity_ms = ms_counter;
}

void room_control_on_button_press(void)
{
    // Toggle de estado entre IDLE y OCCUPIED
    if (current_state == ROOM_IDLE) {
        current_state = ROOM_OCCUPIED;
        apply_pwm_percent(100u);
        uart_send_string("ROOM -> OCCUPIED (button)\r\n");
    } else {
        current_state = ROOM_IDLE;
        apply_pwm_percent(0u);
        uart_send_string("ROOM -> IDLE (button)\r\n");
    }
    last_activity_ms = ms_counter;
}

void room_control_on_uart_receive(char received_char)
{
    switch (received_char) {
        case 'h':
        case 'H':
            apply_pwm_percent(100u);
            uart_send_string("OK h (100%)\r\n");
            last_activity_ms = ms_counter;
            return;

        case 'l':
        case 'L':
            apply_pwm_percent(0u);
            uart_send_string("OK l (0%)\r\n");
            last_activity_ms = ms_counter;
            return;

        case 'O':
        case 'o':
            current_state = ROOM_OCCUPIED;
            apply_pwm_percent(100u);
            uart_send_string("ROOM cmd: OCCUPIED\r\n");
            last_activity_ms = ms_counter;
            return;

        case 'I':
        case 'i':
            current_state = ROOM_IDLE;
            apply_pwm_percent(0u);
            uart_send_string("ROOM cmd: IDLE\r\n");
            last_activity_ms = ms_counter;
            return;

        default:
            // ===== Parser de línea interno para "PWM <n>" =====
            // Acumula todo (letras, espacios, dígitos) hasta EOL y procesa aquí mismo
            if (received_char == '\r' || received_char == '\n') {
                if (idx > 0) {
                    cmd[idx] = 0;
                    apply_command(cmd);   // esto imprime "OK PWM" o "ERR rango"
                    idx = 0;
                    last_activity_ms = ms_counter;
                }
            } else {
                if (idx < sizeof(cmd) - 1) {
                    cmd[idx++] = received_char;
                } else {
                    idx = 0; // overflow simple
                }
            }
            return;
    }
}



void room_control_update(void)
{
    // Timeout a IDLE cuando está en OCCUPIED
    if (current_state == ROOM_OCCUPIED) {
        if ((uint32_t)(ms_counter - last_activity_ms) >= ROOM_TIMEOUT_MS) {
            current_state = ROOM_IDLE;
            apply_pwm_percent(ROOM_LEVEL_IDLE);
            uart_send_string("ROOM timeout -> IDLE\r\n");
        }
    }
}
