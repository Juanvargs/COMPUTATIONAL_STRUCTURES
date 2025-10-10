#include "room_control.h"
#include "gpio.h"     // opcional si usas LEDs por GPIO aparte del PWM
#include "systick.h"
#include "uart.h"
#include "tim.h"
#include <stdint.h>

static uint8_t b_pending = 0;   // 1 = esperando dígito después de 'B'

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
    // --- Trim de espacios iniciales ---
    while (*s == ' ' || *s == '\t') s++;

    // ===== Caso 1: línea de 1 dígito ("5" -> 50%)
    if (s[0] >= '0' && s[0] <= '9' && (s[1] == '\0' || s[1] == '\r' || s[1] == '\n' || s[1] == ' ' || s[1] == '\t')) {
        uint32_t pct = (uint32_t)(s[0] - '0') * 10u;
        apply_pwm_percent(pct);
        uart_send_string("OK digit\r\n");
        last_activity_ms = ms_counter;
        return;
    }

    // ===== Caso 2: "B<d>" o "b<d>"  (B7 -> 70%)
    if ((s[0] == 'B' || s[0] == 'b') && (s[1] >= '0' && s[1] <= '9')) {
        // Acepta espacios después del dígito también
        if (s[2] == '\0' || s[2] == '\r' || s[2] == '\n' || s[2] == ' ' || s[2] == '\t') {
            uint32_t pct = (uint32_t)(s[1] - '0') * 10u;
            apply_pwm_percent(pct);
            uart_send_string("OK Bx\r\n");
            last_activity_ms = ms_counter;
            return;
        }
    }

    // ===== Caso 3: "PWM <n>" (mayus/minus, con o sin espacios)
    if ((s[0]=='P' || s[0]=='p') &&
        (s[1]=='W' || s[1]=='w') &&
        (s[2]=='M' || s[2]=='m')) {

        // Saltar espacios tras "PWM"
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

    // ===== Nada coincidió
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
    // Si veníamos de 'B', esperamos un dígito 0..9
    if (b_pending) {
        // descartar cualquier residuo previo de línea
        idx = 0;

        if (received_char >= '0' && received_char <= '9') {
            uint32_t pct = (uint32_t)(received_char - '0') * 10u;
            apply_pwm_percent(pct);
            uart_send_string("OK Bx\r\n");
            last_activity_ms = ms_counter;
        } else {
            uart_send_string("ERR Bx\r\n");
        }
        b_pending = 0;
        return; // *** importante: no sigas al parser ***
    }

    switch (received_char) {
        case 'h':
        case 'H':
            idx = 0; // limpia buffer de línea
            apply_pwm_percent(100u);
            uart_send_string("OK h (100%)\r\n");
            last_activity_ms = ms_counter;
            return;

        case 'l':
        case 'L':
            idx = 0;
            apply_pwm_percent(0u);
            uart_send_string("OK l (0%)\r\n");
            last_activity_ms = ms_counter;
            return;

        case 'O':
        case 'o':
            idx = 0;
            current_state = ROOM_OCCUPIED;
            apply_pwm_percent(100u);
            uart_send_string("ROOM cmd: OCCUPIED\r\n");
            last_activity_ms = ms_counter;
            return;

        case 'I':
        case 'i':
            idx = 0;
            current_state = ROOM_IDLE;
            apply_pwm_percent(0u);
            uart_send_string("ROOM cmd: IDLE\r\n");
            last_activity_ms = ms_counter;
            return;

        case 'B':
        case 'b':
            // vamos a esperar el siguiente char (debe ser 0..9)
            idx = 0;        // evita que 'B' quede en el buffer de línea
            b_pending = 1;  // activa modo "esperando dígito"
            return;

        default:
            // Atajo: dígito solo = duty 0..90% en pasos de 10
            // Solo si NO estamos escribiendo otra línea (idx == 0) y no hay 'B' pendiente
            if (idx == 0 && b_pending == 0 &&
                received_char >= '0' && received_char <= '9') {
                uint32_t pct = (uint32_t)(received_char - '0') * 10u;
                apply_pwm_percent(pct);
                uart_send_string("OK digit\r\n");
                last_activity_ms = ms_counter;
                return; // *** importante: no sigas al parser ***
            }

            // Mantener el parser "PWM <n>" (línea completa) para todo lo demás
            if (received_char == '\r' || received_char == '\n') {
                if (idx > 0) {
                    cmd[idx] = 0;
                    apply_command(cmd);   // imprime "OK PWM" o "ERR rango"
                    idx = 0;
                    last_activity_ms = ms_counter;
                }
            } else {
                if (idx < sizeof(cmd) - 1) {
                    cmd[idx++] = received_char; // acumula para "PWM 25"
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
