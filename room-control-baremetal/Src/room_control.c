#include "room_control.h"
#include "uart.h"
#include "tim.h"

static char cmd[32];
static uint8_t idx;

void room_control_init(void) {
    idx = 0;
    uart_send_string("Comando: PWM <0..100>\r\n");
}

static void apply_command(const char* s) {
    if (s[0]=='P' && s[1]=='W' && s[2]=='M') {
        int val = -1;
        for (int i = 3; s[i]; i++) {
            if (s[i] >= '0' && s[i] <= '9') {
                val = 0;
                for (; s[i]>='0' && s[i]<='9'; i++) val = val*10 + (s[i]-'0');
                break;
            }
        }
        if (val >= 0 && val <= 100) {
            tim3_ch1_pwm_set_duty_cycle((uint8_t)val);
            uart_send_string("OK PWM\r\n");
        } else {
            uart_send_string("ERR rango (0..100)\r\n");
        }
        return;
    }
    uart_send_string("CMD?\r\n");
}

void room_control_on_rx_char(char c) {
    if (c=='\r' || c=='\n') {
        if (idx>0) { cmd[idx]=0; apply_command(cmd); idx=0; }
        return;
    }
    if (idx < sizeof(cmd)-1) cmd[idx++] = c; else idx = 0;
}
