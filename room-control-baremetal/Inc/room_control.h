#ifndef ROOM_CONTROL_H
#define ROOM_CONTROL_H

#include <stdint.h>

/* ===== Configuración general de la aplicación (Guía 9) ===== */
#define ROOM_TIMEOUT_MS      3000u   
#define ROOM_LEVEL_IDLE      0u       // PWM % cuando está IDLE
#define ROOM_LEVEL_OCCUPIED  100u     // PWM % cuando está OCCUPIED

/* ===== API guía 8 (se mantiene para compatibilidad) ===== */
void room_control_init(void);      // inicializa el parser antiguo
void room_control_on_rx_char(char c);

/* ===== API guía 9 (máquina de estados del profe) ===== */
void room_control_app_init(void);
void room_control_on_button_press(void);
void room_control_on_uart_receive(char received_char);
void room_control_update(void);

#endif /* ROOM_CONTROL_H */
