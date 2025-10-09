#ifndef ROOM_CONTROL_H
#define ROOM_CONTROL_H

#include <stdint.h>

void room_control_init(void);
void room_control_on_rx_char(char c);  // Llamar desde la ISR de USART

#endif
