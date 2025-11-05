#ifndef KEYPAD_DRIVER_H
#define KEYPAD_DRIVER_H

#include "main.h"        // Trae tipos GPIO_TypeDef*, defines de pines creados por CubeMX
#include <stdint.h>      // Tipos enteros fijos (uint8_t, etc.)

// Dimensiones del teclado matricial 4x4
#define KEYPAD_ROWS 4
#define KEYPAD_COLS 4

// Estructura/handle del teclado:
// - row_ports/pins: puertos y pines de las 4 filas (configuradas como SALIDAS)
// - col_ports/pins: puertos y pines de las 4 columnas (ENTRADAS con EXTI + Pull-up)
typedef struct {
    GPIO_TypeDef* row_ports[KEYPAD_ROWS];
    uint16_t      row_pins[KEYPAD_ROWS];
    GPIO_TypeDef* col_ports[KEYPAD_COLS];
    uint16_t      col_pins[KEYPAD_COLS];
} keypad_handle_t;

// Inicializa el teclado (pone filas en estado "reposo" y columnas listas para detectar)
void keypad_init(keypad_handle_t* keypad);

// Escanea cuál tecla corresponde a la columna que disparó la EXTI.
// Recibe el pin de columna (GPIO_Pin) que llegó al callback.
// Devuelve el carácter presionado ('0'..'9','A'..'D','*','#') o '\0' si no detecta válido.
char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin);

#endif // KEYPAD_DRIVER_H
