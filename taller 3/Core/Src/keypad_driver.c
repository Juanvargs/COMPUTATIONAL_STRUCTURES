/* ============================================================
 * keypad_driver.c  
 * Teclado matricial 4x4 con detección por EXTI en columnas.
 * - keypad_init(): pone TODAS las filas en BAJO (reposo del modelo)
 * - keypad_scan(): tras EXTI de columna, identifica la tecla y la devuelve
 *   usando un pequeño debounce por HAL_Delay(50) y espera a la suelta.
 * ============================================================ */

#include "keypad_driver.h"   // Tipos, prototipos y defines (KEYPAD_ROWS/COLS)
#include "stm32l4xx_hal.h"   // HAL GPIO API

/* Mapa de caracteres: [fila][columna] */
// Mapa de teclas [fila][columna]
static const char keypad_map[KEYPAD_ROWS][KEYPAD_COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}
};

/**
 * @brief Inicializa el estado del teclado.
 * Filas como SALIDA ya configuradas por CubeMX → aquí solo las ponemos en BAJO.
 * Columnas deben estar como ENTRADA con PULL-UP + EXTI por flanco de BAJADA.
 */
void keypad_init(keypad_handle_t* keypad)
{
    if (keypad == NULL) {
        return; // nada que inicializar
    }

    /* Pone TODAS las filas en ALTO (reposo):
     * - En este proyecto las columnas tienen PULL-UP y el escaneo
     *   se realiza poniendo una fila a LOW para comprobar la columna.
     * - MX_GPIO_Init() ya configura las filas en ALTO, pero aquí
     *   nos aseguramos del estado de reposo cuando se llama a init.
     */
    for (int i = 0; i < KEYPAD_ROWS; i++) {
        // Asegurar nivel ALTO en cada fila (estado reposo)
        HAL_GPIO_WritePin(keypad->row_ports[i], keypad->row_pins[i], GPIO_PIN_SET);
    }
}

/**
 * @brief Escanea la tecla presionada.
 * Llamar desde HAL_GPIO_EXTI_Callback pasando el pin de columna que disparó.
 * @param keypad   Manejador del teclado (puertos/pines)
 * @param col_pin  Pin de la columna que generó la EXTI (C1..C4)
 * @return Caracter de la tecla ('0'..'9','A'..'D','*','#') o '\0' si no se detecta.
 */
char keypad_scan(keypad_handle_t* keypad, uint16_t col_pin)
{
    char key_pressed = '\0';   // Por defecto, no se detecta nada
    int  col_index   = -1;     // Índice de columna correspondiente a col_pin

    if (keypad == NULL) {
        return '\0';
    }

    /* 1) Debounce simple: dejar asentar el contacto */
    HAL_Delay(50); // breve espera para estabilizar la señal

    /* 2) Encontrar qué columna fue la que disparó (por su pin) */
    for (int c = 0; c < KEYPAD_COLS; c++) {
        if (keypad->col_pins[c] == col_pin) {
            col_index = c;
            break;
        }
    }
    if (col_index < 0) {
        return '\0';           // El pin no corresponde a ninguna columna conocida
    }

    /* 3) Búsqueda de la FILA
     *    - Primero subimos TODAS las filas a ALTO.
     *    - Luego bajamos UNA fila a BAJO y leemos la columna activa.
     *    - Si esa columna lee BAJO, esa es la fila correcta.
     */

    // Todas las filas a ALTO
    for (int r = 0; r < KEYPAD_ROWS; r++) {
        HAL_GPIO_WritePin(keypad->row_ports[r], keypad->row_pins[r], GPIO_PIN_SET); // reposo
    }

    // Prueba fila por fila
    for (int row = 0; row < KEYPAD_ROWS; row++) {

        // Poner SOLO esta fila en BAJO, las demás quedan en ALTO
        HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_RESET); // activar fila

        // Leer la columna que disparó
        if (HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]) == GPIO_PIN_RESET) {
            // Tecla identificada por (row, col_index)
            key_pressed = keypad_map[row][col_index];

            // Esperar a que se SUELTE (vuelva a ALTO) para evitar múltiples lecturas
            while (HAL_GPIO_ReadPin(keypad->col_ports[col_index], keypad->col_pins[col_index]) == GPIO_PIN_RESET) {
                /* espera activa hasta suelta (bloqueante pero breve) */
            }

            // Salimos: ya encontramos la tecla
            break;
        }

        // Restaurar esta fila a ALTO antes de probar la siguiente
        HAL_GPIO_WritePin(keypad->row_ports[row], keypad->row_pins[row], GPIO_PIN_SET);
    }

    /* 4) Restaurar estado de filas según el modelo (todas en BAJO) */
    keypad_init(keypad);

    /* 5) Devolver tecla (o '\0' si no se halló) */
    return key_pressed;
}
