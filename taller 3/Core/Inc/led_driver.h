#ifndef LED_DRIVER_H                     // Evita inclusión múltiple del mismo archivo durante la compilación
#define LED_DRIVER_H

#include <stdint.h>                      // Incluye tipos enteros estándar (uint16_t, uint32_t, etc.)
#include "main.h"                        // Incluye definiciones del proyecto (HAL, GPIO_TypeDef, pines configurados)

/* -------------------------------------------------------------------------
 * Definición de tipos de datos públicos
 * -------------------------------------------------------------------------
 * 'led_handle_t' es una estructura que agrupa la información necesaria para 
 * controlar un LED desde el código de aplicación.
 * Contiene el puerto GPIO y el número de pin asociado al LED.
 * ------------------------------------------------------------------------- */
typedef struct {
    GPIO_TypeDef *port;                  // Puntero al puerto GPIO (por ejemplo: GPIOA, GPIOB, GPIOC)
    uint16_t pin;                        // Número de pin (por ejemplo: GPIO_PIN_5)
} led_handle_t;

/* -------------------------------------------------------------------------
 * API pública (funciones accesibles desde otros módulos)
 * -------------------------------------------------------------------------
 * Cada función recibe un puntero a 'led_handle_t' para saber qué LED controlar.
 * Estas funciones están implementadas en 'led_driver.c'
 * ------------------------------------------------------------------------- */

// Inicializa el LED apagándolo al inicio (pone el pin en nivel bajo)
void led_init(led_handle_t *led);

// Enciende el LED (pone el pin en nivel alto)
void led_on(led_handle_t *led);

// Apaga el LED (pone el pin en nivel bajo)
void led_off(led_handle_t *led);

// Cambia el estado del LED (si está encendido lo apaga y viceversa)
void led_toggle(led_handle_t *led);

#endif // LED_DRIVER_H                   // Marca el final de la protección de inclusión múltiple
