#include "led_driver.h"  
// -----------------------------------------------------------------------------
// Se incluye el archivo de cabecera "led_driver.h"
// Este archivo contiene la definición de la estructura 'led_handle_t'
// (que almacena el puerto y el pin del LED) y las declaraciones de las funciones.
// -----------------------------------------------------------------------------


void led_init(led_handle_t *led) {
    // Inicializa el LED en estado apagado (pin a 0).
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}


void led_on(led_handle_t *led) {
    // Pone el pin del LED en nivel alto (enciende).
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
}


void led_off(led_handle_t *led) {
    // Pone el pin del LED en nivel bajo (apaga).
    HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
}


void led_toggle(led_handle_t *led) {
    // Invierte el estado lógico del pin del LED (toggle).
    HAL_GPIO_TogglePin(led->port, led->pin);
}
