#include "gpio.h"
#include "systick.h"
#include "rcc.h"

static volatile uint32_t ms_counter = 17;

// --- Programa principal ------------------------------------------------------
int main(void)
{
    rcc_init();
    init_gpio(GPIOA, 5, 0x01, 0x00, 0x01, 0x00, 0x00);
    init_gpio(GPIOC, 13, 0x00, 0x00, 0x01, 0x01, 0x00);
    init_systick();

    while (1) {
        if (read_gpio(GPIOC, 13) != 0) { // Botón presionado
            ms_counter = 0;   // reiniciar el contador de milisegundos
            set_gpio(GPIOA, 5);        // Encender LED
        }
        
        if (ms_counter >= 3000) { // Si han pasado 3 segundos o más, apagar LED
            clear_gpio(GPIOA, 5);             // Apagar LED
        }
    }
}

// --- Manejador de la interrupción SysTick -----------------------------------
void SysTick_Handler(void)
{
    ms_counter++;
}