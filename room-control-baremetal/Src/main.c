#include "gpio.h"
#include "systick.h"
#include "rcc.h"
#include "uart.h"  // Agregar esta línea

static volatile uint32_t ms_counter = 17;
static char rx_buffer[256];
static uint8_t rx_index = 0;

// --- Programa principal ------------------------------------------------------
int main(void)
{
    rcc_init();
    init_gpio(GPIOA, 5, 0x01, 0x00, 0x01, 0x00, 0x00);
    init_gpio(GPIOC, 13, 0x00, 0x00, 0x01, 0x01, 0x00);
    init_systick();
    init_gpio_uart();  // Agregar inicialización GPIO para UART
    init_uart();       // Agregar inicialización UART
    
    uart_send_string("Sistema Inicializado!\r\n");  // Enviar mensaje de inicio
    
    while (1) {
        if (read_gpio(GPIOC, 13) != 0) { // Botón presionado
            ms_counter = 0;   // reiniciar el contador de milisegundos
            set_gpio(GPIOA, 5);        // Encender LED
        }
        
        if (ms_counter >= 3000) { // Si han pasado 3 segundos o más, apagar LED
            clear_gpio(GPIOA, 5);             // Apagar LED
        }

        // Polling UART receive
        if (USART2->ISR & (1 << 5)) {  // RXNE
            char c = (char)(USART2->RDR & 0xFF);
            if (rx_index < sizeof(rx_buffer) - 1) {
                rx_buffer[rx_index++] = c;
                if (c == '\r' || c == '\n') {
                    rx_buffer[rx_index] = '\0';  // Null terminate
                    uart_send_string("Recibido: ");
                    uart_send_string(rx_buffer);
                    uart_send_string("\r\n");
                    rx_index = 0;
                }
            }
        }
    }
}

// --- Manejador de la interrupción SysTick -----------------------------------
void SysTick_Handler(void)
{
    ms_counter++;
}