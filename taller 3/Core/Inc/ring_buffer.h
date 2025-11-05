#ifndef RING_BUFFER_H                           // Guardia de inclusión: evita incluir el header más de una vez
#define RING_BUFFER_H

#include <stdint.h>                              // Tipos enteros de tamaño fijo (uint8_t, uint16_t)
#include <stdbool.h>                             // Tipo bool (true/false)

/* Estructura principal del ring buffer (versión con bandera):
 * - buffer   : puntero al arreglo de almacenamiento (lo pone el usuario)
 * - head     : índice de ESCRITURA (dónde se va a guardar el próximo byte)
 * - tail     : índice de LECTURA (de dónde se va a sacar el próximo byte)
 * - capacity : tamaño TOTAL del buffer (en bytes)
 * - is_full  : bandera que indica si el buffer está lleno en este momento
 *
 * Con esta bandera ya no dependemos solo de la comparación head == tail
 * para saber si está vacío o lleno.
 */
typedef struct {
    uint8_t  *buffer;                            // Memoria donde se guardan los datos
    uint16_t  head;                              // Puntero de escritura
    uint16_t  tail;                              // Puntero de lectura
    uint16_t  capacity;                          // Capacidad total del buffer
    bool      is_full;                           // true → buffer lleno, false → no lleno
} ring_buffer_t;

/* Inicializa el ring buffer con un arreglo y capacidad dados */
void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t capacity);
/* 
 * - rb       : estructura del ring buffer a inicializar
 * - buffer   : arreglo externo donde se guardarán los datos
 * - capacity : tamaño del arreglo
 * No reserva memoria dinámica.
 */

/* Escribe un byte en el buffer.
 * En esta versión SIEMPRE escribe.
 * Si está lleno, descarta el dato más viejo (avanzando tail).
 */
bool ring_buffer_write(ring_buffer_t *rb, uint8_t data);

/* Lee un byte del buffer en *data; devuelve false si está vacío */
bool ring_buffer_read(ring_buffer_t *rb, uint8_t *data);

/* Retorna la cantidad de bytes actualmente almacenados */
uint16_t ring_buffer_count(ring_buffer_t *rb);

/* Indica si el buffer está vacío (sin datos por leer) */
bool ring_buffer_is_empty(ring_buffer_t *rb);

/* Indica si el buffer está lleno (no hay espacio libre, el próximo write sobreescribe) */
bool ring_buffer_is_full(ring_buffer_t *rb);

/* Vacía el buffer (descarta el contenido pendiente) */
void ring_buffer_flush(ring_buffer_t *rb);

#endif // RING_BUFFER_H
