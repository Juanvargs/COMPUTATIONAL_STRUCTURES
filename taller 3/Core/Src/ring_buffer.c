// ring_buffer.c
#include "ring_buffer.h"   // Incluye la definición de ring_buffer_t y los prototipos de funciones

/*
 * Implementación de un buffer circular (ring buffer)
 * con bandera de llenado (is_full).
 * 
 * Características:
 * - Detecta fácilmente si el buffer está lleno o vacío.
 * - Cuando se llena, sobrescribe los datos más antiguos.
 */

/* -------------------------------------------------------------------------
 * Inicialización del buffer
 * ------------------------------------------------------------------------- */
void ring_buffer_init(ring_buffer_t *rb, uint8_t *buffer, uint16_t capacity)
{
    rb->buffer   = buffer;      // Dirección del arreglo físico
    rb->capacity = capacity;    // Capacidad total (nº de elementos)
    rb->head     = 0;           // Índice de escritura
    rb->tail     = 0;           // Índice de lectura
    rb->is_full  = false;       // Bandera: buffer vacío al inicio
}

/* -------------------------------------------------------------------------
 * Escritura: guarda un dato nuevo y sobrescribe el más viejo si está lleno
 * ------------------------------------------------------------------------- */
bool ring_buffer_write(ring_buffer_t *rb, uint8_t data)
{
    rb->buffer[rb->head] = data;                       // Guardar dato en head

    // Avanzar head (circular)
    rb->head = (uint16_t)((rb->head + 1u) % rb->capacity);

    // Si el buffer ya estaba lleno, mover tail para descartar el elemento más antiguo
    if (rb->is_full) {
        rb->tail = (uint16_t)((rb->tail + 1u) % rb->capacity);
    }

    // Actualizar flag: si head alcanzó tail, ahora está lleno
    rb->is_full = (rb->head == rb->tail);

    return true;  // Escritura siempre exitosa (sobrescribe si necesario)
}

/* -------------------------------------------------------------------------
 * Lectura: extrae un dato del buffer si hay disponible
 * ------------------------------------------------------------------------- */
bool ring_buffer_read(ring_buffer_t *rb, uint8_t *data)
{
    // Si no está lleno y los índices coinciden, el buffer está vacío
    if (!rb->is_full && (rb->head == rb->tail)) {
        return false;  // No hay nada que leer
    }

    *data = rb->buffer[rb->tail];  // Leer el elemento más antiguo (tail)

    // Tras leer, seguro que no está lleno
    rb->is_full = false;

    // Avanzar tail (circular) para la siguiente lectura
    rb->tail = (uint16_t)((rb->tail + 1u) % rb->capacity);

    return true;  // Lectura correcta
}

/* -------------------------------------------------------------------------
 * Cantidad de elementos almacenados actualmente
 * ------------------------------------------------------------------------- */
uint16_t ring_buffer_count(ring_buffer_t *rb)
{
    uint16_t count;

    if (rb->is_full) {
        count = rb->capacity;  // Lleno → count = capacidad
    } else if (rb->head >= rb->tail) {
        count = rb->head - rb->tail;  // Head adelantado a tail
    } else {
        count = rb->capacity + rb->head - rb->tail;  // Wrap-around: head volvió al inicio
    }

    return count;
}

/* -------------------------------------------------------------------------
 * Estado: devuelve true si el buffer está vacío
 * ------------------------------------------------------------------------- */
bool ring_buffer_is_empty(ring_buffer_t *rb)
{
    // Está vacío si no está lleno y los punteros coinciden
    return (!rb->is_full && (rb->head == rb->tail));
}

/* -------------------------------------------------------------------------
 * Estado: devuelve true si el buffer está lleno
 * ------------------------------------------------------------------------- */
bool ring_buffer_is_full(ring_buffer_t *rb)
{
    return rb->is_full;  // Simplemente devuelve el valor de la bandera
}

/* -------------------------------------------------------------------------
 * Limpieza total del buffer
 * ------------------------------------------------------------------------- */
void ring_buffer_flush(ring_buffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->is_full = false;  // Restablece el estado del buffer a vacío
}
