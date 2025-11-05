/* USER CODE BEGIN Header */                         // --- Sección de encabezado reservada para el usuario (comentarios/metadatos)
/**
  ******************************************************************************
  * @file           : main.c                        // Nombre del archivo fuente principal
  * @brief          : Main program body             // Breve descripción del contenido del archivo
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.          // Derechos de autor
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file  // Referencia a la licencia del software
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.           // Condición en ausencia de licencia
  *
  ******************************************************************************
  */
/* USER CODE END Header */                           // --- Fin de la sección de encabezado del usuario

/* Includes ------------------------------------------------------------------*/
#include "main.h"                                     // Definiciones/prototipos generados por CubeMX (HAL, pines, handlers)

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led_driver.h"                               // Driver del LED (led_handle_t y funciones)
#include "ring_buffer.h"                              // Librería del buffer circular
#include "keypad_driver.h"                            // Driver de keypad 4x4
#include <stdio.h>                                    // printf/puts (si se redirige) 
#include <string.h>                                   // strncmp para comparar la clave
/* USER CODE END Includes */

/* ---------------------------------------------------------------------------
 * Resumen general (comentarios añadidos por el asistente):
 * - Este archivo contiene la inicialización de periféricos y el bucle principal
 *   de la aplicación. La lógica implementada gestiona:
 *     1) Lectura y escaneo de un teclado matricial 4x4 (keypad).
 *     2) Almacenamiento de teclas en un ring buffer y verificación de una
 *        contraseña de 4 dígitos ("2580").
 *     3) Control de dos LEDs: uno interno (LD2, PA5) y otro "door" (PA7).
 *     4) Recepción UART vía interrupciones en un ring buffer y eco en bloques.
 * - He añadido este bloque de documentación y comentarios explicativos.
 * - No se modifica la lógica existente; solo se añaden comentarios y aclaraciones
 *   para facilitar lectura y mantenimiento.
 * ---------------------------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */                                // No se añaden typedefs privados adicionales

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */                                 // No se añaden #define privados

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */                                 // No se añaden macros privadas

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;                            // Control HAL para USART2 (config/estado)

/* USER CODE BEGIN PV */

// LEDs: LD2 (interno) y LED de puerta (externo)
led_handle_t led1 = {GPIOA, GPIO_PIN_5 };             // LD2 PA5
led_handle_t led_door = (led_handle_t){ GPIOA, GPIO_PIN_7 }; // Door LED PA7

/* UART RX: ring buffer para recepción por IRQ (eco cada 5 bytes) */
#define UART2_RX_LEN 16
uint8_t uart2_rx_buffer[UART2_RX_LEN];                 // Memoria del ring buffer UART
ring_buffer_t uart2_rx_rb;                             // Estructura del ring buffer
uint8_t uart2_rx_data;                                 // Byte temporal recibido

/* Variable que almacena el momento exacto (en milisegundos del sistema)
 * en que el LED debe apagarse.
 * Cuando se presiona el botón azul (PC13), se calcula este tiempo como:
 * tiempo_actual + 5000 ms.
 * El while principal se encargará de apagar el LED cuando se cumpla este plazo.
 */
volatile uint32_t door_led_deadline_ms = 0;            // Tiempo (ms) para apagar LED puerta; 0 = none

/* Variable para controlar el "rebote" del botón (debounce).
 * Cuando se presiona el botón, este no genera un solo pulso limpio,
 * sino varios rebotes eléctricos. Para evitar falsos disparos,
 * se registra el último tiempo en el que se presionó.
 * Si se vuelve a presionar antes de 150 ms, se ignora.
 */
// Anti-rebote y control de parpadeo
volatile uint32_t last_button_ms = 0;                  // Tiempo del último pulso (ms)
volatile uint8_t led1_suppress_toggle = 0;             // 1 = suprimir toggle periódico de led1

/* ====== KEYPAD: handle y ring buffer propios ====== */
/* Mapea filas y columnas usando los alias que generó CubeMX
   (tal como en la guía del profe). Deben existir en main.h:
   KEYPAD_R1_Pin / KEYPAD_R1_GPIO_Port, ..., KEYPAD_C4_Pin / KEYPAD_C4_GPIO_Port */
keypad_handle_t keypad = {
    .row_ports = { KEYPAD_R1_GPIO_Port, KEYPAD_R2_GPIO_Port, KEYPAD_R3_GPIO_Port, KEYPAD_R4_GPIO_Port }, // Puertos filas
    .row_pins  = { KEYPAD_R1_Pin,        KEYPAD_R2_Pin,        KEYPAD_R3_Pin,        KEYPAD_R4_Pin        }, // Pines filas
    .col_ports = { KEYPAD_C1_GPIO_Port,  KEYPAD_C2_GPIO_Port,  KEYPAD_C3_GPIO_Port,  KEYPAD_C4_GPIO_Port  }, // Puertos columnas
    .col_pins  = { KEYPAD_C1_Pin,        KEYPAD_C2_Pin,        KEYPAD_C3_Pin,        KEYPAD_C4_Pin        }  // Pines columnas
};

// Ring buffer para teclas del keypad
#define KEYPAD_BUFFER_LEN 16
static uint8_t  keypad_buffer[KEYPAD_BUFFER_LEN];       // Memoria para teclas
ring_buffer_t   keypad_rb;                              // Estructura del ring buffer

// Mensaje base para enviar la tecla por UART
static uint8_t kp_msg[14] = "Tecla: ?\r\n";             // kp_msg[7] reemplaza '?'

/* ====== EXTI→main: pin pendiente del keypad ====== */
volatile uint16_t last_exti_pin = 0;                    // ISR guarda la columna que disparó (0 = none)
// last_exti_pin usado para comunicar ISR -> main loop

/* ====== PASSWORD (4 dígitos) ====== */
// Contraseña de 4 dígitos y buffer de entrada
#define PASSWORD_LEN 4
static const char PASSWORD[PASSWORD_LEN] = {'2','5','8','0'}; // Clave: 2 5 8 0
static char  input_buffer[PASSWORD_LEN];                // Buffer de ingreso
static uint8_t input_index = 0;                         // Posición actual

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);                           // Prototipo: configuración de reloj del sistema
static void MX_GPIO_Init(void);                          // Prototipo: configuración de GPIOs
static void MX_USART2_UART_Init(void);                   // Prototipo: configuración de USART2
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */                                  // No se añaden prototipos adicionales del usuario

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* El envío por UART se gestiona desde syscalls.c (implementación de __io_putchar),
   por tanto no es necesario redefinir _write aquí. */

/* Callback de recepción UART por interrupción:
 * - Rearma recepción de 1 byte
 * - Filtra '\r' y '\n'
 * - Escribe los demás bytes en el ring buffer UART RX
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  // Se llama automáticamente cuando llega 1 byte por UART
{
  if (huart->Instance == USART2) {                     // Evento de USART2
    // Rearmar recepción inmediato para no perder bytes
    HAL_UART_Receive_IT(&huart2, &uart2_rx_data, 1);

    // Filtrar retornos de carro/linefeed comunes del terminal
    if (uart2_rx_data == '\r' || uart2_rx_data == '\n') {
      return;                                      // Ignorar estos caracteres
    }

    // Encolar el byte recibido para procesarlo en el main loop
    ring_buffer_write(&uart2_rx_rb, uart2_rx_data);
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.                // Punto de entrada
  * @retval int                                         // En MCU no se usa el valor de retorno
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */                                  // Espacio para pre-inicializaciones del usuario

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();                                            // Inicializa HAL: reset periféricos, SysTick 1ms, NVIC base

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */                               // Espacio para inicializaciones del usuario tras HAL_Init()

  /* Configure the system clock */
  SystemClock_Config();                                  // Configura osciladores/PLL y relojes de buses

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */                            // Espacio para inicialización de bajo nivel del usuario

  /* Initialize all configured peripherals */
  MX_GPIO_Init();                                        // Inicializa GPIOs (relojes de puertos, modos de pines)
  MX_USART2_UART_Init();                                 // Inicializa USART2 (115200-8N1)
  /* USER CODE BEGIN 2 */
  led_init(&led1);                                       // Asegura LD2 (PA5) apagado al inicio
  ring_buffer_init(&uart2_rx_rb, uart2_rx_buffer, UART2_RX_LEN); // Inicializa ring buffer de UART RX
  HAL_UART_Receive_IT(&huart2, &uart2_rx_data, 1);               // Habilita recepción por interrupciones (1 byte)
  led_init(&led_door);                                   // Apaga LED_DOOR (PA7) al inicio del programa

  /* ====== KEYPAD: inicializaciones ====== */
  ring_buffer_init(&keypad_rb, keypad_buffer, KEYPAD_BUFFER_LEN); // Ring buffer de teclas
  keypad_init(&keypad);                                           // Filas en reposo (alto); columnas ya por CubeMX

  /* Mensaje de “sistema listo” por UART2 (en vez de printf) */
  {
      const uint8_t boot_msg[] = "Sistema listo. Clave 4 digitos: 2 5 8 0\r\n"; // Mensaje de bienvenida con clave
      HAL_UART_Transmit(&huart2, (uint8_t*)boot_msg, sizeof(boot_msg)-1, HAL_MAX_DELAY);    // Enviar por UART2
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {                                                          // Bucle principal: corre para siempre

    /* ====== 1) Procesar evento pendiente del keypad (fuera de la ISR) ====== */
    /* Active polling: recorrer filas, poner cada fila a LOW y leer columnas.
     * Esto detecta teclas aunque EXTI no esté funcionando. Cuando se
     * detecta una columna a LOW llamamos a keypad_scan() con el pin
     * correspondiente para reutilizar la lógica existente.
     */
    if (last_exti_pin == 0) {
      // Asegurar filas en estado reposo ALTO
      for (int r = 0; r < KEYPAD_ROWS; r++) {
        HAL_GPIO_WritePin(keypad.row_ports[r], keypad.row_pins[r], GPIO_PIN_SET);
      }

      // Probar cada fila
      for (int row = 0; row < KEYPAD_ROWS && last_exti_pin == 0; row++) {
        // Activar (LOW) solo la fila actual para detectar qué tecla está presionada
        HAL_GPIO_WritePin(keypad.row_ports[row], keypad.row_pins[row], GPIO_PIN_RESET);

        // Pequeña espera para que la línea se estabilice (debounce simple)
        HAL_Delay(5);

        // Leer columnas: si alguna lee LOW, la tecla en esta fila+col fue pulsada
        for (int c = 0; c < KEYPAD_COLS; c++) {
          if (HAL_GPIO_ReadPin(keypad.col_ports[c], keypad.col_pins[c]) == GPIO_PIN_RESET) {
            // Llamar al escaneo con el pin de columna detectado
            uint16_t detected_col_pin = keypad.col_pins[c];
            char key = keypad_scan(&keypad, detected_col_pin);
            if (key != '\0') {
              ring_buffer_write(&keypad_rb, (uint8_t)key);
            }

            // Restaurar la fila a reposo ALTO antes de continuar
            HAL_GPIO_WritePin(keypad.row_ports[row], keypad.row_pins[row], GPIO_PIN_SET);

            // Evitar lecturas repetidas: pequeña espera tras detección
            HAL_Delay(150);
            break;
          }
        }

        // Restaurar fila a ALTO antes de probar la siguiente
        HAL_GPIO_WritePin(keypad.row_ports[row], keypad.row_pins[row], GPIO_PIN_SET);
      }
    } else {
      // La ISR dejó una columna en last_exti_pin; usar el escaneo que ya existe
      char key = keypad_scan(&keypad, last_exti_pin);
      last_exti_pin = 0;                                             // Marcar atendido

      if (key != '\0') {                                            // Si keypad_scan encontró una tecla válida
          ring_buffer_write(&keypad_rb, (uint8_t)key);               // Encolar tecla
      }

      // Re-habilitar las IRQ de EXTI tras procesar el evento
      HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
      HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    }

    /* ====== 2) Consumir teclas del keypad y verificar PASSWORD ====== */
    {
        uint8_t k;                                                     // Byte local para leer tecla
        if (ring_buffer_read(&keypad_rb, &k)) {                      // Si hay tecla pendiente
          kp_msg[7] = (uint8_t)k;                                  // Poner tecla en el mensaje
          HAL_UART_Transmit(&huart2, kp_msg, sizeof(kp_msg)-1, HAL_MAX_DELAY);

          // Guardar en buffer de entrada si no está lleno
          if (input_index < PASSWORD_LEN) {
            input_buffer[input_index++] = (char)k;
          }

          // Si completamos 4 dígitos, verificar contraseña
          if (input_index == PASSWORD_LEN) {
            if (strncmp(input_buffer, PASSWORD, PASSWORD_LEN) == 0) {
              // Clave correcta: mostrar mensaje y encender LEDs
              const uint8_t ok_msg[] = "Clave CORRECTA (2580). LED ON.\r\n";
              HAL_UART_Transmit(&huart2, (uint8_t*)ok_msg, sizeof(ok_msg)-1, HAL_MAX_DELAY);
              led1_suppress_toggle = 1;      // Suprimir parpadeo automático
              led_on(&led1);                 // Encender LED interno
              led_on(&led_door);             // Encender LED puerta
              door_led_deadline_ms = HAL_GetTick() + 5000U; // Apagar puerta en 5s
            } else {
              // Clave incorrecta: informar y parpadear ambos LEDs 3 veces
              const uint8_t bad_msg[] = "Clave INCORRECTA. LED parpadea.\r\n";
              HAL_UART_Transmit(&huart2, (uint8_t*)bad_msg, sizeof(bad_msg)-1, HAL_MAX_DELAY);
              led1_suppress_toggle = 1;      // Suspender toggle durante el parpadeo
              for (int i = 0; i < 3; i++) {
                led_on(&led1);
                led_on(&led_door);
                HAL_Delay(150);
                led_off(&led1);
                led_off(&led_door);
                HAL_Delay(150);
              }
              led1_suppress_toggle = 0;      // Reanudar toggle periódico
            }
            input_index = 0; // Reiniciar buffer de entrada para la próxima clave
          }
        }
    }

    /* ====== 3) Apagado diferido del LED_DOOR (si venció plazo de 5 s) ====== */
    if (door_led_deadline_ms != 0U) {                                  // ¿Hay un apagado programado?
        if ((int32_t)(HAL_GetTick() - door_led_deadline_ms) >= 0) {    // ¿Se cumplió el tiempo?
            led_off(&led_door);                                        // Apaga LED externo (PA7)
            door_led_deadline_ms = 0U;                                 // Limpia alarma
        }
    }

    /* (sin código de debug temporal) */

    /* ====== 4) Gestión UART: eco en bloques de 5 bytes desde su ring buffer ====== */
    if (ring_buffer_count(&uart2_rx_rb) >= 5) {                        // ¿Hay 5+ bytes acumulados?
        for (int i = 0; i < 5; i++) {                                  // Procesa exactamente 5
            if (ring_buffer_read(&uart2_rx_rb, &uart2_rx_data)) {      // Saca el más antiguo
                HAL_UART_Transmit(&huart2, &uart2_rx_data, 1, HAL_MAX_DELAY); // Lo envía por UART2
            }
        }
    }

    /* ====== 5) Tarea de fondo: parpadeo del LED interno ====== */
    if (!led1_suppress_toggle) {
      led_toggle(&led1);                                             // Cambia estado de LD2 (PA5)
    }
    HAL_Delay(500);                                                    // Espera 500 ms (parpadeo ~1 Hz)
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration                      // Configura osciladores y árboles de reloj
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};          // Estructura: config de osciladores (HSI/PLL)
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};          // Estructura: config de buses (HCLK, PCLK)

  /** Configure the main internal regulator output voltage */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) { // Escala de voltaje para la frecuencia objetivo
    Error_Handler();                                     // Manejo de error
  }

  /** Initializes the RCC Oscillators according to the specified parameters */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI; // Fuente principal: HSI
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;                  // Enciende HSI
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Calibración por defecto
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;              // Habilita PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;      // PLL toma HSI como fuente
  RCC_OscInitStruct.PLL.PLLM = 1;                           // Predivisor M
  RCC_OscInitStruct.PLL.PLLN = 10;                          // Multiplicador N
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;               // Postdivisor P
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;               // Postdivisor Q
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;               // Postdivisor R (SYSCLK)
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {    // Aplica configuración de osciladores
    Error_Handler();                                        // Manejo de error
  }

  /** Initializes the CPU, AHB and APB buses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK
                              | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1
                              | RCC_CLOCKTYPE_PCLK2;        // Selección de relojes a configurar
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // SYSCLK desde PLL
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;        // HCLK = SYSCLK
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;         // PCLK1 = HCLK
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;         // PCLK2 = HCLK

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) { // Aplica árboles de reloj/latencia de Flash
    Error_Handler();                                        // Manejo de error
  }
}

/**
  * @brief USART2 Initialization Function                   // Inicializa USART2 a 115200-8N1
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{
  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;                         // Selecciona periferico USART2
  huart2.Init.BaudRate = 115200;                    // 115200 baudios
  huart2.Init.WordLength = UART_WORDLENGTH_8B;      // 8 bits de dato
  huart2.Init.StopBits = UART_STOPBITS_1;           // 1 bit de parada
  huart2.Init.Parity = UART_PARITY_NONE;            // Sin paridad
  huart2.Init.Mode = UART_MODE_TX_RX;               // TX y RX habilitados
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;      // Sin control de flujo HW
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;  // Oversampling x16
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE; // Deshabilita muestreo a 1 bit
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT; // Sin features avanzados
  if (HAL_UART_Init(&huart2) != HAL_OK) {           // Aplica la configuración
    Error_Handler();                                // Manejo de error
  }
  /* USER CODE BEGIN USART2_Init 2 */
  /* USER CODE END USART2_Init 2 */
}

/**
  * @brief GPIO Initialization Function                     // Configura GPIOs usados en el proyecto
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};           // Estructura temporal para configurar pines
  /* USER CODE BEGIN MX_GPIO_Init_1 */
  /* USER CODE END MX_GPIO_Init_1 */                // Punto de inserción inicial (no usado)

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();                     // Clock para GPIOC
  __HAL_RCC_GPIOH_CLK_ENABLE();                     // Clock para GPIOH
  __HAL_RCC_GPIOA_CLK_ENABLE();                     // Clock para GPIOA
  __HAL_RCC_GPIOB_CLK_ENABLE();                     // Clock para GPIOB

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|LED_EXT_Pin, GPIO_PIN_RESET); // Apaga LD2 y LED_EXT al inicio

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;                     // Selecciona el pin del botón de usuario (B1 = PC13)
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;      // Interrupción por flanco de bajada
  GPIO_InitStruct.Pull = GPIO_NOPULL;               // Sin resistencias internas (depende del hardware del botón)
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);    // Aplica la configuración al puerto del botón

  /*Configure GPIO pins : LD2_Pin LED_EXT_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|LED_EXT_Pin;        // Selecciona pines del LED interno y LED externo
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;       // Salida push-pull (conduce a Vcc o GND)
  GPIO_InitStruct.Pull = GPIO_NOPULL;               // Sin resistencias internas
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;      // Velocidad baja (menos ruido/consumo)
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);           // Aplica configuraciones en puerto A

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);       // Prioridad NVIC para líneas EXTI 10..15
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);               // Habilita la interrupción EXTI correspondiente

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* ========= KEYPAD 4x4 =========
   * Filas (R1..R4): salidas push-pull en ALTO (reposo)
   * Columnas (C1..C4): entradas con PULL-UP + EXTI por flanco de bajada
   *
   * Pines según tus defines en main.h:
   *   R1 = PA10, R2 = PB3, R3 = PB5, R4 = PB4
   *   C1 = PB10 (EXTI15_10), C2 = PA8 (EXTI9_5), C3 = PA9 (EXTI9_5), C4 = PC7 (EXTI9_5)
   */

  GPIO_InitTypeDef GPIO_InitStructK = {0};          // Estructura específica para el bloque de keypad

  /* --- FILAS: salida push-pull, sin pull, nivel ALTO inicial --- */
  GPIO_InitStructK.Mode  = GPIO_MODE_OUTPUT_PP;     // Salida push-pull
  GPIO_InitStructK.Pull  = GPIO_NOPULL;             // Sin resistencias internas en salida
  GPIO_InitStructK.Speed = GPIO_SPEED_FREQ_LOW;     // Velocidad baja suficiente

  // R1 = PA10
  GPIO_InitStructK.Pin = KEYPAD_R1_Pin;             // Selecciona pin de fila R1
  HAL_GPIO_Init(KEYPAD_R1_GPIO_Port, &GPIO_InitStructK);                  // Configura como salida
  HAL_GPIO_WritePin(KEYPAD_R1_GPIO_Port, KEYPAD_R1_Pin, GPIO_PIN_SET);    // Reposo en ALTO

  // R2 = PB3
  GPIO_InitStructK.Pin = KEYPAD_R2_Pin;             // Selecciona pin de fila R2
  HAL_GPIO_Init(KEYPAD_R2_GPIO_Port, &GPIO_InitStructK);                  // Configura como salida
  HAL_GPIO_WritePin(KEYPAD_R2_GPIO_Port, KEYPAD_R2_Pin, GPIO_PIN_SET);    // Reposo en ALTO

  // R3 = PB5
  GPIO_InitStructK.Pin = KEYPAD_R3_Pin;             // Selecciona pin de fila R3
  HAL_GPIO_Init(KEYPAD_R3_GPIO_Port, &GPIO_InitStructK);                  // Configura como salida
  HAL_GPIO_WritePin(KEYPAD_R3_GPIO_Port, KEYPAD_R3_Pin, GPIO_PIN_SET);    // Reposo en ALTO

  // R4 = PB4
  GPIO_InitStructK.Pin = KEYPAD_R4_Pin;             // Selecciona pin de fila R4
  HAL_GPIO_Init(KEYPAD_R4_GPIO_Port, &GPIO_InitStructK);                  // Configura como salida
  HAL_GPIO_WritePin(KEYPAD_R4_GPIO_Port, KEYPAD_R4_Pin, GPIO_PIN_SET);    // Reposo en ALTO

  /* --- COLUMNAS: entrada con PULL-UP + EXTI FALLING --- */
  GPIO_InitStructK.Mode = GPIO_MODE_IT_FALLING;     // Interrupción por flanco de bajada
  GPIO_InitStructK.Pull = GPIO_PULLUP;              // Pull-up interno (línea en ALTO en reposo)

  // C1 = PB10 (usa EXTI15_10 ya habilitado arriba)
  GPIO_InitStructK.Pin = KEYPAD_C1_Pin;             // Selecciona pin de columna C1
  HAL_GPIO_Init(KEYPAD_C1_GPIO_Port, &GPIO_InitStructK);                  // Configura como entrada/EXTI

  // C2 = PA8  (usa EXTI9_5)
  GPIO_InitStructK.Pin = KEYPAD_C2_Pin;             // Selecciona pin de columna C2
  HAL_GPIO_Init(KEYPAD_C2_GPIO_Port, &GPIO_InitStructK);                  // Configura como entrada/EXTI

  // C3 = PA9  (usa EXTI9_5)
  GPIO_InitStructK.Pin = KEYPAD_C3_Pin;             // Selecciona pin de columna C3
  HAL_GPIO_Init(KEYPAD_C3_GPIO_Port, &GPIO_InitStructK);                  // Configura como entrada/EXTI

  // C4 = PC7  (usa EXTI9_5)
  GPIO_InitStructK.Pin = KEYPAD_C4_Pin;             // Selecciona pin de columna C4
  HAL_GPIO_Init(KEYPAD_C4_GPIO_Port, &GPIO_InitStructK);                  // Configura como entrada/EXTI

  /* --- NVIC para las líneas EXTI usadas por las columnas de 5..9 --- */
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 1, 0);         // Prioridad para EXTI líneas 5..9 (PA8, PA9, PC7)
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);                 // Habilita IRQ EXTI9_5

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Callback de líneas EXTI:
 * - Si el pin coincide con B1_Pin (PC13), enciende LED_DOOR y programa apagado (5s) con anti-rebote.
 * - Si el pin coincide con alguna columna del keypad, **NO** escanea aquí: solo guarda el pin y deshabilita EXTI.
 *   El escaneo real se hace en el while() principal, y luego se vuelven a habilitar las IRQs.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == B1_Pin) {                        // ¿Interrupción del botón azul (PC13)?
        uint32_t now = HAL_GetTick();                // Lee el tiempo actual del sistema (ms desde el arranque)

        if ((now - last_button_ms) < 150U) {         // Anti-rebote: 150 ms entre pulsos válidos
            return;                                  // Ignora rebote
        }
        last_button_ms = now;                        // Marca este pulso como válido
        led_on(&led_door);                           // Enciende LED externo (PA7)
        door_led_deadline_ms = now + 5000U;          // Agenda apagado en 5 segundos (no bloqueante)
        return;                                      // Termina aquí para el botón
    }

    /* Columnas del keypad: registrar y deshabilitar IRQs hasta que el main() procese */
    if ( (GPIO_Pin == KEYPAD_C1_Pin) ||
         (GPIO_Pin == KEYPAD_C2_Pin) ||
         (GPIO_Pin == KEYPAD_C3_Pin) ||
         (GPIO_Pin == KEYPAD_C4_Pin) ) {

      last_exti_pin = GPIO_Pin;                    // Guarda qué pin (columna) disparó

      HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);           // Evita reentradas de 5..9 durante el procesamiento
      HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);         // Evita reentradas de 15..10 durante el procesamiento
      return;                                      // El escaneo lo hace el while()
    }

    /* Si no es B1 ni columnas del keypad, no hacemos nada aquí */
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.  // Manejador genérico de errores
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */ // Comentario: el usuario puede añadir reportes por UART, LED, etc.
  __disable_irq();                                   // Deshabilita interrupciones globales para evitar comportamientos erráticos
  while (1) {                                        // Bucle infinito: detiene el flujo normal para depurar
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number       // Reporte para asserts fallidos (solo si USE_FULL_ASSERT)
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name                                // Puntero a la cadena con nombre de archivo
  * @param  line: assert_param error line source number                           // Número de línea donde ocurrió el fallo
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */ // Aquí se podría imprimir por UART para diagnóstico
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */                        // Fin del bloque condicional de assert
