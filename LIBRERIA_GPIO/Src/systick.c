#include "systick.h"

#define HSI_FREQ    4000000U                                   // Reloj interno 4 MHz

void init_systick(void)
{
    SYSTICK->RVR = HSI_FREQ / 1000 - 1;                      // Recarga = 4000 - 1
    SYSTICK->CSR = (1 << 0) | (1 << 1) | (1 << 2);    // ENABLE|TICKINT|CLKSOURCE
}