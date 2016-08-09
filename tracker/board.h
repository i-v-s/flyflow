#ifndef BOARD_H
#define BOARD_H

#define STM32F427X
//#include <armv7-ar/thumb/bits/cpu_defines.h>
//#include "CMSIS/Device/ST/STM32F0xx/Include/stm32f051x8.h"
#include "CMSIS/Device/ST/STM32F4xx/Include/stm32f4xx.h"

#define __LDREX __LDREXW
#define __STREX __STREXW
//#define __CLREX __CLREXW
//#define __LDREX(a) (*(a))
//#define __STREX(a, b) ((*(b) = (a)), false)
//#define __CLREX()

#endif // BOARD_H
