/**
  ******************************************************************************
  * @file    system_stm32h5xx.c
  * @brief   CMSIS Cortex-M33 Device Peripheral Access Layer System Source File.
  ******************************************************************************
  */

#include "stm32h5xx.h"

/* System Clock Frequency (Core Clock) */
uint32_t SystemCoreClock = 64000000U; /* HSI 64 MHz */

/**
  * @brief  Setup the microcontroller system.
  * @param  None
  * @retval None
  */
void SystemInit(void)
{
  /* FPU settings - Enable CP10 and CP11 coprocessors */
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));  /* set CP10 and CP11 Full Access */
#endif

  /* Configure the Vector Table location add offset address */
#ifdef VECT_TAB_SRAM
  SCB->VTOR = SRAM1_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
#else
  SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
#endif
}

/**
  * @brief  Update SystemCoreClock variable according to Clock Register Values.
  * @param  None
  * @retval None
  */
void SystemCoreClockUpdate(void)
{
  /* For this minimal implementation, we use HSI @ 64 MHz */
  SystemCoreClock = 64000000U;
}
