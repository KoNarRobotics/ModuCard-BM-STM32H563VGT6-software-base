/**
  ******************************************************************************
  * @file      startup_stm32h563xx.s
  * @brief     STM32H563xx Devices vector table for GCC toolchain.
  ******************************************************************************
  */

.syntax unified
.cpu cortex-m33
.fpu softvfp
.thumb

.global g_pfnVectors
.global Default_Handler

/* start address for the initialization values of the .data section */
.word _sidata
/* start address for the .data section */
.word _sdata
/* end address for the .data section */
.word _edata
/* start address for the .bss section */
.word _sbss
/* end address for the .bss section */
.word _ebss

/**
 * @brief  This is the code that gets called when the processor first
 *         starts execution following a reset event.
 */
.section .text.Reset_Handler
.weak Reset_Handler
.type Reset_Handler, %function
Reset_Handler:
  ldr   r0, =_estack
  mov   sp, r0          /* set stack pointer */

/* Copy the data segment initializers from flash to SRAM */
  ldr r0, =_sdata
  ldr r1, =_edata
  ldr r2, =_sidata
  movs r3, #0
  b LoopCopyDataInit

CopyDataInit:
  ldr r4, [r2, r3]
  str r4, [r0, r3]
  adds r3, r3, #4

LoopCopyDataInit:
  adds r4, r0, r3
  cmp r4, r1
  bcc CopyDataInit

/* Zero fill the bss segment. */
  ldr r2, =_sbss
  ldr r4, =_ebss
  movs r3, #0
  b LoopFillZerobss

FillZerobss:
  str  r3, [r2]
  adds r2, r2, #4

LoopFillZerobss:
  cmp r2, r4
  bcc FillZerobss

/* Call the clock system initialization function.*/
  bl  SystemInit
/* Call static constructors */
  bl __libc_init_array
/* Call the application's entry point.*/
  bl main

LoopForever:
  b LoopForever

.size Reset_Handler, .-Reset_Handler

/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.
 */
.section .text.Default_Handler,"ax",%progbits
Default_Handler:
Infinite_Loop:
  b Infinite_Loop
  .size Default_Handler, .-Default_Handler

/******************************************************************************
*
* The minimal vector table for a Cortex-M33.
*
******************************************************************************/
.section .isr_vector,"a",%progbits
.type g_pfnVectors, %object
.size g_pfnVectors, .-g_pfnVectors

g_pfnVectors:
  .word _estack
  .word Reset_Handler
  .word NMI_Handler
  .word HardFault_Handler
  .word MemManage_Handler
  .word BusFault_Handler
  .word UsageFault_Handler
  .word SecureFault_Handler
  .word 0
  .word 0
  .word 0
  .word SVC_Handler
  .word DebugMon_Handler
  .word 0
  .word PendSV_Handler
  .word SysTick_Handler
  .word WWDG_IRQHandler                   /* Window WatchDog              */
  .word PVD_PVM_IRQHandler                /* PVD/PVM through EXTI         */
  .word RTC_IRQHandler                    /* RTC                          */
  .word RTC_S_IRQHandler                  /* RTC secure                   */
  .word TAMP_IRQHandler                   /* Tamper                       */
  .word RAMCFG_IRQHandler                 /* RAMCFG                       */
  .word FLASH_IRQHandler                  /* FLASH                        */
  .word FLASH_S_IRQHandler                /* FLASH secure                 */
  .word GTZC_IRQHandler                   /* GTZC                         */
  .word RCC_IRQHandler                    /* RCC                          */
  .word RCC_S_IRQHandler                  /* RCC secure                   */
  .word EXTI0_IRQHandler                  /* EXTI Line0                   */
  .word EXTI1_IRQHandler                  /* EXTI Line1                   */
  .word EXTI2_IRQHandler                  /* EXTI Line2                   */
  .word EXTI3_IRQHandler                  /* EXTI Line3                   */
  .word EXTI4_IRQHandler                  /* EXTI Line4                   */
  .word EXTI5_IRQHandler                  /* EXTI Line5                   */
  .word EXTI6_IRQHandler                  /* EXTI Line6                   */
  .word EXTI7_IRQHandler                  /* EXTI Line7                   */
  .word EXTI8_IRQHandler                  /* EXTI Line8                   */
  .word EXTI9_IRQHandler                  /* EXTI Line9                   */
  .word EXTI10_IRQHandler                 /* EXTI Line10                  */
  .word EXTI11_IRQHandler                 /* EXTI Line11                  */
  .word EXTI12_IRQHandler                 /* EXTI Line12                  */
  .word EXTI13_IRQHandler                 /* EXTI Line13                  */
  .word EXTI14_IRQHandler                 /* EXTI Line14                  */
  .word EXTI15_IRQHandler                 /* EXTI Line15                  */

/*******************************************************************************
*
* Provide weak aliases for each Exception handler to the Default_Handler.
* As they are weak aliases, any function with the same name will override
* this definition.
*
*******************************************************************************/

  .weak      NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak      HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak      MemManage_Handler
  .thumb_set MemManage_Handler,Default_Handler

  .weak      BusFault_Handler
  .thumb_set BusFault_Handler,Default_Handler

  .weak      UsageFault_Handler
  .thumb_set UsageFault_Handler,Default_Handler

  .weak      SecureFault_Handler
  .thumb_set SecureFault_Handler,Default_Handler

  .weak      SVC_Handler
  .thumb_set SVC_Handler,Default_Handler

  .weak      DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak      PendSV_Handler
  .thumb_set PendSV_Handler,Default_Handler

  .weak      SysTick_Handler
  .thumb_set SysTick_Handler,Default_Handler

  .weak      WWDG_IRQHandler
  .thumb_set WWDG_IRQHandler,Default_Handler

  .weak      PVD_PVM_IRQHandler
  .thumb_set PVD_PVM_IRQHandler,Default_Handler

  .weak      RTC_IRQHandler
  .thumb_set RTC_IRQHandler,Default_Handler

  .weak      RTC_S_IRQHandler
  .thumb_set RTC_S_IRQHandler,Default_Handler

  .weak      TAMP_IRQHandler
  .thumb_set TAMP_IRQHandler,Default_Handler

  .weak      RAMCFG_IRQHandler
  .thumb_set RAMCFG_IRQHandler,Default_Handler

  .weak      FLASH_IRQHandler
  .thumb_set FLASH_IRQHandler,Default_Handler

  .weak      FLASH_S_IRQHandler
  .thumb_set FLASH_S_IRQHandler,Default_Handler

  .weak      GTZC_IRQHandler
  .thumb_set GTZC_IRQHandler,Default_Handler

  .weak      RCC_IRQHandler
  .thumb_set RCC_IRQHandler,Default_Handler

  .weak      RCC_S_IRQHandler
  .thumb_set RCC_S_IRQHandler,Default_Handler

  .weak      EXTI0_IRQHandler
  .thumb_set EXTI0_IRQHandler,Default_Handler

  .weak      EXTI1_IRQHandler
  .thumb_set EXTI1_IRQHandler,Default_Handler

  .weak      EXTI2_IRQHandler
  .thumb_set EXTI2_IRQHandler,Default_Handler

  .weak      EXTI3_IRQHandler
  .thumb_set EXTI3_IRQHandler,Default_Handler

  .weak      EXTI4_IRQHandler
  .thumb_set EXTI4_IRQHandler,Default_Handler

  .weak      EXTI5_IRQHandler
  .thumb_set EXTI5_IRQHandler,Default_Handler

  .weak      EXTI6_IRQHandler
  .thumb_set EXTI6_IRQHandler,Default_Handler

  .weak      EXTI7_IRQHandler
  .thumb_set EXTI7_IRQHandler,Default_Handler

  .weak      EXTI8_IRQHandler
  .thumb_set EXTI8_IRQHandler,Default_Handler

  .weak      EXTI9_IRQHandler
  .thumb_set EXTI9_IRQHandler,Default_Handler

  .weak      EXTI10_IRQHandler
  .thumb_set EXTI10_IRQHandler,Default_Handler

  .weak      EXTI11_IRQHandler
  .thumb_set EXTI11_IRQHandler,Default_Handler

  .weak      EXTI12_IRQHandler
  .thumb_set EXTI12_IRQHandler,Default_Handler

  .weak      EXTI13_IRQHandler
  .thumb_set EXTI13_IRQHandler,Default_Handler

  .weak      EXTI14_IRQHandler
  .thumb_set EXTI14_IRQHandler,Default_Handler

  .weak      EXTI15_IRQHandler
  .thumb_set EXTI15_IRQHandler,Default_Handler
