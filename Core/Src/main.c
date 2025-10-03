/**
  ******************************************************************************
  * @file    main.c
  * @brief   Main program body for STM32H563VGT6
  ******************************************************************************
  */

#include "stm32h5xx.h"

/* Private function prototypes */
void SystemClock_Config(void);
void GPIO_Init(void);
void delay(uint32_t count);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  SystemInit();
  
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize GPIO for LED */
  GPIO_Init();

  /* Infinite loop */
  while (1)
  {
    /* Toggle LED on PA5 (example pin) */
    GPIOA->ODR ^= (1 << 5);
    
    /* Delay */
    delay(1000000);
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  /* For this minimal config, we use the default HSI clock (64 MHz) */
  /* No additional configuration needed */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void GPIO_Init(void)
{
  /* Enable GPIOA clock */
  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN;
  
  /* Small delay after enabling clock */
  volatile uint32_t tmpreg = RCC->AHB4ENR & RCC_AHB4ENR_GPIOAEN;
  (void)tmpreg;
  
  /* Configure PA5 as output (MODER5[1:0] = 01) */
  GPIOA->MODER &= ~(0x3 << (5 * 2));  /* Clear mode bits */
  GPIOA->MODER |= (0x1 << (5 * 2));   /* Set as output */
  
  /* Configure PA5 as push-pull (default, but explicit) */
  GPIOA->OTYPER &= ~(1 << 5);
  
  /* Configure PA5 speed as medium */
  GPIOA->OSPEEDR &= ~(0x3 << (5 * 2));
  GPIOA->OSPEEDR |= (0x1 << (5 * 2));
  
  /* Configure PA5 as no pull-up, pull-down */
  GPIOA->PUPDR &= ~(0x3 << (5 * 2));
}

/**
  * @brief Simple delay function
  * @param count: delay count
  * @retval None
  */
void delay(uint32_t count)
{
  volatile uint32_t i;
  for(i = 0; i < count; i++)
  {
    __asm("nop");
  }
}
