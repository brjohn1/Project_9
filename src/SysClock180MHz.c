//***********************************************************
//  SysClock180MHz.c
//  This file contains the system clock configuration for 180MHz
//***********************************************************

#include "stm32f4xx.h"
#include "SysClock180MHz.h"

#define PLL_M       8ul    // VCO input frequency = 16MHz / 8 = 2MHz
#define PLL_N       180ul  // VCO output frequency = 2MHz * 180 = 360MHz
#define PLL_P       0ul    // SYSCLK = 360MHz / 2 = 180MHz
#define PLL_Q       2ul    // USB OTG FS, SDIO and RNG Clock = 360MHz / 2 = 180MHz

void SystemClock_Config(void) {
  // change the clock from 16MHz to 180MHz
  RCC->CR |= RCC_CR_HSION; // Enable HSI (Note this is default if there is no HSE)
  while (!(RCC->CR & RCC_CR_HSIRDY)); // Wait until HSI is ready
  // Configure PLL: VCO = (16MHz / M) * N = 360MHz, SYSCLK = VCO / P = 180MHz, USB OTG FS, SDIO and RNG Clock = VCO / Q = 180MHz
  RCC->PLLCFGR = (PLL_M) | (PLL_N << 6) | (PLL_P << 16) | (PLL_Q << 24);
  RCC->PLLCFGR &=~ RCC_PLLCFGR_PLLSRC; // Set HSI as PLL source
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_PPRE1_DIV4; // Set prescalers: AHB=SYSCLK/1, APB2=AHB/2, APB1=AHB/4
  // APB1 = 45MHz (TIM2 clock x2 = 90MHz), APB2 = 90MHz
  
  RCC->CR |= RCC_CR_PLLON; // Enable PLL
  while (!(RCC->CR & RCC_CR_PLLRDY));// Wait until PLL is ready
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;// Enable power interface clock
  PWR->CR |= PWR_CR_ODEN;// Enable Over-drive mode
  while (!(PWR->CSR & PWR_CSR_ODRDY));// Wait until Over-drive mode is ready
  PWR->CR |= PWR_CR_ODSWEN;// Enable Over-drive switching
  while (!(PWR->CSR & PWR_CSR_ODSWRDY)) ;// Wait until Over-drive switching is ready
  FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_5WS;//  Set flash latency to 5 wait states
  RCC->CFGR &=~ RCC_CFGR_SW;// Clear SW bits
  RCC->CFGR |= RCC_CFGR_SW_PLL;// Select PLL as system clock source
  while ((RCC->CFGR & RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);// Wait until PLL is used as system clock source
}
