// -----------------------------------------------------------------------------
// Author:      Calum Thornhill
// Date:        3/1/2025
// File:        startup.cpp
// Description: Startup code for for STM32WL55xx Cortex-M4 devices.
//  - sets initial SP
//  - sets initial PC == Reset_Handler
//  - sets the vector table entries with the exceptions ISR address,
//  - branches to main in the C library (which eventually calls main()).
//  - after Reset the Cortex-M4 processor is in Thread mode, priority is 
//    Privileged, and the Stack is set to Main.
// References: 
//  - https://github.com/elzoughby/STM32H7xx-Startup.git
//  - https://github.com/csrohit/stm32-startup-cpp.git
//  - STM32 vendor startup code and linker script
// -----------------------------------------------------------------------------

// ------------------------------- Includes ------------------------------------
#include <stdint.h>

// ----------------------------- Private macros -------------------------------- 
#define RAM_START        (0x20000000U)
#define RAM_SIZE         (64U * 1024U) 
#define RAM_END          (RAM_START + RAM_SIZE)
#define STACK_START      (RAM_END)

// --------------------------- Function prototypes -----------------------------
extern "C"
{
    extern int main();
    extern void __libc_init_array();

    void Default_Handler()                   __attribute__((weak));
    void Reset_Handler()                     __attribute__((weak));
    void NMI_Handler()                       __attribute__((weak));
    void HardFault_Handler()                 __attribute__((weak));
    void MemManage_Handler()                 __attribute__((weak));
    void BusFault_Handler()                  __attribute__((weak));
    void UsageFault_Handler()                __attribute__((weak));
    void SVC_Handler()                       __attribute__((weak));
    void DebugMon_Handler()                  __attribute__((weak));
    void PendSV_Handler()                    __attribute__((weak));
    void SysTick_Handler()                   __attribute__((weak));

    void WWDG_IRQHandler()                   __attribute__ ((weak, alias("Default_Handler")));
    void PVD_PVM_IRQHandler()                __attribute__ ((weak, alias("Default_Handler")));
    void TAMP_STAMP_LSECSS_SSRU_IRQHandler() __attribute__ ((weak, alias("Default_Handler")));
    void RTC_WKUP_IRQHandler()               __attribute__ ((weak, alias("Default_Handler")));
    void FLASH_IRQHandler()                  __attribute__ ((weak, alias("Default_Handler")));
    void RCC_IRQHandler()                    __attribute__ ((weak, alias("Default_Handler")));
    void EXTI0_IRQHandler()                  __attribute__ ((weak, alias("Default_Handler")));
    void EXTI1_IRQHandler()                  __attribute__ ((weak, alias("Default_Handler")));
    void EXTI2_IRQHandler()                  __attribute__ ((weak, alias("Default_Handler")));
    void EXTI3_IRQHandler()                  __attribute__ ((weak, alias("Default_Handler")));
    void EXTI4_IRQHandler()                  __attribute__ ((weak, alias("Default_Handler")));
    void DMA1_Channel1_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMA1_Channel2_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMA1_Channel3_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMA1_Channel4_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMA1_Channel5_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMA1_Channel6_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMA1_Channel7_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void ADC_IRQHandler()                    __attribute__ ((weak, alias("Default_Handler")));
    void DAC_IRQHandler()                    __attribute__ ((weak, alias("Default_Handler")));
    void C2SEV_PWR_C2H_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void COMP_IRQHandler()                   __attribute__ ((weak, alias("Default_Handler")));
    void EXTI9_5_IRQHandler()                __attribute__ ((weak, alias("Default_Handler")));
    void TIM1_BRK_IRQHandler()               __attribute__ ((weak, alias("Default_Handler")));
    void TIM1_UP_IRQHandler()                __attribute__ ((weak, alias("Default_Handler")));
    void TIM1_TRG_COM_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
    void TIM1_CC_IRQHandler()                __attribute__ ((weak, alias("Default_Handler")));
    void TIM2_IRQHandler()                   __attribute__ ((weak, alias("Default_Handler")));
    void TIM16_IRQHandler()                  __attribute__ ((weak, alias("Default_Handler")));
    void TIM17_IRQHandler()                  __attribute__ ((weak, alias("Default_Handler")));
    void I2C1_EV_IRQHandler()                __attribute__ ((weak, alias("Default_Handler")));
    void I2C1_ER_IRQHandler()                __attribute__ ((weak, alias("Default_Handler")));
    void I2C2_EV_IRQHandler()                __attribute__ ((weak, alias("Default_Handler")));
    void I2C2_ER_IRQHandler()                __attribute__ ((weak, alias("Default_Handler")));
    void SPI1_IRQHandler()                   __attribute__ ((weak, alias("Default_Handler")));
    void SPI2_IRQHandler()                   __attribute__ ((weak, alias("Default_Handler")));
    void USART1_IRQHandler()                 __attribute__ ((weak, alias("Default_Handler")));
    void USART2_IRQHandler()                 __attribute__ ((weak, alias("Default_Handler")));
    void LPUART1_IRQHandler()                __attribute__ ((weak, alias("Default_Handler")));
    void LPTIM1_IRQHandler()                 __attribute__ ((weak, alias("Default_Handler")));
    void LPTIM2_IRQHandler()                 __attribute__ ((weak, alias("Default_Handler")));
    void EXTI15_10_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
    void RTC_Alarm_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
    void LPTIM3_IRQHandler()                 __attribute__ ((weak, alias("Default_Handler")));
    void SUBGHZSPI_IRQHandler()              __attribute__ ((weak, alias("Default_Handler")));
    void IPCC_C1_RX_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
    void IPCC_C1_TX_IRQHandler()             __attribute__ ((weak, alias("Default_Handler")));
    void HSEM_IRQHandler()                   __attribute__ ((weak, alias("Default_Handler")));
    void I2C3_EV_IRQHandler()                __attribute__ ((weak, alias("Default_Handler")));
    void I2C3_ER_IRQHandler()                __attribute__ ((weak, alias("Default_Handler")));
    void SUBGHZ_Radio_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));
    void AES_IRQHandler()                    __attribute__ ((weak, alias("Default_Handler")));
    void RNG_IRQHandler()                    __attribute__ ((weak, alias("Default_Handler")));
    void PKA_IRQHandler()                    __attribute__ ((weak, alias("Default_Handler")));
    void DMA2_Channel1_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMA2_Channel2_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMA2_Channel3_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMA2_Channel4_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMA2_Channel5_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMA2_Channel6_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMA2_Channel7_IRQHandler()          __attribute__ ((weak, alias("Default_Handler")));
    void DMAMUX1_OVER_IRQHandler()           __attribute__ ((weak, alias("Default_Handler")));


// ------------------------------ Global variables -----------------------------
    extern const volatile uint32_t _estack;     // highest address of user mode stack
    extern uint32_t _sidata;                    // start address for init values in .data 
    extern uint32_t _sdata;                     // start address for .data section
    extern uint32_t _edata;                     // end address for .data section
    extern uint32_t _sbss;                      // start address for .bss section
    extern uint32_t _ebss;                      // end address for .bss section

// --------------------------- Interrupt vector table --------------------------
    __attribute__ ((section(".isr_vector")))
    uint32_t isr_vector[]
    {
        STACK_START, 
        (uint32_t)&Reset_Handler,
        (uint32_t)&NMI_Handler,
        (uint32_t)&HardFault_Handler,
        (uint32_t)&MemManage_Handler,
        (uint32_t)&BusFault_Handler,
        (uint32_t)&UsageFault_Handler,
        0,                  // Reserved
        0,                  // Reserved
        0,                  // Reserved
        0,                  // Reserved
        (uint32_t)&SVC_Handler,
        (uint32_t)&DebugMon_Handler,
        0,                  // Reserved
        (uint32_t)&PendSV_Handler,
        (uint32_t)&SysTick_Handler,

        // External interrupts
        (uint32_t)&WWDG_IRQHandler,
        (uint32_t)&PVD_PVM_IRQHandler,
        (uint32_t)&TAMP_STAMP_LSECSS_SSRU_IRQHandler,
        (uint32_t)&RTC_WKUP_IRQHandler,
        (uint32_t)&FLASH_IRQHandler,
        (uint32_t)&RCC_IRQHandler,
        (uint32_t)&EXTI0_IRQHandler,
        (uint32_t)&EXTI1_IRQHandler,
        (uint32_t)&EXTI2_IRQHandler,
        (uint32_t)&EXTI3_IRQHandler,
        (uint32_t)&EXTI4_IRQHandler,
        (uint32_t)&DMA1_Channel1_IRQHandler,
        (uint32_t)&DMA1_Channel2_IRQHandler,
        (uint32_t)&DMA1_Channel3_IRQHandler,
        (uint32_t)&DMA1_Channel4_IRQHandler,
        (uint32_t)&DMA1_Channel5_IRQHandler,
        (uint32_t)&DMA1_Channel6_IRQHandler,
        (uint32_t)&DMA1_Channel7_IRQHandler,
        (uint32_t)&ADC_IRQHandler,
        (uint32_t)&DAC_IRQHandler,
        (uint32_t)&C2SEV_PWR_C2H_IRQHandler,         
        (uint32_t)&COMP_IRQHandler,                 
        (uint32_t)&EXTI9_5_IRQHandler,             
        (uint32_t)&TIM1_BRK_IRQHandler,            
        (uint32_t)&TIM1_UP_IRQHandler,            
        (uint32_t)&TIM1_TRG_COM_IRQHandler,      
        (uint32_t)&TIM1_CC_IRQHandler,          
        (uint32_t)&TIM2_IRQHandler,            
        (uint32_t)&TIM16_IRQHandler,                 
        (uint32_t)&TIM17_IRQHandler,                
        (uint32_t)&I2C1_EV_IRQHandler,             
        (uint32_t)&I2C1_ER_IRQHandler,              
        (uint32_t)&I2C2_EV_IRQHandler,             
        (uint32_t)&I2C2_ER_IRQHandler,            
        (uint32_t)&SPI1_IRQHandler,              
        (uint32_t)&SPI2_IRQHandler,             
        (uint32_t)&USART1_IRQHandler,          
        (uint32_t)&USART2_IRQHandler,         
        (uint32_t)&LPUART1_IRQHandler,       
        (uint32_t)&LPTIM1_IRQHandler,       
        (uint32_t)&LPTIM2_IRQHandler,      
        (uint32_t)&EXTI15_10_IRQHandler,  
        (uint32_t)&RTC_Alarm_IRQHandler, 
        (uint32_t)&LPTIM3_IRQHandler,                 
        (uint32_t)&SUBGHZSPI_IRQHandler,             
        (uint32_t)&IPCC_C1_RX_IRQHandler,           
        (uint32_t)&IPCC_C1_TX_IRQHandler,          
        (uint32_t)&HSEM_IRQHandler,               
        (uint32_t)&I2C3_EV_IRQHandler,           
        (uint32_t)&I2C3_ER_IRQHandler,          
        (uint32_t)&SUBGHZ_Radio_IRQHandler,
        (uint32_t)&AES_IRQHandler,                   
        (uint32_t)&RNG_IRQHandler,                  
        (uint32_t)&PKA_IRQHandler,                 
        (uint32_t)&DMA2_Channel1_IRQHandler,      
        (uint32_t)&DMA2_Channel2_IRQHandler,   
        (uint32_t)&DMA2_Channel3_IRQHandler,         
        (uint32_t)&DMA2_Channel4_IRQHandler,        
        (uint32_t)&DMA2_Channel5_IRQHandler,       
        (uint32_t)&DMA2_Channel6_IRQHandler,      
        (uint32_t)&DMA2_Channel7_IRQHandler,     
        (uint32_t)&DMAMUX1_OVER_IRQHandler     
    }; // isr_vector[]

} // extern "C"

// -------------------------- Function definitions -----------------------------
void Default_Handler() 
{
    while(1) {}
}

void Reset_Handler() 
{
    // Set stack pointer
    __asm("    ldr    sp, =_estack");

    /*
    // In STM vendor file it is implemented like so: 
    __asm("    ldr    r0, =_estack\n"
          "    mov    sp, r0");
    */

    // Copy the data segment initializers from flash to SRAM 
    uint32_t* pui32Src, *pui32Dest;
    pui32Src = &_sidata;
    for(pui32Dest = &_sdata; pui32Dest < &_edata; )
    {
        *pui32Dest++ = *pui32Src++;
    }

    // Zero fill the bss segment 
    __asm("    ldr     r0, =_sbss\n"
          "    ldr     r1, =_ebss\n"
          "    mov     r2, #0\n"
          "    .thumb_func\n"
          "zero_loop:\n"
          "        cmp     r0, r1\n"
          "        it      lt\n"
          "        strlt   r2, [r0], #4\n"
          "        blt     zero_loop");

    // Call static constructors 
    __libc_init_array();

    // Call the application's entry point 
    main();
}

void NMI_Handler() 
{
    while(1) {}
}

void HardFault_Handler() 
{
    while(1) {}
}

void MemManage_Handler() 
{
    while(1) {}
}

void BusFault_Handler() 
{
    while(1) {}
}

void UsageFault_Handler() 
{
    while(1) {}
}

void SVC_Handler() 
{
    while(1) {}
}

void DebugMon_Handler() 
{
    while(1) {}
}

void PendSV_Handler() 
{
    while(1) {}
}

void SysTick_Handler() 
{
    while(1) {}
}
