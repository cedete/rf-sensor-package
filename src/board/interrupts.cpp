#include "interrupts.hpp"

using namespace STM32WL55JC1;

//
// Interrupt handler that connects peripheral driver function to ISR
//

static Interrupt_Handler usart1_handler;
static Interrupt_Handler usart2_handler;

//
// Implementation of IRQ handler that handles the different peripherals'
// interrupt handlers
//

Interrupt_Handler& IRQ_Handlers::irq_handler(UART::Instance uart)
{
    switch (uart)
    {
        case UART::Instance::USART_1: return usart1_handler; break;
        case UART::Instance::USART_2: return usart2_handler; break;
    }
    assert(false);
}

//
// Vendor supplied ISRs
//

extern "C" void USART1_IRQHandler()
{
    usart1_handler.call();
}

extern "C" void USART2_IRQHandler()
{
    usart2_handler.call();
}
