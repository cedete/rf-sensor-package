//
// This file contains a helper class to handle all the peripheral interrupt
// handlers.
//
// The vendor supplied interrupt handlers (i.e. USART1_IRQHandler() ) are wrapped
// in an extern "C" since they are C functions, and they contain interrupt 
// handler objects which are declared here. The interrupt handler class is found
// in "interrupt_handler.hpp". That class is responsible for connecting the 
// respective vendor interrupt handler with a function contained within a
// peripheral driver class.
//
// This is important because the C-style vendor interrupt handlers can only call
// either static members of a class or a class that is in global scope (with some
// other conditions met). This process allows a normal member of a class to be
// called without it being in global scope and without it being static.
//

#ifndef BOARD_INTERRUPTS_HPP
#define BOARD_INTERRUPTS_HPP

#include <cstdint>
#include <cassert>

#include "../board/board.hpp"
#include "../common/interrupt_handler.hpp"

namespace STM32WL55JC1
{
    struct IRQ_Handlers
    {
        static Interrupt_Handler& irq_handler(UART::Instance uart);

        static Interrupt_Handler& irq_handler();

        // create other instances that have different TypeDef parameters
    };

} // namespace STM32WL55JC1

#endif // BOARD_INTERRUPTS_HPP
