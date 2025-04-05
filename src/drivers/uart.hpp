#ifndef DRIVERS_UART_HPP
#define DRIVERS_UART_HPP

#include <cstdint>

#include "../board/board.hpp"
#include "../board/clock.hpp"
#include "../board/interrupts.hpp"

#include "../common/ring_buffer.h"

namespace STM32WL55JC1
{
    namespace UART
    {
        class Driver 
        {
        public:
            explicit Driver(const Config& config);

            Driver()                             = delete;
            Driver(const Driver& rhs)            = delete;
            Driver(Driver&& rhs)                 = delete;
            Driver& operator=(const Driver& rhs) = delete;
            Driver& operator=(Driver&& rhs)      = delete;

            ~Driver();

            void isr_read();

            // blocking
            uint8_t read();     
            void write(uint8_t byte);

            // internal buffer
            bool buffer_full();
            bool buffer_empty();

            void push(uint8_t data);
            void pop(uint8_t& data);

            bool rx_done(uint8_t delimiter);

        private:

            #ifndef UNIT_TEST
                void enable_peripheral(const Instance& uart);
                void disable_peripheral();
                void enable_interrupts(const Instance& uart);
            #endif // UNIT_TEST

            void enable_uart();
            void disable_uart();


            void set_mode(const Mode& mode);
            void set_baud(const uint32_t& rate);
            void set_data_length(const Data& length);
            void set_stop_bits(const Stop& bits);
            void set_oversampling(const Oversampling& oversampling);

            void set_mode(const GPIO::Pin& p, volatile GPIO_TypeDef* reg);
            void set_function(const GPIO::Pin& p, volatile GPIO_TypeDef* reg);

            volatile USART_TypeDef* UART_Reg;
            volatile GPIO_TypeDef* TX_Pin;
            volatile GPIO_TypeDef* RX_Pin; 

            Ring_Buffer<uint8_t, 100> buffer; 
        };

    } // namespace UART

} // namespace STM32WL55JC1

#endif // DRIVERS_UART_HPP
