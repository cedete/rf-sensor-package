#ifndef TEST_TEST_HARDWARE
#define TEST_TEST_HARDWARE

#include "../src/app/support.hpp"
#include "../src/drivers/uart.hpp"
#include "../src/common/ring_buffer.h"

namespace Test
{
    using namespace STM32WL55JC1;

    class Terminal
    {
    public:
        Terminal(UART::Driver& tx) : usart{tx}
        {}

        ~Terminal()
        {}
    
        void print(Ring_Buffer<uint8_t, 255>& buffer)
        {
            while (!(buffer.empty())) 
            {
                usart.write(buffer.get());
            }               
        }

        //
        // This function tests the USART print to terminal functionality
        // combined with the ring buffer. Because the ring buffer is only
        // 3 characters long, this should only print a, b, and c on repeat.
        // If they do print, it is a defect with the ring buffer. 

        void test_print()
        {
            Ring_Buffer<uint8_t, 3> ring_buffer;

            ring_buffer.put('a');
            ring_buffer.put('b');
            ring_buffer.put('c');
            ring_buffer.put('d');   
            ring_buffer.put('e');
            
            while (!(ring_buffer.empty()))
            {
                usart.write(ring_buffer.get());
            }  
        }

    private:
        UART::Driver& usart;
    };

} // namespace Test

#endif // TEST_TEST_HARDWARE
