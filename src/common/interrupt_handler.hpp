//
// @date    28 March 2025
// @author  Calum Thornhill
// @brief   This implementation allows for a C-based interrupt handler routine 
//          to call a non-static function within a non-global peripheral object.
//
// @details This implementation was modified from an example given by a user on
//          Reddit. See the link: 
//
//          https://www.reddit.com/r/embedded/comments/way0ta/how_can_i_write_effective_isr_handlers_in_c_code/
//
//          The class contains a callback to any possible non void function,
//          which specifically would be an ISR routine within a peripheral
//          driver class (such as UART, GPIO input, etc.). It uses C++17 
//          features and lambdas to erase the type of the given object or 
//          function, and register, "or connect," an ISR with a given peripheral 
//          driver object. 
//
//          This is necessary because the C-based interrupt handlers can only
//          call free functions, static functions within a class, or functions 
//          in global objects. A program design that requires callbacks within 
//          a class needs this functionality. 
//

#ifndef DRIVERS_INTERRUPT_HANDLER_HPP
#define DRIVERS_INTERRUPT_HANDLER_HPP

namespace STM32WL55JC1
{
    class Interrupt_Handler
    {
    public:
        template <auto Func, typename Class>
        void connect(Class* obj)
        {
            callback.object = obj;                  
            callback.function = [](void* data)
            {
                Class* object = static_cast<Class*>(data);
                (object->*Func)();
            };
        }

        template <void (*Func)()>
        void connect()
        {
            callback.object = nullptr;
            callback.function = [](void* data)
            {
                Func();
            };
        }

        void call() const
        {
            if (callback.function)
            {
                callback.function(callback.object);
            }
        }
    
    private:
        struct Callback
        {
            using Func = void (*)(void*);
            void* object = nullptr;
            Func function = nullptr;
        };

        Callback callback;
    };   
 
} // namespace STM32WL55JC1

#endif // DRIVERS_INTERRUPT_HANDLER_HPP
