#ifndef DRIVERS_GPIO_HPP
#define DRIVERS_GPIO_HPP

#include "../board/board.hpp"

namespace STM32WL55JC1
{
    namespace GPIO
    {
		class Output
		{
		public:
			explicit Output(const Config& config);

            Output()                             = delete;
			Output(const Output& rhs)            = delete;
			Output(Output&& rhs)                 = delete;
			Output& operator=(const Output& rhs) = delete;
			Output& operator=(Output&& rhs)      = delete;

			~Output();

   			void reset();
			void set();
            void toggle();

		private:

            #ifndef UNIT_TEST
                void enable_peripheral(const Port& port);
                void disable_peripheral();
            #endif // UNIT_TEST

            void set_mode(const Mode& mode);                
            void set_output_type(const Type& type);        
            void set_speed(const Speed& speed);           
            void set_resistor(const Resistor& resistor); 
            void set_function(const Function& function);

			volatile GPIO_TypeDef* GPIO;

			uint8_t pin;
		};

    }; // namespace GPIO

} // namespace STM32WL55JC1

#endif // DRIVERS_GPIO_HPP
