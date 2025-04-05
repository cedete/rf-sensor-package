#ifndef DRIVERS_SPI_HPP
#define DRIVERS_SPI_HPP

#include <cstdint>

#include "../board/board.hpp"
#include "../board/clock.hpp"

namespace STM32WL55JC1
{
    namespace SPI
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
       
            // blocking 
            uint8_t transaction(uint8_t byte);

        private:

            #ifndef UNIT_TEST
                void enable_peripheral(const Instance& spi);
                void disable_peripheral();
            #endif // UNIT_TEST

            void enable_spi();
            void disable_spi();

            void set_device(const Device& device);
            void set_mode(const Mode& mode);
            void set_baud(const Baud& baud);
            void set_data_length(const Data& length);
            void set_frame_format(const Frame& format);
            void set_chip_select(const Chip_Select& cs);

            void set_mode(const GPIO::Pin& p, volatile GPIO_TypeDef* reg);
            void set_function(const GPIO::Pin& p, volatile GPIO_TypeDef* reg);

            volatile SPI_TypeDef* SPI_Reg;
            volatile GPIO_TypeDef* MISO_Reg;
            volatile GPIO_TypeDef* MOSI_Reg;
            volatile GPIO_TypeDef* SCK_Reg;
        
        };

    } // namespace SPI
    
} // namespace STM32WL55JC1

#endif // DRIVERS_SPI_HPP
