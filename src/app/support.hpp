#ifndef APP_SUPPORT_HPP
#define APP_SUPPORT_HPP

#include "../drivers/gpio.hpp"
#include "../drivers/uart.hpp"
#include "../drivers/spi.hpp"
// include radio

namespace Sensor_Package
{
    using namespace STM32WL55JC1;

    //
    // Configurations for application peripherals 
    //

    const GPIO::Config led1_config
    {
        GPIO::Port::B,
        GPIO::Pin::P15,
        GPIO::Mode::GPIO,
        GPIO::Type::PUSH_PULL,    
        GPIO::Speed::LOW,
        GPIO::Resistor::NONE,
        GPIO::Function::AF0
    };

    const GPIO::Config pb12_config
    {
        GPIO::Port::B,
        GPIO::Pin::P12,
        GPIO::Mode::GPIO,
        GPIO::Type::PUSH_PULL,    
        GPIO::Speed::LOW,
        GPIO::Resistor::NONE,
        GPIO::Function::AF0
    };

    const UART::Config usart1_config
    {
        UART::Instance::USART_1,
        GPIO::Port::B,  // tx
        GPIO::Port::B,  // rx
        GPIO::Pin::P6,  // tx
        GPIO::Pin::P7,  // rx
        UART::Mode::TX_RX,
        9600
    };

    const UART::Config usart2_config
    {
        UART::Instance::USART_2,
        GPIO::Port::A,  // tx
        GPIO::Port::A,  // rx
        GPIO::Pin::P2,  // tx
        GPIO::Pin::P3,  // rx
        UART::Mode::TX_RX,
        115200
    };

    const SPI::Config spi1_config
    {
        SPI::Instance::SPI_1,
        GPIO::Port::A,
        GPIO::Pin::P5,
        GPIO::Port::A,
        GPIO::Pin::P6,
        GPIO::Port::A,
        GPIO::Pin::P7,
        SPI::Device::MASTER,  
        SPI::Mode::FULL_DUPLEX,
        SPI::Baud::DIVIDER_32,
        SPI::Data::LENGTH_16_BITS,
        SPI::Frame::MSB_FIRST,
    };

    const SUBGHZ::Config radio_config = 
    {
        SUBGHZ::Radio::Packet::LORA,
        255,        // packet length
        SUBGHZ::Lora::Header::IMPLICIT_FIXED_LENGTH,
        255,        // header number of symbols
        SUBGHZ::Lora::Spread::FACTOR_7,
        SUBGHZ::Lora::Bandwidth::BW_125,
        SUBGHZ::Lora::Coding::RATE_4_5,
        SUBGHZ::Radio::PA::Power::HP_PLUS_17_DBM,
        SUBGHZ::Radio::Ramp::TIME_800_US,
        SUBGHZ::Radio::PA::HP_PLUS_17_DBM_SETTING,
        915000000   // frequency
    };

    // 
    // Static instances of used peripherals
    //
/*
    // LED for debugging
    GPIO::Output& led1()
    {
        static GPIO::Output obj(led1_config);

        return obj;
    }
*/
    
    // SPI chip select 
    GPIO::Output& pb12()
    {
        static GPIO::Output obj(pb12_config);
        
        return obj;
    }

    // Read gps    
    UART::Driver& usart1()
    {
        static UART::Driver obj(usart1_config);

        return obj;
    } 

    // Serial terminal output for debugging
    UART::Driver& usart2()
    {
        static UART::Driver obj(usart2_config);
    
        return obj;
    }

    // Read rf meter
    SPI::Driver& spi1()
    {
        static SPI::Driver obj(spi1_config);
        
        return obj; 
    } 

    //T0D0:

    // SUBGHZ 

} // namespace Sensor_Package

#endif // APP_SUPPORT_HPP
