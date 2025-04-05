#ifndef BOARD_BOARD_HPP
#define BOARD_BOARD_HPP

#include <cstdint>

#include "../CMSIS/Device/ST/STM32WLxx/Include/stm32wl55xx.h"

namespace STM32WL55JC1
{

    // -------------------------------- GPIO -----------------------------------
    // -------------------------------------------------------------------------

    namespace GPIO
    {
        //
        // GPIO configuration settings
        //

        enum class Port : uint32_t
        {
            A = GPIOA_BASE,
            B = GPIOB_BASE,
            C = GPIOC_BASE,
            H = GPIOH_BASE
        };

        enum class Pin : uint8_t
        {   
            P0,  P1,  P2,  P3,  P4,  P5,  P6,  P7,  P8,  P9,  P10,  P11,  P12, 
            P13, P14, P15    
        };

        enum class Mode : uint8_t
        {   
            INPUT, GPIO, ALT, ANALOG    
        };

        enum class Type: uint8_t
        {   
            PUSH_PULL, OPEN_DRAIN   
        };

        enum class Speed : uint8_t
        {   
            LOW, MEDIUM, FAST, HIGH   
        };

        enum class Resistor : uint8_t
        {   
            NONE, PULLUP, PULLDOWN  
        };

        enum class Function : uint8_t
        {   
            AF0,  AF1,  AF2,  AF3,  AF4,  AF5,  AF6,  AF7,  AF8,  AF9,  AF10,
            AF11, AF12, AF13, AF14, AF15  
        };

        // Pin configuration struct

        struct Config
        {
        #ifdef UNIT_TEST
            uintptr_t port;
        #else
            Port      port; 
        #endif // UNIT_TEST
            Pin       pin;
            Mode      mode;
            Type      type;
            Speed     speed;
            Resistor  resistor;
            Function  function;
        };

    } // namespace GPIO

    // -------------------------------- Timer ----------------------------------
    // -------------------------------------------------------------------------

    namespace Timer
    {
        //
        // Timer configuration settings
        // 

        enum class Instance : uint32_t
        {
            TIMER_1  = TIM1_BASE,
            TIMER_2  = TIM2_BASE,
            TIMER_16 = TIM16_BASE,
            TIMER_17 = TIM17_BASE
        };

        //T0D0
        // - capture compare
        // - interrupts

        // Timer configuration struct        

        struct Config
        {
        #ifdef UNIT_TEST
            uintptr_t timer;
        #else
            Instance  timer;
        #endif // UNIT_TEST
            uint32_t  frequency;
            uint32_t  reload;
        };

    } // namespace Timer

    // -------------------------------- UART -----------------------------------
    // -------------------------------------------------------------------------

    namespace UART
    {
        // 
        // UART configuration settings
        //

        enum class Instance : uint32_t
        {
            USART_1 = USART1_BASE,
            USART_2 = USART2_BASE
        };

        enum class Mode : uint8_t
        {
            TX, RX, TX_RX
        };

        enum class Data: uint8_t
        {
            LENGTH_8 = 0x0,
            LENGTH_9 = 0x1,
            LENGTH_7 = 0x2
        };

        enum class Stop : uint8_t
        {
            BITS_1   = 0x0,
            BITS_0_5 = 0x1, // 0.5 bits
            BITS_2   = 0x2,
            BITS_1_5 = 0x3  // 1.5 bits
        };

        enum class Oversampling : uint8_t
        {
            BY_16 = 0,
            BY_8  = 1
        };

        // UART configuration struct

        struct Config
        {
        #ifdef UNIT_TEST
            uintptr_t      uart;
            uintptr_t      tx_port;
            uintptr_t      rx_port;
        #else
            Instance       uart; 
            GPIO::Port     tx_port;
            GPIO::Port     rx_port;
        #endif // UNIT_TEST
            GPIO::Pin      tx_pin;
            GPIO::Pin      rx_pin;
            Mode           mode;
            uint32_t       baud_rate;
        }; 

    } // namespace UART

    // --------------------------------- SPI -----------------------------------
    // -------------------------------------------------------------------------

    namespace SPI
    {
        // 
        // SPI configuration settings
        //

        enum class Instance : uint32_t
        {
            SPI_1 = SPI1_BASE, 
            SPI_2 = SPI2_BASE,
            SUBGHZ_SPI = SUBGHZSPI_BASE   
        };

        enum class Device : uint8_t
        {
            SLAVE, MASTER 
        };

        enum class Mode : uint8_t
        {
            FULL_DUPLEX, HALF_DUPLEX_TX, HALF_DUPLEX_RX, SIMPLEX_RX
        };
    
        enum class Baud : uint8_t
        {
            DIVIDER_2,  DIVIDER_4,   DIVIDER_8,   DIVIDER_16,   DIVIDER_32,
            DIVIDER_64, DIVIDER_128, DIVIDER_256 
        };

        enum class Data : uint8_t
        {
            LENGTH_4_BITS = 0x3,
            LENGTH_5_BITS = 0x4,
            LENGTH_6_BITS = 0x5,
            LENGTH_7_BITS = 0x6,
            LENGTH_8_BITS = 0x7,
            LENGTH_9_BITS = 0x8,
            LENGTH_10_BITS = 0x9,
            LENGTH_11_BITS = 0xA,
            LENGTH_12_BITS = 0xB,
            LENGTH_13_BITS = 0xC,
            LENGTH_14_BITS = 0xD,
            LENGTH_15_BITS = 0xE,
            LENGTH_16_BITS = 0xF
        };

        enum class Frame : uint8_t
        {
            MSB_FIRST, LSB_FIRST
        };

        enum class Chip_Select : uint8_t
        {
            HARDWARE, SOFTWARE
        };

        // SPI configuration struct

        struct Config
        {
        #ifdef UNIT_TEST
            uintptr_t   spi;
            uintptr_t   sck;
            GPIO::Pin   sck_pin;
            uintptr_t   miso;
            GPIO::Pin   miso_pin;
            uintptr_t   mosi;
            GPIO::Pin   mosi_pin;
        #else 
            Instance    spi; 
            GPIO::Port  sck;
            GPIO::Pin   sck_pin;
            GPIO::Port  miso;
            GPIO::Pin   miso_pin;
            GPIO::Port  mosi;
            GPIO::Pin   mosi_pin;
        #endif // UNIT_TEST
            Device      device;
            Mode        mode;
            Baud        rate;
            Data        length;
            Frame       format;
        };

    } // namespace SPI

    // ------------------------------- SUBGHZ ----------------------------------
    // -------------------------------------------------------------------------

    namespace SUBGHZ
    {
        // 
        // SUBGHZ configuration settings
        //

        enum class Bandwidth : uint8_t
        {
        };

        enum class Spread : uint8_t
        {
            FACTOR_10_PS,
            FACTOR_20_PS,
            FACTOR_40_PS,
            FACTOR_80_PS,
            FACTOR_200_PS,
            FACTOR_800_PS,
            FACTOR_1700_PS,
            FACTOR_3400_PS
        };

        enum class Power : uint8_t
        {
        };

        enum class Ramp_Time : uint8_t 
        {
        };

        enum class Packet : uint8_t
        {
            FSK,  
            LORA,
            BPSK,
            MSK
        };

        enum class Frame : uint8_t
        {
        };

        // SUBGHZ configuration struct

        struct Config
        {
            // these will be hardcoded for now
        };
    
    } // namespace SUBGHZ

} // namespace STM32WL55JC1

#endif // BOARD_BOARD_HPP
