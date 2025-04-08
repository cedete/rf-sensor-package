#ifndef BOARD_BOARD_HPP
#define BOARD_BOARD_HPP

#include <cstdint>
#include <array>

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

        //T0D0: redo this
        /*
        struct Config
        {
        #ifdef UNIT_TEST
            uintptr_t      uart;
            uintptr_t      tx_port;
            GPIO::Pin      tx_pin;
            uintptr_t      rx_port;
            GPIO::Pin      rx_pin;
        #else
            Instance       uart; 
            GPIO::Port     tx_port;
            GPIO::Pin      tx_pin;
            GPIO::Port     rx_port;
            GPIO::Pin      rx_pin;
        #endif // UNIT_TEST
            Mode           mode;
            uint32_t       baud_rate;
        }; 
        */

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

        namespace Radio
        {
            enum class Packet : uint8_t
            {
                GFSK = 0x00,  
                LORA = 0x01,
                BPSK = 0x02,
                GMSK = 0x03,
                NONE = 0x0F
            };

            enum class Ramp : uint8_t 
            {
                TIME_10_US   = 0x00,
                TIME_20_US   = 0x01,
                TIME_40_US   = 0x02,
                TIME_80_US   = 0x03,
                TIME_200_US  = 0x04,
                TIME_800_US  = 0x05,
                TIME_1700_US = 0x06,
                TIME_3400_US = 0x07
            };

            enum class Preamble : uint8_t
            {
                DETECTOR_OFF     = 0x00,
                DETECTOR_08_BITS = 0x04,
                DETECTOR_16_BITS = 0x05,
                DETECTOR_24_BITS = 0x06,
                DETECTOR_32_BITS = 0x07
            };

            namespace PA
            {
                enum class Power : uint8_t
                {
                    LP_PLUS_15_DBM = 0x0E,
                    LP_PLUS_14_DBM = 0x0E,
                    LP_PLUS_10_DBM = 0x0D,
                    HP_PLUS_22_DBM = 0x16,
                    HP_PLUS_20_DBM = 0x16,
                    HP_PLUS_17_DBM = 0x16,
                    HP_PLUS_14_DBM = 0x16
                };
    
                // indexes for PA config array
                static constexpr uint8_t DUTY_CYCLE = 0;
                static constexpr uint8_t HP_MAX     = 1;
                static constexpr uint8_t SEL        = 2;

                // PA config array
                static constexpr std::array<uint8_t, 3> LP_PLUS_15_DBM_SETTING = {0x7, 0x0, 0x1};
                static constexpr std::array<uint8_t, 3> LP_PLUS_14_DBM_SETTING = {0x4, 0x0, 0x1};
                static constexpr std::array<uint8_t, 3> LP_PLUS_10_DBM_SETTING = {0x1, 0x0, 0x1};
                static constexpr std::array<uint8_t, 3> HP_PLUS_22_DBM_SETTING = {0x4, 0x7, 0x0};
                static constexpr std::array<uint8_t, 3> HP_PLUS_20_DBM_SETTING = {0x3, 0x5, 0x0};
                static constexpr std::array<uint8_t, 3> HP_PLUS_17_DBM_SETTING = {0x2, 0x3, 0x0};
                static constexpr std::array<uint8_t, 3> HP_PLUS_14_DBM_SETTING = {0x2, 0x2, 0x0};

            } // namespace PA

            enum class TCXO : uint8_t
            {
                TRIM_1_6_V = 0x0,
                TRIM_1_7_V = 0x1,
                TRIM_1_8_V = 0x2,
                TRIM_2_2_V = 0x3,
                TRIM_2_4_V = 0x4,
                TRIM_2_7_V = 0x5,
                TRIM_3_0_V = 0x6,
                TRIM_3_3_V = 0x7
            };

        } // namespace Radio

        namespace Gfsk
        {
            //T0D0

        }; // namespace Gfsk

        namespace Lora
        {
            enum class Spread : uint8_t
            {
                FACTOR_5  = 0x05,
                FACTOR_6  = 0x06,
                FACTOR_7  = 0x07,
                FACTOR_8  = 0x08,
                FACTOR_9  = 0x09,
                FACTOR_10 = 0x0A,
                FACTOR_11 = 0x0B,
                FACTOR_12 = 0x0C
            };

            enum class Bandwidth : uint8_t
            {
                BW_500 = 6,
                BW_250 = 5,
                BW_125 = 4,
                BW_062 = 3,
                BW_041 = 10,
                BW_031 = 2,
                BW_020 = 9,
                BW_015 = 1,
                BW_010 = 8,
                BW_007 = 0,
            };

            enum class Coding : uint8_t
            {
                RATE_4_5 = 0x01,
                RATE_4_6 = 0x02,
                RATE_4_7 = 0x03,
                RATE_4_8 = 0x04,
            };

            enum class LDRO : uint8_t
            {
                DISABLE = 0x0,
                ENABLE  = 0x1
            };

            enum class Header : uint8_t
            {
                EXPLICIT_VARIABLE_LENGTH = 0x0,
                IMPLICIT_FIXED_LENGTH    = 0x1  
            };

            enum class CRC_Type : uint8_t 
            {
                DISABLE = 0x0,
                ENABLE  = 0x1
            };
   
            enum class Invert_IQ : uint8_t
            {
                STANDARD = 0x0,
                INVERTED = 0x1
            }; 

        } // namespace Lora

        namespace Bpsk
        {
            //T0D0

        } // namespace Bpsk

        namespace Gmsk
        {
            //T0D0

        } // namespace Gmsk
       
        // SUBGHZ configuration struct
        
        struct Config
        {
            Radio::Packet          packet_type;
            uint8_t                packet_length;
            Lora::Header           header_type;
            uint16_t               header_length;        

            Lora::Spread           spread_factor;
            Lora::Bandwidth        bandwidth;
            Lora::Coding           coding_rate;

            Radio::PA::Power       power; 
            Radio::Ramp            ramp_time;
            std::array<uint8_t, 3> pa_config;

            uint32_t               frequency;       
        };
    
    } // namespace SUBGHZ

} // namespace STM32WL55JC1

#endif // BOARD_BOARD_HPP
