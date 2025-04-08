#ifndef DRIVERS_SUBGHZ_HPP
#define DRIVERS_SUBGHZ_HPP

#include <array>

#include "../board/board.hpp"
#include "../board/clock.hpp"
#include "../board/interrupts.hpp"
#include "../common/ring_buffer.h"

namespace STM32WL55JC1
{
    namespace SUBGHZ
    {

        //T0D0:
        // maybe put static constexpr variables here for: 
        // - RF freq calculations
        // - 

        class Driver
        {            
        public:
            enum class Mode : uint8_t              // radio mode
            {
                DEEP_SLEEP  = 0x0,
                SLEEP       = 0x1,
                STANDBY_RC  = 0x2,
                STANDBY_HSE = 0x3,
                FS          = 0x4,
                RX          = 0x5,
                TX          = 0x6 
            };

            enum class Status : uint8_t             // error status
            {
                NOT_DONE,
                DONE
            }; 

            enum class Command_Status : uint8_t     // command status
            {
                RESERVED       = 0x0,
                RFU            = 0x1,
                DATA_AVAILABLE = 0x2,
                TIMEOUT        = 0x3,
                PROCESS_ERROR  = 0x4,
                EXEC_FAILURE   = 0x5,
                TX_DONE        = 0x6 
            };

            explicit Driver(const Config& config);  

            Driver()                             = delete;
            Driver(const Driver& rhs)            = delete;
            Driver(Driver&& rhs)                 = delete;
            Driver& operator=(const Driver& rhs) = delete;
            Driver& operator=(Driver&& rhs)      = delete;

            ~Driver();

            void transmit(Ring_Buffer<uint8_t, 255>& buffer);  
            void receive();
            void transfer_data(Ring_Buffer<uint8_t, 255>& buffer); 

            uint8_t get_packet_type(); 
            Mode get_mode();
            Command_Status get_command_status();
/*
            uint32_t get_packet_status();        // change return type to enum
            // get rx buffer status not needed by public interface
*/
        private:
            void set_mode(Mode mode);

            class SX126X
            {
            public:
                SX126X();
                ~SX126X();

                void transmit(Ring_Buffer<uint8_t, 255>& buffer);
                void receive();
                void isr();
                void transfer_data(Ring_Buffer<uint8_t, 255>& buffer);


                void set_mode(Driver::Mode mode);                
                void set_tcxo(Radio::TCXO trim, 
                              uint32_t timeout = 0);
                void set_packet_type(Radio::Packet packet);
                void set_modulation_params(Lora::Spread factor, 
                                           Lora::Bandwidth bandwidth,
                                           Lora::Coding rate,
                                           Lora::LDRO ldro);
                void set_packet_params(uint16_t preamble_length,
                                       Lora::Header header,
                                       uint8_t payload_length,
                                       Lora::CRC_Type type,
                                       Lora::Invert_IQ setup);
                void set_frequency(uint32_t frequency);
                void set_tx_params(Radio::PA::Power power, 
                                   Radio::Ramp time);
                void set_pa_config(const std::array<uint8_t, 3>& settings);
                void set_buffer_base_addr(uint8_t tx_addr, 
                                          uint8_t rx_addr);
                void set_interrupts();

                uint8_t get_packet_type(); 
                uint8_t get_mode();
                uint8_t get_command_status();      
                /*
                uint32_t get_packet_status();        
                */
                uint16_t get_rx_buffer_status();

                // void wake_up(); 
                // void reset(); 

            private:
                // Radio command opcodes
                enum class Get : uint8_t
                {
                    ERROR            = 0x17,
                    IRQ_STATUS       = 0x12,
                    PACKET_STATUS    = 0x14,
                    PACKET_TYPE      = 0x11,
                    RSSI_INST        = 0x15,
                    RX_BUFFER_STATUS = 0x13,
                    STATS            = 0x10,
                    STATUS           = 0xC0 
                };

                enum class Set : uint8_t
                {
                    BUFFER_BASE_ADDRESS       = 0x8F,
                    CAD                       = 0xC5,
                    CAD_PARAMS                = 0x88,
                    FS                        = 0xC1,
                    LORA_SYMB_TIMEOUT         = 0xA0,
                    MODULATION_PARAMS         = 0x8B,
                    PA_CONFIG                 = 0x95,
                    PACKET_PARAMS             = 0x8C,
                    PACKET_TYPE               = 0x8A,
                    REGULATOR_MODE            = 0x96,
                    RF_FREQUENCY              = 0x86,
                    RX                        = 0x82,
                    RX_DUTY_CYCLE             = 0x94,
                    TX_RX_FALLBACK_MODE       = 0x93,
                    SLEEP                     = 0x84,
                    STANDBY                   = 0x80,
                    STOP_RX_TIMER_ON_PREAMBLE = 0x9F,
                    TCXO_MODE                 = 0x97,
                    TX                        = 0x83,
                    TX_CONTINUOUS_WAVE        = 0xD1,
                    TX_CONTINUOUS_PREAMBLE    = 0xD2,
                    TX_PARAMS                 = 0x8E
                };

                enum class Read : uint8_t
                {
                    BUFFER   = 0x1E,
                    REGISTER = 0x1D
                };

                enum class Write : uint8_t
                {
                    BUFFER   = 0x0E,
                    REGISTER = 0x0D
                };

                enum class Config : uint8_t
                {
                    IRQ = 0x08
                };

                enum class Clear : uint8_t
                {
                    IRQ = 0x02
                };

                // DIO interrupts
                static constexpr uint8_t  TX_DONE           = (1 << 0);
                static constexpr uint8_t  RX_DONE           = (1 << 1);
                static constexpr uint8_t  PREAMBLE_DETECTED = (1 << 2);
                static constexpr uint8_t  SYNC_DETECTED     = (1 << 3); 
                static constexpr uint8_t  HEADER_VALID      = (1 << 4); 
                static constexpr uint8_t  HEADER_ERR        = (1 << 5); 
                static constexpr uint8_t  ERR               = (1 << 6); 
                static constexpr uint8_t  CAD_DONE          = (1 << 7); 
                static constexpr uint16_t CAD_DETECTED      = (1 << 8); 
                static constexpr uint16_t TIMEOUT           = (1 << 9); 

                void enable_interrupts();
                void enable();
                void disable();

                template<std::size_t cmd_size, std::size_t status_size>
                void read(const std::array<uint8_t, cmd_size>& command, 
                          std::array<uint8_t, status_size>& status); 
                
                template<std::size_t size>
                void write(const std::array<uint8_t, size>& command); 

                // so read/write functions know what offset to write to
                uint8_t rx_buffer_offset;  
                uint8_t tx_buffer_offset;

                bool irq_active;
                
                volatile RCC_TypeDef* RCC_Reg;
                volatile PWR_TypeDef* PWR_Reg;
        
                class SPI
                {
                public:
                    SPI();
                    ~SPI();
               
                    uint8_t transaction(uint8_t byte);

                private:
                    void enable();
                    void disable();
                    void configure();

                    volatile RCC_TypeDef* RCC_Reg;
                    volatile SPI_TypeDef* SPI_Reg;

                };  // class SPI

                SPI spi;

            };  // class SX126X

            SX126X radio;

        }; // class Driver

    } // namespace SUBGHZ

} // namespace STM32WL55JC1

#endif // DRIVERS_SUBGHZ_HPP
