#include "subghz.hpp"

using namespace STM32WL55JC1;

// -----------------------------------------------------------------------------
// @class Driver
//
// @brief Public interface of SX126x radio module and its SPI communication
//        interface. 
// -----------------------------------------------------------------------------

SUBGHZ::Driver::Driver(const Config& config) : radio()
{
    // radio initialization before it can receive other commands
    radio.set_tcxo(Radio::TCXO::TRIM_3_3_V, 64000);    
    set_mode(Mode::STANDBY_HSE);        // potentially change to RC to test staying in TX mode

    // configure radio settings
    radio.set_packet_type(config.packet_type);
    radio.set_modulation_params(config.spread_factor, config.bandwidth, 
                                config.coding_rate, Lora::LDRO::DISABLE);
    radio.set_packet_params(config.packet_length, config.header_type, 
                            config.header_length, Lora::CRC_Type::DISABLE, 
                            Lora::Invert_IQ::STANDARD);
    radio.set_frequency(config.frequency);      
    radio.set_tx_params(config.power, config.ramp_time);
    radio.set_pa_config(config.pa_config);
    radio.set_buffer_base_addr(0x80, 0x00); // default 
    radio.set_interrupts();   
}

SUBGHZ::Driver::~Driver()
{}

void SUBGHZ::Driver::transmit(Ring_Buffer<uint8_t, 255>& buffer)
{
    radio.transmit(buffer);
}

void SUBGHZ::Driver::receive()
{
    radio.receive();
}

void SUBGHZ::Driver::transfer_data(Ring_Buffer<uint8_t, 255>& buffer)
{
    radio.transfer_data(buffer);
}

uint8_t SUBGHZ::Driver::get_packet_type()
{
    return radio.get_packet_type();
}

SUBGHZ::Driver::Mode SUBGHZ::Driver::get_mode()
{
    return static_cast<SUBGHZ::Driver::Mode>(radio.get_mode());
}

SUBGHZ::Driver::Command_Status SUBGHZ::Driver::get_command_status()
{
    return static_cast<SUBGHZ::Driver::Command_Status>(radio.get_command_status());
}

/*
uint32_t SUBGHZ::Driver::get_packet_status()
{
}
*/
void SUBGHZ::Driver::set_mode(Mode mode)
{
    //T0D0:
    // implement error checking to where the state is only changed if set_mode
    // returns a successful message (it will send a get status and verify that
    // the mode was switched to.

    radio.set_mode(mode);
}

// -----------------------------------------------------------------------------
// @class SX126X 
//
// @brief Higher level interaction with Semtech SX126x radio module to configure
//        and use the device. 
// -----------------------------------------------------------------------------

SUBGHZ::Driver::SX126X::SX126X() 
    : rx_buffer_offset{0x00}, tx_buffer_offset{0x80}, irq_active{false}, spi() 
{
    #ifdef UNIT_TEST
//        RCC_Reg = reinterpret_cast<volatile RCC_TypeDef*>(config.mock_rcc);
//        PWR_Reg = reinterpret_cast<volatile PWR_TypeDef*>(config.mock_pwr);
    #else
        RCC_Reg = reinterpret_cast<volatile RCC_TypeDef*>(RCC_BASE);
        PWR_Reg = reinterpret_cast<volatile PWR_TypeDef*>(PWR_BASE);
    #endif

    static_assert(sizeof(*RCC_Reg) == 0x188, "RCC registers not aligned"); 
    static_assert(sizeof(*PWR_Reg) == 0x09C, "PWR registers not aligned"); 

    enable();
    enable_interrupts();

    IRQ_Handlers::irq_handler().connect<&SUBGHZ::Driver::SX126X::isr>(this);
}

SUBGHZ::Driver::SX126X::~SX126X()
{
    disable();
}

void SUBGHZ::Driver::SX126X::transmit(Ring_Buffer<uint8_t, 255>& buffer)
{
    std::array<uint8_t, 257> cmd = {};
    cmd[0] = static_cast<uint8_t>(Write::BUFFER);
    cmd[1] = tx_buffer_offset;
    for (std::size_t i = 2; !buffer.empty(); ++i)
    {
        cmd[i] = buffer.get(); 
    }

    write<cmd.size()>(cmd);

    set_mode(Mode::TX); // after tx complete, it will go back to standby mode
}

void SUBGHZ::Driver::SX126X::receive()
{
    set_mode(Mode::RX);
}

void SUBGHZ::Driver::SX126X::isr()
{
    // send command to determine which interrupt fired 
    std::array<uint8_t, 1> irq_cmd = { static_cast<uint8_t>(Get::IRQ_STATUS) };
    std::array<uint8_t, 3> status = {};

    read<irq_cmd.size(), status.size()>(irq_cmd, status);

    // read irq
    uint16_t interrupt = (status[1] << 8) | status[2]; 

    if (interrupt == RX_DONE)
    {
        irq_active = true; 
    }

    std::array<uint8_t, 3> clr_cmd = 
    {
        static_cast<uint8_t>(Clear::IRQ),
        0x00,
        0x02    // clear RX Done
    };

    write<clr_cmd.size()>(clr_cmd);
}

void SUBGHZ::Driver::SX126X::transfer_data(Ring_Buffer<uint8_t, 255>& buffer)
{
    if (irq_active)
    {
        std::array<uint8_t, 2> rx_cmd = 
        { 
            static_cast<uint8_t>(Read::BUFFER),  
            rx_buffer_offset
        };
        std::array<uint8_t, 255> data = {};
        
        read<rx_cmd.size(), data.size()>(rx_cmd, data);

        for (auto& idx : data)
        {
            buffer.put(idx);
        }
       
        irq_active = false;
    }
}


void SUBGHZ::Driver::SX126X::set_mode(Driver::Mode mode)
{
    switch(mode)
    {
        case Mode::DEEP_SLEEP:
        {
            std::array<uint8_t, 2> cmd = 
            { 
                static_cast<uint8_t>(Set::SLEEP),
                0x00        //T0D0: make these contexpr variables
            }; 
            write<cmd.size()>(cmd);
            break;
        }
        case Mode::SLEEP:
        {
            std::array<uint8_t, 2> cmd = 
            { 
                static_cast<uint8_t>(Set::SLEEP),
                0x04        //T0D0: make these contexpr variables
            }; 
            write<cmd.size()>(cmd);
            break;
        }
        case Mode::STANDBY_RC:
        {
            std::array<uint8_t, 2> cmd = 
            { 
                static_cast<uint8_t>(Set::STANDBY),
                0x00
            }; 
            write<cmd.size()>(cmd);
            break;
        }
        case Mode::STANDBY_HSE:
        {
            std::array<uint8_t, 2> cmd = 
            { 
                static_cast<uint8_t>(Set::STANDBY),
                0x01
            }; 
            write<cmd.size()>(cmd);
            break;
        }
        case Mode::FS:
        {
            std::array<uint8_t, 1> cmd = 
            { 
                static_cast<uint8_t>(Set::FS) 
            }; 
            write<cmd.size()>(cmd);
            break;
        }
        case Mode::TX:
        {
            std::array<uint8_t, 4> cmd = 
            { 
                static_cast<uint8_t>(Set::TX),
                0x00,       //T0D0 convert the timeout to an actual parameter
                0x00,
                0x00
            }; 
            write<cmd.size()>(cmd);
            break;
        }
        case Mode::RX:
        {
            std::array<uint8_t, 4> cmd = 
            { 
                static_cast<uint8_t>(Set::RX),
                0xFF,       //T0D0 convert the timeout to an actual parameter
                0xFF,
                0xFF
            }; 
            write<cmd.size()>(cmd);
            break;
        }
    }
}

void SUBGHZ::Driver::SX126X::set_tcxo(Radio::TCXO trim, uint32_t timeout)
{
    std::array<uint8_t, 5> cmd = 
    {
        static_cast<uint8_t>(Set::TCXO_MODE),
        static_cast<uint8_t>(trim),
        static_cast<uint8_t>(timeout >> 16),
        static_cast<uint8_t>(timeout >> 8),
        static_cast<uint8_t>(timeout >> 0)
    };   
 
    write<cmd.size()>(cmd);
}

void SUBGHZ::Driver::SX126X::set_packet_type(Radio::Packet packet)
{
    std::array<uint8_t, 2> cmd =
    {
        static_cast<uint8_t>(Set::PACKET_TYPE), 
        static_cast<uint8_t>(packet)
    };

    write<cmd.size()>(cmd);
}

void SUBGHZ::Driver::SX126X::set_modulation_params(Lora::Spread factor, 
                                                   Lora::Bandwidth bandwidth,
                                                   Lora::Coding rate,
                                                   Lora::LDRO ldro)
{
    std::array<uint8_t, 5> cmd =
    {
        static_cast<uint8_t>(Set::MODULATION_PARAMS),
        static_cast<uint8_t>(factor),
        static_cast<uint8_t>(bandwidth),
        static_cast<uint8_t>(rate),
        static_cast<uint8_t>(ldro)
    };

    write<cmd.size()>(cmd);
}

void SUBGHZ::Driver::SX126X::set_packet_params(uint16_t preamble_length,
                                               Lora::Header header,
                                               uint8_t payload_length,
                                               Lora::CRC_Type type,
                                               Lora::Invert_IQ setup)
{
    std::array<uint8_t, 7> cmd =
    {
        static_cast<uint8_t>(Set::PACKET_PARAMS),
        static_cast<uint8_t>(preamble_length >> 8),
        static_cast<uint8_t>(preamble_length >> 0),
        static_cast<uint8_t>(header),
        payload_length,
        static_cast<uint8_t>(type),
        static_cast<uint8_t>(setup)
    };     

    write<cmd.size()>(cmd);
}

void SUBGHZ::Driver::SX126X::set_frequency(uint32_t frequency)
{
    uint64_t pll_freq = frequency << 25 / 32000000; // pll step calculation
    pll_freq = static_cast<uint32_t>(pll_freq);     // narrow down just in case??

    std::array<uint8_t, 5> cmd = 
    {
        static_cast<uint8_t>(Set::RF_FREQUENCY),
        static_cast<uint8_t>(pll_freq >> 24),
        static_cast<uint8_t>(pll_freq >> 16),
        static_cast<uint8_t>(pll_freq >> 8),
        static_cast<uint8_t>(pll_freq >> 0)
    };

    write<cmd.size()>(cmd);
}

void SUBGHZ::Driver::SX126X::set_tx_params(Radio::PA::Power power, Radio::Ramp time)
{
    std::array<uint8_t, 3> cmd = 
    {
        static_cast<uint8_t>(Set::TX_PARAMS),
        static_cast<uint8_t>(power),
        static_cast<uint8_t>(time)
    };

    write<cmd.size()>(cmd);
}

void SUBGHZ::Driver::SX126X::set_pa_config(const std::array<uint8_t, 3>& settings)
{
    std::array<uint8_t, 5> cmd =
    {
        static_cast<uint8_t>(Set::PA_CONFIG), 
        settings[Radio::PA::DUTY_CYCLE],
        settings[Radio::PA::HP_MAX],
        settings[Radio::PA::SEL], 
        0x01                // reserved, must be this
    };

    write<cmd.size()>(cmd);
}

void SUBGHZ::Driver::SX126X::set_buffer_base_addr(uint8_t tx_addr, uint8_t rx_addr)
{
    std::array<uint8_t, 3> cmd =
    {
        static_cast<uint8_t>(Set::BUFFER_BASE_ADDRESS),
        tx_addr,
        rx_addr 
    };

    write<cmd.size()>(cmd);

    // Update base address for radio buffers for use by write/read functions
    tx_buffer_offset = tx_addr;
    rx_buffer_offset = rx_addr;
}

void SUBGHZ::Driver::SX126X::set_interrupts()
{
    uint16_t dio_mask = RX_DONE;

    std::array<uint8_t, 9> cmd =
    {
        static_cast<uint8_t>(Config::IRQ),
        static_cast<uint8_t>(dio_mask >> 8),    // DIO global mask
        static_cast<uint8_t>(dio_mask >> 0),
        static_cast<uint8_t>(dio_mask >> 8),    // DIO1 mask 
        static_cast<uint8_t>(dio_mask >> 0),
        0x00,                                   // DIO2 mask (unused) 
        0x00,                           
        0x00,                                   // DIO3 mask (unused)
        0x00
    };

    write<cmd.size()>(cmd);
}

uint8_t SUBGHZ::Driver::SX126X::get_packet_type()
{
    std::array<uint8_t, 1> cmd = { static_cast<uint8_t>(Get::PACKET_TYPE) };
    std::array<uint8_t, 2> status = {};

    read<cmd.size(), status.size()>(cmd, status);

    return status[1];
}

uint8_t SUBGHZ::Driver::SX126X::get_mode()
{
    std::array<uint8_t, 2> cmd = { static_cast<uint8_t>(Get::STATUS) };
    std::array<uint8_t, 1> status = {};

    read<cmd.size(), status.size()>(cmd, status);

    return ( (status[0] & 0x70) >> 4 );
}

uint8_t SUBGHZ::Driver::SX126X::get_command_status()
{
    std::array<uint8_t, 2> cmd = { static_cast<uint8_t>(Get::STATUS) };
    std::array<uint8_t, 1> status = {};

    read<cmd.size(), status.size()>(cmd, status);

    return ( (status[0] & 0x0E) >> 1 );
}

/*
uint32_t SUBGHZ::Driver::SX126X::get_packet_status()
{
}
*/

uint16_t SUBGHZ::Driver::SX126X::get_rx_buffer_status()
{
    std::array<uint8_t, 1> cmd = { static_cast<uint8_t>(Get::RX_BUFFER_STATUS) };    
    std::array<uint8_t, 3> status = {};

    read<cmd.size(), status.size()>(cmd, status);

    return ( static_cast<uint16_t>(status[1] << 8) & status[2] );
}

void SUBGHZ::Driver::SX126X::enable_interrupts()
{
    PWR_Reg->CR3 |= PWR_CR3_EWRFIRQ;                    // enable IRQ in PWR reg

    NVIC_SetPriorityGrouping(0);                        // high priority

    uint32_t encoding = NVIC_EncodePriority(0,10,0);    // equal priority to USART

    NVIC_SetPriority(SUBGHZ_Radio_IRQn, encoding);

    NVIC_EnableIRQ(SUBGHZ_Radio_IRQn);

    EXTI->IMR2 |= EXTI_IMR2_IM44;                       // enable radio specific IRQ
}

void SUBGHZ::Driver::SX126X::enable()
{
    RCC_Reg->CR |= RCC_CR_HSEBYPPWR;            // enable HSE VDDTCXO output

    while (RCC_Reg->CR & RCC_CR_HSERDY) {};     // wait until TCXO is activated

    RCC_Reg->CR |= RCC_CR_HSEON;                // enable HSE clock

    RCC_Reg->CSR &= ~RCC_CSR_RFRST;             // disable radio reset

    PWR_Reg->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS;// set chip select high

    PWR_Reg->CR3 |= PWR_CR3_EWRFBUSY;           // enable radio wakeup 

    PWR_Reg->SCR = PWR_SCR_CWRFBUSYF;           // clear radio busy flag
}

void SUBGHZ::Driver::SX126X::disable()
{
    RCC_Reg->CSR |= RCC_CSR_RFRST;              // reset radio

    RCC_Reg->CR &= ~RCC_CR_HSEON;               // disable HSE clock

    RCC_Reg->CR &= ~RCC_CR_HSEBYPPWR;           // disable HSE VDDTCXO output
}

template<std::size_t cmd_size, std::size_t status_size>
void SUBGHZ::Driver::SX126X::read(const std::array<uint8_t, cmd_size>& command,
                                  std::array<uint8_t, status_size>& status)
{
    //T0D0:
    // if (state == sleep)
    // wake_up_device();

    PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS;   // set chip select low 

    for (auto& idx : command)
    {
        static_cast<void>(spi.transaction(idx));// send opcode and parameters
    }
    
    for (auto& idx : status)
    {
        idx = spi.transaction(0xFF);            // read received status bytes
    }

    PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS;    // set chip select high

    while (PWR->SR2 & PWR_SR2_RFBUSYS) {};      // wait until process completed
}

template<std::size_t size>
void SUBGHZ::Driver::SX126X::write(const std::array<uint8_t, size>& command)  
{
    //T0D0:
    // if (state == sleep)
    //wake_up_device();

    PWR->SUBGHZSPICR &= ~PWR_SUBGHZSPICR_NSS;   // set chip select low 

    for (auto& idx : command)                   
    {
        static_cast<void>(spi.transaction(idx));// send opcode and parameters
    }
    
    static_cast<void>(spi.transaction(0xFF));   // dummy read 

    PWR->SUBGHZSPICR |= PWR_SUBGHZSPICR_NSS;    // set chip select high

    if (command[0] != static_cast<uint8_t>(Set::SLEEP)) 
    {
        while (PWR->SR2 & PWR_SR2_RFBUSYS) {};  // wait until process completed
    }
}

// -----------------------------------------------------------------------------
// @class SPI
//
// @brief Custom SPI communication interface with SX126x radio module.
// -----------------------------------------------------------------------------

SUBGHZ::Driver::SX126X::SPI::SPI()
{
    #ifdef UNIT_TEST
//        RCC_Reg = reinterpret_cast<volatile RCC_TypeDef*>(config.mock_rcc);
//        SPI_Reg = reinterpret_cast<volatile SPI_TypeDef*>(config.mock_spi);
    #else
        RCC_Reg = reinterpret_cast<volatile RCC_TypeDef*>(RCC_BASE);
        SPI_Reg = reinterpret_cast<volatile SPI_TypeDef*>(SUBGHZSPI_BASE);
    #endif

    static_assert(sizeof(*RCC_Reg) == 0x188, "RCC registers not aligned"); 
    static_assert(sizeof(*SPI_Reg) == 0x024, "SPI registers not aligned"); 

    enable();
    configure();
}

SUBGHZ::Driver::SX126X::SPI::~SPI()
{
    disable();
}

uint8_t SUBGHZ::Driver::SX126X::SPI::transaction(uint8_t byte)
{
    uint8_t data;

    while (!(SPI_Reg->SR & SPI_SR_TXE)) {};     // wait for empty register
    *(reinterpret_cast<volatile uint8_t*>(&SPI_Reg->DR)) = byte;  // send data

    while (!(SPI_Reg->SR & SPI_SR_RXNE)) {};    // wait for empty register
    data = static_cast<uint8_t>(SPI_Reg->DR);   // read data
    
    return data; 
}

void SUBGHZ::Driver::SX126X::SPI::enable()
{
    RCC_Reg->APB3ENR |= RCC_APB3ENR_SUBGHZSPIEN;    // enable SPI clock 
}

void SUBGHZ::Driver::SX126X::SPI::disable()
{
    RCC_Reg->APB3RSTR |= RCC_APB3RSTR_SUBGHZSPIRST; // disable SPI clock 
}

void SUBGHZ::Driver::SX126X::SPI::configure()
{
    SPI_Reg->CR1 &= ~SPI_CR1_SPE;       // disable SPI to configure

    SPI_Reg->CR1 = (SPI_CR1_MSTR |      // master mode
                    SPI_CR1_SSI  |      // software slave management 
                    SPI_CR1_BR_0 |      // baud rate divider by 128 
                    SPI_CR1_BR_1 |    
                    SPI_CR1_SSM);       // slave select management

    SPI_Reg->CR2 = (SPI_CR2_FRXTH |     // read 8 bit data 
                    SPI_CR2_DS_0  |     // configure data size as 8 bits 
                    SPI_CR2_DS_1  |   
                    SPI_CR2_DS_2);    

    SPI_Reg->CR1 |= SPI_CR1_SPE;        // enable SPI
}
