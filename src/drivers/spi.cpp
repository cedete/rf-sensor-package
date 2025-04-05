#include "spi.hpp"

using namespace STM32WL55JC1;

//
// Register bit definitions
//

namespace
{
    // control register 1
    static constexpr uint32_t BIDIMODE = (1 << 15);// Bidirectional Data Mode Enable
    static constexpr uint32_t BIDIOE   = (1 << 14);// Bidirectional Output Enable
    static constexpr uint32_t RXONLY   = (1 << 10);// Receive Only Enabled
    static constexpr uint8_t  SSM      = 9;        // Software Slave Management
    static constexpr uint32_t SSI      = (1 << 8); // Interal Slave Select
    static constexpr uint8_t  LSBFIRST = 7;        // Frame Format
    static constexpr uint32_t SPE      = (1 << 6); // SPI Enable
    static constexpr uint8_t  BR       = 3;        // Baud Rate[0]
    static constexpr uint8_t  MSTR     = 2;        // Master selection

    // control register 2
    static constexpr uint32_t FRXTH = (1 << 12);   // FIFO Reception Threshold
    static constexpr uint8_t DS = 8;               // Data Size

    // status register
    constexpr uint32_t BSY     = (1 << 7);         // Busy
	constexpr uint8_t TXE	   = (1 << 1);         // Transmit Buffer Empty
	constexpr uint8_t RXNE	   = (1 << 0);         // Receive Buffer Not Empty
}

//
// Constructor
//

SPI::Driver::Driver(const SPI::Config& config)
{
    SPI_Reg = reinterpret_cast<volatile SPI_TypeDef*>(config.spi);
    SCK_Reg = reinterpret_cast<volatile GPIO_TypeDef*>(config.sck);
    MISO_Reg = reinterpret_cast<volatile GPIO_TypeDef*>(config.miso);
    MOSI_Reg = reinterpret_cast<volatile GPIO_TypeDef*>(config.mosi);

    static_assert(sizeof(*SPI_Reg) == 0x24, "SPI Registers not aligned");
    static_assert(sizeof(*SCK_Reg) == 0x2C, "SCK Registers not aligned");
    static_assert(sizeof(*MISO_Reg) == 0x2C, "MISO Registers not aligned");
    static_assert(sizeof(*MOSI_Reg) == 0x2C, "MOSI Registers not aligned");

    #ifndef UNIT_TEST
        enable_peripheral(config.spi);
    #endif // UNIT_TEST

    set_mode(config.sck_pin, SCK_Reg);
    set_mode(config.miso_pin, MISO_Reg);
    set_mode(config.mosi_pin, MOSI_Reg);
    set_function(config.sck_pin, SCK_Reg);
    set_function(config.miso_pin, MISO_Reg);
    set_function(config.mosi_pin, MOSI_Reg);

    set_device(config.device);
    set_mode(config.mode);
    set_baud(config.rate);
    set_data_length(config.length);
    set_frame_format(config.format);
    set_chip_select(Chip_Select::SOFTWARE);    
}

//
// Destructor
//

SPI::Driver::~Driver()
{
    #ifndef UNIT_TEST
        disable_peripheral(); 
    #endif // UNIT_TEST
}

//
// Interface
//

uint8_t SPI::Driver::transaction(uint8_t byte)
{
    uint8_t data;
  
    if (!(SPI_Reg->CR1 & SPE))
    {
        enable_spi();
    }

    while (!(SPI_Reg->SR & TXE)) {};  // datasheet recommends not using 
                                            //  the BSY flag for full duplex
    SPI_Reg->DR = byte;  

    while (!(SPI_Reg->SR & RXNE)) {};

    data = static_cast<uint8_t>(SPI_Reg->DR);

    return data;
}

#ifndef UNIT_TEST
    //
    // Enable and disable peripheral clocks
    // 

    void SPI::Driver::enable_peripheral(const Instance& spi)
    {
        // enable spi
        if (spi == Instance::SPI_1)
        {
            RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
        }
        else if (spi == Instance::SPI_2)
        {
            RCC->APB1ENR1 |= RCC_APB1ENR1_SPI2EN;      
        }
        else if (spi == Instance::SUBGHZ_SPI)
        {
            RCC->APB3ENR |= RCC_APB3ENR_SUBGHZSPIEN;
        }

        // enable ports
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;  
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;  
    }

    void SPI::Driver::disable_peripheral()
    {
        // disable spi
        RCC->APB2RSTR  |= RCC_APB2RSTR_SPI1RST;
        RCC->APB1RSTR1 |= RCC_APB1RSTR1_SPI2RST;      
        RCC->APB3RSTR  |= RCC_APB3RSTR_SUBGHZSPIRST;

        // disable ports
        RCC->AHB2RSTR |= (RCC_AHB2RSTR_GPIOARST | 
                          RCC_AHB2RSTR_GPIOBRST);  
    }
#endif // UNIT_TEST

//
// SPI configuration
//

void SPI::Driver::enable_spi()
{
    SPI_Reg->CR1 |= SPE; 
}

void SPI::Driver::disable_spi()
{
    while (SPI_Reg->SR & BSY) {}; 

    // only has effect is using Software Slave Management (SSM)
    if (SPI_Reg->CR1 & SSI)
    {
        SPI_Reg->CR1 &= ~SSI;
    }

    SPI_Reg->CR1 &= ~SPE; 
}

void SPI::Driver::set_device(const Device& device)
{
    uint32_t mask = static_cast<uint32_t>(device) << MSTR;

    SPI_Reg->CR1 |= mask;
}

void SPI::Driver::set_mode(const Mode& mode)
{
    if (mode == Mode::FULL_DUPLEX)
    {
        SPI_Reg->CR1 &= ~BIDIMODE; 
        SPI_Reg->CR1 &= ~RXONLY; 
    }
    else if (mode == Mode::HALF_DUPLEX_TX)
    {
        SPI_Reg->CR1 &= ~BIDIMODE; 
        SPI_Reg->CR1 |= BIDIOE;
    }
    else if (mode == Mode::HALF_DUPLEX_RX) 
    {
        SPI_Reg->CR1 &= ~BIDIMODE; 
        SPI_Reg->CR1 &= ~BIDIOE;
    }
    else if (mode == Mode::SIMPLEX_RX)
    {
        SPI_Reg->CR1 &= ~BIDIMODE; 
        SPI_Reg->CR1 |= RXONLY;
    }
}

void SPI::Driver::set_baud(const Baud& baud)
{
    SPI_Reg->CR1 &= ~(0x7 << BR);       // set to known state

    uint32_t mask = static_cast<uint32_t>(baud) << BR;
    
    SPI_Reg->CR1 |= mask;
}

void SPI::Driver::set_data_length(const Data& length)
{
    SPI_Reg->CR2 &= ~(0xF << DS);       // set to known state;

    uint32_t mask = static_cast<uint32_t>(length) << DS;
    
    SPI_Reg->CR2 |= mask;

    // Adjust data register read to reflect the data size 
    if (static_cast<uint8_t>(length) <= 8)
    {
        SPI_Reg->CR2 |= FRXTH; 
    }
    else
    {
        SPI_Reg->CR2 &= ~FRXTH;
    }
}

void SPI::Driver::set_frame_format(const Frame& format)
{
    uint32_t mask = static_cast<uint32_t>(format) << LSBFIRST;

    SPI_Reg->CR1 |= mask;
}

void SPI::Driver::set_chip_select(const Chip_Select& cs)
{
    uint32_t mask = static_cast<uint32_t>(cs) << SSM;

    SPI_Reg->CR1 |= mask; 

    if (cs == Chip_Select::SOFTWARE)
    { 
        SPI_Reg->CR1 |= SSI;
    }
}

//
// I/O pin configuration
//

void SPI::Driver::set_mode(const GPIO::Pin& p, volatile GPIO_TypeDef* Reg)
{
    uint8_t pin = static_cast<uint8_t>(p) * 2;
    
	Reg->MODER &= ~(0x3 << pin);		// set bits to known state

    uint32_t mask = static_cast<uint32_t>(GPIO::Mode::ALT) << pin;

    Reg->MODER |= mask;
}

void SPI::Driver::set_function(const GPIO::Pin& p, volatile GPIO_TypeDef* Reg)
{
    uint32_t mask;

    uint8_t pin = static_cast<uint8_t>(p) * 2;

	// Each AFR register handles 7 or 8 pins, so the pins need to be scaled 
    // accordingly 
    if (pin < 15)
    {
    	Reg->AFR[0] &= ~(0xF << (2 * pin));	// set bits to known state

        mask = static_cast<uint32_t>(GPIO::Function::AF5) << (2 * pin);

        Reg->AFR[0] |= mask;
    }
    else if (pin > 15 && pin < 31)
    {
    	Reg->AFR[1] &= ~(0xF << (2 * pin));// set bits to known state

        mask = static_cast<uint32_t>(GPIO::Function::AF5) << pin;

        Reg->AFR[1] |= mask;
    }
}
