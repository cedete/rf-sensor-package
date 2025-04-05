#include "uart.hpp"

using namespace STM32WL55JC1;

//
// Register bit definitions
//

namespace
{
    // UART control_1 register
    static constexpr uint32_t M1     = (1 << 28);    // Word Length (Bit 1) 
    static constexpr uint32_t M0     = (1 << 12);    // Word Length (Bit 0) 
    static constexpr uint8_t  OVER8  =       15;     // Oversampling
    static constexpr uint32_t RXNEIE = (1 << 5);     // RX Interrupt Enable
    static constexpr uint8_t  TE     = (1 << 3);     // Transmitter Enable
    static constexpr uint8_t  RE     = (1 << 2);     // Receiver Enable
    static constexpr uint8_t  UE     = (1 << 0);     // UART Enable

    // UART control_2 register
    static constexpr uint8_t  STOP   =       12;     // Stop[0] bit

    // UART status register
    static constexpr uint32_t RXNE   = (1 << 5);     // RXFIFO Not Empty
    static constexpr uint32_t TXE    = (1 << 7);     // TXFIFO Not Empty
}

//
// Constructor
//

UART::Driver::Driver(const UART::Config& config)
{
    UART_Reg = reinterpret_cast<volatile USART_TypeDef*>(config.uart);
    TX_Pin = reinterpret_cast<volatile GPIO_TypeDef*>(config.tx_port);
    RX_Pin = reinterpret_cast<volatile GPIO_TypeDef*>(config.rx_port);

    static_assert(sizeof(*UART_Reg) == 0x30, "UART registers not aligned");
    static_assert(sizeof(*TX_Pin) == 0x2C, "TX registers are not aligned");
    static_assert(sizeof(*RX_Pin) == 0x2C, "RX registers are not aligned");

    #ifndef UNIT_TEST
        enable_peripheral(config.uart);
        enable_interrupts(config.uart);
    #endif // UNIT_TEST

    // connect isr function to vendor ISR function found in startup file 
    IRQ_Handlers::irq_handler(config.uart).connect<&UART::Driver::isr_read>(this);    

    set_mode(config.tx_pin, TX_Pin);
    set_mode(config.rx_pin, RX_Pin);
    set_function(config.tx_pin, TX_Pin);
    set_function(config.rx_pin, RX_Pin);

    set_baud(config.baud_rate);
    set_data_length(Data::LENGTH_8);
    set_stop_bits(Stop::BITS_1);
    set_oversampling(Oversampling::BY_16);

    enable_uart();
    set_mode(config.mode);
}

//
// Destructor
// 

UART::Driver::~Driver()
{
    #ifndef UNIT_TEST
        disable_peripheral();
    #endif // UNIT_TEST
}

// 
// Public interface
//

void UART::Driver::isr_read()
{
    if (UART_Reg->ISR & RXNE)
    {
        buffer.put(UART_Reg->RDR); 
    }
}

uint8_t UART::Driver::read()
{
    uint8_t data;

    while (!(UART_Reg->ISR & RXNE)) {};
    data = UART_Reg->RDR;

    return data;
}

void UART::Driver::write(uint8_t byte)
{
    while (!(UART_Reg->ISR & TXE)) {};
    UART_Reg->TDR = byte;
}

bool UART::Driver::buffer_empty()
{
    return buffer.empty();
}

bool UART::Driver::buffer_full()
{
    return buffer.full();
}

void UART::Driver::push(uint8_t data)
{
    buffer.put(data);
}

void UART::Driver::pop(uint8_t& data)
{
    data = buffer.get();
}

bool UART::Driver::rx_done(uint8_t delimiter)
{
    if (buffer[buffer.get_unread() - 1] == delimiter)
    {
        return true;     
    }

    return false;
}

#ifndef UNIT_TEST
    //
    // Enable/disable peripheral
    //

    void UART::Driver::enable_peripheral(const Instance& uart)
    {
        if (uart == Instance::USART_1)
        {
            RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 
        }
        else if (uart == Instance::USART_2)
        {
            RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN; 
        }

        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;  
        RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;  
    }

    void UART::Driver::disable_peripheral()
    {
        RCC->APB2RSTR |= RCC_APB2RSTR_USART1RST; 
        RCC->APB1RSTR1 |= RCC_APB1RSTR1_USART2RST; 

        RCC->AHB2RSTR |= (RCC_AHB2RSTR_GPIOARST | 
                          RCC_AHB2RSTR_GPIOBRST);  
    }

    void UART::Driver::enable_interrupts(const Instance& uart)
    {
        // enable NVIC 
        NVIC_SetPriorityGrouping(0);    

        // set priority level
        uint32_t encoding = NVIC_EncodePriority(0,1,0);

        if (uart == Instance::USART_1)
        {
            NVIC_SetPriority(USART1_IRQn, encoding);
            NVIC_EnableIRQ(USART1_IRQn);
        }
        else if (uart == Instance::USART_2)
        {
            NVIC_SetPriority(USART2_IRQn, encoding);
            NVIC_EnableIRQ(USART2_IRQn);
        }

        // enable in peripheral register
        UART_Reg->CR1 |= RXNEIE;       
    }
#endif // UNIT_TEST

//
// UART configuration
//

void UART::Driver::enable_uart()
{
	UART_Reg->CR1 |= UE;   
}

void UART::Driver::set_mode(const Mode& mode)
{
	if (mode == UART::Mode::TX)
	{
		UART_Reg->CR1 |= TE; 
	}
	else if (mode == UART::Mode::RX)
	{
		UART_Reg->CR1 |= RE; 
	}
	else if (mode == UART::Mode::TX_RX)
	{
		UART_Reg->CR1 |= (TE | RE); 
	}
}


void UART::Driver::set_baud(const uint32_t& rate)
{
	uint32_t usartdiv = system_clock_freq / rate;

	UART_Reg->BRR = usartdiv;
}

void UART::Driver::set_data_length(const Data& length)
{
	switch(length)
    {
        case UART::Data::LENGTH_8:
            UART_Reg->CR1 &= ~(M1 | M0);
            break;
        case UART::Data::LENGTH_9:
            UART_Reg->CR1 &= ~M1;
            UART_Reg->CR1 |= M0;
            break;
        case UART::Data::LENGTH_7:
            UART_Reg->CR1 |= M1;
            UART_Reg->CR1 &= ~M0;
            break;
        default:
            UART_Reg->CR1 &= ~(M1 | M0);
            break;
	}
}

void UART::Driver::set_stop_bits(const Stop& bits)
{
	UART_Reg->CR2 &= ~(0x3 << STOP);	// set bits to known state

	uint32_t mask = static_cast<uint32_t>(bits) << STOP;

	UART_Reg->CR2 |= mask;
}

void UART::Driver::set_oversampling(const Oversampling& oversampling)
{
	uint32_t mask = static_cast<uint32_t>(oversampling) << OVER8;	

	UART_Reg->CR1 |= mask;
}

//
// I/O pin configuration 
//

void UART::Driver::set_mode(const GPIO::Pin& p, volatile GPIO_TypeDef* Reg)
{
    uint8_t pin = static_cast<uint8_t>(p) * 2;
    
	Reg->MODER &= ~(0x3 << pin);		// set bits to known state

    uint32_t mask = static_cast<uint32_t>(GPIO::Mode::ALT) << pin;

    Reg->MODER |= mask;
}

void UART::Driver::set_function(const GPIO::Pin& p, volatile GPIO_TypeDef* Reg)
{
    uint32_t mask;

    uint8_t pin = static_cast<uint8_t>(p) * 2;

	// Each AFR register handles 7 or 8 pins, so the pins need to be scaled 
    // accordingly 
    if (pin < 15)
    {
    	Reg->AFR[0] &= ~(0xF << (2 * pin));	// set bits to known state

        mask = static_cast<uint32_t>(GPIO::Function::AF7) << (2 * pin);

        Reg->AFR[0] |= mask;
    }
    else if (pin > 15 && pin < 31)
    {
    	Reg->AFR[1] &= ~(0xF << (2 * pin));// set bits to known state

        mask = static_cast<uint32_t>(GPIO::Function::AF7) << pin;

        Reg->AFR[1] |= mask;
    }
}
