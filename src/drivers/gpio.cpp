#include "gpio.hpp"

using namespace STM32WL55JC1;

//
// Constructor
//

GPIO::Output::Output(const Config& config)
{
    GPIO = reinterpret_cast<volatile GPIO_TypeDef*>(config.port);

    static_assert(sizeof(*GPIO) == 0x2C, "Registers are not aligned in memory");

    // One 32-bit register covers all pins, and each section of the register is
    // 2 bits wide, so the "pin value" needs to be modified accordingly
    pin = static_cast<uint8_t>(config.pin) * 2;

    #ifndef UNIT_TEST
        enable_peripheral(config.port);
    #endif // UNIT_TEST

    set_mode(config.mode);
    set_output_type(config.type);
    set_speed(config.speed);
    set_resistor(config.resistor);
    set_function(config.function);
}

//
// Destructor
//

GPIO::Output::~Output()
{
    #ifndef UNIT_TEST
        disable_peripheral();
    #endif // UNIT_TEST
}

//
// Port State Functions
//

void GPIO::Output::reset()
{
    GPIO->ODR &= ~(1 << (pin / 2));
}

void GPIO::Output::set()
{
    GPIO->ODR |= (1 << (pin / 2));
}

void GPIO::Output::toggle()
{
    GPIO->ODR ^= (1 << (pin / 2));
}

#ifndef UNIT_TEST
    //
    // Enable and disable peripherals via RCC clock
    //

    void GPIO::Output::enable_peripheral(const Port& port)
    {
        // Enable all GPIO peripheral clocks
        if (port == GPIO::Port::A)
        {
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;  
        }
        else if (port == GPIO::Port::B)
        {
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;  
        }
        else if (port == GPIO::Port::C)
        {
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;  
        }
        else if (port == GPIO::Port::H)
        {
            RCC->AHB2ENR |= RCC_AHB2ENR_GPIOHEN;  
        }
    }

    void GPIO::Output::disable_peripheral()
    {
        // Disable all GPIO peripheral clocks
        RCC->AHB2RSTR |= (RCC_AHB2RSTR_GPIOARST | 
                          RCC_AHB2RSTR_GPIOBRST | 
                          RCC_AHB2RSTR_GPIOCRST | 
                          RCC_AHB2RSTR_GPIOHRST);
    }
#endif // UNIT_TEST

//
// Port Configuration Functions
//

void GPIO::Output::set_mode(const Mode& mode)
{
	GPIO->MODER &= ~(0x3 << pin);		// set bits to known state

    uint32_t mask = static_cast<uint32_t>(mode) << pin;

    GPIO->MODER |= mask;
}

void GPIO::Output::set_output_type(const Type& type)
{
    // Each pin corresponds to a bit in this register, so no reset is needed

    uint32_t mask = static_cast<uint32_t>(type) << (pin / 2);

    GPIO->OTYPER |= mask;
}

void GPIO::Output::set_speed(const Speed& speed)
{
	GPIO->OSPEEDR &= ~(0x3 << pin);		// set bits to known state

    uint32_t mask = static_cast<uint32_t>(speed) << pin;

    GPIO->OSPEEDR |= mask;
}

void GPIO::Output::set_resistor(const Resistor& resistor)
{
	GPIO->PUPDR &= ~(0x3 << pin);	// set bits to known state

    uint32_t mask = static_cast<uint32_t>(resistor) << pin;

    GPIO->PUPDR |= mask;
}

void GPIO::Output::set_function(const Function& function)
{
	uint32_t mask;

	// Each AFR register handles 7 or 8 pins, so the pins need to be scaled 
    // accordingly 
    if (pin < 15)
    {
    	GPIO->AFR[0] &= ~(0xF << (2 * pin));	// set bits to known state

        mask = static_cast<uint32_t>(function) << (2 * pin);

        GPIO->AFR[0] |= mask;
    }
    else if (pin > 15 && pin < 31)
    {
    	GPIO->AFR[1] &= ~(0xF << (2 * pin));// set bits to known state

        mask = static_cast<uint32_t>(function) << pin;

        GPIO->AFR[1] |= mask;
    }
}
