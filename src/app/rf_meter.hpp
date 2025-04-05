//
// @date    3/29/2025
// @author  Calum Thornhill
// @brief   Read RF sensor data and transfer it to an external buffer. 
//
// @details The sensor used is the Mikroe RF-Meter. It contains a logarmithmic
//          amplifier and an internal 12-bit ADC that outputs raw data. The data 
//          is constantly transmitted via SPI an interface. 
//
//          The SPI driver uses polling for this application since the data is
//          very simple (max size is a 12-bit number) and no there are no serious
//          timing constraints. The public interface of this class reads the data
//          and provides a method to transfer it to an external buffer.
//

#ifndef APP_RF_METER_HPP
#define APP_RF_METER_HPP

#include <array>
#include <cstdio>

#include "../drivers/gpio.hpp"
#include "../drivers/spi.hpp"
#include "../drivers/uart.hpp"
#include "../common/ring_buffer.h"

namespace Sensor_Package
{
    using namespace STM32WL55JC1;

    class RF_Meter 
    {
    public:
        RF_Meter(SPI::Driver& rx, GPIO::Output& cs);
        
        RF_Meter()                               = delete;
        RF_Meter(const RF_Meter& rhs)            = delete;
        RF_Meter(RF_Meter&& rhs)                 = delete;
        RF_Meter& operator=(const RF_Meter& rhs) = delete;
        RF_Meter& operator=(RF_Meter&& rhs)      = delete;

        ~RF_Meter();

        void read();

        void transfer_data(Ring_Buffer<uint8_t, 100>& buffer);

        Ring_Buffer<uint8_t, 25>& get_formatted_data();  // array of chars
        uint16_t get_raw_data() const;                  // integer value

    private:

        SPI::Driver& spi;
        GPIO::Output& chip_select; 

        Ring_Buffer<uint8_t, 25> data_buffer;

        uint16_t raw_data;

    }; // class RF_Meter

} // namespace Sensor_Package

#endif // APP_RF_METER_HPP
