#include "../test/test_hardware.hpp"
#include "app/support.hpp"
#include "app/gps.hpp"
#include "app/rf_meter.hpp"
#include "drivers/subghz.hpp"

using namespace Sensor_Package;

int main()
{
    Ring_Buffer<uint8_t, 255> data_buffer;  // store all data

    RF_Meter rf_meter(spi1(), pb12());              
    GPS gps(usart1());
    SUBGHZ::Driver radio(radio_config);
    
    GPIO::Output led1(led1_config);

//    Test::Terminal terminal(usart2());      // view via serial output for debugging

    while(true)
    {
        // debug
        /* led1.reset(); */

        gps.transfer_data(data_buffer);     // gps reads data via interrupt and then tranfers buffer

        if (gps.status() == GPS::DONE)
        {
//            rf_meter.read();
//            rf_meter.transfer_data(data_buffer); 

//            terminal.print(data_buffer);    // view GPS data for debugging

            radio.transmit(data_buffer);    

            // debug
            led1.set();
        }
    }
}
