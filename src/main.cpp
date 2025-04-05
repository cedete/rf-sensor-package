#include "../test/test_hardware.hpp"
#include "app/support.hpp"
#include "app/gps.hpp"
#include "app/rf_meter.hpp"

using namespace Sensor_Package;

int main()
{
    Ring_Buffer<uint8_t, 100> data_buffer;  // store all data

    RF_Meter rf_meter(spi1(), pb12());              
    GPS gps(usart1());

    Test::Terminal terminal(usart2());      // view via serial output for debugging

    while(true)
    {
        gps.transfer_data(data_buffer);     // gps reads data via interrupt and then tranfers buffer

        if (gps.status() == GPS::DONE)
        {
//            rf_meter.read();
//            rf_meter.transfer_data(data_buffer); 

            terminal.print(data_buffer);    // view GPS data for debugging

            //Potential problem: may not have enough space, might have to print after
        }
    }
}
