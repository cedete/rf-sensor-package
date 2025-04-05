#include "rf_meter.hpp"

using namespace Sensor_Package;

RF_Meter::RF_Meter(SPI::Driver& rx, GPIO::Output& cs) : spi{rx}, chip_select{cs}
{}

RF_Meter::~RF_Meter()
{}

void RF_Meter::read()
{
    uint16_t data = 2500; 

//    chip_select.reset(); 

//    data = spi.transaction(0x00);

//    chip_select.set(); 

    raw_data = data;

    char buffer[12];

    snprintf(buffer, sizeof(buffer), ",%u", data);
 
    for (std::size_t i = 0; i < sizeof(buffer); ++i)
    {
        data_buffer.put(buffer[i]);         
    }
}

void RF_Meter::transfer_data(Ring_Buffer<uint8_t, 100>& buffer)
{
    while (!data_buffer.empty())  
    {
        buffer.put(data_buffer.get()); 
    }
}

Ring_Buffer<uint8_t, 25>& RF_Meter::get_formatted_data() 
{
    return data_buffer; 
}

uint16_t RF_Meter::get_raw_data() const
{
    return raw_data;
}
