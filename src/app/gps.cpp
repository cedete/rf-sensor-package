#include "gps.hpp"

using namespace Sensor_Package;

GPS::GPS(UART::Driver& rx) : usart{rx}, gps_status{NOT_DONE}
{
    set_pmtk(NMEA_OUTPUT_RMCONLY);
    set_pmtk(NMEA_UPDATE_200_MILLIHZ);
}

GPS::~GPS()
{
    // peripheral constructors take care of resetting peripheral states
}

void GPS::transfer_data(Ring_Buffer<uint8_t, 100>& buffer)
{
    gps_status = NOT_DONE;

    uint8_t data;

    if (usart.rx_done('\n'))
    {
        while(!usart.buffer_empty())
        {
            usart.pop(data);    // pop from internal buffer
            buffer.put(data);   // put data into external buffer;
        }

        gps_status = DONE;
    }
}

GPS::Status GPS::status()
{
    return gps_status;
}

void GPS::set_pmtk(const char* command)
{
    for (auto& idx : std::string_view(command))
    {
        usart.write(idx);
    }
}
