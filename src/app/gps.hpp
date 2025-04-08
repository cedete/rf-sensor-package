//
// @date    29 March 2025
// @author  Calum Thornhill
// @brief   Read GPS data and transfers it to an external buffer.
//
// @details The GPS used is the Adafruit Ultimate. See www.adafruit.com.
//          It is constantly transmitting data, and it uses UART to communicate
//          with the connected microcontroller. The baud rate, update rate, and 
//          other settings can be configured by via commands sent to the GPS in 
//          the PMTK format. 
//
//          The GPS outputs NMEA-formatted sentences that are around 50 bytes 
//          long. Because of how fast these the data within these sentences is
//          transmitted, interrupts are required. Because of this, the data read
//          implementation is hidden within the UART interrupt handler, and the
//          public interface of this class is simply transferring the received
//          data to another location.
//
//          GPS:         Adafruit Ultimate
//          Baud Rate:   9600
//          Update Rate: 200 mHz (once every 5 seconds)
//

#ifndef APP_GPS_HPP
#define APP_GPS_HPP

#include <string_view>              // used for iterating through PMTK C strings

#include "../drivers/uart.hpp"
#include "../common/ring_buffer.h"

namespace Sensor_Package
{
    using namespace STM32WL55JC1;

    class GPS
    {
    public:

        enum Status
        {
            DONE,
            NOT_DONE    
        };

        GPS(UART::Driver& rx);
        
        GPS()                          = delete;
        GPS(const GPS& rhs)            = delete;    
        GPS(GPS&& rhs)                 = delete;
        GPS& operator=(const GPS& rhs) = delete;
        GPS& operator=(GPS&& rhs)      = delete;

        ~GPS();

        void transfer_data(Ring_Buffer<uint8_t, 255>& buffer);

        Status status();

    private:

        UART::Driver& usart;        // communication with GPS

        Status gps_status;

        //
        //  PMTK commands to configure GPS
        //

        // Update rate for NMEA sentences
        static constexpr const char* NMEA_UPDATE_100_MILLIHZ = "$PMTK220,10000*2F\r\n"; // untested
        static constexpr const char* NMEA_UPDATE_200_MILLIHZ = "$PMTK220,5000*1B\r\n";
        static constexpr const char* NMEA_UPDATE_1_HZ        = "$PMTK220,1000*1F\r\n";
        static constexpr const char* NMEA_UDPATE_2_HZ        = "$PMTK220,500*2B\r\n";   // untested
        static constexpr const char* NMEA_UPDATE_5_HZ        = "$PMTK220,200*2C\r\n";   // untested
        static constexpr const char* NMEA_UPDATE_10_HZ       = "$PMTK220,100*2F\r\n";   // untested

        // Specify which sentences to output
        static constexpr const char* NMEA_OUTPUT_RMCONLY = "$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n";

        // Query device release and version
        static constexpr const char* Q_RELEASE = "$PMTK605*31\r\n";         

        //
        // Helper to send GPS commands
        //

        void set_pmtk(const char* command);

    };  // class GPS

} // namespace Sensor_Package

#endif // APP_GPS_HPP
