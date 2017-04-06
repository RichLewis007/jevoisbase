/*
MAVLink Integration on Jevois Module

Implementation adapted from PX4Flow MAVLink implementation https://github.com/PX4/Flow

This is a Jevois Component
//TODO: Move as a separate component once I figure out away 
to pass settings to compnent without including header
@author Ali AlSaibie
@email ali@alsaibie.com

*/

#include <jevois/Component/Component.H>
#include <jevois/Core/Serial.H>
//#include "mavlink_bridge_header.h"
#include <mavlink.h>

#include "MAVLinkTest_settings.h"

#ifdef JEVOIS_PLATFORM
    //! On platform hardware, device for the 4-pin hardware serial port
    #define JEVOIS_SERIAL_DEFAULT "/dev/ttyS0"
#else
    //! On generic computer hardware, device for serial port should always be stdio to use an StdioInterface
    #define JEVOIS_SERIAL_DEFAULT "stdio" //Change to another serial device or perhaps implement UDP
#endif

namespace MAVLinkCommunication
{
    class UserInterface;
    //! Parameter 
    // JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(serialdev, std::string, "Hardware (4-pin connector) serial device name, "
    //                                            "or 'stdio' to use the console, or empty for no hardware serial port",
    //                                            JEVOIS_SERIAL_DEFAULT, ParamCateg);
    //! Parameter 
    JEVOIS_DECLARE_PARAMETER(serialdev, std::string, "Hardware (4-pin connector) serial device name, "
                                               "or 'stdio' to use the console, or empty for no hardware serial port",
                                               JEVOIS_SERIAL_DEFAULT, ParamCateg);
}

class MAVLinkCommunication : public jevois::Component            
{
public:
    //! Constructor
    MAVLinkCommunication(std::string const & instance, struct MAVLink_data_struct * MAVData);

    //! Destructor
    virtual ~MAVLinkCommunication();
    //! System State
    void sendSystemState(void);

    //! Parameters
    void sendParameters(void);

    //! Receive 
    void receive(void);

    //! Handle MAVLink Messages - Defined in implementation module
    void handle_mavlink_message(mavlink_message_t* msg);

    // static to access from c function implementation
    static MAVLinkCommunication      *get_instance(unsigned instance = 0);

    mavlink_system_t mavlink_system;
    static uint32_t m_parameter_i;

    // Assume we need one instance now, expand to vector (See PX4 MAVLink module implementation
    // and the use of hash list header utlist.h)
    // Does it need to be static?
    static MAVLinkCommunication    *itsInstance;

private:
    int numbytes = 0;
    uint8_t MAVLinkReceiveBuf[64];

   //! Methods to get internal status of MAVLink
   // mavlink_status_t* mavlink_get_channel_status(uint8_t channel);
   // mavlink_message_t* mavlink_get_channel_buffer(uint8_t channel);
protected:
    std::shared_ptr<Serial> itsSerial;
    struct MAVLink_data_struct *itsMAVLinkData;
}; // class MAVLinkCommunication

