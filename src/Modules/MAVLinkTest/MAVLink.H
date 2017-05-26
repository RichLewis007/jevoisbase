/*
MAVLink Integration on Jevois Module

Implementation adapted from PX4Flow MAVLink implementation https://github.com/PX4/Flow

This is a Jevois Component
//TODO: Move as a separate component once I figure out away
to pass settings to compnent without including header
@author Ali AlSaibie
@email ali@alsaibie.com

*/
#pragma once

#include <jevois/Component/Component.H>
#include <jevois/Types/Enum.H>
#include <mavlink/common/mavlink.h>
#include "MAVLinkTest_settings.h"

//#ifdef JEVOIS_PLATFORM
//    //! On platform hardware, device for the 4-pin hardware serial port
#define JEVOIS_SERIAL_DEFAULT "/dev/ttyS0"
//#else
//    //! On generic computer hardware, device for serial port should always be stdio to use an StdioInterface
//    #define JEVOIS_SERIAL_DEFAULT "stdio" //Change to another serial device or perhaps implement UDP
//#endif

namespace mavlink
{
    static jevois::ParameterCategory const ParamCateg("MAVLink Options");
    //! Parameter
    JEVOIS_DECLARE_PARAMETER(serialdev, std::string, "Hardware (4-pin connector) serial device name, "
                                               "or 'stdio' to use the console, or empty for no hardware serial port",
                             JEVOIS_SERIAL_DEFAULT, ParamCateg);
}

class MAVLink : public jevois::Component
{
public:
    //! Constructor
    MAVLink(std::string const & instance, struct MAVLink_data_struct * MAVData);

    //! Destructor
    virtual ~MAVLink();

    //! System State
    void send_system_state(void);

    //! Parameters
    void send_parameters(void);

    //! Receive
    void receive(void);

    //! Handle MAVLink Messages - Defined in implementation module
    void handle_mavlink_message(mavlink_message_t* msg);

    // static to access from c function implementation
    static MAVLink      *get_instance();

    mavlink_status_t  *get_status() { return &_mavlink_status;}
    mavlink_message_t *get_buffer() { return &_mavlink_buffer;}

    mavlink_system_t mavlink_system;
    static uint32_t m_parameter_i;

private:
    int numbytes = 0;
    uint8_t MAVLinkReceiveBuf[64];
    mavlink_message_t _mavlink_buffer;
    mavlink_status_t  _mavlink_status;
   //! Methods to get internal status of MAVLink
protected:
    // itsSerials are instaces of UserInterface which can take HARD serial, USB
    std::shared_ptr<jevois::UserInterface> itsSerials; //TODO:
    struct MAVLink_data_struct *itsMAVLinkData;

}; // class MAVLink
