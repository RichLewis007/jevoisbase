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

#define TRUE 1
#define FALSE 0

#include <jevois/Component/Component.H>
#include <jevois/Core/Serial.H>
#include <jevois/Types/Enum.H>
#include "mavlink_bridge_header.h"
#include <jevoismavlink/mavlink.h>
#include "MAVLink_settings.h"
#include <memory>

// TODO: uncomment when compiling
#ifdef JEVOIS_PLATFORM
// ! On platform hardware, device for the 4-pin hardware serial port
#define JEVOIS_SERIAL_DEFAULT "/dev/ttyS0"
#define JEVOIS_USBSERIAL_DEFAULT "/dev/ttyGS0"
#else
// ! On generic computer hardware, device for serial port should be the MAVLink USB Device
#define JEVOIS_SERIAL_DEFAULT "/dev/ttyACM0"
#define JEVOIS_USBSERIAL_DEFAULT "/dev/ttyACM0" //TODO: Change to another serial device or implement UDP
#endif

namespace mavlink
{
    static jevois::ParameterCategory const ParamCateg("MAVLink Options");
    //! Parameter \relates mavlink
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(serialdev, std::string, "Hardware (4-pin connector) serial device name, "
                                               "or 'stdio' to use the console, or empty for no hardware serial port",
                             JEVOIS_SERIAL_DEFAULT, ParamCateg);
    //! Parameter \relates mavlink
    JEVOIS_DECLARE_PARAMETER_WITH_CALLBACK(usbserialdev, std::string, "Over-the-USB serial device name, or empty",
                                           JEVOIS_USBSERIAL_DEFAULT, ParamCateg);
}

class MAVLink : public jevois::Component
{
public:
    //! Enum for the interface type
    typedef enum class Type_t { Hard, USB };
    //! Constructor
    MAVLink(std::string const & instance, struct MAVLink_data_struct * MAVData, Type_t type);

    //! Destructor
    virtual ~MAVLink();

    //! System State
    void send_system_state(void);

    //! Parameters
    void send_parameters(bool Force = FALSE);

    //! Receive
    void receive(void);

    //! Handle MAVLink Messages - Defined in application module
    void handle_mavlink_message(mavlink_message_t* msg);

    // static to access from c function implementation
    static std::shared_ptr<MAVLink> get_instance(Type_t type);
    void set_instance(std::shared_ptr<MAVLink> & inst);
    mavlink_status_t  *get_status() { return &_mavlink_status;}
    mavlink_message_t *get_buffer() { return &_mavlink_buffer;}

    mavlink_system_t mavlink_system;
    uint32_t m_parameter_i;

    // itsSerial is a Serial instance, can be either USB or Hard Serial. Instead of having
    // multiple itsSerials, we can have multiple MAVLinks and each with its single itsSerial
    std::shared_ptr<jevois::Serial> itsSerial;

private:
    int numbytes = 0;
    uint8_t MAVLinkReceiveBuf[64];
    mavlink_message_t _mavlink_buffer;
    mavlink_status_t  _mavlink_status;
   //! Methods to get internal status of MAVLink
protected:
    struct MAVLink_data_struct *itsMAVLinkData; // TODO: change to smart ptr
    mavlink_channel_t mavlink_channel;
    Type_t itsType;
}; // class MAVLink

namespace  mavlink {
    static  std::map<MAVLink::Type_t, std::shared_ptr<MAVLink> > gMAVLink_instances;
}