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
#include <jevois/Core/Serial.H>
#include <jevois/Types/Enum.H>
#include "mavlink_bridge_header.h"
#include <mavlink/jevoismavlink/mavlink.h>
#include <memory>

// TODO: uncomment when compiling
#ifdef JEVOIS_PLATFORM
// ! On platform hardware, device for the 4-pin hardware serial port
#define JEVOIS_SERIAL_DEFAULT "/dev/ttyS0"
#define JEVOIS_USBSERIAL_DEFAULT "/dev/ttyGS0"
#else
// ! On generic computer hardware, device for serial port should be the MAVLink USB Device
#define JEVOIS_SERIAL_DEFAULT "/dev/ttyACM0"
#define JEVOIS_USBSERIAL_DEFAULT "/dev/ttyUSB0"
//TODO: Add UDP?
#endif

#define ONBOARD_PARAM_NAME_LENGTH 	15

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
    typedef enum
    {
        READ_ONLY   = 0,
        READ_WRITE  = 1,
    } ParameterAccess_TypeDef;

    typedef struct
    {
        /* nothing here until now */

    } SysState_TypeDef;


    enum MAVLink_param_id_t
    {
        PARAM_SYSTEM_ID = 0,
        PARAM_COMPONENT_ID,
        PARAM_SENSOR_ID,
        PARAM_SYSTEM_TYPE,
        PARAM_AUTOPILOT_TYPE,
        PARAM_SYSTEM_STATUS,
        PARAM_SW_VERSION,
        PARAM_SYSTEM_SEND_STATE,

        /* Keep this last */
        ONBOARD_PARAM_COUNT

    };

    struct MAVLink_data_struct
    {
        SysState_TypeDef system_state;
        float param[ONBOARD_PARAM_COUNT];
        char param_name[ONBOARD_PARAM_COUNT][ONBOARD_PARAM_NAME_LENGTH];
        ParameterAccess_TypeDef param_access[ONBOARD_PARAM_COUNT];
    };
}

using namespace mavlink;

class MAVLink : public jevois::Component
{
public:
    //! Enum for the interface type
    typedef enum class Type_t { Hard, USB };
    //! Constructor
    MAVLink(std::string const & instance, struct MAVLink_data_struct * MAVData, Type_t type);

    //! Destructor
    virtual ~MAVLink();

    //! Initialize, create the detector and read the config files
    void postInit() override;

    //! Un-initialize, nuke allocated resources
    void postUninit() override;


    //! System State
    void send_system_state(void);

    //! Parameters
    void send_parameters(bool Force = false);

    //! Receive
    void receive(void);

    //! Handle MAVLink Messages - Defined in application module
    void handle_mavlink_message(mavlink_message_t* msg);

    //! reset parameters - Defined in application module
    void reset_param_defaults(void);

    // static to access from c function implementation
    static std::shared_ptr<MAVLink> get_instance(Type_t type);
    void set_instance(std::shared_ptr<MAVLink> & inst);
    mavlink_status_t  *get_status();
    mavlink_message_t *get_buffer();

//    mavlink_system_t mavlink_system;
    uint32_t m_parameter_i;

    // itsSerial is a Serial instance, can be either USB or Hard Serial. Instead of having
    // multiple itsSerials, we can have multiple MAVLinks and each with its single itsSerial
    std::shared_ptr<jevois::Serial> itsSerial;
    void printsomething(void);
private:
    int numbytes = 0;
    uint8_t MAVLinkReceiveBuf[64];
   //! Methods to get internal status of MAVLink

protected:
    struct MAVLink_data_struct *itsMAVLinkData; // TODO: change to smart ptr
    mavlink_channel_t mavlink_channel;
    Type_t itsType;
}; // class MAVLink
