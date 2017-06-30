/*
   MAVLink Integration on Jevois Module

   @author Ali AlSaibie
   @email ali@alsaibie.com

 */


#include <list>
#include <jevois/Debug/Log.H>
#include <jevois/Core/StdioInterface.H>
#include <jevoisbase/Components/MAVLink/mavlink_bridge_header.h>
#include <jevoisbase/Components/MAVLink/MAVLink.H>



namespace  mavlink {
    static  std::map<MAVLink::Type_t, std::shared_ptr<MAVLink> > gMAVLink_instances;
}
using namespace mavlink;

mavlink_system_t mavlink_system;
/*
 *
 */
MAVLink::MAVLink(std::string const &instance, struct MAVLink_data_struct *MAVData, Type_t type) :
        jevois::Component(instance), itsMAVLinkData(MAVData), itsType(type), m_parameter_i
        (ONBOARD_PARAM_COUNT)
{
    LDEBUG("MAVLink Constructor");
    // Add subcomponents here,
    // Serial is a SubComponent of MAVLink. Either HARD or USB
    if (itsType == Type_t::Hard){
        mavlink_channel = MAVLINK_COMM_0;
        itsSerial = addSubComponent<jevois::Serial>("serial", jevois::UserInterface::Type::Hard);
        std::string const newval = JEVOIS_SERIAL_DEFAULT;
        itsSerial->setParamVal("devname", newval);
        LINFO("Using [" << newval << "] hardware (4-pin connector) serial port for MAVLink");
    }
    else if (itsType == Type_t::USB){
        mavlink_channel = MAVLINK_COMM_1;
        itsSerial = addSubComponent<jevois::Serial>("usbserial", jevois::UserInterface::Type::USB);
        std::string const newval = JEVOIS_USBSERIAL_DEFAULT;
        itsSerial->setParamVal("devname", newval);
        LINFO("Using [" << newval << "] USB serial port for MAVLink");
    }
    else{
        LERROR("Could not start serial");
    }
}

void MAVLink::postInit() {

    LDEBUG("MAVLink PostInit");

    // Set devname in param.cfg so it doesn't need to be re-setup here.
    // Reset MAVLink Parameters to Default
    reset_param_defaults();

    // Set system ID
    LINFO("Setting MAVLink System ID: " <<itsMAVLinkData->param[PARAM_SYSTEM_ID] <<" and component ID "<< itsMAVLinkData->param[PARAM_COMPONENT_ID]);
    mavlink_system.sysid = itsMAVLinkData->param[PARAM_SYSTEM_ID]; // System ID, 1-255
    mavlink_system.compid = itsMAVLinkData->param[PARAM_COMPONENT_ID]; // Component/Subsystem ID, 1-255
}

void MAVLink::postUninit() {

    LDEBUG("MAVLink PostUnInit");

    mavlink::gMAVLink_instances[itsType] = std::shared_ptr<MAVLink>(); //Assign Empty of type MAVLink
    //TODO: Is this needed? also. implement iterator.
}

MAVLink::~MAVLink(){}


void MAVLink::send_system_state(void) {
    LDEBUG("Heartbeat Message System Type: "<<itsMAVLinkData->param[PARAM_SYSTEM_TYPE] <<" Autopilot Type: " << itsMAVLinkData->param[PARAM_AUTOPILOT_TYPE]);
    mavlink_msg_heartbeat_send(mavlink_channel, itsMAVLinkData->param[PARAM_SYSTEM_TYPE],
                               itsMAVLinkData->param[PARAM_AUTOPILOT_TYPE], 0, 0, 0);
}

void MAVLink::send_parameters(bool Force) {

    LDEBUG("MAVLink::send_parameters");

    if (Force) m_parameter_i = 0;

    if (m_parameter_i < ONBOARD_PARAM_COUNT) {
        mavlink_msg_param_value_send(mavlink_channel,
                                     itsMAVLinkData->param_name[m_parameter_i],
                                     itsMAVLinkData->param[m_parameter_i], MAVLINK_TYPE_FLOAT,
                                     ONBOARD_PARAM_COUNT, m_parameter_i);
        m_parameter_i++;
    }

}


void MAVLink::receive(void) {


    mavlink_message_t msg;
    mavlink_status_t status = {0};

    /* Minimum 20 bytes at a time */
    const unsigned character_count = 20;
    /* Read up to buffer size. Loop through characters read and parse. Handle message when packet is complete */
    // TODO: Only read if characters are available
    if ((numbytes = itsSerial->read2(MAVLinkReceiveBuf, sizeof(MAVLinkReceiveBuf))) >=
        (size_t) character_count) {
        //LINFO("Read Buffer, Now Parsing");
        for (size_t i = 0; i < numbytes; i++) {
            if (mavlink_parse_char(mavlink_channel, MAVLinkReceiveBuf[i], &msg, &status)) {
                /* Handle message */
                LDEBUG("New MAVLink Message Received");
                handle_mavlink_message(&msg);
            }
        }
    }
}

 std::shared_ptr<MAVLink> MAVLink::get_instance(Type_t type){

     std::shared_ptr<MAVLink> inst = mavlink::gMAVLink_instances[type];
     if (inst){
         return  inst;
     }
     else{
         LERROR("MAVLink instance does not exist or expired!");
         //throw "Instance does not exist or expired!";
         return nullptr;
     }

}

void MAVLink::set_instance(std::shared_ptr<MAVLink> &inst) {
    LDEBUG("Saving MAVLink instance into map");
    mavlink::gMAVLink_instances[itsType] = inst;
}

void MAVLink::printsomething(void){

    // LINFO("I'm Here, yes I'm here");
    itsSerial->writeString("This is a test only");
}

mavlink_status_t* MAVLink::get_status() {
    static mavlink_status_t  _mavlink_status;
    return &_mavlink_status;

}

mavlink_message_t* MAVLink::get_buffer() {
    static mavlink_message_t _mavlink_buffer;
    return &_mavlink_buffer;

}


/*
 * Below are mavlink_bridge_header.h definitions
 */

/*
   This MAVLink convenience function, once defined, will be used by MAVLink to send packed bytes.
   It provides MAVLink with the device serial interface to send bytes.
 */
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, uint16_t length) {

    LDEBUG("Sending Bytes on Channel: "<< chan);

    if (chan == MAVLINK_COMM_0){
        //auto m = MAVLink::get_instance(MAVLink::Type_t::Hard);
        auto m = mavlink::gMAVLink_instances[MAVLink::Type_t::Hard];
        try { m->itsSerial->writeNoCheck(ch, length); } catch (...) { jevois::warnAndIgnoreException(); }
    }
    else if (chan == MAVLINK_COMM_1){
        auto m = MAVLink::get_instance(MAVLink::Type_t::USB);
        try { m->itsSerial->writeNoCheck(ch, length); } catch (...) { jevois::warnAndIgnoreException(); }
    }
}


/*
 * Internal function to give access to the channel status for each channel
 */
mavlink_status_t *mavlink_get_channel_status(uint8_t chan) {

    LDEBUG("Get channel status");
    if (chan == MAVLINK_COMM_0){
        auto m = MAVLink::get_instance(MAVLink::Type_t::Hard);
        return m->get_status();
    }
    else if (chan == MAVLINK_COMM_1){
        auto m = MAVLink::get_instance(MAVLink::Type_t::USB);
        return m->get_status();
    }
}

/*
 * Internal function to give access to the channel buffer for each channel
 */
mavlink_message_t *mavlink_get_channel_buffer(uint8_t chan) {

    LDEBUG("Get channel buffer");
    if (chan == MAVLINK_COMM_0){
        auto m = MAVLink::get_instance(MAVLink::Type_t::Hard);
        return m->get_buffer();
    }
    else if (chan == MAVLINK_COMM_1){
        auto m = MAVLink::get_instance(MAVLink::Type_t::USB);
        return m->get_buffer();
    }
}
