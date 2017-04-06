//! MAVLink Integration Test
/*!
    @author Ali AlSaibie

    \ingroup modules */

#include <jevois/Core/Module.H>
#include "MAVLinkCommunication.h"
#include "MAVLinkTest_settings.h"

namespace MAVLinkn {


}

class MAVLinkTest : public jevois::Module
{
public:
    //! Default base class constructor ok
    MAVLinkTest(std::string const & instance) : jevois::Module(instance)
    { itsMAVLink = addSubComponent<MAVLinkCommunication>("MAVLink communication", &MAVLink_data); }

    //! Virtual destructor for safe inheritance
    virtual ~MAVLinkTest() { }

    //! Processing function
    virtual void process(jevois::InputFrame && inframe, jevois::OutputFrame && outframe) override
    {
        // Send Heartbeat Every 1 second
        itsMAVLink->senSystemState();

        // Read Periodically
        itsMAVLink->receive();

        // Send Parameters
        itsMAVLink->sendParameters();

    }

    static mavlink_attitude_t attitude;

protected:
    jevois::Timer itsProcessingTimer;
    std::shared_ptr<MAVLinkCommunication> itsMAVLink;
};

//! Define handle_mavlink_message here - it's specific to the application
MAVLinkCommunication::handle_mavlink_message(mavlink_message_t* msg) {

    switch (msg->msgid) {
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    {
        LDEBUG("MSG ID PARAM REQUEST READ");
    } break;
    case MAVLINK_MSG_ID_PARAM_SET:
    {
        LDEBUG("MSG ID PARAM REQUEST SET");
    } break;
    case MAVLINK_MSG_ID_ATTITUDE:
    {
        LDEBUG("MSG ID ATTITUDE");
        mavlink_msg_attitude_decode(msg, & MAVLinkTest::attitude);
    } break;


    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        /* Start sending parameters */
        LDEBUG("MSG ID PARAM SEND LIST");
        m_parameter_i = 0;
    } break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
    {
        LDEBUG("MSG ID COMMAND LONG");
    } break;
    case MAVLINK_MSG_ID_PING:
    {
        LDEBUG("MSG ID PING");
    } break;

    }

}
// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(MAVLinkTest);
