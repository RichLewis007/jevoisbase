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

        // Send Parameters Periodically
        itsMAVLink->sendParameters();

    }
    
    mavlink_attitude_t attitude;

protected:
    jevois::Timer itsProcessingTimer;
    std::shared_ptr<MAVLinkCommunication> itsMAVLink;
};

//! Define handle_mavlink_message here
MAVLinkCommunication::handle_mavlink_message(mavlink_message_t* msg){

//Somewhere in here
    case MAVLINK_MSG_ID_ATTITUDE:
    mavlink_msg_attitude_decode(msg, & MAVLinkTest::attitude);

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    {
        /* Start sending parameters */
        m_parameter_i = 0;
    }
}
// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(MAVLinkTest);
