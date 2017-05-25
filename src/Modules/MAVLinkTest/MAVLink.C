/*
   MAVLink Integration on Jevois Module

   @author Ali AlSaibie
   @email ali@alsaibie.com

 */

#include <jevoisbase/src/Components/MAVLinkCommunication/MAVLink.H>
#include "mavlink_bridge_header.h"
#include "MAVLink.H"

static MAVLink *_mavlink_instance = nullptr;


/*

 */
MAVLink::MAVLink(std::string const & instance, struct MAVLink_data_struct * MAVData) :
        jevois::Component(instance), itsMAVLinkData(MAVData), m_parameter_i(ONBOARD_PARAM_COUNT)

{

        ::_mavlink_instance = this;

        // I need to access this parameter the proper way in order to call onParChange
        //jevois::engine::serialdev = "";

        // Add Serial locally here
        itsSerial = addComponent<jevois::Serial>("serial", mavlink::UserInterface::Type::Hard);

        // devname set in params.cfg
        // itsSerial->setParamVal("devname", MAVLinkCommunication::serialdev);
        // Or?  What is MAVLinkCommunication::serialdev type??
        // itsSerial-> setParamString("devname", MAVLinkCommunication::serialdev);

        // Reset MAVLink Parameters to Default

        MAVLink_data_reset_param_defaults();
        
        // Set system ID
        mavlink_system.sysid = itsMAVLinkData->param[PARAM_SYSTEM_ID]; // System ID, 1-255
        mavlink_system.compid = itsMAVLinkData->param[PARAM_COMPONENT_ID]; // Component/Subsystem ID, 1-255
}


MAVLink::~MAVLink()
{
        ::_mavlink_instance = nullptr;
}


void MAVLink::send_system_state(void) {

        mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAVData->param[PARAM_SYSTEM_TYPE], MAVData->param[PARAM_AUTOPILOT_TYPE], 0, 0, 0);
}

void MAVLink::send_parameters(void) {

        if (m_parameter_i < ONBOARD_PARAM_COUNT)
        {
                mavlink_msg_param_value_send(MAVLINK_COMM_0,
                                             itsMAVLinkData->pparam_name[m_parameter_i],
                                             itsMAVLinkData->pparam[m_parameter_i], MAVLINK_TYPE_FLOAT, ONBOARD_PARAM_COUNT, m_parameter_i);
                m_parameter_i++;
        }
}


void MAVLink::receive(void){

        mavlink_message_t msg;
        mavlink_status_t status = { 0 }

        /* 20 bytes at a time */
        const unsigned character_count = 20;

        /* Read up to buffer size. Loop through characters read and parse. Handle message when packet is compelte */
        if ( (numbytes = itsSerial->read(MAVLinkReceiveBuf, sizeof(MAVLinkReceiveBuf) )) > (size_t)character_count) {

                for (size_t i = 0; i < numbytes; i++) {
                        if (mavlink_parse_char(MAVLINK_COMM_0, MAVLinkReceiveBuf[i], &msg, &status))
                        {
                                /* Handle message */
                                LDEBUG("New MAVLink Message Received");
                                handle_mavlink_message(&msg);

                        }
                }
        }
}

MAVLink *
MAVLink::get_instance(void)
{
        MAVLink *inst = ::_mavlink_instance;
        if (inst != nullptr) {
                return inst;
        }
        else {
                LERROR("No MAVLink instance available");
                inst = new MAVLink;
                return inst;
        }
}

void MAVLink::handle_mavlink_message(mavlink_message_t *msg) {

}


/*
   This MAVLink convenience function, once defined, will be used by MAVLink to send packed bytes.
   It provides MAVLink with the device serial interface to send bytes.
 */
void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t * ch, uint16_t length) {
        // For now just implement on Hard Serial, then incorporate USB Serial
        if (chan == MAVLINK_COMM_0)
        {
                /* send to Hard UART grab from module.
                   Need an implementation that accepts character and length here*/
                MAVLink *m = MAVLink::get_instance();
                if (m != nullptr) {
                        m->itsSerial->write(ch, length);
                }
        }
}



/*
 * Internal function to give access to the channel status for each channel
 */
mavlink_status_t* mavlink_get_channel_status(uint8_t channel)
{
  if (chan == MAVLINK_COMM_0)
  {
          MAVLink *m = MAVLink::get_instance();
          if (m != nullptr) {
                  m->get_status();
          }
          else{
            return nullptr;
          }
  }

}

/*
 * Internal function to give access to the channel buffer for each channel
 */
mavlink_message_t* mavlink_get_channel_buffer(uint8_t channel)
{
  if (chan == MAVLINK_COMM_0)
  {
          MAVLink *m = MAVLink::get_buffer();
          if (m != nullptr) {
                  m->get_status();
          }
          else{
            return nullptr;
          }
        }
}
