/*
MAVLink Integration on Jevois Module

@author Ali AlSaibie
@email ali@alsaibie.com
 
*/
#include <jevoisbase/src/Components/MAVLinkCommunication/MAVLinkCommunication.H>


// ####################################################################################################
MAVLinkCommunication::MAVLinkCommunication(std::string const & instance, struct MAVLink_data_struct * MAVData) :
jevois::Component(instance)

{
  // I need to access this parameter the proper way in order to call onParChange
  jevois::engine::serialdev =""; 
	
  // Add Serial locally here 
  itsSerial = addSubComponent<jevois::Serial>("serial", MAVLinkCommunication::UserInterface::Type::Hard);
  itsSerial-> setParamVal("devname", MAVLinkCommunication::serialdev); 
  // Or?  What is MAVLinkCommunication::serialdev type? 
  // itsSerial-> setParamString("devname", MAVLinkCommunication::serialdev); 

	// Reset MAVLink Parameters to Default (includes setting System ID etc)
	MAVLink_data_reset_param_defaults();
}

// ####################################################################################################
MAVLinkCommunication::~MAVLinkCommunication()
{ 
  // On module exit revert back to normal serial operation. 
  jevois::engine::serialdev = MAVLinkCommunication::serialdev; 
}

// ####################################################################################################
void MAVLinkCommunication::sendSystemState(void){
  mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAVData->param[PARAM_SYSTEM_TYPE], MAVData->param[PARAM_AUTOPILOT_TYPE], 0, 0, 0);
}

// ####################################################################################################
void MAVLinkCommunication::sendParameters(void){

}

// ####################################################################################################
void MAVLinkCommunication::receive(void){
  mavlink_message_t msg;
  mavlink_status_t status = { 0 };

  //This is blocking, need to change, but general idea is
  while (charavailable)
  {
    uint8_t c = itsSerial->read(MAVLink_buffer, 1);

    /* Try to get a new message */
    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status))
    {
      /* Handle message */
      handle_mavlink_message(&msg);
    }
    /* And get the next one */
  }

}

void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t * ch, uint16_t length){
  if (chan == MAVLINK_COMM_0)
  {
    /* send to Hard UART grab from module. 
    Need an implementation that accepts character and length here*/
    MAVLinkTest::sendSerial()
  }
}
