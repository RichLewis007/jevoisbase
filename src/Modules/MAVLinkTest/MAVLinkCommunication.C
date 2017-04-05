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
  itsSerial = addSubComponent<jevois::Serial>("serial", jevois::UserInterface::Type::Hard);
  itsSerial->setParamVal("devname", MAVLinkCommunication::serialdev);


	// Reset MAVLink Parameters to Default
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

}

// ####################################################################################################
void MAVLinkCommunication::sendParameters(void){

}

// ####################################################################################################
void MAVLinkCommunication::receive(void){

}

void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t * ch, uint16_t length){
  if (chan == MAVLINK_COMM_0)
  {
    /* send to Hard UART grab from module. 
    Need an implementation that accepts character and length here*/
    MAVLinkTest::sendSerial()
  }
}
