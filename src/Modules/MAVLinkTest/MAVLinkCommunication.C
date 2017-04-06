/*
MAVLink Integration on Jevois Module

@author Ali AlSaibie
@email ali@alsaibie.com

*/
#include <jevoisbase/src/Components/MAVLinkCommunication/MAVLinkCommunication.H>

// ####################################################################################################
MAVLinkCommunication::MAVLinkCommunication(std::string const & instance, struct MAVLink_data_struct * MAVData) :
  jevois::Component(instance), itsInstance(this), itsMAVLinkData(MAVData), m_parameter_i(0)

{
  // I need to access this parameter the proper way in order to call onParChange
  //jevois::engine::serialdev = "";

  // Add Serial locally here
  itsSerial = addSubComponent<jevois::Serial>("serial", MAVLinkCommunication::UserInterface::Type::Hard);
  itsSerial->setParamVal("devname", MAVLinkCommunication::serialdev);
  // Or?  What is MAVLinkCommunication::serialdev type?
  // itsSerial-> setParamString("devname", MAVLinkCommunication::serialdev);

  // Reset MAVLink Parameters to Default
  MAVLink_data_reset_param_defaults();

  // Set system ID
  mavlink_system.sysid = itsMAVLinkData->param[PARAM_SYSTEM_ID]; // System ID, 1-255
  mavlink_system.compid = itsMAVLinkData->param[PARAM_COMPONENT_ID]; // Component/Subsystem ID, 1-255
}

// ####################################################################################################
MAVLinkCommunication::~MAVLinkCommunication()
{
  // On module exit revert back to normal serial operation.
  //jevois::engine::serialdev = MAVLinkCommunication::serialdev;
}

// ####################################################################################################
void MAVLinkCommunication::sendSystemState(void) {

  mavlink_msg_heartbeat_send(MAVLINK_COMM_0, MAVData->param[PARAM_SYSTEM_TYPE], MAVData->param[PARAM_AUTOPILOT_TYPE], 0, 0, 0);
}

// ####################################################################################################
void MAVLinkCommunication::sendParameters(void) {

  if (m_parameter_i < ONBOARD_PARAM_COUNT)
  {
    mavlink_msg_param_value_send(MAVLINK_COMM_0,
                                 itsMAVLinkData->pparam_name[m_parameter_i],
                                 itsMAVLinkData->pparam[m_parameter_i], MAVLINK_TYPE_FLOAT, ONBOARD_PARAM_COUNT, m_parameter_i);
    m_parameter_i++;
  }
}

// ####################################################################################################
void MAVLinkCommunication::receive(void) {

  mavlink_message_t msg;
  mavlink_status_t status = { 0 };

  /* 20 bytes at a time */
  const unsigned character_count = 20;

  /* Read up to buffer size. Loop through characters read and parse. Handle message when packet is compelte */
  if ( (numbytes = itsSerial->read(MAVLinkReceiveBuf, sizeof(MAVLinkReceiveBuf)) > (size_t)character_count) {
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

MAVLinkCommunication *
MAVLinkCommunication::get_instance(unsigned instance)
{
  if (itsInstance != nullptr) {
    return itsInstance;
  }

  LERROR("No MAVlink Instance to return");
  return nullptr;
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
    MAVLinkCommunication *m = MAVLinkCommunication::get_instance();
    if (m != nullptr) {
      m->itsSerial->write(ch, length);
    }
  }
}
