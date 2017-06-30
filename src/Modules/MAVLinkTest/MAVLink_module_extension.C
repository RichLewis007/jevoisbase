/*
 * Summary: this file contains extensions of the MAVLink components, with definitions specific to the module implementation
 */

#include <string.h>
#include <jevoisbase/Components/MAVLink/MAVLink.H>
#include <mavlink/jevoismavlink/mavlink.h>

inline int FLOAT_EQ_FLOAT(float f1, float f2) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
	return (f1 == f2);
#pragma GCC diagnostic pop
}

//! reset all parameters to default
using namespace mavlink;

void MAVLink::reset_param_defaults(void){

	itsMAVLinkData->param[PARAM_SYSTEM_ID] = 85;
	strcpy(itsMAVLinkData->param_name[PARAM_SYSTEM_ID], "SYS_ID");
	itsMAVLinkData->param_access[PARAM_SYSTEM_ID] = READ_WRITE;

	itsMAVLinkData->param[PARAM_COMPONENT_ID] = MAV_COMP_ID_CAMERA;
	strcpy(itsMAVLinkData->param_name[PARAM_COMPONENT_ID], "SYS_COMP_ID");
	itsMAVLinkData->param_access[PARAM_COMPONENT_ID] = READ_WRITE;

	itsMAVLinkData->param[PARAM_SENSOR_ID] = 100;
	strcpy(itsMAVLinkData->param_name[PARAM_SENSOR_ID], "SYS_SENSOR_ID");
	itsMAVLinkData->param_access[PARAM_SENSOR_ID] = READ_WRITE;

	itsMAVLinkData->param[PARAM_SYSTEM_TYPE] = MAV_TYPE_GENERIC;
	strcpy(itsMAVLinkData->param_name[PARAM_SYSTEM_TYPE], "SYS_TYPE");
	itsMAVLinkData->param_access[PARAM_SYSTEM_TYPE] = READ_WRITE;

	itsMAVLinkData->param[PARAM_AUTOPILOT_TYPE] = MAV_AUTOPILOT_GENERIC;
	strcpy(itsMAVLinkData->param_name[PARAM_AUTOPILOT_TYPE], "SYS_AP_TYPE");
	itsMAVLinkData->param_access[PARAM_AUTOPILOT_TYPE] = READ_WRITE;

    itsMAVLinkData->param[PARAM_SYSTEM_STATUS] = MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    strcpy(itsMAVLinkData->param_name[PARAM_SYSTEM_STATUS], "SYS_STATUS_SENSOR");
    itsMAVLinkData->param_access[PARAM_SYSTEM_STATUS] = READ_WRITE;

	itsMAVLinkData->param[PARAM_SW_VERSION] = 1000;
	strcpy(itsMAVLinkData->param_name[PARAM_SW_VERSION], "SYS_SW_VER");
	itsMAVLinkData->param_access[PARAM_SW_VERSION] = READ_WRITE;

	itsMAVLinkData->param[PARAM_SYSTEM_SEND_STATE] = 1;
	strcpy(itsMAVLinkData->param_name[PARAM_SYSTEM_SEND_STATE], "SYS_SEND_STATE");
	itsMAVLinkData->param_access[PARAM_SYSTEM_SEND_STATE] = READ_WRITE;

}

//! Define handle_mavlink_message here - it's specific to the application
void MAVLink::handle_mavlink_message(mavlink_message_t *msg) {

	LINFO("Handling Message ID: " <<msg->msgid);

	switch (msg->msgid) {
		case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			/* Copied from PX4Flow Implementation */
			mavlink_param_request_read_t set;
			mavlink_msg_param_request_read_decode(msg, &set);

			/* Check if this message is for this system */
			if ((uint8_t) set.target_system == (uint8_t) itsMAVLinkData->param[PARAM_SYSTEM_ID]
				&& (uint8_t) set.target_component == (uint8_t) itsMAVLinkData->param[PARAM_COMPONENT_ID]) {
				char *key = (char *) set.param_id;
				if (set.param_id[0] != (char) -1) {
					/* Choose parameter based on index */
					if ((set.param_index >= 0) && (set.param_index < ONBOARD_PARAM_COUNT)) {
						/* Report back value */
						mavlink_msg_param_value_send(mavlink_channel,
													 itsMAVLinkData->param_name[set.param_index],
													 itsMAVLinkData->param[set.param_index], MAVLINK_TYPE_FLOAT,
													 ONBOARD_PARAM_COUNT, set.param_index);
					}
				} else /* Based on full name */
				{
					for (int i = 0; i < ONBOARD_PARAM_COUNT; i++) {
						bool match = true;
						for (int j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++) {
							/* Compare */
							if (((char) (itsMAVLinkData->param_name[i][j])) != (char) (key[j])) {
								match = false;
								/* No need to continue checking */
								break;
							}

							/* End matching if null termination is reached */
							if (((char) itsMAVLinkData->param_name[i][j]) == '\0') {
								break;
							}
						}

						/* Check if matched */
						if (match) {
							/* Report back value */
							mavlink_msg_param_value_send(mavlink_channel,
														 itsMAVLinkData->param_name[i],
														 itsMAVLinkData->param[i], MAVLINK_TYPE_FLOAT,
														 ONBOARD_PARAM_COUNT, i);
						}
					}
				}
			}
			LDEBUG("MSG ID PARAM REQUEST READ ");
		}
			break;
		case MAVLINK_MSG_ID_PARAM_SET: {
			mavlink_param_set_t set;
			mavlink_msg_param_set_decode(msg, &set);

			/* Check if this message is for this system */
			if ((uint8_t) set.target_system
				== (uint8_t) itsMAVLinkData->param[PARAM_SYSTEM_ID]
				&& (uint8_t) set.target_component
				   == (uint8_t) itsMAVLinkData->param[PARAM_COMPONENT_ID]) {
				char *key = (char *) set.param_id;

				for (int i = 0; i < ONBOARD_PARAM_COUNT; i++) {
					bool match = true;
					for (int j = 0; j < ONBOARD_PARAM_NAME_LENGTH; j++) {
						/* Compare */
						if (((char) (itsMAVLinkData->param_name[i][j]))
							!= (char) (key[j])) {
							match = false;
							/* No need to continue checking */
							break;
						}

						/* End matching if null termination is reached */
						if (((char) itsMAVLinkData->param_name[i][j]) == '\0') {
							break;
						}
					}

					/* Check if matched */
					if (match) {
						/* Only write and emit changes if there is actually a difference
                         * AND only write if new value is NOT "not-a-number"
                         * AND is NOT infinity
                         * AND has access
                         */
						if (!FLOAT_EQ_FLOAT(itsMAVLinkData->param[i], set.param_value) && !isnan(set.param_value)
							&& !isinf(set.param_value) && itsMAVLinkData->param_access[i]) {
							itsMAVLinkData->param[i] = set.param_value;

							/* handle sensor position */
//                        if(i == PARAM_SENSOR_POSITION)
//                        {
//
//                        }

							/* report back new value */
							mavlink_msg_param_value_send(mavlink_channel, itsMAVLinkData->param_name[i],
														 itsMAVLinkData->param[i], MAVLINK_TYPE_FLOAT,
														 ONBOARD_PARAM_COUNT, i);


						} else {
							/* send back current value because it is not accepted or not write access*/
							mavlink_msg_param_value_send(mavlink_channel,
														 itsMAVLinkData->param_name[i],
														 itsMAVLinkData->param[i], MAVLINK_TYPE_FLOAT,
														 ONBOARD_PARAM_COUNT, i);
						}
					}
				}
			}

			LDEBUG("MSG ID PARAM REQUEST SET");
		}
			break;

		case MAVLINK_MSG_ID_PING: {
			mavlink_ping_t ping;
			mavlink_msg_ping_decode(msg, &ping);
			if (ping.target_system == 0 && ping.target_component == 0) {
//                uint64_t r_timestamp = get_boot_time_us();
//                mavlink_msg_ping_send(MAVLINK_COMM_0, ping.seq, msg->sysid, msg->compid, r_timestamp);
			}
		}
			break;

		case MAVLINK_MSG_ID_ATTITUDE: {
			LDEBUG("MSG ID ATTITUDE");
//            mavlink_msg_attitude_decode(msg, &(MAVLinkTest::attitude));

		}
			break;


		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			/* Start sending parameters */
			LDEBUG("MSG ID PARAM SEND LIST");
			m_parameter_i = 0;
		}
			break;
	}
}

