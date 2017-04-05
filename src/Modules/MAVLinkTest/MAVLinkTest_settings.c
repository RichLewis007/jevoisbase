#include "MAVLinkTest_settings.h"
#include <mavlink.h>

enum MAVLink_param_id_t MAVLink_param_id;
struct MAVLink_data_struct MAVLink_data;
/**
 * @brief reset all parameters to default
 */
void MAVLink_data_reset_param_defaults(void){

	MAVLinkData.param[PARAM_SYSTEM_ID] = 81;
	strcpy(MAVLinkData.param_name[PARAM_SYSTEM_ID], "SYS_ID");
	MAVLinkData.param_access[PARAM_SYSTEM_ID] = READ_WRITE;

	MAVLinkData.param[PARAM_COMPONENT_ID] = 50;
	strcpy(MAVLinkData.param_name[PARAM_COMPONENT_ID], "SYS_COMP_ID");
	MAVLinkData.param_access[PARAM_COMPONENT_ID] = READ_WRITE;

	MAVLinkData.param[PARAM_SENSOR_ID] = 77;
	strcpy(MAVLinkData.param_name[PARAM_SENSOR_ID], "SYS_SENSOR_ID");
	MAVLinkData.param_access[PARAM_SENSOR_ID] = READ_WRITE;

	MAVLinkData.param[PARAM_SYSTEM_TYPE] = MAV_TYPE_GENERIC;
	strcpy(MAVLinkData.param_name[PARAM_SYSTEM_TYPE], "SYS_TYPE");
	MAVLinkData.param_access[PARAM_SYSTEM_TYPE] = READ_WRITE;

	MAVLinkData.param[PARAM_AUTOPILOT_TYPE] = MAV_AUTOPILOT_GENERIC;
	strcpy(MAVLinkData.param_name[PARAM_AUTOPILOT_TYPE], "SYS_AP_TYPE");
	MAVLinkData.param_access[PARAM_AUTOPILOT_TYPE] = READ_WRITE;

	MAVLinkData.param[PARAM_SW_VERSION] = 1300;
	strcpy(MAVLinkData.param_name[PARAM_SW_VERSION], "SYS_SW_VER");
	MAVLinkData.param_access[PARAM_SW_VERSION] = READ_WRITE;

	MAVLinkData.param[PARAM_SYSTEM_SEND_STATE] = 1;
	strcpy(MAVLinkData.param_name[PARAM_SYSTEM_SEND_STATE], "SYS_SEND_STATE");
	MAVLinkData.param_access[PARAM_SYSTEM_SEND_STATE] = READ_WRITE;

	MAVLinkData.param[DEBUG_VARIABLE] = 1;
	strcpy(MAVLinkData.param_name[DEBUG_VARIABLE], "DEBUG");
	MAVLinkData.param_access[DEBUG_VARIABLE] = READ_WRITE;

}