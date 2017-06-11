#include <string.h>
#include "MAVLink_settings.h"
#include <jevoismavlink/mavlink.h>

//enum MAVLink_param_id_t MAVLink_param_id;
struct MAVLink_data_struct MAVLink_data;
/**
 * @brief reset all parameters to default
 */
void MAVLink_data_reset_param_defaults(void){

	MAVLink_data.param[PARAM_SYSTEM_ID] = 85;
	strcpy(MAVLink_data.param_name[PARAM_SYSTEM_ID], "SYS_ID");
	MAVLink_data.param_access[PARAM_SYSTEM_ID] = READ_WRITE;

	MAVLink_data.param[PARAM_COMPONENT_ID] = MAV_COMP_ID_CAMERA;
	strcpy(MAVLink_data.param_name[PARAM_COMPONENT_ID], "SYS_COMP_ID");
	MAVLink_data.param_access[PARAM_COMPONENT_ID] = READ_WRITE;

	MAVLink_data.param[PARAM_SENSOR_ID] = 100;
	strcpy(MAVLink_data.param_name[PARAM_SENSOR_ID], "SYS_SENSOR_ID");
	MAVLink_data.param_access[PARAM_SENSOR_ID] = READ_WRITE;

	MAVLink_data.param[PARAM_SYSTEM_TYPE] = MAV_TYPE_GENERIC;
	strcpy(MAVLink_data.param_name[PARAM_SYSTEM_TYPE], "SYS_TYPE");
	MAVLink_data.param_access[PARAM_SYSTEM_TYPE] = READ_WRITE;

	MAVLink_data.param[PARAM_AUTOPILOT_TYPE] = MAV_AUTOPILOT_GENERIC;
	strcpy(MAVLink_data.param_name[PARAM_AUTOPILOT_TYPE], "SYS_AP_TYPE");
	MAVLink_data.param_access[PARAM_AUTOPILOT_TYPE] = READ_WRITE;

    MAVLink_data.param[PARAM_SYSTEM_STATUS] = MAV_SYS_STATUS_SENSOR_VISION_POSITION;
    strcpy(MAVLink_data.param_name[PARAM_SYSTEM_STATUS], "SYS_STATUS_SENSOR");
    MAVLink_data.param_access[PARAM_SYSTEM_STATUS] = READ_WRITE;

	MAVLink_data.param[PARAM_SW_VERSION] = 1000;
	strcpy(MAVLink_data.param_name[PARAM_SW_VERSION], "SYS_SW_VER");
	MAVLink_data.param_access[PARAM_SW_VERSION] = READ_WRITE;

	MAVLink_data.param[PARAM_SYSTEM_SEND_STATE] = 1;
	strcpy(MAVLink_data.param_name[PARAM_SYSTEM_SEND_STATE], "SYS_SEND_STATE");
	MAVLink_data.param_access[PARAM_SYSTEM_SEND_STATE] = READ_WRITE;


}