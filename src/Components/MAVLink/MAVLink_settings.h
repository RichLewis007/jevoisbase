


#define ONBOARD_PARAM_NAME_LENGTH 	15

/**
  * @brief  parameter access
  */
#ifdef __cplusplus
extern "C"
{
#endif

	typedef enum
	{
	  READ_ONLY   = 0,
	  READ_WRITE  = 1,
	} ParameterAccess_TypeDef;

	typedef struct
	{
		/* nothing here until now */

	} SysState_TypeDef;

	enum MAVLink_param_id_t
	{
		PARAM_SYSTEM_ID = 0,
		PARAM_COMPONENT_ID,
		PARAM_SENSOR_ID,
		PARAM_SYSTEM_TYPE,
		PARAM_AUTOPILOT_TYPE,
        PARAM_SYSTEM_STATUS,
        PARAM_SW_VERSION,
		PARAM_SYSTEM_SEND_STATE,
		 
		/* Keep this last */
		ONBOARD_PARAM_COUNT

	};

	struct MAVLink_data_struct
	{
		SysState_TypeDef system_state;
		float param[ONBOARD_PARAM_COUNT];
		char param_name[ONBOARD_PARAM_COUNT][ONBOARD_PARAM_NAME_LENGTH];
		ParameterAccess_TypeDef param_access[ONBOARD_PARAM_COUNT];
	};

	/* global declarations */
	// extern enum MAVLink_param_id_t MAVLink_param_id; //TODO: is this needed?
	extern struct MAVLink_data_struct MAVLink_data;

	/******************************************************************
	  * DEFAULT SETTINGS FUNCTIONS
	  */

	void MAVLink_data_reset_param_defaults(void);

#ifdef __cplusplus
}
#endif
