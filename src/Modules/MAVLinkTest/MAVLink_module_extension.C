// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// JeVois Smart Embedded Machine Vision Toolkit - Copyright (C) 2016 by Laurent Itti, the University of Southern
// California (USC), and iLab at USC. See http://iLab.usc.edu and http://jevois.org for information about this project.
//
// This file is part of the JeVois Smart Embedded Machine Vision Toolkit.  This program is free software; you can
// redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software
// Foundation, version 2.  This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
// License for more details.  You should have received a copy of the GNU General Public License along with this program;
// if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
//
// Contact information: Laurent Itti - 3641 Watt Way, HNB-07A - Los Angeles, CA 90089-2520 - USA.
// Tel: +1 213 740 3527 - itti@pollux.usc.edu - http://iLab.usc.edu - http://jevois.org
// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
 * Summary: this file contains extensions of the MAVLink components, with definitions specific to the module implementation
 */
/*! \file */

#include <string.h>
#include <jevoisbase/Components/MAVLink/MAVLink.H>
#include <mavlink/jevoismavlink/mavlink.h>

inline int FLOAT_EQ_FLOAT(float f1, float f2) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    return (f1 == f2);
#pragma GCC diagnostic pop
}

extern uint64_t get_boot_time_us(void);
//! reset all parameters to default

using namespace mavlink;

void MAVLink::reset_param_defaults(void) {

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

    itsMAVLinkData->param[PARAM_JEVOIS_BRIGHTNESS] = 0;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_BRIGHTNESS], "JEVOIS_BRIGHTNESS");
    itsMAVLinkData->param_access[PARAM_JEVOIS_BRIGHTNESS] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_CONTRAST] = 3;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_CONTRAST], "JEVOIS_CONTRAST");
    itsMAVLinkData->param_access[PARAM_JEVOIS_CONTRAST] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_SATURATION] = 2;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_SATURATION], "JEVOIS_SATURATION");
    itsMAVLinkData->param_access[PARAM_JEVOIS_SATURATION] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_AUTOWB] = 1;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_AUTOWB], "JEVOIS_AUTOWB");
    itsMAVLinkData->param_access[PARAM_JEVOIS_AUTOWB] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_DOWB] = 0;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_DOWB], "JEVOIS_DOWB");
    itsMAVLinkData->param_access[PARAM_JEVOIS_DOWB] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_REDBAL] = 128;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_REDBAL], "JEVOIS_REDBAL");
    itsMAVLinkData->param_access[PARAM_JEVOIS_REDBAL] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_BLUEBAL] = 128;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_BLUEBAL], "JEVOIS_BLUEBAL");
    itsMAVLinkData->param_access[PARAM_JEVOIS_BLUEBAL] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_AUTOGAIN] = 1;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_AUTOGAIN], "JEVOIS_AUTOGAIN");
    itsMAVLinkData->param_access[PARAM_JEVOIS_AUTOGAIN] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_GAIN] = 16;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_GAIN], "JEVOIS_GAIN");
    itsMAVLinkData->param_access[PARAM_JEVOIS_GAIN] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_HFLIP] = 0;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_HFLIP], "JEVOIS_HFLIP");
    itsMAVLinkData->param_access[PARAM_JEVOIS_HFLIP] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_VFLIP] = 0;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_VFLIP], "JEVOIS_VFLIP");
    itsMAVLinkData->param_access[PARAM_JEVOIS_VFLIP] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_POWERFREQ] = 2;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_POWERFREQ], "JEVOIS_POWERFREQ");
    itsMAVLinkData->param_access[PARAM_JEVOIS_POWERFREQ] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_SHARPNESS] = 6;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_SHARPNESS], "JEVOIS_SHARPNESS");
    itsMAVLinkData->param_access[PARAM_JEVOIS_SHARPNESS] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_AUTOEXP] = 0;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_AUTOEXP], "JEVOIS_AUTOEXP");
    itsMAVLinkData->param_access[PARAM_JEVOIS_AUTOEXP] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_ABSEXP] = 500;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_ABSEXP], "JEVOIS_ABSEXP");
    itsMAVLinkData->param_access[PARAM_JEVOIS_ABSEXP] = READ_WRITE;

    itsMAVLinkData->param[PARAM_JEVOIS_PRESETWB] = 1;
    strcpy(itsMAVLinkData->param_name[PARAM_JEVOIS_PRESETWB], "JEVOIS_PRESETWB");
    itsMAVLinkData->param_access[PARAM_JEVOIS_PRESETWB] = READ_WRITE;

}

//! Define handle_mavlink_message here - it's specific to the application

void MAVLink::handle_mavlink_message(mavlink_message_t *msg) {

    LINFO("Handling Message ID: " << msg->msgid);

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

                            // TODO: Here I incorporate each of the changes. by checking i against param.
                            // TODO: Do it in chunks, say if i is between first camera and last camera parameter, call setCamParameter function
                            // and if its between values for a specific application then make a function to handle those ranges etc.

                            /* handle sensor position */
                            if (i >= PARAM_JEVOIS_BRIGHTNESS && i <= PARAM_JEVOIS_PRESETWB) {
                                LINFO("Setting Camera Control Parameter");
                                switch (i) {
                                    // TODO: I need access to itsCamera in Engine, to call setControl.
                                    // There is no built in access. So need to create a separate implementation of engine.
                                    // TODO: Also, make sure you do proper type conversion. MAVLinks are ints I believe
                                    case PARAM_JEVOIS_BRIGHTNESS: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_CONTRAST: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_SATURATION: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_AUTOWB: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_DOWB: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_REDBAL: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_BLUEBAL: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_AUTOGAIN: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_GAIN: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_HFLIP: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_VFLIP: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_POWERFREQ: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_SHARPNESS: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_AUTOEXP: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_ABSEXP: {
                                    }
                                        break;
                                    case PARAM_JEVOIS_PRESETWB: {
                                    }
                                        break;
                                }
                            }

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
                uint64_t r_timestamp = get_boot_time_us();
                mavlink_msg_ping_send(mavlink_channel, ping.seq, msg->sysid, msg->compid, r_timestamp);
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
