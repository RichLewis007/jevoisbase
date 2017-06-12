//! MAVLink Integration Test
/*!
    @author Ali AlSaibie

    \ingroup modules */

#include <Eigen/Dense>
#include <jevois/Core/Module.H>
#include <jevois/Debug/Timer.H>
#include <jevoisbase/src/Components/MAVLink/MAVLink.H>
#include <jevois/Image/RawImageOps.H>
#include <linux/videodev2.h>
#include <sys/sysinfo.h>

inline int FLOAT_EQ_FLOAT(float f1, float f2) {
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wfloat-equal"
    return (f1 == f2);
#pragma GCC diagnostic pop
}

uint64_t get_boot_time_us(void) {
    std::chrono::milliseconds uptime(0u);
    struct sysinfo x;
    if (sysinfo(&x) == 0) {
        uptime = std::chrono::milliseconds(static_cast<unsigned long long>(x.uptime) * 1000ULL);
        return (uint64_t) uptime.count();
    }
    return 0;

}

class MAVLinkTest : public jevois::Module {
public:
    //! Default base class constructor ok
    MAVLinkTest(std::string const &instance) : jevois::Module(instance),
                                               itsProcessingTimer("Processing", 30, LOG_DEBUG) {
        LDEBUG("Adding MAVLink Instance");
        itsMAVLink = addSubComponent<MAVLink>("MAVLink", &MAVLink_data, MAVLink::Type_t::USB);
        itsMAVLink->set_instance(itsMAVLink);
        dummy_increment = 0;
        vision_position = Eigen::Vector3f::Zero();
    }

    //! Virtual destructor for safe inheritance
    virtual ~MAVLinkTest() {}

    //! MAVLink Task
    void MAVLinkTask(){

        LINFO("Grabbing a mavlink instance in MAVLinkTest");
        //auto m = MAVLink::get_instance(MAVLink::Type_t::USB);

        //auto m = mavlink::gMAVLink_instances[MAVLink::Type_t::USB];
        auto m = itsMAVLink;
        //if (m == nullptr) LERROR("No Valid Instance. Count");
        // Send Heartbeat Every 1 second
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;
        if (elapsed_seconds.count() >= 0.50) {
            m->send_system_state();
            LINFO("elapsed time: " << elapsed_seconds.count());
            start = std::chrono::system_clock::now();
        }
        LINFO("Receive");
        // Read Periodically
        //m->receive();
        //TODO: run mavlink in separate thread, then block when reading to allow for full packet transmit.

        // Send Parameters
        //m->send_parameters(TRUE);
//
//        /* Process Dummy Data */
//        {
//            dummy_increment++;
//            vision_position << 50, 100, 150;
//            vision_position += 2 * dummy_increment * Eigen::Vector3f::Ones();
//            vision_position += 5 * Eigen::Vector3f::Random();
//            if (dummy_increment > 3) dummy_increment = 0;
//        }
//        /* Send Dummy Data */
//        {
//            mavlink_msg_vision_position_estimate_send(MAVLINK_COMM_1, get_boot_time_us(),
//                                                      vision_position(0), vision_position(1), vision_position(2),
//                                                      0, 0, 0);
//        }
    }

    // ####################################################################################################
    //! Processing function, no video output
    // ####################################################################################################
    virtual void process(jevois::InputFrame && inframe) override {

        itsProcessingTimer.start();
        std::chrono::high_resolution_clock::now();
        // PASSTHROUGH //

        // Wait for next available camera image:
        jevois::RawImage const inimg = inframe.get(true);
        inframe.done(); // NOTE: optional here, inframe destructor would call it anyway

        MAVLinkTask();

        std::string const &mavlinkcputime = itsProcessingTimer.stop();
    }

    // ####################################################################################################
    //! Processing function with video output to USB
    // ####################################################################################################
    virtual void process(jevois::InputFrame && inframe, jevois::OutputFrame && outframe) override {

        itsProcessingTimer.start();
        std::chrono::high_resolution_clock::now();

        // PASSTHROUGH //

        // Wait for next available camera image:
        jevois::RawImage const inimg = inframe.get(true);

        // Wait for an image from our gadget driver into which we will put our results:
        jevois::RawImage outimg = outframe.get();

        // Enforce that the input and output formats and image sizes match:
        outimg.require("output", inimg.width, inimg.height, inimg.fmt);

        // Just copy the pixel data over:
        memcpy(outimg.pixelsw<void>(), inimg.pixels<void>(), std::min(inimg.buf->length(), outimg.buf->length()));

        // Camera outputs RGB565 in big-endian, but most video grabbers expect little-endian:
        if (outimg.fmt == V4L2_PIX_FMT_RGB565) jevois::rawimage::byteSwap(outimg);

        // Let camera know we are done processing the input image:
        inframe.done(); // NOTE: optional here, inframe destructor would call it anyway

        // Send the output image with our processing results to the host over USB:
        outframe.send(); // NOTE: optional here, outframe destructor would call it anyway

        MAVLinkTask();

        std::string const &mavlinkcputime = itsProcessingTimer.stop();
    }

    static mavlink_attitude_t attitude;
    std::shared_ptr<MAVLink> itsMAVLink;

protected:
    jevois::Timer itsProcessingTimer;

private:
    Eigen::Vector3f vision_position;
    uint8_t dummy_increment;
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

mavlink_attitude_t MAVLinkTest::attitude;

//! Define handle_mavlink_message here - it's specific to the application
void MAVLink::handle_mavlink_message(mavlink_message_t *msg) {

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
            LDEBUG("MSG ID PARAM REQUEST READ");
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
                uint64_t r_timestamp = get_boot_time_us();
                mavlink_msg_ping_send(MAVLINK_COMM_0, ping.seq, msg->sysid, msg->compid, r_timestamp);
            }
        }
            break;

        case MAVLINK_MSG_ID_ATTITUDE: {
            LDEBUG("MSG ID ATTITUDE");
            mavlink_msg_attitude_decode(msg, &(MAVLinkTest::attitude));

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
// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(MAVLinkTest);
