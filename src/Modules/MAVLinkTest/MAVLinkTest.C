//! MAVLink Integration Test
/*!
    @author Ali AlSaibie

    \ingroup modules */

#include <Eigen/Dense>
#include <jevois/Core/Module.H>
#include <jevois/Debug/Timer.H>
#include <jevoisbase/Components/MAVLink/MAVLink.H>
#include <jevois/Image/RawImageOps.H>
#include <linux/videodev2.h>
#include <sys/sysinfo.h>


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

        itsMAVLink = addSubComponent<MAVLink>("MAVLink", &MAVLinkData, MAVLink::Type_t::USB);
        itsMAVLink->set_instance(itsMAVLink);
        dummy_increment = 0;
        vision_position = Eigen::Vector3f::Zero();
    }

    //! Virtual destructor for safe inheritance
    virtual ~MAVLinkTest() {}

    //! MAVLink Task
    void MAVLinkTask(){

        auto m = MAVLink::get_instance(MAVLink::Type_t::USB);

        if (!m) LERROR("No Valid MAVLink Instance");

        // Send Heartbeat Every 1 second
        end = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end - start;


        if (elapsed_seconds.count() >= 1.0) {
            m->send_system_state();
            LINFO("elapsed time: " << elapsed_seconds.count());
            start = std::chrono::system_clock::now();
        }
        // Read Periodically
#ifdef JEVOIS_PLATFORM
        m->receive();
#else
        m->receive();
#endif
        //TODO: run mavlink in separate thread, then block when reading to allow for full packet transmit.

        // Send Parameters
//      LINFO("Send Parameters");
        m->send_parameters(true);

        /* Process Dummy Data */
        {
            dummy_increment++;
            vision_position << 50, 100, 150;
            vision_position += 2 * dummy_increment * Eigen::Vector3f::Ones();
            vision_position += 5 * Eigen::Vector3f::Random();
            if (dummy_increment > 3) dummy_increment = 0;
        }

        /* Send Dummy Data */
        {
            mavlink_msg_vision_position_estimate_send(MAVLINK_COMM_1, get_boot_time_us(),
                                                      vision_position(0), vision_position(1), vision_position(2),
                                                      0, 0, 0);
        }
    }

    // ####################################################################################################
    //! Processing function, no video output
    // ####################################################################################################
    virtual void process(jevois::InputFrame && inframe) override {

        itsProcessingTimer.start();
        // Wait for next available camera image:
        jevois::RawImage inimg = inframe.get();

        // Convert it to gray:
        //cv::Mat imggray = jevois::rawimage::convertToCvGray(inimg);

        // Compute the vanishing point, with no drawings:
        //jevois::RawImage visual; // unallocated pixels, will not draw anything

        // Let camera know we are done processing the input image:
        inframe.done();

        // PASSTHROUGH //

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
    mavlink::MAVLink_data_struct MAVLinkData;
    std::shared_ptr<MAVLink> itsMAVLink;

protected:
    jevois::Timer itsProcessingTimer;

private:
    Eigen::Vector3f vision_position;
    uint8_t dummy_increment;
    std::chrono::time_point<std::chrono::system_clock> start, end;
};

mavlink_attitude_t MAVLinkTest::attitude;


// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(MAVLinkTest);
