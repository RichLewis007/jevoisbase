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

#include <Eigen/Dense>
#include <jevois/Core/Module.H>
#include <jevois/Debug/Timer.H>
#include <jevoisbase/Components/MAVLink/MAVLink.H>
#include <jevois/Image/RawImageOps.H>
#include <linux/videodev2.h>
#include <sys/sysinfo.h>

/*! \file */

//! MAVLink Integration Test - Work in progress and contributiions welcome.
/*! This application is a basic demo for the MAVLink Component.

    This is a development test demo. This demo can be run with or without USB output. With USB Output the module does
    a passthrough.

    Either USB or Hard serial can be used with this module, to use either one, it must not be used by engine. This is
    done by assigning an empty string to the respective serial name macro in Engine.h or in the params.cfg file.

    A Mavlink instance is added as a subcomponent to the module, the returned shared_ptr is assigned to a global map
    which is accessed via get_instance method by free functions required by the mavlink implementation. This has the
    potential to allow for more than one mavlink instance to be used, on both USB and Hard serial, but note that
    jevois must have one serial terminal connected in order to run.

    --------
    You can daisy chain a jevois host and jevois platform to replicate MAVLink messages and have a debugger running
    on the host.

    Or, if you you two FTDI adapters you can loop back into the PC then run a jevois host instance and QGroundControl.

    You can connect jevois platform directly to QGroundControl, you should see the list of parameters
    and analyze the dummy output vision position values in a running plot.

    And of course, you can connect it to a mavlink capable autopilot once a useful vision processing module is
    incorporated. Refer to the mavlink repository https://github.com/mavlink/mavlink and website
    http://qgroundcontrol.org/mavlink/start to learn more on how to add specific message definitions in xml files and
    regenerate the mavlink header files.

    When using hard serial for mavlink, remember to turn off any boot messages (uncomment first entry in uEnv.txt).
    You can still print out jevois messages via Serial-over-USB by setting serout and serlog to USB only. Also
    remember that when using No-USB out videomapping, you either have to specify streamon in initscript or manually
    enter it through an avialable console.


    @author Ali AlSaibie

    @videomapping NONE 0 0 0 YUYV 640 480 30.0 JeVois MAVLinkTest
    @videomapping YUYV 640 480 30.0 YUYV 640 480 30.0 JeVois MAVLinkTest
    @email ali@alsaibie.com
    @address University of Southern California, HNB-07A, 3641 Watt Way, Los Angeles, CA 90089-2520, USA
    @copyright Copyright (C) 2016 by Laurent Itti, iLab and the University of Southern California
    @mainurl http://jevois.org
    @supporturl http://jevois.org/doc
    @otherurl http://iLab.usc.edu
    @license GPL v3
    @distribution Unrestricted
    @restrictions None
    \ingroup modules */

//! Utility function to get system uptime
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
                                               itsProcessingTimer("Processing", 30, LOG_DEBUG), dummy_increment(0) {
      LDEBUG("Adding MAVLink Instance");
      itsMAVLink = addSubComponent<MAVLink>("MAVLink", &MAVLinkData, MAVLink::Type_t::Hard);
      itsMAVLink->set_instance(itsMAVLink);
      vision_position = Eigen::Vector3f::Zero(); // TODO: use
    }

    //! Virtual destructor for safe inheritance
    virtual ~MAVLinkTest() {}

    //! MAVLink Task
    void MAVLinkTask() {

      auto m = MAVLink::get_instance(MAVLink::Type_t::Hard);

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
      m->receive();

      // Send Parameters
      m->send_parameters(true); // True for force send all

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
        mavlink_msg_vision_position_estimate_send(MAVLINK_COMM_0, get_boot_time_us(),
                                                  vision_position(0), vision_position(1), vision_position(2),
                                                  0, 0, 0);
      }
    }

    // ####################################################################################################
    //! Processing function, no video output
    // ####################################################################################################
    virtual void process(jevois::InputFrame &&inframe) override {

      itsProcessingTimer.start();
      // Wait for next available camera image and do nothing.
      jevois::RawImage inimg = inframe.get();
      inframe.done();


      // Perform mavlink communication tasks
      MAVLinkTask();

      std::string const &mavlinkcputime = itsProcessingTimer.stop();
    }

    // ####################################################################################################
    //! Processing function with video output to USB
    // ####################################################################################################
    virtual void process(jevois::InputFrame &&inframe, jevois::OutputFrame &&outframe) override {

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

      // Perform mavlink communication tasks
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
