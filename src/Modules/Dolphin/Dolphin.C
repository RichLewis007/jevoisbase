// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*! \file */

#include <jevois/Core/Module.H>

#include <jevois/Image/RawImageOps.H>
#include <linux/videodev2.h>


//! Dolphin Module - Template
/*! 

    @author Ali AlSAibie

    @videomapping YUYV 1280 1024 7.5 YUYV 1280 1024 7.5 JeVois Dolphin
    @videomapping YUYV 640 480 30.0 YUYV 640 480 30.0 JeVois Dolphin

    @email ali\@alsaibie.com
    @address Georgia Institute of Technology
    @copyright Copyright (C) 2017 by Ali AlSaibie
    @mainurl 
    @supporturl 
    @otherurl 
    @license GPL v3
    @distribution Unrestricted
    @restrictions None
    \ingroup modules */
class Dolphin : public jevois::Module
{
  public:
    //! Default base class constructor ok
    using jevois::Module::Module;

    //! Virtual destructor for safe inheritance
    virtual ~Dolphin() { }

    //! Processing function
    virtual void process(jevois::InputFrame && inframe, jevois::OutputFrame && outframe) override
    {
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
    }
};

// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(Dolphin);
