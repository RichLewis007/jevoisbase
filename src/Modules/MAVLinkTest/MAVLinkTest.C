//! MAVLink Integration Test
/*! 
    @author Ali AlSaibie

    \ingroup modules */

#include <jevois/Core/Module.H>

#include "settings.h"

class MAVLinkTest : public jevois::Module
{
  public:
    //! Default base class constructor ok
    MAVLinkTest(std::string const & instance) : jevois::Module(instance)
    { itsMAVLink = addSubComponent<MAVLinkCOmmunication>("MAVLink communication"); }

    //! Virtual destructor for safe inheritance
    virtual ~MAVLinkTest() { }

    //! Processing function
    virtual void process(jevois::InputFrame && inframe, jevois::OutputFrame && outframe) override
    {

    }

    virtual void sendSerial(std::string const & str) override
    {

    }
    virtual void parseSerial(std::string const & str, std::shared_ptr<UserInterface> s){
      
    }
};

// Allow the module to be loaded as a shared object (.so) file:
JEVOIS_REGISTER_MODULE(MAVLinkTest);
