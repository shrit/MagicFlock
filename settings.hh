#ifndef SETTINGS_HH_
#define SETTINGS_HH_


# include <iostream>

# include <boost/program_options.hpp>
# include <boost/property_tree/ptree.hpp>
# include <boost/property_tree/json_parser.hpp>

# include "global.hh"

/**
 * Settings for the controller code
 * it parses the command line to run the program 
 * TODO: make the configuration readable from a file
*/


namespace lt = local_types;

class Settings {

public:
  
  Settings(int argc, char* argv[]);
  


  
private:
  
  
  /// Drone on which to connect to.
  lt::ip_type	drone_ip_;

  
  // number of udp ports to connects to quads


  std::string connection_url_;
  lt::connection_type socket_;
  
  
};




#endif
