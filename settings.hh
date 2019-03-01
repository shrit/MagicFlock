#ifndef SETTINGS_HH_
#define SETTINGS_HH_

# include <iostream>

# include <boost/program_options.hpp>

# include "global.hh"

namespace lt = local_types;

class Settings {

public:
  
  Settings(int argc, char* argv[]);
  
  
private:
   
  std::string ini_file_;
  std::string connection_url_;
  lt::connection_type socket_;
  
  
};



#endif
