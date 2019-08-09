#ifndef SETTINGS_HH_
#define SETTINGS_HH_

# include <iostream>

# include <boost/program_options.hpp>

# include "global.hh"

namespace lt = local_types;

class Settings {

public:

  Settings(int argc, char* argv[]);

  bool generate() const;
  bool training() const;
  bool testing() const;
private:

  bool generate_;
  bool training_;
  bool testing_;
  std::string ini_file_;
  std::string dataset_file_;

};



#endif
