#ifndef SETTINGS_HH_
#define SETTINGS_HH_

# include <iostream>
# include <boost/program_options.hpp>

# include "global.hh"

namespace lt = local_types;

class Settings {

public:

  Settings(int argc, char* argv[]);

  bool flying() const;
  bool generate() const;
  bool training() const;
  bool testing() const;
  bool classification() const;
  bool regression() const;
  bool trajectory() const;
  std::string dataset() const;
  
private:
  bool flying_{false};
  bool generate_{false};
  bool training_{false};  
  bool testing_{false};
  bool classification_{false};
  bool regression_{false};
  bool trajectory_noise_{false};
  std::string ini_file_;
  std::string dataset_file_;
};

#endif
