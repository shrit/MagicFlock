# include <iostream>
# include <string>


# include "ini.hh"
# include "global.hh"

namespace lt = local_types;

class Configs
{

public:

  Configs();
  
  void parse_ini();
  
  std::string positions() const;
  
  std::string rssi_1_2() const;
  std::string rssi_1_3() const;
  std::string rssi_2_3() const;

  std::string reset_1() const;
  std::string reset_2() const;
  std::string reset_3() const;
  
  std::string qtable_file_name() const;
  std::string map_file_name() const;
  
  int quad_number() const;

  std::vector<lt::port_type> quads_ports() const;
  
  float speed() const;

  bool train() const;
  

private:

  mINI::INIStructure ini_;

  int number_of_quads_;
  
  lt::port_type	port_;

  std::string qtable_file_name_;
  std::string map_file_name_;

  std::string algo_name_;
  
  std::vector<lt::port_type> ports_;
  std::vector<std::string> quad_names_;
  
  float speed_;
  
  bool train_, keyboard_, joystick_;

  std::string positions_;
  
  std::string rssi_1_2_;
  std::string rssi_1_3_;
  std::string rssi_2_3_;

  std::string reset_1_;
  std::string reset_2_;
  std::string reset_3_;
  
  
};

