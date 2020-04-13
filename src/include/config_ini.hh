#ifndef CONFIG_INI_HH_
#define CONFIG_INI_HH_

#include <iostream>
#include <string>

#include "ini.hh"

class Configs
{
  using port_type = uint16_t;

public:
  Configs();
  Configs(std::string file_name);

  void parse_ini();

  std::string positions() const;

  std::string rssi_1_2() const;
  std::string rssi_1_3() const;
  std::string rssi_2_3() const;

  std::string reset_1() const;
  std::string reset_2() const;
  std::string reset_3() const;
  std::string reset_4() const;

  int quad_number() const;
  std::vector<std::string> quad_names() const;
  std::vector<port_type> quads_ports() const;

  float speed() const;

  bool train() const;

  bool just_fly() const;

private:
  mINI::INIStructure ini_;
  int number_of_quads_;
  port_type port_;
  std::vector<port_type> ports_;
  std::vector<std::string> quad_names_;

  float speed_;
  bool keyboard_, joystick_, just_fly_;
  std::string positions_;

  std::string rssi_1_2_;
  std::string rssi_1_3_;
  std::string rssi_2_3_;

  std::string reset_1_;
  std::string reset_2_;
  std::string reset_3_;
  std::string reset_4_;
};

#endif
