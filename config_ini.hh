# include <iostream>
# include <string>


# include "ini.hh"
# include "global.hh"


class Configs
{

public:

  Configs();


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
  
  std::vector<lt::port_type> ports_;
  
  float speed_;
  
  bool train_;
  
  
};

