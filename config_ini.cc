# include "config_ini.hh"

Configs::Configs()
{
  
  mINI::INIFile file("quad.ini");  
  file.read(ini_);


  parse_ini();
  
}


void Configs::parse_ini()
{

  number_of_quads_ = std::stoi(ini.get("simulation").get("quad_number"));
  
  
}


int Configs::quad_number() const
{
  return number_of_quads_;
}

std::vector<lt::port_type> Configs::quads_ports() const
{
  return ports_;
}

std::string Configs::get_file_name() const
{
  return file_name_;
}

float Configs::speed() const
{
  return speed_;
}

bool Configs::train() const
{
  return train_;
}





 std::cout << number << std::endl;
 
 std::string name = ini.get("quadcopter_01").get("model_name");
 std::cout << name << std::endl;
