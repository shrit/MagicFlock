# include "config_ini.hh"

Configs::Configs()
{
  
  mINI::INIFile file("quad.ini");  
  file.read(ini_);


  parse_ini();
  
}


void Configs::parse_ini()
{

  /*Parse Simulation section*/
  
  number_of_quads_ = std::stoi(ini.get("simulation").get("quad_number"));

  std::string joystick = ini.get("simulation").get("use_joystick");
  istringstream(joystick) >> std::boolalpha >> joystick_;

  
  std::string keyboard = ini.get("simulation").get("use_keyboard");
  istringstream(keyboard) >> std::boolalpha >> keybooard_;
    
  speed_ = std::stoi(ini.get("simulation").get("speed"));


    /*Parse quadcopter section*/
  
  quad_names_.push_back(ini.get("quadcopter_01").get("model_name"));
  quad_names_.push_back(ini.get("quadcopter_02").get("model_name"));
  quad_names_.push_back(ini.get("quadcopter_03").get("model_name"));

  ports.push_back(); // verify convert to unsigned integer
  ports.push_back(); // verify convert to unsigned integer
  ports.push_back(); // verify convert to unsigned integer


  /*Parse algorithm section*/
  
  algo_name = ini.get("algorithm").get("name");

  std::string  train = ini.get("algorithm").get("train");
  istringstream(train) >> std::boolalpha >> train_;

  qtable_file_name_ = ini.get("algorithm").get("qtable");
  
  map_file_name_ = ini.get("algorithm").get("map_qtable");
  
  
}

int Configs::quad_number() const
{
  return number_of_quads_;
}

std::vector<lt::port_type> Configs::quads_ports() const
{
  return ports_;
}

std::string Configs::qtable_file_name() const
{
  return qtable_file_name_;
}

std::string Configs::map_file_name() const
{
  return map_file_name_;
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
