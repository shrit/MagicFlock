# include "config_ini.hh"

Configs::Configs()
{
  
  mINI::INIFile file("../quad.ini");  
  file.read(ini_);
  
  parse_ini();
  
}


void Configs::parse_ini()
{

  /*Parse Simulation section*/
  
  number_of_quads_ = std::stoi(ini_.get("simulation").get("quad_number"));

  std::string joystick = ini_.get("simulation").get("use_joystick");
  std::istringstream(joystick) >> std::boolalpha >> joystick_;

  
  std::string keyboard = ini_.get("simulation").get("use_keyboard");
  std::istringstream(keyboard) >> std::boolalpha >> keyboard_;
    
  speed_ = std::stoi(ini_.get("simulation").get("speed"));


    /*Parse quadcopter section*/
  
  quad_names_.push_back(ini_.get("quadcopter_01").get("model_name"));
  quad_names_.push_back(ini_.get("quadcopter_02").get("model_name"));
  quad_names_.push_back(ini_.get("quadcopter_03").get("model_name"));

  ports_.push_back(static_cast<lt::port_type>
		   (std::stoi(ini_.get("quadcopter_01").get("port"))));
  
  // ports_.push_back(static_cast<lt::port_type>
  // 		   (std::stoi(ini_.get("quadcopter_02").get("port"))));
  
  // ports_.push_back(static_cast<lt::port_type>
  // 		   (std::stoi(ini_.get("quadcopter_03").get("port")))); 


  /*Parse algorithm section*/
  
  algo_name_ = ini_.get("algorithm").get("name");

  std::string  train = ini_.get("algorithm").get("train");
  std::istringstream(train) >> std::boolalpha >> train_;

  qtable_file_name_ = ini_.get("algorithm").get("qtable");
  
  map_file_name_ = ini_.get("algorithm").get("map_qtable");

  /* Parse subscribe topic section */

  positions_ = ini_.get("subscribe_topics").get("positions");
  
  rssi_1_2_ = ini_.get("subscribe_topics").get("rssi_1_2");
  rssi_1_3_ = ini_.get("subscribe_topics").get("rssi_1_3");
  rssi_2_3_ = ini_.get("subscribe_topics").get("rssi_2_3"); 

  /* Parse publish topic section */

  reset_1_ = ini_.get("publish_topics").get("reset_1");
  reset_2_ = ini_.get("publish_topics").get("reset_2");
  reset_3_ = ini_.get("publish_topics").get("reset_3"); 
  
  
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

std::string Configs::positions() const
{
  return positions_;
}

std::string Configs::rssi_1_2() const
{
  return rssi_1_2_;
}

std::string Configs::rssi_1_3() const
{
  return rssi_1_3_;
}

std::string Configs::rssi_2_3() const
{
  return rssi_2_3_;
}

std::string Configs::reset_1() const
{
  return reset_1_;
}

std::string Configs::reset_2() const
{
  return reset_2_;
}

std::string Configs::reset_3() const
{
  return reset_3_;
}

float Configs::speed() const
{
  return speed_;
}

bool Configs::train() const
{
  return train_;
}
