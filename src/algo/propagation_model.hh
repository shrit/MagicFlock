#pragma once

# include <cmath>

# include "../global.hh"
# include "../log.hh"
# include "../math_tools.hh"

constexpr double PI           = 3.14159265358979323846;  /* pi */
constexpr double speed_of_light = 299792458;

template<class simulator_t,
	 class T>
class Propagation_model {

public:

  Propagation_model(std::shared_ptr<simulator_t> sim_interface);
 
  T dbm_to_watt(T value);
  
  lt::triangle<double> distances_2D();
  
  void channel_to_frequency(int channel);
        
  void wave_length();

private:
   
  T convert_watt_to_distance(T receiver_power);
  T convert_dbm_to_distance(T receiver_power);
    
  double frequency_; // in HZ
  double wave_length_;
  double transmitter_power_; // in Watt
  double transmitter_gain_ ;
  double receiver_gain_ ;
  double system_loss_;

  Math_tools mtools_;
  
  std::shared_ptr<simulator_t> sim_interface_;
  
};
  
# include "propagation_mode.hxx"
