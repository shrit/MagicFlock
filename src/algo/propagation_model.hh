#pragma once

# include <cmath>

# include "../log.hh"


constexpr double PI           = 3.14159265358979323846;  /* pi */
constexpr double speed_of_light = 299792458;

class Propagation_model {

public:

  Propagation_model();
  
  template <typename T>
  T convert_to_distance(T receiver_power);

  
  template <typename T>
  T dbm_to_watt(T value);
    
  void channel_to_frequencey(int channel);
        
  void wave_length();

private:

  
  double frequency_; // in HZ
  double wave_length_;
  double transmitter_power_; // in Watt
  double transmitter_gain_ ;
  double receiver_gain_ ;
  double system_loss_;

};
  
# include "propagation_mode.hxx"
