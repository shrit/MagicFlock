#pragma once

# include <cmath>

# include "../log.hh"


constexpr double PI           = 3.14159265358979323846;  /* pi */
constexpr double speed_of_light = 299792458;

class Propagation_model {

public:

  Propagation_model();
  
  template <typename T>
  lt::triangle<T> convert_to_distance(T value);

  
  template <typename T>
  T dbm_to_watt(T value);
    
  void channel_to_frequencey(int channel);
        
  void wave_length();

private:

  
  double frequency_; // in HZ
  double rssi_watt_; // in Watt
  double wave_length_;
  
};
  
# include "propagation_mode.hxx"
