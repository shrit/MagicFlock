#pragma once

# include "propagation_model.hh"

template <class simulator_t, class T>
Propagation_model<simulator_t, T>::
Propagation_model(std::shared_ptr<simulator_t> sim_interface)
  :sim_interface_(std::move(sim_interface))
{
  channel_to_frequency();
  wave_length();  
}

template <class simulator_t,
	  class T>
T Propagation_model<simulator_t, T>::
convert_to_distance(T receiver_power)
{
  /*  ensure initialization  */
  receiver_power = dbm_to_watt(receiver_power);
  T distance = wave_length_ /(4*PI) *
    std::sqrt(transmitter_power_ * transmitter_gain_ * receiver_gain_
	      / receiver_power * system_loss_);
  /*  Now we have the 3D disance between two points, So far we need to
   convert it to 2D since our nn model take on account only 2D
   distances */
  double total_heights = sim_interface_->positions().f2.z;
  double original_heights = sim_interface_->positions().f1.z;

  double diff_heights = total_heights - original_heights;
  
  distance =  mtools_.pythagore_leg(diff_heights, distance);
  
  return distance;  
}

template <class simulator_t, class T>
lt::triangle<double> Propagation_model<simulator_t, T>::
distances_2D()
{
  lt::triangle<double> dist;
  dist.f1 = convert_to_distance (sim_interface_.rssi().f1) ;
  dist.f2 = convert_to_distance (sim_interface_.rssi().f2) ;
  dist.f3 = convert_to_distance (sim_interface_.rssi().f3) ;
  return dist;
}

template <class simulator_t,
	  class T>
T Propagation_model<simulator_t, T>::dbm_to_watt(T value)
{
  double rssi_watt;
  return rssi_watt = std::pow(10, (value - 30)/10 );  
}

template <class simulator_t, class T>
void Propagation_model<simulator_t, T>::channel_to_frequency(int channel)
{
  if (channel > 13 and channel < 1) {
    LogErr() << "No valid channel number";
  }
  double ref_channel = 1;
  
  frequency_ = ((channel - ref_channel)*5 +  2412) * 1000000 ;      
}

template <class simulator_t, class T>
void Propagation_model<simulator_t, T>::wave_length()
{
  wave_length_ = speed_of_light / frequency_;
}

