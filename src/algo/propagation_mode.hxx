#pragma once

# include "propagation_model.hh"

template <class simulator_t, class T>
Propagation_model<simulator_t, T>::
Propagation_model(std::shared_ptr<simulator_t> sim_interface):
  transmitter_power_(10),
  transmitter_gain_(0),
  receiver_gain_(0),
  system_loss_(0),  
  sim_interface_(std::move(sim_interface))
{
  channel_to_frequency(1);
  wave_length();  
}

/*  Do not use, something is missing in the watt version */
template <class simulator_t,
	  class T>
T Propagation_model<simulator_t, T>::
convert_watt_to_distance(T receiver_power)
{
  /*  maybe it needs to be rooted... */
  /*  ensure initialization  */
  /*  Convert everything to watt before calculation */
  receiver_power = dbm_to_watt(receiver_power);
  
  LogInfo () << "R Power in Watt: " << receiver_power;
  LogInfo () << "Wave length: " << wave_length_;
  
  T distance = wave_length_ /(4*PI) *
    std::sqrt(transmitter_power_ * transmitter_gain_ * receiver_gain_
	      / receiver_power * system_loss_);
  /*  Now we have the 3D disance between two points, So far we need to
   convert it to 2D since our nn model take on account only 2D
   distances */
  LogInfo () << "distance 3D: " << distance;
  double total_heights = sim_interface_->positions().f2.z;
  double original_heights = sim_interface_->positions().f1.z;

  double diff_heights = total_heights - original_heights;
  
  distance =  mtools_.pythagore_leg(diff_heights, distance);
  
  return distance;  
}

template <class simulator_t,
	  class T>
T Propagation_model<simulator_t, T>::
convert_dbm_to_distance(T receiver_power)
{

  double value = (receiver_power - transmitter_power_ -
		  transmitter_gain_ - receiver_gain_) / 20 ;
  double D_log = std::log10(wave_length_) - std::log10(4*PI) - value;
  T distance = std::pow(10, D_log);
  LogInfo () << "distance 3D: " << distance;

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
  lt::rssi signal = sim_interface_->rssi();
  LogInfo() << "Signal received in PropModel: "
	    << signal ;
  
  dist.f1 = convert_dbm_to_distance (signal.f1()) ;
  dist.f2 = convert_dbm_to_distance (signal.f2()) ;
  dist.f3 = convert_dbm_to_distance (signal.f3()) ;
  LogInfo() << "Distance from the PropModel: "
	    << dist ;

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

