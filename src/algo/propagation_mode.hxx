Propagation_model<simulator_t>()::Propagation_model()
  :sim_interface_(std::move(sim_interface))
{}

template <typename simulator_t,
	  typename T>
T Propagation_model<simulator_t>::
convert_to_distance(T receiver_power)
{
  /*  ensure initialization  */
  receiver_power = dbm_to_watt(receiver_power);
  T distance = wave_length_ /(4*PI) *
    std::sqrt(transmitter_power_ * transmitter_gain_ * receiver_gain_
	      / receiver_power * system_loss_);
  return distance;  
}

template <typename simulator_t,
	  typename T>
T Propagation_model<simulator_t>::dbm_to_watt(T value)
{
  return rssi_watt = std::pow(10, (value - 30)/10 );  
}

template <typename simulator_t>
void Propagation_model<simulator_t>::channel_to_frequency(int channel)
{
  if (channel > 13 and channel < 1) {
    LogErr() << "No valid channel number";
  }
  double ref_channel = 1;
  
  frequency_ = ((channel - ref_channel)*5 +  2412) * 1000000 ;      
}

template <typename simulator_t>
void Propagation_model<simulator_t>::wave_length()
{
  wave_length_ = speed_of_light / frequency_;
}

