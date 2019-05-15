

Propagation_model()::Propagation_model()
{}

template <typename T>
T Propagation_model::dbm_to_watt(T value)
{
  rssi_watt_ = std::pow(10, (value - 30)/10 );  
}

void Propagation_model::channel_to_frequency(int channel)
{

  if (channel > 13 and channel < 1) {
    LogErr() << "No valid channel number";
  }
  double ref_channel = 1;
  
  frequency_ = ((channel - ref_channel)*5 +  2412) * 1000000 ;    
  
}

void Propagation_model::wave_length()
{
  wave_length_ = speed_of_light / frequency_;
}

