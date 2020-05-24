#pragma once

#include "propagation_model.hh"

template<class rssi_t>
Propagation_model<rssi_t>::Propagation_model(
  std::shared_ptr<rssi_t> rssi_interface)
  : transmitter_power_(10)
  , transmitter_gain_(0)
  , receiver_gain_(0)
  , system_loss_(0)
  , alpha_(2.12)
  , beta_(29.3)
  , gamma_(2.11)
  , sigma_(0)
  , rssi_interface_(std::move(rssi_interface))
{
  channel_to_frequency(1);
  wave_length();
}

/*  Do not use, something is missing in the watt version */
template<class rssi_t>
double
Propagation_model<rssi_t, double>::friis_convert_watt_to_distance(
  double receiver_power)
{
  /*  ensure initialization  */
  /*  Convert everything to watt before calculation */
  receiver_power = dbm_to_watt(receiver_power);
  logger::logger_->info("R Power in Watt: {}", receiver_power);
  logger::logger_->info("Wave length: {}", wave_length_);

  double distance = wave_length_ / (4 * PI) *
                    std::sqrt(transmitter_power_ * transmitter_gain_ *
                              receiver_gain_ / receiver_power * system_loss_);
  return distance;
}

template<class rssi_t>
double
Propagation_model<rssi_t, double>::friis_convert_dbm_to_distance(
  double receiver_power)
{
  double value =
    (receiver_power - transmitter_power_ - transmitter_gain_ - receiver_gain_) /
    20;
  double D_log = std::log10(wave_length_) - std::log10(4 * PI) - value;
  double distance = std::pow(10, D_log);
  return distance;
}

template<class rssi_t>
double
Propagation_model<rssi_t>::ITU_convert_dbm_to_distance(double receiver_power)
{
  double frequency = Hz_to_Ghz(frequency_);
  double D_log = (receiver_power - transmitter_power_ - beta -
                  10 * gamma_ * std::log10(frequency) - sigma_) /
                 10 * alpha_;

  double distance = std::pow(10, D_log);
  return distance;
}

template<class rssi_t>
std::vector<double>
Propagation_model<rssi_t>::estimated_distances_3D()
{
  std::vector<double> dist;
  lt::rssi signal = rssi_interface_->rssi();

  dist.push_back(friis_convert_dbm_to_distance(signal.f1()));
  dist.push_back(friis_convert_dbm_to_distance(signal.f2()));
  dist.push_back(friis_convert_dbm_to_distance(signal.f3()));

  return dist;
}

template<class rssi_t>
double
Propagation_model<rssi_t>::dbm_to_watt(double value)
{
  double rssi_watt;
  return rssi_watt = std::pow(10, (value - 30) / 10);
}

template<class rssi_>
double
Propagation_model<rssi_t>::Hz_to_Ghz(double value)
{
  double f_ghz;
  return f_ghz = value / 1000000000;
}

template<class rssi_t>
void
Propagation_model<rssi_t>::channel_to_frequency(int channel)
{
  if (channel > 13 and channel < 1) {
    logger::logger_->error("No valid channel number");
  }
  double ref_channel = 1;
  frequency_ = ((channel - ref_channel) * 5 + 2412) * 1000000;
}

template<class rssi_t>
void
Propagation_model<rssi_t>::wave_length()
{
  wave_length_ = speed_of_light / frequency_;
}
