#pragma once

#include <cmath>

#include <MagicFlock/util/logger.hpp>

using namespace ILMR;

constexpr double PI = 3.14159265358979323846; /* pi */
constexpr double speed_of_light = 299792458;

template<class rssi_t>
class Propagation_model
{

public:
  Propagation_model(std::vector<double> position);

  double dbm_to_watt(double value);
  double Hz_to_Ghz(double value);

  std::vector<double> estimated_distances_3D();

  void channel_to_frequency(int channel);
  void wave_length();

private:
  double friis_convert_watt_to_distance(double receiver_power);
  double friis_convert_dbm_to_distance(double receiver_power);
  double ITU_convert_dbm_to_distance(double receiver_power);

  double frequency_; // in HZ
  double wave_length_;
  double transmitter_power_; // in Watt
  double transmitter_gain_;
  double receiver_gain_;
  double system_loss_;

  double alpha_;
  double beta_;
  double gamma_;
  double sigma_;

  std::shared_ptr<rssi_t> rssi_interface_;
};

#include "propagation_model_impl.hpp"
