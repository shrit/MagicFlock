/**
 * This file contains state types that can be used
 * as state for these cases.
 */

class ReceivedSignal
{
public:
  unsigned int id;
  double antenna_1; /* Value measured on antenna 1*/
  double antenna_2; /* Value measured on antenna 2*/
  std::string name; /* Name of the transmitter quadrotor */
};

class FullWiFi
{
public:
  unsigned int id;
  double antenna;   /* Value measured on antenna 1 */
  double azimuth;
  double azimuth_0; /* angle on the x,y surface */
  double azimuth_1; /* angle on the x,y surface */
  double azimuth_2; /* angle on the x,y surface */
  double elevation; /* angle on the y,z surface */
};

class AntennaDists
{
public:
  unsigned int id;
  double dist_antenna_1;
  double dist_antenna_2;
};

class LaserScan
{
public:
  // std::string frame;
  // double angle_min;
  // double angle_max;
  // double angle_step;
  // double range_min;
  // double range_max;
  // std::uint32 count;
  // double vertical_angle_min;
  // double vertical_angle_max;
  // double vertical_angle_step;
  // std::uint32 vertical_count;
  std::vector<double> ranges;
  std::vector<double> intensities;
};

class CrapyData
{};
