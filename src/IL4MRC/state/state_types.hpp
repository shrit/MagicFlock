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

class AntennaDists
{
public:
  double dist_antenna_1;
  double dist_antenna_2;
};

class CrapyData
{
  
};
