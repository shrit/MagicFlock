#pragma once

#include <iostream>

template<class T>
struct position_GPS
{
  T latitude_deg;
  T longitude_deg;
  T absolute_altitude_m;
  T relative_altitude_m;
};

/*  Overloading the << operator to print local structs, vectors and classes */
template<typename T>
std::ostream&
operator<<(std::ostream& out, const local_types::position_GPS<T>& p)
{
  out << "[" << p.latitude_deg << ", " << p.longitude_deg << ", "
      << p.absolute_altitude_m << ", " << p.relative_altitude_m << "]";
  return out;
}

