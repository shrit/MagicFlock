#ifndef GLOBAL_HH_
#define GLOBAL_HH_

# include <iostream>
# include <fstream>
# include <vector>
# include <iterator>

# include "log.hh"


/*  Global name space for the simulation
    It contain a namespace with several
    defintion used in all files
*/
namespace local_types {

  template <class T>
  struct dist3D {
    T d1;
    T d2;
    T d3;
  };
 
  template <class T>
  struct position3D {
    T x;
    T y;
    T z;
  };
  
  template <class T>
  struct position_GPS {
    T latitude_deg ;
    T longitude_deg ;
    T absolute_altitude_m;
    T relative_altitude_m;
  };
  
  template <typename T>
  struct orientation {
    T x;
    T y;
    T z;
    T w;
  };
  
  template<typename T>
  struct triangle
  {
    T f1; /* Distance between the leader and the true follower */
    T f2; /* Distance between the true follower and the fake follower */
    T f3; /* Distance between the leader and the fake follower  */
  };

  template <typename T>
  struct error {
    T x;
    T y;
  };

  template <typename T>
  class rssi {

  public:

    rssi<T>(){}

    const T f1() const
    { return f1_;}
    const T f2() const
    {return f2_;}
    const T f3() const
    { return f3_;}

    void f1(T value)
    { f1_ = value; }
    void f2(T value)
    { f2_ = value; }
    void f3(T value)
    { f3_ = value; }

  private:
    T f1_;
    T f2_;
    T f3_;
  };

  using port_type = uint16_t;
  using connection_type = std::string;
  using topic_name = std::string;
}

/*  Overloading the << operator to print local structs, vectors and classes */
template <typename T>
std::ostream& operator<< (std::ostream& out, const local_types::dist3D<T>& v)
{
  out << "["<< v.d1 <<", " << v.d2 <<", " << v.d3 <<"]";
  return out;
}

/*  Overloading the << operator to print local structs, vectors and classes */
template <typename T>
std::ostream& operator<< (std::ostream& out, const local_types::position3D<T>& p)
{
  out << "["<< p.x <<", " << p.y <<", " << p.z <<"]";
  return out;
}

template <typename T>
std::ostream& operator<< (std::ostream& out, const local_types::position_GPS<T>& p)
{
  out << "["<< p.latitude_deg <<", "
      << p.longitude_deg <<", "
      << p.absolute_altitude_m <<", "
      << p.relative_altitude_m <<"]";
  return out;
}

template <typename T>
std::ostream& operator<< (std::ostream& out, const local_types::rssi<T>& r)
{/*  Print in order  f1, f2 ,f3, according to TF FF TL */
  out << r.f1() <<"," << r.f2() <<"," << r.f3() ;
  return out;
}

template <typename T>
std::ostream& operator<< (std::ostream& out, const local_types::triangle<T>& t)
{/*  Print in order  f1, f2 ,f3, according to TF FF TL */
  out << t.f1 <<"," << t.f2 <<"," << t.f3 ;
  return out;
}

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v)
{
  if ( !v.empty() ) {
    for (size_t i = 0; i< v.size(); i++) {
      out << v.at(i) ;

      if (i != v.size() - 1 )
	out << ",";
    }
  }
  return out;
}

template <typename T>
const std::vector<T>  operator- (const std::vector<T>& v, const std::vector<T>& v1)
{
  std::vector<T> result;
  result.resize (v);
  std::transform(v.begin(), v.end(), v1.begin(), result.begin(), std::minus<T>());
  return result;
}

template <typename T>
const std::vector<T> operator+ (const std::vector<T>& v, const std::vector<T>& v1)
{
  std::vector<T> result;
  result.resize (v);
  std::transform(v.begin(), v.end(), v1.begin(), result.begin(), std::plus<T>());
  return result;
}

template <typename T>
const std::vector<T>  operator* (const std::vector<T>& v, const std::vector<T>& v1)
{
  std::vector<T> result;
  result.resize (v);
  std::transform(v.begin(), v.end(), v1.begin(), result.begin(), std::multiplies<T>());
  return result;
}


template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<std::vector<T>>& v)
{
  if ( !v.empty() ) {
    out << '[';
    for (int i = 0; i < v.size(); i++ )
      std::copy (v.at(i).begin(), v.at(i).end(), std::ostream_iterator<T>(out, ","));
    out << "\b\b]";
  }
  return out;
}

#endif
