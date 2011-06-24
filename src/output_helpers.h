//
//  output_helpers.h
//  AU_UAV_ROS
//
//  Created by Tyler Young on 6/23/11.
//

#include <iomanip>


#ifndef TO_STRING
#define TO_STRING
template <class T>
inline std::string to_string( const T& t )
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}
#endif

#ifndef DOUBLE_TO_STRING
#define DOUBLE_TO_STRING
std::string double_to_string(const double & d)
{
  std::stringstream ss;
  ss << std::setprecision( std::numeric_limits<double>::digits10+2);
  ss << d;
  return ss.str();
}
#endif

