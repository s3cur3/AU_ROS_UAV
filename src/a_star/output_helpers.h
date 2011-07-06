//
//  output_helpers.h
//  AU_UAV_ROS
//
//  Created by Tyler Young on 6/23/11.
//

#include <iomanip>
#include <sstream>


#ifndef TO_STRING
#define TO_STRING
/**
 * Casts a generic type as a string
 * @param t The thing you want to cast
 * @return a string version of the thing you wanted to cast
 */
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

/**
 * Used for printing doubles with full precision (by default, the standard out
 * cuts things off at around 4 decimal places)
 * @param d The double version of the number
 * @return A string 
 */ 
std::string double_to_string(const double & d)
{
  std::stringstream ss;
  ss << std::setprecision( std::numeric_limits<double>::digits10+2);
  ss << d;
  return ss.str();
}
#endif

