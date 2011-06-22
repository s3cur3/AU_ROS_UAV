//
//  Position_tester.cpp
//  AU_UAV_ROS
//
//  Created by Tyler Young on 6/21/11.
//

#include <cstdlib>
#include <string>
#include <iostream>
#include "best_cost_with_fields.h"
#include "map_tools.h"
#include "Plane.h"
#include <time.h>
#include <vector>
#include <iomanip>

#define DEBUG


// For Tyler's sandbox
//#include <sstream>
//#include <time.h>
//#include <iostream>
//#include <fstream>


using namespace std;

#ifndef natural
#define natural unsigned int
#endif

// The following vars are required to define a position
// If it says "const," it's because it remains constant for our
// airfield.
double resolution = 10; // meters per grid square
const double upper_left_longitude = -85.490363;
const double upper_left_latitude = 32.592425;
const double width_in_degrees_longitude = 0.005002;
const double height_in_degrees_latitude = -0.003808;

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

int main()
{
  int run = 2;
  
  // The origin
  Position plane_1_start( upper_left_longitude, upper_left_latitude,
                         width_in_degrees_longitude, height_in_degrees_latitude,
                         12, 7, resolution );
  Position other_plane_start( upper_left_longitude, upper_left_latitude,
                             width_in_degrees_longitude, height_in_degrees_latitude,
                             14, 7, resolution );
  Position plane_1_end( upper_left_longitude, upper_left_latitude,
                             width_in_degrees_longitude, height_in_degrees_latitude,
                             10, 10, resolution );
  cout << endl << "Pos is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
  
   
  if( run == 0 )
  {
    cout << "Setting to ( 46, 0 ) via lat-lon" << endl;
    plane_1_start.setLatLon( upper_left_latitude, upper_left_longitude + width_in_degrees_longitude - 0.00001 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 0, 42 ) via lat-lon" << endl;
    plane_1_start.setLatLon( 32.588625, -85.490364 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 46, 42 ) via lat-lon" << endl;
    plane_1_start.setLatLon( upper_left_latitude + height_in_degrees_latitude,
                            upper_left_longitude + width_in_degrees_longitude );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 12, 12 )" << endl;
    plane_1_start.setXY( 12, 12 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    
    cout << "The TRUE test! Setting ( 12, 12 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 19, 19 )" << endl;
    plane_1_start.setXY( 19, 19 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    
    cout << "The TRUE test! Setting ( 19, 19 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 20, 20 )" << endl;
    plane_1_start.setXY( 20, 20 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting ( 20, 20 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 20, 21 )" << endl;
    plane_1_start.setXY( 20, 21 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting ( 20, 21 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 21, 21 )" << endl;
    plane_1_start.setXY( 21, 21 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting ( 21, 21 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 30, 30 )" << endl;
    plane_1_start.setXY( 30, 30 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting ( 30, 30 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 20, 19 )" << endl;
    plane_1_start.setXY( 20, 19 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting (" << plane_1_start.getX() << ", " << plane_1_start.getY() << ")'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 20, 18 )" << endl;
    plane_1_start.setXY( 20, 18 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting (" << plane_1_start.getX() << ", " << plane_1_start.getY() << ")'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 20, 22 )" << endl;
    plane_1_start.setXY( 20, 22 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting (" << plane_1_start.getX() << ", " << plane_1_start.getY() << ")'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 20, 23 )" << endl;
    plane_1_start.setXY( 20, 23 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting (" << plane_1_start.getX() << ", " << plane_1_start.getY() << ")'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 19, 23 )" << endl;
    plane_1_start.setXY( 19, 23 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting (" << plane_1_start.getX() << ", " << plane_1_start.getY() << ")'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 19, 20 )" << endl;
    plane_1_start.setXY( 19, 20 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting (" << plane_1_start.getX() << ", " << plane_1_start.getY() << ")'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 19, 31 )" << endl;
    plane_1_start.setXY( 19, 31 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting (" << plane_1_start.getX() << ", " << plane_1_start.getY() << ")'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 5, 31 )" << endl;
    plane_1_start.setXY( 5, 31 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting (" << plane_1_start.getX() << ", " << plane_1_start.getY() << ")'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 33, 3 )" << endl;
    plane_1_start.setXY( 33, 3 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting (" << plane_1_start.getX() << ", " << plane_1_start.getY() << ")'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 34, 41 )" << endl;
    plane_1_start.setXY( 34, 41 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting (" << plane_1_start.getX() << ", " << plane_1_start.getY() << ")'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 34, 42 )" << endl;
    plane_1_start.setXY( 34, 42 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting (" << plane_1_start.getX() << ", " << plane_1_start.getY() << ")'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 24, 1 )" << endl;
    plane_1_start.setXY( 24, 1 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    cout << "The TRUE test! Setting (" << plane_1_start.getX() << ", " << plane_1_start.getY() << ")'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 46, 42 )" << endl;
    plane_1_start.setXY( 46, 42 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "The TRUE test! Setting ( 46, 42 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 0, 0 )" << endl;
    plane_1_start.setXY( 0, 0 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "The TRUE test! Setting ( 0, 0 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    
    // The farthest corner
    Position plane_1_end( upper_left_longitude, upper_left_latitude,
                         width_in_degrees_longitude, height_in_degrees_latitude,
                         upper_left_longitude + width_in_degrees_longitude - 0.00001,
                         upper_left_latitude + height_in_degrees_latitude + 0.00001, resolution );
    Position other_plane_end( upper_left_longitude, upper_left_latitude,
                             width_in_degrees_longitude, height_in_degrees_latitude,
                             20, 40, resolution );
    
    cout << "Setting to ( 10, 0 )" << endl;
    plane_1_start.setXY( 10, 0 );
    cout << "Pos in BC Tester is " << plane_1_start.getLat() << ", " << plane_1_start.getLon() << endl << endl;

    cout << "Setting to ( 0, 10 )" << endl;
    plane_1_start.setXY( 0, 10 );
    cout << "Pos in BC Tester is " << plane_1_start.getLat() << ", " << plane_1_start.getLon() << endl << endl;

  }
  
  if( run == 1 )
  {
    cout << "Setting to ( 18, 36 )" << endl;
    plane_1_start.setXY( 18, 36 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    
    cout << "The TRUE test! Setting ( 18, 36 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 9, 37 )" << endl;
    plane_1_start.setXY( 9, 37 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    
    cout << "The TRUE test! Setting ( 9, 37 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 42, 9 )" << endl;
    plane_1_start.setXY( 42, 9 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    
    cout << "The TRUE test! Setting ( 42, 9 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 1, 19 )" << endl;
    plane_1_start.setXY( 1, 19 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    
    cout << "The TRUE test! Setting ( 1, 19 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 29, 11 )" << endl;
    plane_1_start.setXY( 29, 11 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    
    cout << "The TRUE test! Setting ( 29, 11 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 12,0 )" << endl;
    plane_1_start.setXY( 12, 0 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    
    cout << "The TRUE test! Setting ( 12, 0 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 13, 1 )" << endl;
    plane_1_start.setXY( 13, 1 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    
    cout << "The TRUE test! Setting ( 13, 1 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 13, 8 )" << endl;
    plane_1_start.setXY( 13, 8 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    
    cout << "The TRUE test! Setting ( 13, 8 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
    
    cout << "Setting to ( 13, 7 )" << endl;
    plane_1_start.setXY( 13, 7 );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl;
    
    cout << "The TRUE test! Setting ( 13, 7 )'s lat/lon to itself." << endl;
    plane_1_start.setLatLon( plane_1_start.getLat(), plane_1_start.getLon() );
    cout << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
  }
  
  if( run == 2 )
  {
    cout << "Plane 0 start" << endl;
    cout << "  Setting to ( 0, 40 ) via xy" << endl;
    plane_1_start.setXY( 0, 40 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
         << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    
    cout << "Plane 0 end" << endl;
    cout << "Setting to ( 40, 0 ) via lat-lon" << endl;
    plane_1_start.setXY( 40, 0 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 1 start" << endl;
    cout << "Setting to ( 42, 20 ) via lat-lon" << endl;
    plane_1_start.setXY( 42, 20 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 1 end" << endl;
    cout << "Setting to ( 40, 20 ) via lat-lon" << endl;
    plane_1_start.setXY( 40, 20 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 2 start" << endl;
    cout << "Setting to ( 38, 22 ) via lat-lon" << endl;
    plane_1_start.setXY( 38, 22 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 2 end" << endl;
    cout << "Setting to ( 20, 40 ) via lat-lon" << endl;
    plane_1_start.setXY( 20, 40 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 4 start" << endl;
    cout << "Setting to ( 40, 40 ) via lat-lon" << endl;
    plane_1_start.setXY( 40, 40 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 4 end" << endl;
    cout << "Setting to ( 0, 0 ) via lat-lon" << endl;
    plane_1_start.setXY( 0, 0 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 8 start" << endl;
    cout << "Setting to ( 1, 40 ) via lat-lon" << endl;
    plane_1_start.setXY( 1, 40 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 8 end" << endl;
    cout << "Setting to ( 39, 0 ) via lat-lon" << endl;
    plane_1_start.setXY( 39, 0 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 9 start" << endl;
    cout << "Setting to ( 41, 1 ) via lat-lon" << endl;
    plane_1_start.setXY( 41, 1 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 9 end" << endl;
    cout << "Setting to ( 0, 39 ) via lat-lon" << endl;
    plane_1_start.setXY( 0, 39 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 10 start" << endl;
    cout << "Setting to ( 10, 15 ) via lat-lon" << endl;
    plane_1_start.setXY( 10, 15 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 10 wp" << endl;
    cout << "Setting to ( 20, 15 ) via lat-lon" << endl;
    plane_1_start.setXY( 20, 15 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 10 wp" << endl;
    cout << "Setting to ( 20, 25 ) via lat-lon" << endl;
    plane_1_start.setXY( 20, 25 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
    
    cout << "Plane 10 wp" << endl;
    cout << "Setting to ( 10, 25 ) via lat-lon" << endl;
    plane_1_start.setXY( 10, 25 );
    cout << "  Pos in BC Tester is " << double_to_string( plane_1_start.getLat() )
    << ", " << double_to_string( plane_1_start.getLon() ) << endl << endl;
  }
}











