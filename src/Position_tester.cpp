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
  
  if( run == 2 )
  {
    int plane = 8;
    srand( plane );
    
    cout << "# Plane " << plane << " start" << endl;
    plane_1_start.setXY( 0, 25 );
    printf( "#  (%d, %d)\n%d\t%f\t%f\t300\n\n", plane_1_start.getX(), plane_1_start.getY(),
           plane, plane_1_start.getLat(), plane_1_start.getLon() );

    cout << "# Plane " << plane << " wps" << endl;
    for( int i = 1; i < 12; i++ )
    {
      int the_x = rand() % 46;
      int the_y = rand() % 42;
      plane_1_start.setXY( the_x, the_y );
      printf( "# WP %d:  (%d, %d)\n%d\t%f\t%f\t300\n", i, the_x, the_y, plane,
              plane_1_start.getLat(), plane_1_start.getLon() );

    }      
  
  }
}











