//
//  best_cost_tester.cpp
//  AU_UAV_ROS
//
//  Created by Tyler Young on 5/31/11.
//

#include <cstdlib>
#include <string>
#include <iostream>
#include "best_cost_straight_lines.h"
#include "map_tools.h"
#include <time.h>
#include <vector>


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

// Constants for the Auburn field
const double upper_left_longitude = -85.490363;
const double upper_left_latitude = 32.592425;
const double width_in_degrees_longitude = 0.005002;
const double height_in_degrees_latitude = -0.003808;

// Constants for the 1km final field
//const double upper_left_longitude = -115.808173;
//const double upper_left_latitude = 37.244956;
//const double width_in_degrees_longitude = 0.011283;
//const double height_in_degrees_latitude = -0.009023;

// for testing only; returns a position whose latitude and longitude are randomized
Position randomized_position()
{
  // for the Auburn test field
  double longitude = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
  double latitude = -( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;
  
  // For the final 1km test field
//  double longitude = upper_left_longitude + ( (double)( rand() % 11282999 ) / 1000000000 );
//  double latitude = upper_left_latitude - ( (double)( rand() % 9023009 ) / 1000000000 );
  
  // cout << "Starting long " << longitude << endl << "Starting lat " << latitude << endl;
  
  return( Position( upper_left_longitude, upper_left_latitude,
                   width_in_degrees_longitude, height_in_degrees_latitude,
                   longitude, latitude, resolution ) );
}

std::map< int, Plane > randomized_planes( natural num_planes )
{
  std::map< int, Plane > the_planes;
  // The following vars are required to set a plane's position
  Position new_pos;
  Position destination;
  double bearing;
  double speed;
  
  // Fill the array with planes, set their positions randomly
  for( natural i = 0; i < num_planes; i++ )
  {
    new_pos = randomized_position();
    //cout << " I.e., starting x = " << new_pos.getX() << " and y = " << new_pos.getY() << endl;
    destination = randomized_position();
    bearing = (double)( rand() % 360 );
    speed = (double)( rand() % 60 );
    speed = 30; // don't want to upset the randomness!
    
    the_planes[ i ] = Plane(i, new_pos, destination);
  }
  return the_planes;
}

// Test the danger_grid class by:
// - initializing a d_g with a set of aircraft and some map information
// - accessing the danger ratings of a square using the overloaded ( ) operator
// Note that the all-important methods fill_danger_space() and calculate_future_pos()
// will be called automatically when initializing.
int main()
{
  // Tyler's sandbox:
/*  stringstream ss( stringstream::out );
  int planeID = 2;
  int index = 500;
  ss << "/mnt/hgfs/Dropbox/school/Auburn/Code/AU_UAV_stack/AU_UAV_ROS/teledata/";
  ss << planeID << "_" << index << ".txt";
  string filename = ss.str();
  cout << filename << endl;*/
  
  time_t seconds;
  seconds = time(NULL);
  cout << "Start time: " << seconds<<endl;
  
  srand ( 20 );
  
  natural num_planes = 20;
  
  std::map< int, Plane > test_set;
  
  // The origin
  Position plane_1_start( upper_left_longitude, upper_left_latitude,
                          width_in_degrees_longitude, height_in_degrees_latitude,
                          0, 0, resolution );
  Position other_plane_start( upper_left_longitude, upper_left_latitude,
                              width_in_degrees_longitude, height_in_degrees_latitude,
                              10, 10, resolution );  
  // The farthest corner
  Position plane_1_end(  upper_left_longitude, upper_left_latitude,
                       width_in_degrees_longitude, height_in_degrees_latitude,
                       0, 40, resolution );
  Position other_plane_end(  upper_left_longitude, upper_left_latitude,
                       width_in_degrees_longitude, height_in_degrees_latitude,
                       20, 40, resolution );
  
  Position plane_2_start(  upper_left_longitude, upper_left_latitude,
                       width_in_degrees_longitude, height_in_degrees_latitude,
                       0, 41, resolution );
  
  test_set[ 0 ] = Plane( 0, other_plane_end, plane_1_end );
  test_set[ 0 ].update( plane_1_start, plane_1_end, 30 );
  
  test_set[ 1 ] = Plane( 1, plane_2_start, other_plane_end );
  
  
  Position plane_3_start(  upper_left_longitude, upper_left_latitude,
                         width_in_degrees_longitude, height_in_degrees_latitude,
                         15, 20, resolution );
  
  plane_3_start.setLatLon(upper_left_latitude, 
                          -85.489251);
  test_set[ 2 ] = Plane( 2, plane_3_start, other_plane_end );
  
  test_set[0].setDestination(5, 5);
  
  
  
  
  
  vector< std::map< int, Plane > > all_planes;
  all_planes.push_back( test_set );
  
  for( int i = 0; i < 39; i++ )
    all_planes.push_back( randomized_planes( num_planes ) );
  
  double width_of_field = 
  map_tools::calculate_distance_between_points( upper_left_latitude, upper_left_longitude,
                                               upper_left_latitude, upper_left_longitude +
                                               width_in_degrees_longitude,
                                               "meters");
  double height_of_field = 
  map_tools::calculate_distance_between_points( upper_left_latitude, upper_left_longitude,
                                               upper_left_latitude + 
                                               height_in_degrees_latitude,
                                               upper_left_longitude,
                                               "meters");
  
  cout << "Here, width is " << width_of_field << " and height is " << height_of_field << endl;
  
//  while( time(NULL) - seconds < 60 )

  vector< best_cost > all_bc_grids;
  for( int i = 0; i < all_planes.size(); i++ )
  {
    all_bc_grids.push_back( best_cost( &all_planes[ i ], width_of_field, height_of_field, resolution, 0 ) );  
  }
  
  all_bc_grids[ 0 ].dump( 0 );
  all_bc_grids[ 0 ].dump_csv( 0, "", "asdf" );
//  
//  bc1.dump( 1 );
//  
//  bc1.dump( 7 );
//  
//  bc1.dump( 20 );

 
  seconds = time(NULL);
  cout << "End time:   " << seconds << endl;
  cout << endl << "Done!" << endl;
  
  cout << endl << endl << "Success!" << endl;
}