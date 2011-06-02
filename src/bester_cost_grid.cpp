//
//  bester_cost_grid.cpp
//  AU_UAV_ROS
//
//  Created by Tyler Young on 5/31/11.
//

#include <cstdlib>
#include <string>
#include "bester_cost_grid.h"
#include "map_tools.h"

#include <vector>

using namespace std;

#ifndef natural
#define natural unsigned int
#endif

// The following vars are required to define a position
// If it says "const," it's because it remains constant for our
// airfield.
double resolution = 10; // meters per grid square
const double upper_left_longitude = 85.490363;
const double upper_left_latitude = 32.592425;
const double width_in_degrees_longitude = 0.005002;
const double height_in_degrees_latitude = 0.003808;

// for testing only; returns a position whose latitude and longitude are randomized
Position randomized_position()
{
  double longitude = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
  double latitude = ( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;
  
  // cout << "Starting long " << longitude << endl << "Starting lat " << latitude << endl;
  
  return( Position( upper_left_longitude, upper_left_latitude,
                   width_in_degrees_longitude, height_in_degrees_latitude,
                   longitude, latitude, resolution ) );
}

vector< Plane > randomized_planes( int num_planes )
{
  vector< Plane > the_planes;
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
    
    the_planes.push_back( Plane(i, new_pos, destination) );
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
  srand ( 20 );
  
  natural num_planes = 50;
  
  vector< Plane > test_set = randomized_planes( num_planes );
  vector< Plane > test_set1 = randomized_planes( num_planes );
  vector< Plane > test_set2 = randomized_planes( num_planes );
  vector< Plane > test_set3 = randomized_planes( num_planes );
  vector< Plane > test_set4 = randomized_planes( num_planes );
  vector< Plane > test_set5 = randomized_planes( num_planes );
  vector< Plane > test_set6 = randomized_planes( num_planes );
  vector< Plane > test_set7 = randomized_planes( num_planes );
  vector< Plane > test_set8 = randomized_planes( num_planes );
  vector< Plane > test_set9 = randomized_planes( num_planes );
  vector< Plane > test_set10 = randomized_planes( num_planes );
  vector< Plane > test_set11 = randomized_planes( num_planes );
  vector< Plane > test_set12 = randomized_planes( num_planes );
  vector< Plane > test_set13 = randomized_planes( num_planes );
  vector< Plane > test_set14 = randomized_planes( num_planes );
  vector< Plane > test_set15 = randomized_planes( num_planes );
  vector< Plane > test_set16 = randomized_planes( num_planes );
  vector< Plane > test_set17 = randomized_planes( num_planes );
  vector< Plane > test_set18 = randomized_planes( num_planes );
  vector< Plane > test_set19 = randomized_planes( num_planes );
  
  double width_of_field = 
  map_tools::calculate_distance_between_points( upper_left_latitude, upper_left_longitude,
                                               upper_left_latitude, upper_left_longitude +
                                               width_in_degrees_longitude,
                                               "meters");
  double height_of_field = 
  map_tools::calculate_distance_between_points( upper_left_latitude, upper_left_longitude,
                                               upper_left_latitude - 
                                               height_in_degrees_latitude,
                                               upper_left_longitude,
                                               "meters");
  
  bester_cost_grid bc( test_set, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc1( test_set1, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc2( test_set2, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc3( test_set3, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc4( test_set4, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc5( test_set5, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc6( test_set6, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc7( test_set7, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc8( test_set8, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc9( test_set9, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc10( test_set10, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc11( test_set11, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc12( test_set12, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc13( test_set13, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc14( test_set14, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc15( test_set15, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc16( test_set16, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc17( test_set17, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc18( test_set18, width_of_field, height_of_field, resolution, 2 );
  bester_cost_grid bc19( test_set19, width_of_field, height_of_field, resolution, 2 );

  
  //bc.dump( 0 );
  
  //bc.dump( 1 );
  
  //bc.dump( 5 );
  
  cout << endl << endl << "Success!" << endl;
}