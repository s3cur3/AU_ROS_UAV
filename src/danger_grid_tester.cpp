//
// danger_grid_tester.cpp
// AU_UAV_ROS
//
// Created by Tyler Young on 5/23/11.
//
// A simple tester for the danger_grid class

#include <cstdlib>
#include <string>
#include "danger_grid.h"
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

// Test the danger_grid class by:
// - initializing a d_g with a set of aircraft and some map information
// - accessing the danger ratings of a square using the overloaded ( ) operator
// Note that the all-important methods fill_danger_space() and calculate_future_pos()
// will be called automatically when initializing.
int main()
{
  srand ( 9 );
  
  vector< Plane > test_set;
  
  natural num_planes = 12;
  
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
    
    test_set.push_back( Plane(i, new_pos, destination) );
    //cout << " Updating plane " << i << "'s position." << endl;
    //test_set[i].update( new_pos, destination, bearing, speed );
  }
  
  cout << endl << "Plane 1's initial position is (" << test_set[0].getLocation().getX() << ", " <<
  test_set[0].getLocation().getY() << ")" << endl;
  cout << "Plane 2's initial position is (" << test_set[1].getLocation().getX() << ", " <<
  test_set[1].getLocation().getY() << ")" << endl;
  cout << "Plane 3's initial position is (" << test_set[2].getLocation().getX() << ", " <<
  test_set[2].getLocation().getY() << ")" << endl;
  
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
  
  /*cout << "width_of_field: " << width_of_field << endl;
   cout << " true width around 469 m" << endl;
   cout << "height_of_field: " << height_of_field << endl;
   cout << " true height around 422 m" << endl;*/
  
  danger_grid dg( test_set, width_of_field, height_of_field, resolution );
  cout << "The danger rating at (3, 3, 3) (x, y, t) is: " << dg( 3, 3, 3 ) << endl;
  cout << "The danger rating at (4, 5, 6) (x, y, t) is: " << dg( 4, 5, 6 ) << endl;
  cout << "The danger rating at (10, 20, 9) (x, y, t) is: " << dg( 10, 20, 9 ) << endl;
  cout << "The danger rating at (8, 13, 15) (x, y, t) is: " << dg( 8, 13, 15 ) << endl;
  cout << "The danger rating at (30, 17, 24) (x, y, t) is: " << dg( 30, 17, 14 ) << endl;
  cout << "The danger rating at (30, 17, -2) (x, y, t) is: " << dg( 30, 17, -2 ) << endl;
  
  for( int i = -2; i <= 15; ++i )
  {
    cout << endl << " AT TIME " << i << ": " << endl;
    dg.dump( i );
  }
  
  cout << endl << " All times overlayed: " << endl;
  dg.dump( 10000 );
  
  /* cout << endl << " AT TIME -2: " << endl;
   dg.dump( -2 );
   cout << endl << " AT TIME -1: " << endl;
   dg.dump( -1 );
   cout << endl << " AT TIME 0: " << endl;
   dg.dump( 0 );
   cout << endl << " AT TIME 1: " << endl;
   dg.dump( 1 );
   cout << endl << " AT TIME 10: " << endl;
   dg.dump( 10 );*/
  
  cout << endl << endl << "Success!" << endl;
}