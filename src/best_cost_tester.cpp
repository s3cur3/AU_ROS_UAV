//
//  best_cost_tester.cpp
//  AU_UAV_ROS
//
//  Created by Tyler Young on 5/31/11.
//

#include <cstdlib>
#include <string>
#include <iostream>
#include "best_cost_with_fields.h"
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
const double upper_left_longitude = -85.490363;
const double upper_left_latitude = 32.592425;
const double width_in_degrees_longitude = 0.005002;
const double height_in_degrees_latitude = -0.003808;

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

vector< Plane > randomized_planes( natural num_planes )
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
    speed = 30; // don't want to upset the randomness!
    
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
  
  //vector< Plane > test_set = randomized_planes( num_planes );
  
  vector< Plane > test_set;
  
  // The origin
  Position plane_1_start( upper_left_longitude, upper_left_latitude,
                          width_in_degrees_longitude, height_in_degrees_latitude,
                          0, 0, resolution );
  cout << endl << "Pos is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;

  plane_1_start.setXY( 0, 20 );
  cout << endl << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
  
  plane_1_start.setXY( 12, 12 );
  cout << endl << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
  
  plane_1_start.setXY( 20, 20 );
  cout << endl << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
  
  plane_1_start.setXY( 46, 43 );
  cout << endl << "Pos in BC Tester is " << plane_1_start.getX() << ", " << plane_1_start.getY() << endl << endl;
  
  // The farthest corner
  Position plane_1_end(  upper_left_longitude, upper_left_latitude,
                         width_in_degrees_longitude, height_in_degrees_latitude,
                         upper_left_longitude + width_in_degrees_longitude - 0.00001,
                         upper_left_latitude + height_in_degrees_latitude - 0.00001, resolution );
  
  test_set.push_back( Plane( 0, plane_1_start, plane_1_end ) );
  
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
                                               upper_left_latitude + 
                                               height_in_degrees_latitude,
                                               upper_left_longitude,
                                               "meters");
  
  cout << "Here, width is " << width_of_field << " and height is " << height_of_field << endl;
  
  best_cost bc( &test_set, width_of_field, height_of_field, resolution, 0 );
  best_cost bc1( &test_set1, width_of_field, height_of_field, resolution, 2 );
  best_cost bc2( &test_set2, width_of_field, height_of_field, resolution, 2 );
  best_cost bc3( &test_set3, width_of_field, height_of_field, resolution, 2 );
  best_cost bc4( &test_set4, width_of_field, height_of_field, resolution, 2 );
  best_cost bc5( &test_set5, width_of_field, height_of_field, resolution, 2 );
  best_cost bc6( &test_set6, width_of_field, height_of_field, resolution, 2 );
  best_cost bc7( &test_set7, width_of_field, height_of_field, resolution, 2 );
  best_cost bc8( &test_set8, width_of_field, height_of_field, resolution, 2 );
  best_cost bc9( &test_set9, width_of_field, height_of_field, resolution, 2 );
  best_cost bc10( &test_set10, width_of_field, height_of_field, resolution, 2 );
  best_cost bc11( &test_set11, width_of_field, height_of_field, resolution, 2 );
  best_cost bc12( &test_set12, width_of_field, height_of_field, resolution, 2 );
  best_cost bc13( &test_set13, width_of_field, height_of_field, resolution, 2 );
  best_cost bc14( &test_set14, width_of_field, height_of_field, resolution, 2 );
  best_cost bc15( &test_set15, width_of_field, height_of_field, resolution, 2 );
  best_cost bc16( &test_set16, width_of_field, height_of_field, resolution, 2 );
  best_cost bc17( &test_set17, width_of_field, height_of_field, resolution, 2 );
  best_cost bc18( &test_set18, width_of_field, height_of_field, resolution, 2 );
  best_cost bc19( &test_set19, width_of_field, height_of_field, resolution, 2 );

  
  bc1.dump( 0 );
  
  bc1.dump( 1 );
  
  bc1.dump( 2 );
 
  seconds = time(NULL);
  cout << "End time:   " << seconds << endl;
  cout << endl << "Done!" << endl;
  
  cout << endl << endl << "Success!" << endl;
}