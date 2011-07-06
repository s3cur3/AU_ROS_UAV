//
//  Astar_stress_tester.cpp
//  au_uav_ros
//
//  Created by Tyler Young on 7/3/11.
//

#define Testing
#define Outputting
#define DEBUG // for now, this should ALWAYS be defined for the sake of rigor

#define TYLERS_PC

//standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <map>

#include "a_star/Plane_fixed.h"
#include "a_star/best_cost_straight_lines.h"
#include "a_star/astar_sparse0.cpp"
#include "a_star/Position.h"

#ifdef DEBUG
#include "output_helpers.h"
#endif

#ifndef EPSILON
#define EPSILON 0.00000001
#endif

using namespace std;

//required global values
double upperLeftLon;
double upperLeftLat;
double lonWidth;
double latWidth;
double res;
double fieldWidth;
double fieldHeight;

//where the planes are
std::map<int,Plane> planes;

#ifdef COLLISIONTESTING
vector< point > plane_locs;
#endif

//the number of files written per plane to teledata
int output_indices[12];
int planesmade;

//keeps count of the number of services requested
int the_count;

std::map< int, int > last_callback_updated;
std::map< int, double > prev_dist;
std::map< int, bool > needs_a_push;

void makeField();

/**
 * Checks a plane's location against all others to see if there is a collision
 * @param id_to_check The ID of the plane that was just updated; all other planes'
 * locations will be checked against this one
 * @return TRUE if a collision occurred, FALSE if it's business as usual
 */
bool collision_occurred( int id_to_check );

// for testing only; returns a position whose latitude and longitude are randomized
Position randomized_position()
{
  // for the Auburn test field
  //  double longitude = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
  //  double latitude = -( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;
  
  // For the final 1km test field
  double longitude = upperLeftLon + ( (double)( rand() % 1282999 ) / 1000000000 );
  double latitude = upperLeftLat - ( (double)( rand() % 23009 ) / 1000000000 );
  
  // cout << "Starting long " << longitude << endl << "Starting lat " << latitude << endl;
  
  return( Position( upperLeftLon, upperLeftLat,
                   lonWidth, latWidth,
                   longitude, latitude, res ) );
}

int main()
{
  makeField();
  
  time_t seconds;
  seconds = time(NULL);
  cout << "Start time: " << seconds<<endl;
  while( time(NULL) - seconds < 300 )
  {
    
  the_count++;
  
  int planeId=rand() % 32;
  
  // make the positions
  //update the plane's current location
  Position current = randomized_position();
  
  // If it's a new plane . . .
  // . . . give it an initial destination obtained through the telemetry update  
  if( planes.find(planeId) == planes.end() )
  {
    
    Position next = randomized_position();

    planes[ planeId ] = Plane( planeId, current, next );
    
#ifdef DEBUG
    assert( (int)next.getLat() != 0 );
    assert( (int)current.getLat() != 0 );
#endif
  }
  else // it's an old plane, so . . .
  {
    // we need only update it's current location with the info from the telemetry
    // update
    planes[ planeId ].update_current( current );
  }
  
  // Make a note that this plane got a callback
  last_callback_updated[ planeId ] = the_count;
  
  double dist_from_goal = 100;
  
  Position dummy = randomized_position();
  // If the plane is in a loop, give it a fake "break-out" goal
  if( dist_from_goal < 45 && prev_dist[ planeId ] < dist_from_goal )
  {
    planes[ planeId ].setFinalDestination( dummy.getLat(), dummy.getLon() );
  }
  else // plane isn't in a loop
  {
    // Set the plane's final destination based on what the goal service told us
    planes[planeId].setFinalDestination( dummy.getLon(), dummy.getLat() );
  }
  
  // grab stuff for A*, if thats his real name
  int startx = planes[planeId].getLocation().getX();
  int starty = planes[planeId].getLocation().getY();
  int endx = planes[planeId].getFinalDestination().getX();
  int endy = planes[planeId].getFinalDestination().getY();
  map_tools::bearing_t bearingNamed = planes[planeId].get_named_bearing();
  
  // Begin A*ing
  best_cost bc = best_cost( &planes, fieldWidth, fieldHeight, res, planeId);
  
  point a_Star;
  a_Star = astar_point( &bc, startx, starty, endx, endy, planeId, bearingNamed, &planes );
  
  // Prior to artificially updating the plane's location, make a note of where 
  // the plane is
  prev_dist[ planeId ] = 
  map_tools::calculate_distance_between_points( dummy.getLat(), dummy.getLon(), 
                                               current.getLat(), current.getLon(),
                                               "meters" );
  
  //where to go next
  Position aStar(upperLeftLon,upperLeftLat,lonWidth,latWidth,a_Star.x,a_Star.y,res);
  
#ifdef DEBUG
  assert( (int)aStar.getLat() != 0 );
#endif
  
  
  //                         Update the plane object                             //
  Position next = Position( upperLeftLon, upperLeftLat, lonWidth, latWidth,
                           aStar.getLon(), aStar.getLat(), res);
  planes[planeId].update_intermediate_wp( next );  
  
  }
  cout << "End of loop " << endl << endl;
    
}


void makeField()
{
  ifstream the_file( "/Volumes/DATA/Dropbox/school/Auburn/Code/AU_UAV_stack/AU_UAV_ROS/field_1000.txt" );
  if ( the_file.is_open() )
  {
    the_file >> upperLeftLon;
    the_file >> upperLeftLat;
    the_file >> lonWidth;
    the_file >> latWidth;
    the_file >> res;
    the_file.close();
    
    fieldWidth= /* in meters */
    map_tools::calculate_distance_between_points( upperLeftLat, upperLeftLon,
                                                 upperLeftLat, upperLeftLon + lonWidth,
                                                 "meters");
    
    fieldHeight = 
    map_tools::calculate_distance_between_points( upperLeftLat, upperLeftLon,
                                                 upperLeftLat + latWidth, upperLeftLon,
                                                 "meters"); 
  }
  else
    assert( false );
}