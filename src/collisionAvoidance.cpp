//  
//  This is a collision avoidance module for the Auburn University UAV ATTRACT
//  project. It uses Dynamic Sparse A* Search to plan optimal or nearly-optimal, 
//  flyable paths for UAVs.
//
//  This code is a collaborative effort between Thomas Crescenzi of Marist College,
//  Tyler Young, and Andrew Kaizer, both of Truman State University.
//
//  It was created during an 8-week, NSF-funded Research Experience for 
//  Undergraduates (REU) at Auburn University, between May and July of 2011.
//
//  As it stands, the project sadly does not live up to its full potential. Whereas  
//  we confirm that A* can safely and reliably guide up to 15 aircraft in a 500 m 
//  square airspace in simulations without tight turning constraints, enforcing real-
//  world turning capabilities has proven much more time consuming than we anticipated.
//  Given more time, we are confident these results could be obtained in realistic 
//  simulations. As it stands, however, our simulations show 1 to 2 potential 
//  collisions for as few as 8 aircraft in a 500 m by 500 m airspace, with as many 
//  or more for 16 aircraft in the same space.
//
//

// For the sake of rigor, this should ALWAYS be defined when testing
#define DEBUG
#define COLLISIONTESTING
//#define VISUALIZATION_OUTPUT
#define TYLERS_PC // you can include your own path variables for visualization output

// Standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <map>

// Our framework
#include "a_star/Plane_fixed.h"
#include "a_star/best_cost_straight_lines.h"
#include "a_star/astar_sparse0.cpp"
#include "a_star/Position.h"

#ifdef DEBUG
#include "a_star/output_helpers.h"
#endif

// ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"

#ifndef EPSILON
#define EPSILON 0.00000001
#endif

using namespace std;

//ROS service client for calling a service from the coordinator
ros::ServiceClient client;
ros::ServiceClient findGoal;

// Global values for storing this airplane's telemetry information
double upperLeftLon;
double upperLeftLat;
double lonWidth;
double latWidth;
double res;
double fieldWidth;
double fieldHeight;

// Where the planes are stored
std::map< int, Plane> planes;

#ifdef COLLISIONTESTING
// TODO: This should be changed to a std::map to mirror the planes std::map
vector< point > plane_locs;
#endif

#ifdef VISUALIZATION_OUTPUT
// The number of files each plane has written to the visualization telemetry data
int output_indices[12];
int planesmade;
#endif

// Keeps count of the number of callbacks received
int the_count;

// Stores the number of the callback (the_count) each time a plane gets its location
// updated (Used in deciding if planes are dead during the garbage collection step)
std::map< int, int > last_callback_updated;

// The distance between a plane and its goal during its previous telemetry update;
// if this increases within a certain threshold, it indicates the aircraft is 
// stuck in a loop 
std::map< int, double > prev_dist;

// If aircraft is stuck in a loop (as indicated by its spot in prev_dist), this 
// signals that it needs a "break-out" waypoint away from its goal in order to come
// around
std::map< int, bool > needs_a_push;

/**
 * Reads a field.txt file from the /var directory. This stores the upper left point
 * of the airfield (effectively, our origin), as well as the width and length of
 * the field, in degrees latitude and longitude. It also stores the resolution used
 * in our grids.
 * 
 * TODO: Make the field locations and resolution chosen on-the-fly based on the
 *       point returned by a telemetry update and the aircraft nearby.
 */
void makeField();

/**
 * Checks a plane's location against all others to see if there is a collision
 * @param id_to_check The ID of the plane that was just updated; all other planes'
 * locations will be checked against this one
 * @return TRUE if a collision occurred, FALSE if it's business as usual
 */
bool collision_occurred( int id_to_check );

/**
 * This function does all of the interesting things in our framework. Each time a 
 * plane's location is updated, this function is called. It can command an aircraft's
 * autopilot to deviate toward some "avoidance" waypoint, or it can send it along
 * to its destination.
 * 
 * @param msg The telemetry "message" sent in by the ROS framework
 */
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
  the_count++;
  
  // Store the information from the message in local variables
  // (only to limit indirection)
  int planeId = msg->planeID;
  double currentLon = msg->currentLongitude;
  double currentLat = msg->currentLatitude;
  double currentAlt = msg->currentAltitude;
  double destLon = msg->destLongitude;
  double destLat = msg->destLatitude;
  double destAlt = msg->destAltitude;
  double gSpeed = msg->groundSpeed;
  double bearing = msg->targetBearing;
  
#ifdef DEBUG
  if( planeId < 0 )
  {
    ROS_ERROR( "This should not have happened. You're using an illegal plane ID." );
  }
#endif
  
  // Initialize the goal service
  AU_UAV_ROS::RequestWaypointInfo goalSrv;
  
  // Populate the request for the destination information (returned by ROS framework)
  goalSrv.request.planeID = planeId;
  goalSrv.request.isAvoidanceWaypoint = false;
  goalSrv.request.positionInQueue = 0;
  
  // Ask the coordinator nicely
  if( !findGoal.call(goalSrv) )
    ROS_ERROR("No goal was returned");
  
#ifdef DEBUG
  ROS_INFO("The goal of plane %d returned was %f,%f",
           planeId,goalSrv.response.longitude, goalSrv.response.latitude);
  ROS_INFO("The current location of plane %d is %f,%f",
           planeId, currentLon, currentLat);
#endif
  
  // If this is not a dummy update . . .
  if( !(goalSrv.response.latitude < -900 && goalSrv.response.longitude < -900) &&
     !( (int)destLon == 0 && (int)destLat == 0 ) )
  {
    // The following is used to output telemetry files for visualization with X-Plane
#ifdef VISUALIZATION_OUTPUT
     // Print out the tele data for use with x-plane
     ofstream tele;
     string path;
    
#ifdef TYLERS_PC
     path = "/mnt/hgfs/Dropbox/school/Auburn/Code/AU_UAV_stack/AU_UAV_ROS/teledata/";
#endif
    
     path+=to_string(planeId);
     path+="_";
     ++output_indices[planeId];
     path+=to_string( output_indices[planeId] );
     path+=".txt";
     tele.open(path.c_str(), ios::out);
     if( tele.is_open() )
     {
       // should give time in ms; be sure to include time.h
       unsigned int timestamp = clock() / (CLOCKS_PER_SEC / 1000); 
       // Create the file for x-plane
       tele << planeId <<"\n"
         << double_to_string(currentLat) << "\n" 
         << double_to_string(currentLon) << "\n" 
         << currentAlt << "\n" 
         << gSpeed << "\n" 
         << double_to_string(destLat) << "\n" 
         << double_to_string(destLon) << "\n" 
         << double_to_string(bearing) << "\n"
         << timestamp << "\n";
       tele.close();
     }
#endif

#ifdef DEBUG
    assert( (int)currentLon != 0 && (int)currentLat != 0 );
    assert( (int)destLon != 0 && (int)destLat != 0 );
#endif
    
    // Prepare to update the plane's current location
    Position current = Position(upperLeftLon,upperLeftLat,lonWidth,latWidth,currentLon,currentLat,res);
    
#ifdef COLLISIONTESTING
    // If this plane ID is not in the list of plane locations, increase the size of 
    // the vector to accomodate it
    while( planeId >= (int)plane_locs.size() )
    {
      point dummy;
      dummy.x = -1;
      dummy.y = -1;
      dummy.t = -1;
      plane_locs.push_back( dummy );
    }
    
    // store the current plane's current location in the vector
    plane_locs[ planeId ].x = current.getX();
    plane_locs[ planeId ].y = current.getY();
    plane_locs[ planeId ].t = 0;
    
    // If a collision occured, don't die, just print
    if( collision_occurred( planeId ) )
      ROS_ERROR( " You've made a mistake, Sir." );
#endif
    
    // If it's a new plane . . .
    if( planes.find(planeId) == planes.end() )
    {
      // . . . give it an initial destination obtained through the telemetry update
      Position next = Position( upperLeftLon, upperLeftLat, lonWidth, latWidth, 
                                destLon, destLat, res);
      
      // Create and store the plane object
      planes[ planeId ] = Plane( planeId, current, next );
      
      // Create a spot for this plane in the list of planes which need a "push"
      // away from their goal
      needs_a_push[ planeId ] = false;
    }
    else // it's an old plane, so . . .
    {
      // we need only update its current location with info from the telemetry update
      planes[ planeId ].update_current( current );
    }
    
    // Make a note that this plane got a callback
    last_callback_updated[ planeId ] = the_count;
    
    // Set the plane's final destination based on what the goal service told us
    planes[ planeId ].setFinalDestination( goalSrv.response.longitude, 
                                           goalSrv.response.latitude);
#ifdef DEBUG
    ROS_INFO("You set plane %d's final destination to: %f,%f", 
             planeId, goalSrv.response.longitude, goalSrv.response.latitude);
#endif
    
    double dist_from_goal =
      map_tools::calculate_distance_between_points( goalSrv.response.latitude, 
                                                    goalSrv.response.longitude,
                                                    current.getLat(), current.getLon(),
                                                    "meters" );
    
    // If the plane is in a loop, give it a fake "break-out" goal
    if( dist_from_goal < 45 && prev_dist[ planeId ] < dist_from_goal )
    {
      // "Break-out" goal is 75 meters in opposite direction of the plane's
      // bearing to the real destination
      bearing_t bearing_to_break_out =
        map_tools::reverse_bearing( planes[ planeId ].get_named_bearing_to_dest() );
      
      double break_out_lat, break_out_lon;
      map_tools::calculate_point( goalSrv.response.latitude, goalSrv.response.longitude,
                                  75, map_tools::bearing_to_double( bearing_to_break_out ),
                                  break_out_lat, break_out_lon );
      
      // Set the plane's INTERMEDIATE destination to a break-out waypoint
      planes[ planeId ].setDestination( break_out_lon, break_out_lat );
            
      needs_a_push[ planeId ] = true;
      
#ifdef DEBUG
      ROS_INFO( "Set plane %d's breakout waypoint to %f, %f", planeId, break_out_lon, break_out_lat );
      ROS_INFO( "Plane %d had a bearing of %f ", planeId, planes[ planeId ].getBearing() );
#endif
    }
    
    // Grab stuff for A*
    int startx = planes[planeId].getLocation().getX();
    int starty = planes[planeId].getLocation().getY();
    int endx = planes[planeId].getFinalDestination().getX();
    int endy = planes[planeId].getFinalDestination().getY();
    map_tools::bearing_t bearingNamed = planes[ planeId ].get_named_bearing();
    
    if( needs_a_push[ planeId ] ) // have A* solve to its intermediate waypoint, NOT
    {                            // to the final destination as usual
      endx = planes[ planeId ].getDestination().getX();
      endy = planes[ planeId ].getDestination().getY();
      
      needs_a_push[ planeId ] = false;
    }
    
    // Begin A*ing
    best_cost bc = best_cost( &planes, fieldWidth, fieldHeight, res, planeId);
    
    point commanded_pt;
    commanded_pt = astar_point( &bc, startx, starty, endx, endy, planeId,
                                bearingNamed, &planes );
    // Prepare to send the plane to the commanded point
    Position aStar( upperLeftLon, upperLeftLat, lonWidth, latWidth,
                   commanded_pt.x, commanded_pt.y, res);
    
#ifdef DEBUG
    ROS_INFO("A* says for plane %d to go here: \033[22;32m\nx: %d\ny: %d", 
             planeId, commanded_pt.x, commanded_pt.y);
    ROS_INFO("From here:\nx: %d\ny: %d", startx, starty);
#endif
    
    // Set up the service to command a plane to go to a location
    AU_UAV_ROS::GoToWaypoint srv;
    
    srv.request.planeID = planeId;
    srv.request.longitude = aStar.getLon();
    srv.request.latitude = aStar.getLat();
    srv.request.altitude = goalSrv.response.altitude; // Don't change altitude
    
    // The point we're sending is an avoidance maneuver waypoint AND
    // it will clear the avoidance queue
    srv.request.isAvoidanceManeuver = true;
    srv.request.isNewQueue = true;
    
    // Send the command! If the command isn't received, let the user know.
    if( !client.call(srv) )
      ROS_ERROR("Service failed to go through!");
    
    // Update the plane object
    planes[planeId].update_intermediate_wp( aStar );
    
    // Make a note of where the plane is now for the sake of checking next time 
    // if it's in a loop
    prev_dist[ planeId ] = dist_from_goal;
    
    // Garbage collection (delete dead planes)
    vector< int > delete_these_keys;
    for( map< int, Plane >::iterator crnt_plane = planes.begin(); 
        crnt_plane != planes.end(); ++crnt_plane )
    {
      int crnt_id = (*crnt_plane).second.getId();
      
      if( last_callback_updated[ crnt_id ] < (the_count - (3 * planes.size()) ) &&
         the_count > 30 && crnt_id >= 0 )
      {
        // Current plane hasn't been updated in the last 3 rounds of callbacks.
        // This *probably* means it's dead.
        delete_these_keys.push_back( crnt_id );
      }
    }
    for( vector< int >::iterator key = delete_these_keys.begin(); key != delete_these_keys.end();
        ++key )
    {
      planes.erase( (*key) );
      ROS_ERROR(" Deleting plane %d", (*key) );
    }    
  } // end if this is an okay goal
  else // Bad goals are normal in the first couple rounds of updates
  {
    ROS_ERROR("Either you had a dest. lat-lon of (0, 0) or you had a bad goal returned: %f, %f",
              goalSrv.response.latitude, goalSrv.response.longitude);
  }
  
  ROS_INFO ("End of callback \n" );
}

/**
 * This method is only called once by the ROS framework; after initializing all
 * planes, it calls only the telemetry update callback
 */
int main(int argc, char **argv)
{
  the_count = 0;
  
  //standard ROS startup
  ros::init(argc, argv, "collisionAvoidance");
  ros::NodeHandle n;
  
  //setup the required data for our stuff
  
  
  //subscribe to telemetry outputs and create client for the avoid collision service and the goal giving service
  ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
  client = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
  findGoal = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");
  
  //initialize counting
  the_count = 0;
  
  makeField();
  
  //needed for ROS to wait for callbacks
  ros::spin();
  
  // NOTE: We tried both versions of the multithreaded spinning and got 
  //       distastrous results with both. Use at your own risk--ROS is a fickle beast.
  /*ros::MultiThreadedSpinner spinner(4); // Use 4 threads
  spinner.spin(); */
  
  /*ros::AsyncSpinner spinner(4); // Use 4 threads
  spinner.start();
  ros::waitForShutdown();*/
  
  return 0;
}

#ifdef COLLISIONTESTING
bool collision_occurred( int id_to_check )
{
  for( unsigned int crnt_id = 0; crnt_id < plane_locs.size(); crnt_id++ )
  {
    if( crnt_id != id_to_check && /* this is a different plane */
       plane_locs[ crnt_id ].x == plane_locs[ id_to_check ].x && /* with the same x pos */
       plane_locs[ crnt_id ].y == plane_locs[ id_to_check ].y && /* and the same y pos */
       plane_locs[ crnt_id ].t != -1 /* and it HAS been initialized */)
    {
      cout << " Collision occurred between planes " << crnt_id << " and "
      << id_to_check << " at (" << plane_locs[ crnt_id ].x << ", " <<
      plane_locs[ crnt_id ].y << ")" << endl;
      cout << " (Note: presumably plane " << id_to_check
      << " was updated most recently)" << endl;
      return true;
    }
  }
  
  return false;
}
#endif


void makeField()
{
  ifstream the_file( "/var/field.txt" );
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
    
    /*
     cout << endl;
     cout << "You've selected a field with upper left longitude: " << upperLeftLon << endl;
     cout << " upper left latitude: " << upperLeftLat << endl;
     cout << " width in deg longitude: " << lonWidth << endl;
     cout << " width in deg latitude: " << latWidth << endl;
     cout << " width in meters: " << fieldWidth << endl;
     cout << " height in meters: " << fieldHeight << endl;
     */
  }
  else
  {
    ROS_ERROR("Cannot open field data");
  }
}

