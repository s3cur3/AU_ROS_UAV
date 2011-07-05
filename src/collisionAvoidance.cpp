#define Testing
#define Outputting
#define DEBUG // for now, this should ALWAYS be defined for the sake of rigor
//#define GODDAMMIT
//#define COLLISIONTESTING

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
#include "a_star/telemetry_data_out.h"
#include "a_star/Position.h"

#ifdef DEBUG
#include "a_star/output_helpers.h"
#endif

//ROS headers
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

void makeField();

/**
 * Checks a plane's location against all others to see if there is a collision
 * @param id_to_check The ID of the plane that was just updated; all other planes'
 *                    locations will be checked against this one
 * @return TRUE if a collision occurred, FALSE if it's business as usual
 */
bool	collision_occurred( int id_to_check );

void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
  the_count++;
  
  int planeId=msg->planeID;
  double currentLon=msg->currentLongitude;
  double currentLat=msg->currentLatitude;
  double currentAlt=msg->currentAltitude;
  double destLon=msg->destLongitude;
  double destLat=msg->destLatitude;
  double destAlt=msg->destAltitude;
  double gSpeed=msg->groundSpeed;
  double bearing=msg->targetBearing;
  
#ifdef DEBUG
  if( planeId < 0 )
  {
    ROS_ERROR( "This should not have happened. You're using an illegal plane." );
  }
#endif
  
  // initialize the goal service
  AU_UAV_ROS::RequestWaypointInfo goalSrv;
  
  //populate the request for the destination
  goalSrv.request.planeID=planeId;
  goalSrv.request.isAvoidanceWaypoint=false;
  goalSrv.request.positionInQueue=0;
  
  //ask the coordinator nicely
  if(!findGoal.call(goalSrv))
    ROS_ERROR("No goal was returned");
  
#if 	defined(Testing) || defined(Outputting)
  ROS_INFO("The goal of plane %d returned was %f,%f",
           planeId,goalSrv.response.longitude, goalSrv.response.latitude);
  ROS_INFO("The current location of plane %d is %f,%f", 
           planeId, currentLon, currentLat);
#endif
  

  // If this is not a dummy goal
  if( !(goalSrv.response.latitude < -900 && goalSrv.response.longitude < -900) &&
      !( (int)destLon == 0 && (int)destLat == 0 ) )
  {
    // The following is used to output telemetry files for visualization with X-Plane
    /*
    //print out the tele data for use with x-plane
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
      unsigned int timestamp = clock() / (CLOCKS_PER_SEC / 1000); // should give ms; be sure to include time.h
      
      //  create files for x-plane
      tele << planeId <<"\n"<< double_to_string(currentLat) << "\n" << double_to_string(currentLon);
      tele << "\n" << currentAlt << "\n" <<	gSpeed << "\n" << double_to_string(destLat);
      tele << "\n" << double_to_string(destLon) << "\n" << double_to_string(bearing) << "\n";
      tele << timestamp << "\n";
      
      tele.close();
    }
     */
#ifdef DEBUG
    assert( (int)currentLon != 0 && (int)currentLat != 0 );
    assert( (int)destLon != 0 && (int)destLat != 0 );
#endif
        
    // make the positions
    //update the plane's current location
    Position current = Position(upperLeftLon,upperLeftLat,lonWidth,latWidth,currentLon,currentLat,res);
    
#ifdef COLLISIONTESTING
    // if this plane ID is not in the plane_locs vector, increase the size of the
    // vector to accomodate it
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
    
    // Don't die, just print
    if( collision_occurred( planeId ) )
      cout << " You've made a mistake, Sir." << endl;
#endif
    
    // If it's a new plane . . .
    if( planes.find(planeId) == planes.end() )
    {
      // . . . give it an initial destination obtained through the telemetry update  
      Position next = Position(upperLeftLon,upperLeftLat,lonWidth,latWidth,destLon,destLat,res);
      
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
    
    double dist_from_goal =
      map_tools::calculate_distance_between_points( goalSrv.response.latitude, goalSrv.response.longitude, 
                                                    current.getLat(), current.getLon(),
                                                    "meters" );
    
    // If the plane is in a loop, give it a fake "break-out" goal
    if( dist_from_goal < 45 && prev_dist[ planeId ] < dist_from_goal )
    {
      // "Break-out" goal is 100 meters in opposite direction of the plane's 
      //  bearing to the real destination
      double break_out_lat, break_out_lon;
      bearing_t bearing_to_break_out = 
        map_tools::reverse_bearing( planes[ planeId ].get_named_bearing_to_dest() );
      map_tools::calculate_point( goalSrv.response.latitude, goalSrv.response.longitude,
                                  50, map_tools::bearing_to_double( bearing_to_break_out ),
                                  break_out_lat, break_out_lon );

      planes[ planeId ].setFinalDestination( break_out_lon, break_out_lat );
#ifdef DEBUG
      ROS_INFO( "Set plane %d's breakout waypoint to %f, %f", planeId, break_out_lon, break_out_lat );
      ROS_INFO( "Plane %d had a bearing of %f ", planeId, planes[ planeId ].getBearing() );
#endif
    }
    else // plane isn't in a loop
    {
      // Set the plane's final destination based on what the goal service told us
      planes[planeId].setFinalDestination(goalSrv.response.longitude, goalSrv.response.latitude);
      
#ifdef DEBUG
      ROS_INFO("You set plane %d's final destination to: %f,%f", planeId, goalSrv.response.longitude,goalSrv.response.latitude);
#endif
    }
    
    // grab stuff for A*, if thats his real name
    int startx = planes[planeId].getLocation().getX();
    int starty = planes[planeId].getLocation().getY();
    int endx = planes[planeId].getFinalDestination().getX();
    int endy = planes[planeId].getFinalDestination().getY();
    map_tools::bearing_t bearingNamed = planes[planeId].get_named_bearing();
    
    // Begin A*ing
    best_cost bc = best_cost( &planes, fieldWidth, fieldHeight, res, planeId);
    
    /*
    if( planeId == 4 || planeId == 5 )
    {
      bc.dump( 0 );
      bc.dump( 2 );
      bc.dump( 4 );
    }*/
      
    point a_Star;
    a_Star = astar_point( &bc, startx, starty, endx, endy, planeId, bearingNamed, &planes );

#ifdef DEBUG
    ROS_ERROR("\033[22;31m Still here, doing my dynamic, sparse, A* thing on plane %d . . .", planeId );
    cout << "Told A* plane has a bearing of " << bearing_to_string( bearingNamed ) << endl;
#endif
        
#if defined(Outputting) || defined(GODDAMMIT)
    ROS_INFO("A* says for plane %d to go here: \033[22;32m\nx: %d\ny: %d\nFrom here:\nx: %d\ny: %d",
             planeId, a_Star.x, a_Star.y, startx, starty);
    
#endif
    
    // Make a note of where the plane is for the sake of checking if it's in a loop
    prev_dist[ planeId ] = 
      map_tools::calculate_distance_between_points( goalSrv.response.latitude, goalSrv.response.longitude, 
                                                    current.getLat(), current.getLon(),
                                                    "meters" );
    
    //where to go next
    Position aStar(upperLeftLon,upperLeftLat,lonWidth,latWidth,a_Star.x,a_Star.y,res);
    
#ifdef DEBUG
    assert( (int)aStar.getLat() != 0 );
#endif
    
    //        Set up the service to command a plane to go to a location           //
    AU_UAV_ROS::GoToWaypoint srv;
    
    srv.request.planeID = planeId;
    srv.request.longitude = aStar.getLon();
    srv.request.latitude = aStar.getLat();
    srv.request.altitude = goalSrv.response.altitude;  
    
    //these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue(if there was a new plane)
    srv.request.isAvoidanceManeuver = true;
    srv.request.isNewQueue = true;
    
    if(!client.call(srv))
      ROS_ERROR("Service failed to go through!");
    
    //                         Update the plane object                             //
    Position next = Position( upperLeftLon, upperLeftLat, lonWidth, latWidth,
                             aStar.getLon(), aStar.getLat(), res);
    planes[planeId].update_intermediate_wp( next );
    
    //                        Garbage collection                                   //
    vector< int > delete_these_keys;
    for( map< int, Plane >::iterator crnt_plane = planes.begin(); crnt_plane != planes.end();
         ++crnt_plane )
    {      
      int crnt_id = (*crnt_plane).second.getId();
      
      if( last_callback_updated[ crnt_id ] < (the_count - (3 * planes.size()) ) &&
          the_count > 30 && crnt_id >= 0 )
      {
        // Current plane hasn't been updated in the last 2 rounds of callbacks.
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
    //                        End garbage collection                               //
    
  } // end if this is an okay goal
  else // Bad goals are normal in the first couple rounds of updates
  {
    ROS_ERROR("Either you had a dest. lat-lon of (0, 0) or you had a bad goal returned: %f, %f",
              goalSrv.response.latitude,goalSrv.response.longitude);
  }
  
  cout << "End of callback " << endl << endl;
}

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

	//needed for ROS to wait for callbacks
	makeField();
    
  ros::spin();
  
//  ros::MultiThreadedSpinner spinner(4); // Use 4 threads
//  spinner.spin();

	return 0;
}

#ifdef COLLISIONTESTING
bool	collision_occurred( int id_to_check )
{
  for( unsigned int crnt_id = 0; crnt_id < plane_locs.size(); crnt_id++ )
  {
    if( crnt_id != id_to_check && /* this is a different plane */
        plane_locs[ crnt_id ].x == plane_locs[ id_to_check ].x && /* with the same x pos */
        plane_locs[ crnt_id ].y == plane_locs[ id_to_check ].y && /* and the same y pos */
        plane_locs[ crnt_id ].t != -1 /* and it HAS been initialized */)
    {
      cout << "   Collision occurred between planes " << crnt_id << " and "
        << id_to_check << " at (" << plane_locs[ crnt_id ].x << ", " <<
        plane_locs[ crnt_id ].y << ")" << endl;
      cout << "              (Note: presumably plane " << id_to_check
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
    cout << "                              upper left latitude: " << upperLeftLat << endl;
    cout << "                           width in deg longitude: " << lonWidth << endl;
    cout << "                            width in deg latitude: " << latWidth << endl;
    cout << "                                  width in meters: " << fieldWidth << endl;
    cout << "                                 height in meters: " << fieldHeight << endl;
    */
  }
  
	else
	{
		ROS_ERROR("Cannot open field data");
	}
}
