#define Testing
#define Outputting
#define DEBUG // for now, this should ALWAYS be defined for the sake of rigor
//#define GODDAMMIT
#define COLLISIONTESTING

#define TYLERS_PC

//standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <time.h>
#include "Plane_fixed.h"
#include "best_cost_straight_lines.h"
#include "astar_sparse0.cpp"
#include "telemetry_data_out.h"
#include "Position.h"

#ifdef DEBUG
#include "output_helpers.h"
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
// Position crash[5];
// int crashSize=5;
#endif

//the number of files written per plane to teledata
int output_indices[12];
int planesmade;

//keeps count of the number of services requested
int the_count;

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
  if( !(goalSrv.response.latitude + 1000 < EPSILON && goalSrv.response.longitude + 1000 < EPSILON) )
  {
    
#ifdef DEBUG
    //print out the tele data for use with x-plane
    ofstream tele;
    string path;
    
#ifdef TYLERS_PC
    //path = "/mnt/hgfs/Dropbox/school/Auburn/Code/AU_UAV_stack/AU_UAV_ROS/teledata/";
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
      
      //  stuff to print for x-plane
      //int Plane ID
      //double currentLatitude
      //double currentLongitude
      //double currentAltitude
      //double groundSpeed
      //double destLatitude
      //double destLongitude
      //double targetBearing
      //double Timestamp (in milliseconds, doesn't matter where you start)
      
      
      tele << planeId <<"\n"<< double_to_string(currentLat) << "\n" << double_to_string(currentLon);
      tele << "\n" << currentAlt << "\n" <<	gSpeed << "\n" << double_to_string(destLat);
      tele << "\n" << double_to_string(destLon) << "\n" << double_to_string(bearing) << "\n";
      tele << timestamp << "\n";
      
      tele.close();
    }
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
    
    //Don't die, just print
    if( collision_occurred( planeId ) )
      cout << " Ya dun fucked up." << endl;

#endif
    
    // The plane's next location, be it the goal or an avoidance waypoint
    Position next;
    //if next is good . . .
    if(destLat<=upperLeftLat && destLon>=upperLeftLon && destLat>0 && destLon<0)
    {
      next = Position(upperLeftLon,upperLeftLat,lonWidth,latWidth,destLon,destLat,res);
#ifdef DEBUG
      /*
       assert( next.getLon() < -85.484 );
       assert( next.getLon() > -85.4915 );
       assert( next.getLat() > 32.5882 );
       assert( next.getLat() < 32.593 );
       */
#endif
    }
    else
    {
      next = Position(current);
      
#ifdef DEBUG
      /*
       assert( next.getLon() < -85.484 );
       assert( next.getLon() > -85.4915 );
       assert( next.getLat() > 32.5882 );
       assert( next.getLat() < 32.593 );
       */
#endif
      ROS_INFO("\033[22;31m next is current");
    }
    
    // If it's a new plane
    //its a new plane
    if( planes.find(planeId) == planes.end() )
    {
      planes[ planeId ] = Plane( planeId, current, next );
      
#ifdef DEBUG
      cout<<"Current location for plane"<<planeId<<" "<< planes[ planeId ].getLocation().getX()<<
      " "<<planes[ planeId ].getLocation().getY()<<endl;
      printf("   Which is %f, %f \n", planes[ planeId ].getLocation().getLon(), 
             planes[ planeId ].getLocation().getLat() );
      cout<<"Next location for plane"<<planeId<<" "<<planes[ planeId ].getDestination().getX()<<
      " "<<planes[ planeId ].getDestination().getY()<<endl;
#endif
    }
    
    // Set the plane's final destination based on what the goal service told us
    planes[planeId].setFinalDestination(goalSrv.response.longitude, goalSrv.response.latitude);
    
#ifdef DEBUG
    ROS_INFO("You set plane %d's final destination to: %f,%f", planeId, goalSrv.response.longitude,goalSrv.response.latitude);
#endif
    
    
    //update the plane                                                                           ////////////
    planes[planeId].update(current,next,gSpeed);
    
    //grab stuff for A*, if thats his real name
    int startx=planes[planeId].getLocation().getX();
    int starty=planes[planeId].getLocation().getY();
    int endx=planes[planeId].getFinalDestination().getX();
    int endy=planes[planeId].getFinalDestination().getY();
    map_tools::bearing_t bearingNamed = planes[planeId].get_named_bearing();
    
#ifdef DEBUG
    /*
     assert( startx < 51 );
     assert( startx >= 0);
     assert( starty >= 0 );
     assert( starty < 51 );
     assert( endx < 51 );
     assert( endx >= 0);
     assert( endy >= 0 );
     assert( endy < 51 );
     */
#endif
    
#ifdef DEBUG
    if( planeId == 10 )
    {
      ROS_INFO("for plane %d\n the startx:%d\n the starty:%d\n the endx:%d\n the endy:%d\n end lat: %f \n end lon %f",
               planeId,startx,starty,endx,endy, planes[planeId].getFinalDestination().getLat(), 
               planes[planeId].getFinalDestination().getLon() );
    }
#endif
    
    // Begin A*ing
    point a_Star;
    vector<Plane> planes2;
    for( int i=0; i<planes.size(); i++ )
      planes2.push_back(planes[i]);
    
    best_cost bc = best_cost(&planes2,fieldWidth,fieldHeight,res,planeId);
    
    a_Star=astar_point(&bc,startx,starty,endx,endy,planeId,bearingNamed);
    ROS_ERROR("\033[22;31m Still here, doing my A* thing . . .");
    
#ifdef DEBUG
    //  unsigned int time = clock() / (CLOCKS_PER_SEC / 1000);
    //  unsigned int t = 0;
    //  
    //  stringstream prefix;
    //  prefix << "For plane," << planeId << ",\nGoal:," << endx << "," << endy << ",\n";
    //  prefix << "Start:," << startx << "," << starty << ",\n";
    //  prefix << "Timestep:,0,\n";
    //  stringstream name;
    //  name << "plane_" << planeId << "_t_" << t << "_" << time;
    //  bc.dump_csv( t, prefix.str(), name.str() );
    //  
    //  t = 1;
    //  stringstream prefix1;
    //  prefix1 << "For plane," << planeId << ",\nGoal:," << endx << "," << endy << ",\n";
    //  prefix1 << "Start:," << startx << "," << starty << ",\n";
    //  prefix1 << "Timestep:," << t << ",\n";
    //  stringstream name1;
    //  name1 << "plane_" << planeId << "_t_" << t << "_" << time;
    //  bc.dump_csv( t, prefix1.str(), name1.str() );
    //  
    //  cout << "This plane's lat == " << double_to_string( planes[ planeId ].getLocation().getLat()) << endl;
    //  cout << "This plane's lon == " << double_to_string( planes[ planeId ].getLocation().getLon()) << endl;
    //  cout << "This plane's x == " << double_to_string( planes[ planeId ].getLocation().getX()) << endl;
    //  cout << "This plane's y == " << double_to_string( planes[ planeId ].getLocation().getY()) << endl;
    cout << "In case you forgot, this is plane " << planeId << ", traveling " << 
    bearing_to_string( planes[ planeId ].get_named_bearing() ) << endl;
    cout << "A* says to go to (" << a_Star.x << ", " << a_Star.y << ")" << endl;
    
    bc.dump( 0 );
    bc.dump( 1 );
#endif
    
    if( planeId == 0 )
    {
      ROS_INFO("Created bc, going into A*");
    }
    
#ifdef DEBUG
    //  stringstream prefixcomp;
    //  prefixcomp << "For comparison \n";
    //  prefixcomp << "For plane," << planeId << ",\nGoal:," << endx << "," << endy << ",\n";
    //  prefixcomp << "Start:," << startx << "," << starty << ",\n";
    //  prefixcomp << "Timestep:,1,\n";
    //  stringstream namecomp;
    //  t = 1;
    //  namecomp << "plane_" << planeId << "_t_" << t << "_" << time << "_after";
    //  bc.dump_csv( t, prefixcomp.str(), namecomp.str() );
#endif
    
#if defined(Outputting) || defined(GODDAMMIT)
    ROS_INFO("A* says for plane %d to go here: \033[22;32m\nx: %d\ny: %d\nFrom here:\nx: %d\ny: %d",
             planeId, a_Star.x, a_Star.y, startx, starty);
    
#endif
    
    int currentx=startx;
    int currenty=starty;
    
    // Change the plane's "current" location for the sake of keeping planes in sync //
    int width = planes[ planeId ].getLocation().getWidth(); // in grid sqrs
    int height = planes[ planeId ].getLocation().getHeight(); // in grid sqrs
    
    if( bearingNamed == map_tools::N ) //headed north
    {
      if(currenty > 0) // safe to decrement...
        currenty--;
    }
    else if( bearingNamed == map_tools::NW ) //headed northwest
    {
      if(currentx > 0 && currenty > 0)
      {
        currentx--;
        currenty--;
      }
    }
    else if( bearingNamed == map_tools::W ) //headed west
    {
      if(currentx > 0)
        currentx--;
    }        
    else if( bearingNamed == map_tools::SW ) //headed southwest
    {
      if(currentx > 0 && currenty + 1 < height) 
      {
        currentx--;
        currenty++;
      }
    }
    else if( bearingNamed == map_tools::S ) //headed south
    {
      if(currenty + 1 < height)
        currenty++;
    }
    else if( bearingNamed == map_tools::SE ) //headed southeast
    {
      if(currentx + 1 < width && currenty + 1 < height)
      {
        currentx++;
        currenty++;
      }
    }
    else if( bearingNamed == map_tools::E ) //headed east
    {
      if(currentx + 1 < width)
        currentx++;
    }
    else //headed northeast
    {
      if(currentx + 1 < width && currenty > 0)
      {
        currentx++;
        currenty--;
      }
    }
    
    //set up where the plane 'will' be
    current.setXY(currentx,currenty);
    
#ifdef DEBUG
    /*
     assert( upperLeftLon < -85.49 && upperLeftLon > -85.491 );
     assert( upperLeftLat > 32.592 && upperLeftLat < 32.593 );
     assert( lonWidth > 0.005 && lonWidth < 0.00501 );
     assert( latWidth > -0.00381 && latWidth < -0.00380 );
     
     assert( a_Star.x < 47 );
     assert( a_Star.x >= 0 );
     assert( a_Star.y < 43 );
     assert( a_Star.y >= 0 );
     
     assert( res > 9.99 && res < 10.1 );
     */
#endif
    
    //where to go next
    Position aStar(upperLeftLon,upperLeftLat,lonWidth,latWidth,a_Star.x,a_Star.y,res);
    
#ifdef DEBUG
    cout << "aStar.getLat() == " << aStar.getLat() << endl;
    cout << "aStar.getLon() == " << aStar.getLon() << endl;
    cout << "aStar.getX() == " << aStar.getX() << endl;
    cout << "aStar.getY() == " << aStar.getY() << endl;
    
    cout << "You set it's (x, y) to (" << a_Star.x << ", " << a_Star.y << ")" << endl;
    /*
     assert( aStar.getLat() > 32.5882 );
     assert( aStar.getLat() < 32.593 );
     assert( aStar.getLon() < -85.484 );
     assert( aStar.getLon() > -85.4915 );
     */
#endif
    
    // The service to command a plane to go to a location
    bool newQueue=false;
    AU_UAV_ROS::GoToWaypoint srv;
    
    srv.request.planeID = planeId;
    srv.request.longitude = aStar.getLon();
    srv.request.latitude = aStar.getLat();
    srv.request.altitude = goalSrv.response.altitude;//? not sure if this is allowed but hey i like cheating    
    
    next.setLatLon( aStar.getLat(), aStar.getLon() );
    
    //these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue(if there was a new plane)
    srv.request.isAvoidanceManeuver = true;
    srv.request.isNewQueue = true;
    
    if(!client.call(srv))
      ROS_ERROR("YOUR SERVICE DIDN'T GO THROUGH YOU GONA CRASH!!!");
    
    /*goalSrv.request.planeID=planeId;
     goalSrv.request.isAvoidanceWaypoint=true;
     goalSrv.request.positionInQueue=0;
     if(findGoal.call(goalSrv))
     cout<<goalSrv.response.longitude<<" "<<goalSrv.response.latitude<<endl;*/
    
    //update plane so others see it going to new goal
    planes[planeId].update(current,next,gSpeed);  
    
  } // end if this is an okay goal
  else if( the_count > 60 )
  {
    ROS_ERROR("\n Um... did you kill a plane? \n");
    
    // This plane had a collision; kill it!
    planes.erase( planeId );
  }
  else // Bad goals are normal in the first couple rounds of updates
  {
    ROS_ERROR("Bad goal returned %f,%f",goalSrv.response.latitude,goalSrv.response.longitude);
    planes[planeId].setFinalDestination(planes[planeId].getLocation().getLon(), planes[planeId].getLocation().getLat());
  }
  
  cout << "End of callback " << endl << endl;
}

int main(int argc, char **argv)
{
	the_count=0;
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
    cout << endl;
    cout << "You've selected a field with upper left longitude: " << upperLeftLon << endl;
    cout << "                              upper left latitude: " << upperLeftLat << endl;
    cout << "                           width in deg longitude: " << lonWidth << endl;
    cout << "                            width in deg latitude: " << latWidth << endl;
    cout << "                                  width in meters: " << fieldWidth << endl;
    cout << "                                 height in meters: " << fieldHeight << endl;
  }
  
	else
	{
		ROS_ERROR("Cannot open field data");
	}
}

/*
map_tools::bearing_t name_bearing( double the_bearing )
{
  the_bearing = fmod(the_bearing, 360); // modular division for floats
  
  if( the_bearing > -22.5 && the_bearing <= 22.5 )
    return map_tools::N;
  else if( the_bearing > 22.5 && the_bearing <= 67.5 )
    return map_tools::NE;
  else if( the_bearing > 67.5 && the_bearing <= 112.5 )
    return map_tools::E;
  else if( the_bearing > 112.5 && the_bearing <= 157.5 )
    return map_tools::SE;
  else if( the_bearing > 157.5 && the_bearing <= 202.5 )
    return map_tools::S;
  else if( the_bearing > 202.5 && the_bearing <= 247.5 )
    return map_tools::SW;
  else if( the_bearing > 247.5 && the_bearing <= 292.5 )
    return map_tools::W;
  else if( the_bearing > 292.5 && the_bearing <= 337.5 )
    return map_tools::NW;
  else if( the_bearing > -67.5 && the_bearing <= -22.5 )
    return map_tools::NW;
  else if( the_bearing > -112.5 && the_bearing <= -67.5 )
    return map_tools::W;
  else if( the_bearing > -157.5 && the_bearing <= -112.5 )
    return map_tools::SW;
  else if( the_bearing > -202.5 && the_bearing <= -157.5 )
    return map_tools::S;
  else if( the_bearing > -247.5 && the_bearing <= -202.5 )
    return map_tools::SE;
  else if( the_bearing > -292.5 && the_bearing <= -247.5 )
    return map_tools::E;
  else if( the_bearing > -337.5 && the_bearing <= -292.5 )
    return map_tools::NW;
  else
  {
#ifdef DEBUG
    assert( the_bearing > -361 && the_bearing < 361 );
#endif
    return map_tools::N;
  }
}
 */

