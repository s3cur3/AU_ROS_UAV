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
#include "best_cost_final.h"
#include "astar_sparse2.cpp"
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
vector<Plane> planes;
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
 * locations will be checked against this one
 * @return TRUE if a collision occurred, FALSE if it's business as usual
 */
bool collision_occurred( int id_to_check );

void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
  bool next_is_current = false;
  
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
  
  /*
   #ifdef GODDAMMIT
   // If this doesn't fail immediately, it means the msg from the telemetry update is OK
   assert( destLat - currentLat > EPSILON || destLat - currentLat < -EPSILON );
   assert( destLon - currentLon > EPSILON || destLon - currentLon < -EPSILON );
   #endif
   */
#ifdef DEBUG
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
    
    /*stuff to print for x-plane
     int Plane ID
     double currentLatitude
     double currentLongitude
     double currentAltitude
     double groundSpeed
     double destLatitude
     double destLongitude
     double targetBearing
     double Timestamp (in milliseconds, doesn't matter where you start)
     */
    
    tele << planeId <<"\n"<< double_to_string(currentLat) << "\n" << double_to_string(currentLon);
    tele << "\n" << currentAlt << "\n" << gSpeed << "\n" << double_to_string(destLat);
    tele << "\n" << double_to_string(destLon) << "\n" << double_to_string(bearing) << "\n";
    tele << timestamp << "\n";
    
    tele.close();
  }
#endif
  
  //intilize everything (else) we need
  bool newQueue=false;
  AU_UAV_ROS::RequestWaypointInfo goalSrv;
  AU_UAV_ROS::GoToWaypoint srv;
  
  
  // Make the positions
#ifdef DEBUG
  assert( upperLeftLon < -85.49 && upperLeftLon > -85.491 );
  assert( upperLeftLat > 32.592 && upperLeftLat < 32.593 );
  assert( lonWidth > 0.005 && lonWidth < 0.00501 );
  assert( latWidth > -0.00381 && latWidth < -0.00380 );
  
  assert( currentLon < -85.484 );
  assert( currentLon > -85.4915 );
  assert( currentLat > 32.5882 );
  assert( currentLat < 32.593 );
  
  assert( res > 9.99 && res < 10.1 );
#endif
  
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
  
  assert( !collision_occurred( planeId ) );
  
  /*
   crash[planeId]=current;
   if(the_count > 6)
   {
   int x=0, y=0;
   if(collision(x,y))
   {
   ROS_ERROR("There has been a collision between planes %d and %d at position (%d,%d),(%f,%f) at step# %d"
   ,x,y,crash[x].getX(),crash[x].getY(),crash[x].getLat(),crash[x].getLon(),the_count);
   assert(false);
   }
   }*/
#endif
  
#ifdef DEBUG
  assert( upperLeftLon < -85.49 && upperLeftLon > -85.491 );
  assert( upperLeftLat > 32.592 && upperLeftLat < 32.593 );
  assert( lonWidth > 0.005 && lonWidth < 0.00501 );
  assert( latWidth > -0.00381 && latWidth < -0.00380 );
  
  assert( res > 9.99 && res < 10.1 );
#endif
  
  // The plane's next location, be it the goal or an avoidance waypoint
  Position next;
  //if next is good . . .
  if(destLat<=upperLeftLat && destLon>=upperLeftLon && destLat>0 && destLon<0)
  {
    next = Position(upperLeftLon,upperLeftLat,lonWidth,latWidth,destLon,destLat,res);
#ifdef DEBUG
    assert( next.getLon() < -85.484 );
    assert( next.getLon() > -85.4915 );
    assert( next.getLat() > 32.5882 );
    assert( next.getLat() < 32.593 );
#endif
  }
  else
  {
    next = Position(current);
    next_is_current = true;
#ifdef DEBUG
    assert( next.getLon() < -85.484 );
    assert( next.getLon() > -85.4915 );
    assert( next.getLat() > 32.5882 );
    assert( next.getLat() < 32.593 );
#endif
    ROS_INFO("\033[22;31m next is current");
  }
  
  // If it's a new plane
  if((int)planes.size()<=planeId)
  {
    //planesMap[planeId]=(int)planes.size();
    //ROS_INFO("Plane %d is mapped to %d",planeId,(int)planes.size());
    planes.push_back(Plane (planeId,current,next));
    
    newQueue=true;
    if(the_count>2)
      ROS_ERROR("error in plane stuffs");
  }
  
  //prevent the insanity that double []s bring
  Plane * plane = &(planes[planeId]);
  //update the plane
  plane->update(current,next,gSpeed);
#ifdef DEBUG
  if( true )
  {
    cout<<"Current location for plane"<<planeId<<" "<<plane->getLocation().getX()<<" "<<plane->getLocation().getY()<<endl;
    printf(" Which is %f, %f \n", plane->getLocation().getLon(), plane->getLocation().getLat() );
    cout<<"Next location for plane"<<planeId<<" "<<plane->getDestination().getX()<<" "<<plane->getDestination().getY()<<endl;
  }
#endif
  //populate the request for the destination
  
  goalSrv.request.planeID=planeId;
  goalSrv.request.isAvoidanceWaypoint=false;
  goalSrv.request.positionInQueue=0;
  
  //ask the coordinator nicely
  if(!findGoal.call(goalSrv))
    ROS_ERROR("No goal was returned");
  
#if defined(Testing) || defined(Outputting)
  // TY confirms that the goals are getting set correctly before the first
  // DG output when using Chicken.course
  if( planeId == 0 )
  {
    ROS_INFO("The goal of plane \033[22;32m %d returned was \033[22;31m %f,%f",
             planeId,goalSrv.response.longitude, goalSrv.response.latitude);
    ROS_INFO("The current location of plane \033[22;32m %d is \033[22;31m %f,%f",
             planeId, currentLon, currentLat);
  }
#endif
  
  // Make sure the goal is a real goal none of those lame
  // "become rich and famous goals." We want something real and
  // reachable by a foam airplane
  if( /*inside the area*/(goalSrv.response.latitude<=upperLeftLat&&goalSrv.response.longitude>=upperLeftLon) && goalSrv.response.latitude>0)
  {
#ifdef DEBUG
    assert( goalSrv.response.longitude < -85.484 );
    assert( goalSrv.response.longitude > -85.4915 );
    assert( goalSrv.response.latitude > 32.5882 );
    assert( goalSrv.response.latitude < 32.593 );
#endif
    plane->setFinalDestination(goalSrv.response.longitude, goalSrv.response.latitude);
    
#ifdef Testing || defined(Outputting)
    if( planeId == 0 )
    {
      ROS_INFO("The final destination is: %f,%f",goalSrv.response.longitude,goalSrv.response.latitude);
    }
#endif
    
  }
  else // It's going nowhere fast
  {
#ifdef DEBUG
    assert( plane->getLocation().getLon() < -85.484 );
    assert( plane->getLocation().getLon() > -85.4915 );
    assert( plane->getLocation().getLat() > 32.5882 );
    assert( plane->getLocation().getLat() < 32.593 );
    
    cout << "goalSrv.response.latitude is " << goalSrv.response.latitude << endl;
    cout << "goalSrv.response.longitude is " << goalSrv.response.longitude << endl;
#endif
    plane->setFinalDestination(plane->getLocation().getLon(), plane->getLocation().getLat());
    
#ifdef DEBUG
    ROS_INFO("The final destination does not exist or has been reached");
#endif
  }
  
  
  //grab stuff for A*, if thats his real name
  int startx=plane->getLocation().getX();
  int starty=plane->getLocation().getY();
  int endx=plane->getFinalDestination().getX();
  int endy=plane->getFinalDestination().getY();
#ifdef DEBUG
  assert( startx < 47 );
  assert( startx >= 0);
  assert( starty >= 0 );
  assert( starty < 43 );
  assert( endx < 47 );
  assert( endx >= 0);
  assert( endy >= 0 );
  assert( endy < 43 );
#endif
#ifdef DEBUG
  if( planeId == 10 )
  {
    ROS_INFO("for plane %d\n the startx:%d\n the starty:%d\n the endx:%d\n the endy:%d\n end lat: %f \n end lon %f",
             planeId,startx,starty,endx,endy, plane->getFinalDestination().getLat(),
             plane->getFinalDestination().getLon() );
  }
#endif
  
  planes[planeId]=*plane;
  
  
  point a_Star;
  best_cost bc = best_cost(&planes,fieldWidth,fieldHeight,res,planeId);
#ifdef DEBUG
  unsigned int time = clock() / (CLOCKS_PER_SEC / 1000);
  unsigned int t = 0;
  
  stringstream prefix;
  prefix << "For plane," << planeId << ",\nGoal:," << endx << "," << endy << ",\n";
  prefix << "Start:," << startx << "," << starty << ",\n";
  prefix << "Timestep:,0,\n";
  stringstream name;
  name << "plane_" << planeId << "_t_" << t << "_" << time;
  bc.dump_csv( t, prefix.str(), name.str() );
  //
  // stringstream prefix1;
  // prefix1 << "For plane," << planeId << ",\nGoal:," << endx << "," << endy << ",\n";
  // prefix1 << "Start:," << startx << "," << starty << ",\n";
  // prefix1 << "Timestep:,1,\n";
  // stringstream name1;
  // t = 1;
  // name1 << "plane_" << planeId << "_t_" << t << "_" << time;
  // bc.dump_csv( t, prefix1.str(), name1.str() );
  //
  // stringstream prefix2;
  // prefix2 << "For plane," << planeId << ",\nGoal:," << endx << "," << endy << ",\n";
  // prefix2 << "Start:," << startx << "," << starty << ",\n";
  // prefix2 << "Timestep:,2,\n";
  // stringstream name2;
  // t = 2;
  // name2 << "plane_" << planeId << "_t_" << t << "_" << time;
  // bc.dump_csv( t, prefix2.str(), name2.str() );
  //
  // stringstream prefix3;
  // prefix3 << "For plane," << planeId << ",\nGoal:," << endx << "," << endy << ",\n";
  // prefix3 << "Start:," << startx << "," << starty << ",\n";
  // prefix3 << "Timestep:,2,\n";
  // stringstream name3;
  // t = 3;
  // name3 << "plane_" << planeId << "_t_" << t << "_" << time;
  // bc.dump_csv( t, prefix3.str(), name3.str() );
  //
  // stringstream prefix10;
  // prefix10 << "For plane," << planeId << ",\nGoal:," << endx << "," << endy << ",\n";
  // prefix10 << "Start:," << startx << "," << starty << ",\n";
  // prefix10 << "Timestep:,10,\n";
  // stringstream name10;
  // t = 10;
  // name10 << "plane_" << planeId << "_t_" << t << "_" << time;
  // bc.dump_csv( t, prefix10.str(), name10.str() );
  //
  // stringstream prefix11;
  // prefix11 << "For plane," << planeId << ",\nGoal:," << endx << "," << endy << ",\n";
  // prefix11 << "Start:," << startx << "," << starty << ",\n";
  // prefix11 << "Timestep:,11,\n";
  // stringstream name11;
  // t = 11;
  // name11 << "plane_" << planeId << "_t_" << t << "_" << time;
  // bc.dump_csv( t, prefix11.str(), name11.str() );
#endif
  
  if( planeId == 0 )
  {
    ROS_INFO("Created bc, going into A*");
  }
  
  a_Star=astar_point(&bc,startx,starty,endx,endy,planeId,plane->get_named_bearing());
#ifdef DEBUG
  // stringstream prefixcomp;
  // prefixcomp << "For comparison \n";
  // prefixcomp << "For plane," << planeId << ",\nGoal:," << endx << "," << endy << ",\n";
  // prefixcomp << "Start:," << startx << "," << starty << ",\n";
  // prefixcomp << "Timestep:,1,\n";
  // stringstream namecomp;
  // t = 1;
  // namecomp << "plane_" << planeId << "_t_" << t << "_" << time << "_after";
  // bc.dump_csv( t, prefixcomp.str(), namecomp.str() );
#endif
#if defined(Outputting) || defined(GODDAMMIT)
  ROS_INFO("A* says for plane %d to go here: \033[22;32m\nx: %d\ny: %d\nFrom here:\nx: %d\ny: %d",
           planeId, a_Star.x, a_Star.y, startx, starty);
  
#endif
  
  int currentx=startx;
  int currenty=starty;
  map_tools::bearing_t direction = name_bearing( plane->getBearing() );
  cout<<"plane "<<planeId<<" has a direction of "<<map_tools::bearing_to_string( direction )<<endl;
  switch( direction )
  {
    case map_tools::N://headed north
      if(currenty-1 > -1)
        currenty--;
      break;
      
    case map_tools::NW://headed northwest
      if(currentx > 0 && currenty > 0)
      {
        currentx--;
        currenty--;
      }
      break;
      
    case map_tools::W://headed west
      if(currentx-1 > -1)
        currentx--;
      break;
      
    case map_tools::SW://headed southwest
      if(currentx-1 > -1 && currenty+1 <43)
      {
        currentx--;
        currenty++;
      }
      break;
      
    case map_tools::S://headed south
      if(currenty+1 <43)
        currenty++;
      break;
      
    case map_tools::SE://headed southeast
      if(currentx+1 < 47 && currenty+1 < 43)
      {
        currentx++;
        currenty++;
      }
      break;
      
    case map_tools::E://headed east
      if(currentx+1 < 47)
        currentx++;
      break;
      
    case map_tools::NE://headed northeast
      if(currentx+1 < 47 && currenty-1 > -1)
      {
        currentx++;
        currenty--;
      }
      break;
      
    default:
      cout<<"something has gone wrong";
  }
#ifdef DEBUG
  assert( currentx < 47 );
  assert( currentx >= 0 );
  assert( currenty < 43 );
  assert( currenty >= 0 );
#endif
  current.setXY(currentx,currenty);
  
  if( /*!next_is_current*/ (a_Star.x!=startx&&a_Star.y!=starty)&&(a_Star.x!=endx&&a_Star.y!=endy) )
  {
#ifdef DEBUG
    assert( upperLeftLon < -85.49 && upperLeftLon > -85.491 );
    assert( upperLeftLat > 32.592 && upperLeftLat < 32.593 );
    assert( lonWidth > 0.005 && lonWidth < 0.00501 );
    assert( latWidth > -0.00381 && latWidth < -0.00380 );
    
    assert( a_Star.x < 47 );
    assert( a_Star.x >= 0 );
    assert( a_Star.y < 43 );
    assert( a_Star.y >= 0 );
    
    assert( res > 9.99 && res < 10.1 );
#endif
    
    Position aStar(upperLeftLon,upperLeftLat,lonWidth,latWidth,a_Star.x,a_Star.y,res);
#ifdef Outputting
    if( planeId == 0 )
    {
      printf( "The avoidance wp was set to: %f, %f\n", aStar.getLon(), aStar.getLat() );
      printf( " ...which is: %d, %d\n", aStar.getX(), aStar.getY() );
    }
#endif
#ifdef DEBUG
    if( planeId != 0 )
    {
      cout << ( aStar.getLon() < -85.484 || aStar.getLat() > 32.5882 );
      cout << "aStar.getLat() == " << aStar.getLat() << endl;
      cout << "aStar.getLon() == " << aStar.getLon() << endl;
      cout << "aStar.getX() == " << aStar.getX() << endl;
      cout << "aStar.getY() == " << aStar.getY() << endl;
      
      cout << "You set it's (x, y) to (" << a_Star.x << ", " << a_Star.y << ")" << endl;
    }
    assert( aStar.getLat() > 32.5882 );
    assert( aStar.getLat() < 32.593 );
    assert( aStar.getLon() < -85.484 );
    assert( aStar.getLon() > -85.4915 );
#endif
    
    srv.request.planeID = planeId;
    srv.request.longitude = aStar.getLon();
    srv.request.latitude = aStar.getLat();
    srv.request.altitude = goalSrv.response.altitude;//? not sure if this is allowed but hey i like cheating
    
    //next.setXY( a_Star.x, a_Star.y );
    
    
    next.setLatLon( aStar.getLat(), aStar.getLon() );
    
    //these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue(if there was a new plane)
    srv.request.isAvoidanceManeuver = true;
    srv.request.isNewQueue = true;
    
    if( planeId == 0 )
    {
      ROS_INFO("\033[22;32mCollision detected; maneuvering to avoid");
    }
    
    if(!client.call(srv))
      ROS_ERROR("YOUR SERVICE DIDN'T GO THROUGH YOU GONA CRASH!!!");
    
    /*goalSrv.request.planeID=planeId;
     goalSrv.request.isAvoidanceWaypoint=true;
     goalSrv.request.positionInQueue=0;
     if(findGoal.call(goalSrv))
     cout<<goalSrv.response.longitude<<" "<<goalSrv.response.latitude<<endl;*/
    
    //update plane so others see it going to new goal
    plane->update(current,next,gSpeed);
  }
  else
  {
    plane->update(current,next,gSpeed);
  }
  
  planes[planeId]=*plane;
  
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
      << id_to_check << endl;
      cout << " (Note: presumably plane " << id_to_check
      << " was updated most recently)" << endl;
      return true;
    }
  }
  
  return false;
}
#endif

/*
 bool collision(int &one, int &two)
 {
 //longest declertion ever?
 /*bool dead = crash[3]==crash[4] ||
 crash[2]==crash[3] || crash[2]==crash[4] ||
 crash[1]==crash[2] || crash[1]==crash[3] || crash[1]==crash[4] ||
 crash[0]==crash[1] || crash[0]==crash[2] || crash[0]==crash[3] || crash[0]==crash[4];*/

/*if(crash[3]==crash[4])
 {one=3;two=4;return true;}
 
 if(crash[2]==crash[3])
 {one=2;two=3;return true;}
 if(crash[2]==crash[4])
 {one=2;two=4;return true;}
 
 if(crash[1]==crash[4])
 {one=1;two=4;return true;}
 if(crash[1]==crash[3])
 {one=1;two=3;return true;}
 if(crash[1]==crash[2])
 {one=1;two=2;return true;}
 if(crash[1]==crash[4])
 {one=1;two=4;return true;}
 if(crash[1]==crash[3])
 {one=1;two=3;return true;}
 if(crash[1]==crash[2])
 {one=1;two=2;return true;}
 
 if(crash[0]==crash[4])
 {one=0;two=4;return true;}
 if(crash[0]==crash[3])
 {one=0;two=3;return true;}
 if(crash[0]==crash[2])
 {one=0;two=2;return true;}
 if(crash[0]==crash[1])
 {one=0;two=1;return true;}
 return false;
 } */

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
    
  }
  
  else
  {
    ROS_ERROR("Cannot open field data");
  }
}

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

