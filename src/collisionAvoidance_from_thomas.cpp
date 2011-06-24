#define DEBUG
#define TYLERS_PC


#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <time.h>
#include "Plane.h"
#include "best_cost_with_fields.h"
#include "astar_sparse2.cpp"
#include "telemetry_data_out.h"
#include "Position.h"
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"
#include "output_helpers.h"

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


//keeps count of the number of services requested
int the_count;
int output_indices[12];


void makeField();
int name_bearing(double);

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
	bool newQueue=false;
	AU_UAV_ROS::RequestWaypointInfo goalSrv;
	AU_UAV_ROS::GoToWaypoint srv;
	
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
    tele << "\n" << currentAlt << "\n" <<	gSpeed << "\n" << double_to_string(destLat);
    tele << "\n" << double_to_string(destLon) << "\n" << double_to_string(bearing) << "\n";
    tele << timestamp << "\n";
    
    tele.close();
  }
#endif
  
	//update the planes current location
	Position current = Position(upperLeftLon,upperLeftLat,lonWidth,latWidth,currentLon,currentLat,res);

	//the planes next location be it the goal or a dodging point
	Position next;
	//if next is good
	if(destLat<=upperLeftLat && destLon>=upperLeftLon && destLat>0 && destLon<0)
		next = Position(upperLeftLon,upperLeftLat,lonWidth,latWidth,destLon,destLat,res);
	//if bad
	else
		next=Position(current);

	if((int)planes.size()<=planeId)
	{
		planes.push_back(Plane (planeId,current,next));
		newQueue=true;
		if(the_count>2)
			ROS_ERROR("error in plane stuffs");
	}
	//as little inderection as possiable
	Plane * plane = &(planes[planeId]);
	
	//update the plane
	plane->update(current,next,gSpeed,bearing);
	
	goalSrv.request.planeID=planeId;
	goalSrv.request.isAvoidanceWaypoint=false;
	goalSrv.request.positionInQueue=0;

	if(!findGoal.call(goalSrv))
		ROS_ERROR("No goal was returned");

	if( /*inside the area*/(goalSrv.response.latitude<=upperLeftLat && goalSrv.response.longitude>=upperLeftLon) && goalSrv.response.latitude>0)
		plane->setFinalDestination(goalSrv.response.longitude, goalSrv.response.latitude);

	else
		plane->setFinalDestination(plane->getLocation().getLon(), plane->getLocation().getLat());

	int startx=plane->getLocation().getX();
	int starty=plane->getLocation().getY();
	int endx=plane->getFinalDestination().getX();
	int endy=plane->getFinalDestination().getY();
	ROS_INFO("\033[22;32mPlane #%d \nStart x:%d Start y:%d \nEnd x:%d End y:%d",planeId,startx,starty,endx,endy);
	planes[planeId]=*plane;
	point a_Star;
  	best_cost bc = best_cost(&planes,fieldWidth,fieldHeight,res,planeId);
	a_Star=astar_point(&bc,startx,starty,endx,endy,planeId);

	int currentx=startx, currenty=starty;
	int direction=name_bearing(plane->getBearing());
	cout<<"plane "<<planeId<<" has a direction of "<<direction<<endl;
	switch(direction)
		{
			case 0://headed north
				currenty--;
				break;
				
			case 7://headed northwest
				currentx--;currenty--;
				break;
			
			case 6://headed west
				currentx--;
				break;
			
			case 5://headed southwest
				currentx--;currenty++;
				break;
			
			case 4://headed south
				currenty++;
				break;
			
			case 3://headed southeast
				currentx++;currenty++;
				break;
				
			case 2://headed east
				currentx++;
				break;
			
			case 1://headed northease
				currentx++;currenty--;
				break;
			
			default:
				cout<<"something has gone wrong";
		}
		current.setXY(currentx,currenty);

	if( /*!next_is_current*/ ( a_Star.x!=startx&& a_Star.y!=starty)&&( a_Star.x!=endx&& a_Star.y!=endy) )
	{
		Position aStar(upperLeftLon,upperLeftLat,lonWidth,latWidth,a_Star.x,a_Star.y,res);

				
		srv.request.planeID = planeId;
		srv.request.longitude = aStar.getLon();
		srv.request.latitude = aStar.getLat();
		srv.request.altitude = goalSrv.response.altitude;//? not sure if this is allowed but hey i like cheating
    
		next.setLatLon( aStar.getLat(), aStar.getLon() );

		//these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue(if there was a new plane)
		srv.request.isAvoidanceManeuver = true;
		srv.request.isNewQueue = true;
		cout<<a_Star.x<<"\n"<<a_Star.y<<"\n";
		ROS_INFO("\033[22;32mCollision detected; maneuvering to avoid\n Added:%f,%f to the queue of plane #%d", srv.request.latitude,srv.request.longitude,planeId);

		if(!client.call(srv))
			ROS_ERROR("YOUR SERVICE DIDN'T GO THROUGH YOU GONA CRASH!!!");
		//update plane so others see it going to new goal
		plane->update(current,next,gSpeed,bearing);
	}

	else
		plane->update(current,next,gSpeed,bearing);

	planes[planeId]=*plane;
	
		

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

//names the bearing of the plane
//0 is north and it goes clockwise 1=NE 2=E and so forth

int name_bearing( double the_bearing )
{
  the_bearing = fmod(the_bearing, 360); // modular division for floats
  
  if( the_bearing > -22.5 && the_bearing <= 22.5 )
    return 0;
  else if( the_bearing > 22.5 && the_bearing <= 67.5 )
    return 1;
  else if( the_bearing > 67.5 && the_bearing <= 112.5 )
    return 2;
  else if( the_bearing > 112.5 && the_bearing <= 157.5 )
    return 3;
  else if( the_bearing > 157.5 && the_bearing <= 202.5 )
    return 4;
  else if( the_bearing > 202.5 && the_bearing <= 247.5 )
    return 5;
  else if( the_bearing > 247.5 && the_bearing <= 292.5 )
    return 6;
  else if( the_bearing > 292.5 && the_bearing <= 337.5 )
    return 7;
  else if( the_bearing > -67.5 && the_bearing <= -22.5 )
    return 7;
  else if( the_bearing > -112.5 && the_bearing <= -67.5 )
    return 6;
  else if( the_bearing > -157.5 && the_bearing <= -112.5 )
    return 5;
  else if( the_bearing > -202.5 && the_bearing <= -157.5 )
    return 4;
  else if( the_bearing > -247.5 && the_bearing <= -202.5 )
    return 3;
  else if( the_bearing > -292.5 && the_bearing <= -247.5 )
    return 2;
  else if( the_bearing > -337.5 && the_bearing <= -292.5 )
    return 1;
  else
  {
    assert( the_bearing > -361 && the_bearing < 361 );
    return 0;
  }
}





