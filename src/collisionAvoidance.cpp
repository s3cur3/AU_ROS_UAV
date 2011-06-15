//#define Testing
#define Outputting
//#define collisionTesting

//#define TYLERS_PC

//standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <time.h>
#include "Plane.h"
#include "best_cost_with_fields.h"
#include "astar_point.cpp"
#include "telemetry_data_out.h"

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"
#include "AU_UAV_ROS/RequestWaypointInfo.h"



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
#ifdef collisionTesting
Position crash[5];
//int crashSize=5;
#endif

//the number of files written per plane to teledata
int output_indices[12];
int planesmade;

//keeps count of the number of services requested
int the_count;

template <class T>
inline std::string to_string( const T& t )
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}


void makeField();

#ifdef collisionTesting
bool	collision(int &x, int &y);
#endif

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
  
#ifdef Testing
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
	
	tele <<planeId <<"\n"<< currentLat<<"\n"<<currentLon<<"\n"<<currentAlt<<"\n"<<
	gSpeed<<"\n"<<destLat<<"\n"<<destLon<<"\n"<<bearing<<"\n"<<timestamp<<endl;

	tele.close();
#endif

	//intilize everything(else) we need 
	bool newQueue=false;
	AU_UAV_ROS::RequestWaypointInfo goal;
	AU_UAV_ROS::GoToWaypoint srv;


	//make the positions
	Position current = Position(upperLeftLon,upperLeftLat,lonWidth,latWidth,currentLon,currentLat,res);

	
#ifdef collisionTesting
	crash[planeId]=current;
	if(the_count>6)
	{
		int x=0, y=0;
		if(collision(x,y))
		{	
			ROS_ERROR("There has been a collision between planes %d and %d at position (%d,%d),(%f,%f) at step# %d"
			,x,y,crash[x].getX(),crash[x].getY(),crash[x].getLat(),crash[x].getLon(),the_count);
			assert(false);
		}
	}
#endif

	Position next;
	if(destLat>0&&destLon<0)
		next = Position(upperLeftLon,upperLeftLat,lonWidth,latWidth,destLon,destLat,res);
	else
		next = current;

	//if its a new plane
	if((int)planes.size()<=planeId)
	{
		//planesMap[planeId]=(int)planes.size();
		//ROS_INFO("Plane %d is mapped to %d",planeId,(int)planes.size());
		planes.push_back(Plane (planeId,current,next));
	}

	//prevent the insanity that double []s bring
	Plane plane = planes[planeId];
	//update the plane
	plane.update(current,next,gSpeed,bearing);
	cout<<"Current location for plane"<<planeId<<" "<<plane.getLocation().getX()<<" "<<plane.getLocation().getY()<<endl;
	cout<<"Next location for plane"<<planeId<<" "<<plane.getDestination().getX()<<" "<<plane.getDestination().getY()<<endl;
	//populate the request for the destination
	
	goal.request.planeID=planeId;
	goal.request.isAvoidanceWaypoint=false;
	goal.request.positionInQueue=0;
	
	//ask the coordinator nicely
	if(!findGoal.call(goal))
		ROS_ERROR("No goal was returned");

	//make sure the goal is a real goal none of those lame "become rich and famous goals" we want something real and reachable by a foam airplane

#if 	defined(Testing) || defined(Outputting)
	ROS_INFO("The goal of plane \033[22;32m %d returned was \033[22;31m %f,%f",planeId,goal.response.longitude, goal.response.latitude);
#endif

	if( /*inside the area*/(goal.response.latitude<upperLeftLat&&goal.response.longitude>upperLeftLon) && goal.response.latitude>0)
	{
		plane.setFinalDestination(goal.response.longitude, goal.response.latitude);

#ifdef Testing
		ROS_INFO("The final destination is:%f,%f",goal.response.longitude,goal.response.latitude);
#endif

	}
	//its going nowhere fast
	else
	{	
		plane.setFinalDestination(plane.getLocation().getLon(), plane.getLocation().getLat());

#ifdef Testing
		ROS_INFO("The final destination does not exist or has been reached");
#endif

	}
	
	
	
	//grab stuff for A*, if thats his real name
	int startx=plane.getLocation().getX();
    	int starty=plane.getLocation().getY();
   	int endx=plane.getFinalDestination().getX();
   	int endy=plane.getFinalDestination().getY();

//#ifdef Testing
	ROS_INFO("for plane %d\n the startx:%d\n the starty:%d\n the endx:%d\n the endy:%d",planeId,startx,starty,endx,endy);
//#endif	

	point forSparta;
	best_cost bc = best_cost(&planes,fieldWidth,fieldHeight,res,planeId);
	forSparta=astar_point(&bc,startx,starty,endx,endy,planeId);
	//our code will blot out the sun
	if(the_count>6)
	{
	bc.dump(0);
	cin.get();
	bc.dump(1);
	cin.get();
	}

#ifdef Outputting
	ROS_INFO("A* says\033[22;32m:\nx: %d\ny: %d",forSparta.x, forSparta.y);
#endif

	if( (forSparta.x!=startx&&forSparta.y!=starty)&&(forSparta.x!=endx&&forSparta.y!=endy))
	{
		
		double lon=forSparta.x*( lonWidth / current.getWidth())+upperLeftLon;
		double lat=forSparta.y*( latWidth / current.getHeight())+upperLeftLat;

		srv.request.planeID = planeId;
		srv.request.longitude = lon;
		srv.request.latitude = lat;
		srv.request.altitude = goal.response.altitude;//? not sure if this is allowed but hey i like cheating

		next.setX(forSparta.x);
		next.setY(forSparta.y);

		//these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue(if there was a new plane)
		srv.request.isAvoidanceManeuver = true;
		srv.request.isNewQueue = true;

		ROS_INFO("\033[22;32mCollision detected maunvering to avoid");

		if(!client.call(srv))
			ROS_ERROR("YOUR SERVICE DIDN'T GO THROUGH YOU GONA CRASH!!!");

		goal.request.planeID=planeId;
		goal.request.isAvoidanceWaypoint=true;
		goal.request.positionInQueue=0;
		findGoal.call(goal);
		cout<<goal.response.longitude<<" "<<goal.response.latitude<<endl;
		plane.update(current,next,gSpeed,bearing);
		//cin.get();
	}

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

#ifdef collisionTesting
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
	{one=0;two=2;return true;}*/
	if(crash[0]==crash[1])
	{one=0;two=1;return true;}
	
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

	  fieldWidth=
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
