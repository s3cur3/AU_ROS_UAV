/*
collisionAvoidance
This is where students will be able to program in a collision avoidance algorithm.  The telemetry callback
is already setup along with a dummy version of how the service request would work.
*/

#define DEBUG
#define RUNNING_ROS


//standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include <iostream>
#include <fstream>
#include "Plane.h"
#include "best_cost.h"
#include "astar.cpp"

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
int planeNum;
//where the planes are
vector<Plane> planes;
//a map. the index in the array is the planeid the value is the location of the plane's info in the planes vector
int planesMap[12];


//keeps count of the number of services requested
int the_count;


void makeField();
//this function is run everytime new telemetry information from any plane is recieved
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	ROS_INFO("Received update #[%lld]", msg->currentWaypointIndex);
		std::stringstream ss;
		ss << "Sending service request " << the_count++;
		
		//sets up the service that will ask the coordinator for the current plane's goal
		AU_UAV_ROS::RequestWaypointInfo goal;
		//sets up the service that will tell the coordinator the new waypoint
		AU_UAV_ROS::GoToWaypoint srv;
		ofstream tele;
		tele.open("/home/tyler/Desktop/teledata.txt", ios::out);
		tele << msg->planeID<<"\n"<<msg->currentLongitude<<"\n"<<msg->currentLatitude<<"\n"<<msg->destLongitude<<"\n"<<msg->destLatitude;
		tele.close();
		int planeId=msg->planeID;//limits the amount of indirection
		
    //cout << "HI" << endl;

		//wrap the Telemetry info into positions
		Position current(upperLeftLon,upperLeftLat,lonWidth,latWidth,msg->currentLongitude,msg->currentLatitude,res);
		Position next;
		if((int)msg->destLongitude!=0&&(int)msg->destLatitude!=0)
			next=Position(upperLeftLon,upperLeftLat,lonWidth,latWidth,msg->destLongitude,msg->destLatitude,res);
		
		else
			next=Position(current);
		
		ofstream nex;
		nex.open("/home/tyler/Desktop/nextdata.txt", ios::out);
		nex << next.getLon()<<"\n"<<next.getLat()<<"\n";
		nex.close();
		//create the intial plane if there is no plane in the vector yet
		ROS_INFO("Hey I made it past next stuff");
		if(planesMap[planeId]==-1)
		{
			ROS_INFO("Hey I started making a plane");
			planesMap[planeId]= planes.size();//the location in the vector where the plane will go
			ROS_INFO("Hey I placed stuff in the vector");
			planes.push_back(Plane(planeId,current,next));//pushes back the new plane
			ROS_INFO("Hey I made a plane");
		}
		//ROS_INFO("Hey I made it past next stuff");
		//update the plane with the new telemetry data
		planes[planesMap[planeId]].update(current,next,msg->targetBearing, msg->groundSpeed);
		Plane currentPlane = planes[planesMap[planeId]];
  
		//fill the service with the required info
		goal.request.planeID=planeId;
		goal.request.isAvoidanceWaypoint=false;//ask matt if this is how it works
		goal.request.positionInQueue=0;//ask about this too
	
		//call the service. the catching might be excessive but i doubt that it takes much time
		if(findGoal.call(goal))
			ROS_INFO("Got the destination from the cordinator");
		else
			ROS_ERROR("Failed to receive the destination");
		//fill the plane's final destination in
		if((int)goal.response.latitude!=0&&(int)goal.response.longitude!=0)
		{
			ROS_INFO("The destination is not 0");
			planes[planesMap[planeId]].setFinalDestination(goal.response.longitude,goal.response.latitude);
		}
		else
		{
			ROS_INFO("The destination is 0");
			planes[planesMap[planeId]].setFinalDestination(currentPlane.getLocation().getLon(),currentPlane.getLocation().getLat());
		}
		//int avoidTehCrash[2];
		//this is where all the magic happens
		//Pass A* a map and a plane
	ROS_INFO("HEY I'm about to calculate stuff with maptools");
  ROS_INFO("The stuff for maptools is:\nU_L lat: %f\nU_L lon: %f\nU_L lon + lonWidth:  %f\nU_L lat + latWidth:  %f\nlatWidth: %f\nlonWidth: %f",upperLeftLat,upperLeftLon, upperLeftLon + lonWidth, upperLeftLat + latWidth, latWidth, lonWidth);

    double width_of_field = 
    map_tools::calculate_distance_between_points( upperLeftLat, upperLeftLon,
                                                 upperLeftLat, upperLeftLon + lonWidth,
                                                 "meters");
	ROS_INFO("HEY I calculated stuff with maptools");
    double height_of_field = 
    map_tools::calculate_distance_between_points( upperLeftLat, upperLeftLon,
                                                 upperLeftLat + latWidth, upperLeftLon,
                                                 "meters");
	ROS_INFO("HEY I calculated other stuff with maptools");
    int startx=currentPlane.getLocation().getX();
    int starty=currentPlane.getLocation().getY();
	ROS_INFO("HEY I got the x and y");
    int endx=currentPlane.getFinalDestination().getX();
    int endy=currentPlane.getFinalDestination().getY();
	ROS_INFO("HEY I Got the stuff for A* and I'm going to make a grid now. Wish me luck.");
  ROS_INFO("The stuff for A* is:\nStart x: %d\nStart y: %d\nEnd x:  %d\nEnd y:  %d\nHeight: %f\nWidth:  %f\nRes:    %f",startx,starty,endx,endy, height_of_field, width_of_field, res);
    best_cost bc( planes,width_of_field,height_of_field,res,planesMap[planeId]);
	ROS_INFO("Hey I'm about to run A*");
		astar( &bc,startx,starty,endx,endy,planeId);
		//thats right only one line of magic, don't bother to look at the 1000+ lines behind the curtain 
	ROS_INFO("Hey I ran A*");
		//fill the service with the new waypoint info
		srv.request.planeID = planeId;
		srv.request.latitude = goal.response.latitude;//currentPlane.getDestination().getLat();//send the new lat
		srv.request.longitude =goal.response.longitude; //currentPlane.getDestination().getLon();//send the new lon
		srv.request.altitude = goal.response.altitude;//? not sure if this is allowed but hey i like cheating
		
		//these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue
		srv.request.isAvoidanceManeuver = true;
		srv.request.isNewQueue = true;//not sure if we want to clear it, i think we do

		//check to make sure the client call worked (regardless of return values from service)
		if(client.call(srv))
		{
			ROS_INFO("Received response from service request %d", (the_count-1));
		}
		else
		{
			//client.call(srv); //we might want to call it again to try and force the data through
			ROS_ERROR("Did not receive response");
		}
}

int main(int argc, char **argv)
{
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
  ifstream the_file( "/home/tyler/Desktop/field.txt" );
  if ( the_file.is_open() )
  {
    // The extraction operator (>>) will stop reading at each bit of whitespace
    the_file >> upperLeftLon;
    the_file >> upperLeftLat;
    the_file >> lonWidth;
    the_file >> latWidth;
    the_file >> res;
    the_file >> planeNum;
#ifdef DEBUG
    assert( upperLeftLon > -90 && upperLeftLon < 0 );
    assert( upperLeftLat > 30 && upperLeftLat < 33 );
    assert( lonWidth > 0 && lonWidth < 1 );
    assert( latWidth > -1 && latWidth < 0 );
    assert( res == 10 );
    assert( planeNum == 1 );
#endif
    
    the_file.close();
  }
	ROS_INFO("Hey im not in the for loop!");
	//planesMap[planeNum];
	for(int i=0; i<planeNum; i++)
	{
		ROS_INFO("Hey im in the for loop!");
		planesMap[i]=-1;
	}
}