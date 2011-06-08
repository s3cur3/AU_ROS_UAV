/*
collisionAvoidance
This is where students will be able to program in a collision avoidance algorithm.  The telemetry callback
is already setup along with a dummy version of how the service request would work.
*/

//standard C++ headers
#include <sstream>
#include <stdlib.h>
#include <time.h>
#include <vector>
#include "Plane.h"
#include "best_cost.h"

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
		
		int planeId=msg->planeID;//limits the amount of indirection
		

		//wrap the Telemetry info into positions
		Position current(upperLeftLon,upperLeftLat,lonWidth,latWidth,msg->currentLongitude,msg->currentLatitude,res);
		Position next(upperLeftLon,upperLeftLat,lonWidth,latWidth,msg->destLongitude,msg->destLatitude,res);

		//create the intial plane if there is no plane in the vector yet
		if(planesMap[planeId]==-1)
		{
			planesMap[planeId]= planes.size();//the location in the vector where the plane will go
			planes.push_back(Plane(planeId,current,next));//pushes back the new plane
		}
		
		//update the plane with the new telemetry data
		planes[planesMap[planeId]].update(current,next,msg->targetBearing, msg->groundSpeed);
		Plane currentPlane = planes[planesMap[planeId]];
		//fill the service with the required info
		goal.request.planeID=planeId;
		goal.request.isAvoidanceWaypoint=false;//ask matt if this is how it works
		goal.request.positionInQueue=0;//ask about this too
	
		//call the service. the catching might be excessive but i doubt that it takes much time
		if(findGoal.call(goal))
			ROS_INFO("Got the destination");
		else
			ROS_ERROR("Failed to receive the destination");
		//fill the plane's final destination in
  planes[planesMap[planeId]].setFinalDestination(goal.response.longitude,goal.response.latitude);

		//int avoidTehCrash[2];
		//this is where all the magic happens
		//Pass A* a map and a plane
		//A-Star(/*replace with bester cost*/best_cost_grid(planes,/*find the xwidth*/10,/*find the ywidth*/10,res,planesMap[planeId]),currentPlane);
		//thats right only one line of magic, don't bother to look at the 1000+ lines behind the curtain 

		//fill the service with the new waypoint info
		srv.request.planeID = planeId;
		srv.request.latitude = currentPlane.getDestination().getLat();//send the new lat
		srv.request.longitude = currentPlane.getDestination().getLon();//send the new lon
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
	makeField();
	
	//subscribe to telemetry outputs and create client for the avoid collision service and the goal giving service
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	client = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
	findGoal = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");
	
	//initialize counting
	the_count = 0;

	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}

void makeField()
{
	//get all the data. ?do we need more
	cout<<"Upperleftlon:";
	cin>>upperLeftLon;
	cout<<"Upperleftlat";
	cin>>upperLeftLat;
	cout<<"Lon-width:";
	cin>>lonWidth;
	cout<<"Lat-width:";
	cin>>latWidth;
	cout<<"Resoultion";
	cin>>res;
	cout<<"How Many Planes:";
	cin>>planeNum;
	planesMap[planeNum];
	for(int i=0; i<planeNum; i++)
		planesMap[i]=-1;
}
//150 lines :)
