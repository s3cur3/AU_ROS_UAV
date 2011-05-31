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
#include "best_cost_grid.h"

//ROS headers
#include "ros/ros.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/GoToWaypoint.h"

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
vector<Plane> planes(planeNum+1);//just in case :)
int planesMap[];


//keeps count of the number of services requested
int count;

bool inVector(int id);
void makeField();
//this function is run everytime new telemetry information from any plane is recieved
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	//TODO:Make this function do something useful, aka an avoidance algorithm
	ROS_INFO("Received update #[%lld]", msg->currentWaypointIndex);
		//this will be replaced by students to do more than just send a string
		std::stringstream ss;
		ss << "Sending service request " << count++;
		AU_UAV_ROS::findGoal goal;

		//wrap the info into positions
		Position current(upperLeftLon,upperLeftLat,lonWidth,latWidth,msg->currentLongitude,msg->currentLatitude,res);
		Position next(upperLeftLon,upperLeftLat,lonWidth,latWidth,msg->destLongitude,msg->destLatitude,res);

		//create the intial plane 
		if(planesMap[msg->planeID]==-1)
		{
			planesMap[msg->planeID]=planes.size();
			planes.push_back(Plane(msg->planeID,current,next));
		}
		//update the plane
		planes[planesMap[msg->planeID]].update(current,next,msg->targetBearing, msg->groundSpeed);

		goal.request.planeId=msg->planeID;
		goal.request.isAvoidanceWaypoint=false;
		goal.request.positionInQueue=0;

		findGoal.call(goal);
		planes[planesMap[msg->planeId]].setFinalDestination(goal.response.longitude,goal.response.latitude)

		int avoidTehCrash[2];
		A-Star(best_cost_grid(planes,10,10,10,planesMap[msg->planeId]),avoidTehCrash);

		//dummying up a service request for the REU students to see
		AU_UAV_ROS::GoToWaypoint srv;
		srv.request.planeID = msg->planeID;
		srv.request.latitude = avoidTehCrash[1];//fix with maptools
		srv.request.longitude = avoidTehCrash[0];//fix with maptools
		srv.request.altitude = goal.response.altitude;//? not sure if this is allowed but hey i like cheating
		
		//these settings mean it is an avoidance maneuver waypoint AND to clear the avoidance queue
		srv.request.isAvoidanceManeuver = true;
		srv.request.isNewQueue = true;

		//check to make sure the client call worked (regardless of return values from service)
		if(client.call(srv))
		{
			ROS_INFO("Received response from service request %d", (count-1));
		}
		else
		{
			ROS_ERROR("Did not receive response");
		}
}

int main(int argc, char **argv)
{
	//standard ROS startup
	makeField();
	ros::init(argc, argv, "collisionAvoidance");
	ros::NodeHandle n;
	
	//subscribe to telemetry outputs and create client for the avoid collision service
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	client = n.serviceClient<AU_UAV_ROS::GoToWaypoint>("go_to_waypoint");
	findGoal = n.serviceClient<AU_UAV_ROS::RequestWaypointInfo>("request_waypoint_info");
	
	//random seed for if statement in telemetryCallback, remove when collision avoidance work begins
	srand(time(NULL));
	
	//initialize counting
	count = 0;

	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}

void makeField()
{
	cout<<"Upperleftlon:";
	cin>>upperLeftLon;
	cout>>"Upperleftlat";
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

