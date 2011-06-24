/*
KMLCreator
This component will be used to create standard KML files to be loaded into Google Earth or Google Maps.
This will monitor updates from the planes and then eventually write them to a file.
*/

//standard headers
#include <stdio.h>
#include <map>
#include <queue>

// Added by Tyler Young for distance calculation
#include <vector>
////////////////////// End of additions by TY ///////////////////////////////


//ros headers
#include "ros/ros.h"
#include "ros/package.h"
#include "AU_UAV_ROS/standardDefs.h"
#include "AU_UAV_ROS/TelemetryUpdate.h"
#include "AU_UAV_ROS/SaveFlightData.h"

#define MAX_LINE_TYPES 6

//at startup we are monitoring the UAVs, maybe change this later
bool isMonitoringTelemetry = true;

//this map links a plane ID to a queue of received points
std::map<int, std::queue<struct AU_UAV_ROS::waypoint> > mapOfPaths;

std::string colorComboArray[] = {"7f0000ff", "7f00ff00", "7fff0000", "7f00ffff", "7fff00ff", "7fffff00"};


/**
 * Uses the haversine formula to calculate the distance between two points.
 * Added by Tyler Young
 * 
 * Returns the distance in feet, yards, meters, kilometers, or attoparsecs.
 * @param latitude_1 The latitude (in decimal degrees) for point 1
 * @param longitude_1 The longitude (in decimal degrees) for point 1
 * @param latitude_2 The latitude (in decimal degrees) for point 2
 * @param longitude_2 The longitude (in decimal degrees) for point 2
 * @param units The units for the returned distance. Allowed values are:
 *                   - 'feet'
 *                   - 'yards'
 *                   - 'miles'
 *                   - 'meters'
 *                   - 'kilometers'
 *                   - 'attoparsecs'
 *              Default value is meters.
 * @return The distance, measured in whatever units were specified (default is meters)
 */
double dist_between_pts( double latitude_1, double longitude_1, 
                         double latitude_2, double longitude_2,
                         std::string units )
{    
  const double DEGREES_TO_RAD = 3.14159265 / 180;
  const double earth_radius = 6371000; // meters, on average
  
  double the_distance;
  double d_lat = DEGREES_TO_RAD*( latitude_2 - latitude_1 );
  double d_long = DEGREES_TO_RAD*( longitude_2 - longitude_1 );
  double sin_d_lat = sin( d_lat / 2);
  double sin_d_long = sin( d_long / 2);
  double a = ( sin_d_lat * sin_d_lat +
              cos( DEGREES_TO_RAD*latitude_1 ) * cos( DEGREES_TO_RAD*latitude_2 ) * 
              sin_d_long * sin_d_long );
  double c = 2 * atan2( sqrt(a), sqrt(1 - a) );
  
  the_distance = fabs(earth_radius * c); // make sure it's positive
  
  if( units == "feet" )
    return the_distance * 3.28083989501312;
  if( units == "yards" )
    return the_distance * 3.28083989501312 / 3;
  if( units == "miles" )
    return the_distance * 3.28083989501312 / 5280;
  if( units == "kilometers" )
    return the_distance / 1000;
  if( units == "attoparsecs" )
    return the_distance * 32.4077649;
  else
    return the_distance;
}


/*
telemetryCallback
This is called whenever a new telemetry message is received.  It should simply store the waypoint value
received for writing to a file later.
*/
void telemetryCallback(const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg)
{
	if(isMonitoringTelemetry)
	{
		//construct waypoint to save
		struct AU_UAV_ROS::waypoint temp;
		temp.latitude = msg->currentLatitude;
		temp.longitude =msg->currentLongitude;
		temp.altitude = msg->currentAltitude;
		
		//save that bad boy
		mapOfPaths[msg->planeID].push(temp);
	}
	else
	{
		//we stopped monitoring which means the data is saved, clean exit time
		exit(0);
	}
}

/*
saveFlightData
This is a service called when it's time to save the file.
*/
bool saveFlightData(AU_UAV_ROS::SaveFlightData::Request &req, AU_UAV_ROS::SaveFlightData::Response &res)
{
	isMonitoringTelemetry = false;
	ROS_INFO("Saving to file %s...", req.filename.c_str());

	// This vector used by Tyler Young to store the distances traveled by planes /////
  std::vector< double > d_traveled;
  ////////////////////// End of additions by TY ///////////////////////////////
  
	FILE *fp = fopen((ros::package::getPath("AU_UAV_ROS")+"/flightData/"+req.filename).c_str(), "w");
	if(fp != NULL)
	{
		//opening stuff, never changes
		fprintf(fp, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
		fprintf(fp, "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n");
		fprintf(fp, "  <Document>\n");
		fprintf(fp, "    <name>Test</name>\n");
		fprintf(fp, "    <open>1</open>\n");
		
		for(int count = 0; count < MAX_LINE_TYPES; count++)
		{
			fprintf(fp, "    <Style id=\"lineType%d\">\n", count);
			fprintf(fp, "      <LineStyle>\n");
			fprintf(fp, "        <color>%s</color>\n", colorComboArray[count % MAX_LINE_TYPES].c_str());
			fprintf(fp, "        <width>4</width>\n");
			fprintf(fp, "      </LineStyle>\n");
			fprintf(fp, "      <PolyStyle>\n");
			fprintf(fp, "        <color>7fffffff</color>\n");
			fprintf(fp, "      </PolyStyle>\n");
			fprintf(fp, "    </Style>\n");
		}
		fprintf(fp, "    <Folder>\n");
		fprintf(fp, "      <name>Paths</name>\n");
		fprintf(fp, "      <visibility>1</visibility>\n");

		//here goes the magic
		std::map<int, std::queue<struct AU_UAV_ROS::waypoint> >::iterator ii;
		
		int count = 0;
		for(ii = mapOfPaths.begin(); ii != mapOfPaths.end(); ii++)
		{
			fprintf(fp, "      <Placemark>\n");
			fprintf(fp, "        <name>UAV #%d</name>\n", ii->first);
			fprintf(fp, "        <visibility>1</visibility>\n");
			fprintf(fp, "        <LookAt>\n");
			fprintf(fp, "          <longitude>%lf</longitude>\n", ii->second.front().longitude);
			fprintf(fp, "          <latitude>%lf</latitude>\n", ii->second.front().latitude);
			fprintf(fp, "          <altitude>0</altitude>\n");
			fprintf(fp, "          <heading>0</heading>\n");
			fprintf(fp, "          <tilt>0</tilt>\n");
			fprintf(fp, "          <range>4451.842204068102</range>\n");
			fprintf(fp, "        </LookAt>\n");
			fprintf(fp, "        <styleUrl>#lineType%d</styleUrl>\n", count);
			count = (count + 1) % MAX_LINE_TYPES;
			fprintf(fp, "        <LineString>\n");
			fprintf(fp, "          <extrude>1</extrude>\n");
			fprintf(fp, "          <tessallate>1</tessallate>\n");
			fprintf(fp, "          <altitudeMode>absolute</altitudeMode>\n");
			fprintf(fp, "          <coordinates>\n");
			
      // Added by TY for distance calc                /////////////////////////////// 
      while( ii->first >= (int)d_traveled.size() )
      {
        d_traveled.push_back( 0.0 );
      }
      
      double prev_lat = 0.0;
      double prev_lon = 0.0;
      ///////////////////////// End of additions by TY ////////////////////////
      
			//for each coordinate saved, we want to write to the file
			while(!(ii->second.empty()))
			{
				AU_UAV_ROS::waypoint temp = ii->second.front();
				fprintf(fp, "            %lf, %lf, %lf\n", temp.longitude, temp.latitude, temp.altitude);
				ii->second.pop();
        
        
        
        // Add this pt the distance calculated (by TY) ///////////////////////////////
        if( (prev_lat > 0.00001 || prev_lat < -0.00001)  && 
           (prev_lon > 0.00001 || prev_lon < -0.00001) ) 
        { // Prev position has been initialized
          d_traveled[ ii->first ] += dist_between_pts( temp.latitude, temp.longitude, 
                                                       prev_lat, prev_lon,
                                                       "meters" );
        }
        
        prev_lat = temp.latitude;
        prev_lon = temp.longitude;
        ///////////////////////// End of additions by TY ////////////////////////
			}
			
			fprintf(fp, "          </coordinates>\n");
			fprintf(fp, "        </LineString>\n");
			fprintf(fp, "      </Placemark>\n");      			
		}

		//closing stuff, never changes
		fprintf(fp, "    </Folder>\n");
		fprintf(fp, "  </Document>\n");
		fprintf(fp, "</kml>\n");
    
    
    // Output the distances calculated (by TY) ///////////////////////////////
    fprintf(fp, "<!--\n");
    
    double total_d = 0.0;
    for( int i = 0; i < (int)d_traveled.size(); i++ )
    {
      total_d += d_traveled[ i ];
      fprintf(fp, "     Plane %d distance traveled: %f meters\n", i, d_traveled[ i ]);
    }
    fprintf(fp, "     Total distance traveled: %f meters\n", total_d );
    fprintf(fp, "-->\n");
    ///////////////////////// End of additions by TY ////////////////////////
    
		fclose(fp);
			
		ROS_INFO("Done! It's safe to CTRL-C now.");
		return true;
	}
	else
	{
		res.error = "Error opening the file";
		return false;
	}
}

int main(int argc, char **argv)
{
	//standard ROS startup
	ros::init(argc, argv, "KMLCreator");
	ros::NodeHandle n;
	
	//subscribe to telemetry outputs and create client for the avoid collision service
	ros::Subscriber sub = n.subscribe("telemetry", 1000, telemetryCallback);
	
	//set up services
	ros::ServiceServer saveFlightDataService = n.advertiseService("save_flight_data", saveFlightData);
	
	//needed for ROS to wait for callbacks
	ros::spin();	

	return 0;
}
