//
//  map_tools.h
//  AU_UAV_ROS
//
//  Created by Tyler Young on 5/26/11.
// 
// A utility class for the map and Position classes

#ifndef MAP_TOOLS
#define MAP_TOOLS

#include <math.h>

using namespace std;

static const double earth_radius = 6371000; // meters, on average

#ifndef RADIAN_CONSTANTS
#define RADIAN_CONSTANTS
const double PI = 2*acos(0.0);// pi
const double TWO_PI = 2*PI;
const double RADtoDEGREES = 180/PI;//Conversion factor from Radians to Degrees
const double DEGREEStoRAD = PI/180;//Conversion factor from Degrees to Radians
#endif

namespace map_tools 
{
  /**
   * Using the width, height, and resolution (in whatever system of measurement
   * you're using, such as meters), this returns the width of the field IN SQUARES.
   * @param width_of_field The width of the flyable area, in meters
   * @param height_of_field The height of the flyable area, in meters
   * @param map_resolution The resolution (width and height of a given square),
   *                       in meters
   * @return The width of the map grid in grid squares; pass this in
   *         empty and the function will assign it the proper value.
   */
  unsigned int find_width_in_squares( double width_of_field, 
                                     double height_of_field, 
                                     double map_resolution );
  
  /**
   * Using the width, height, and resolution (in whatever system of measurement
   * you're using, such as meters), this returns the width of the field IN SQUARES.
   * @param width_of_field The width of the flyable area, in meters
   * @param height_of_field The height of the flyable area, in meters
   * @param map_resolution The resolution (width and height of a given square),
   *                       in meters
   * @return The height of the map grid in grid squares; pass this in
   *         empty and the function will assign it the proper value.
   */
  unsigned int find_height_in_squares( double width_of_field, 
                                      double height_of_field, 
                                      double map_resolution );
  
  /**
   * Uses the haversine formula to calculate the distance between two points.
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
  double calculate_distance_between_points( double latitude_1, double longitude_1, 
                                           double latitude_2, double longitude_2,
                                           string units );

  /**
   * Calculates an ending lat-long coordinate given a starting lat-long position, 
   * a distance between the two points, and a bearing from the starting point to the
   * final point.
   * @param latitude_1 The latitude (in decimal degrees) for point 1
   * @param longitude_1 The longitude (in decimal degrees) for point 1
   * @param distance_in_meters Distance between starting and ending points
   * @param bearing_in_deg The bearing from the starting to the ending point
   * @param out_latitude_2 The latitude (in decimal degrees) for the ending point
   * @param out_longitude_2 The longitude (in decimal degrees) for the ending point
   * @param 
   */
  void calculate_point( double latitude_1, double longitude_1, 
                        double distance_in_meters, double bearing_in_deg,
                        double & out_latitude_2, double & out_longitude_2 );
  
  /**
   * Calculate the bearing, in degrees, between two points
   * @param latitude_1 The latitude (in decimal degrees) for point 1
   * @param longitude_1 The longitude (in decimal degrees) for point 1
   * @param latitude_2 The latitude (in decimal degrees) for point 2
   * @param longitude_2 The longitude (in decimal degrees) for point 2
   * @return The bearing, in degrees, from point 1 to point 2
   */
  double calculateBearing( double latitude_1, double longitude_1, 
                           double latitude_2, double longitude_2 );

  /**
   * Calculate the bearing, in radians, between two points (which themselves are 
   * given in decimal degrees)
   * @param latitude_1 The latitude (in decimal degrees) for point 1
   * @param longitude_1 The longitude (in decimal degrees) for point 1
   * @param latitude_2 The latitude (in decimal degrees) for point 2
   * @param longitude_2 The longitude (in decimal degrees) for point 2
   * @return The bearing, in radians, from point 1 to point 2
   */
  double calculate_bearing_in_rad( double latitude_1, double longitude_1, 
                                   double latitude_2, double longitude_2 );
  
  /**
   * Converts an angle, for use in the haversine formula
   * @param angle_in_degrees The angle you wish to convert from degrees to radians
   * @return The angle converted to radians
   */
  double to_radians( double angle_in_degrees );
  
  const double pi = 3.1415926535897932;
}

unsigned int map_tools::find_width_in_squares( double width_of_field, 
                                               double height_of_field, 
                                               double map_resolution )
{
    return (int)( ceil( (double)(width_of_field) / map_resolution ) + 0.1 );
}

unsigned int map_tools::find_height_in_squares( double width_of_field, 
                                                double height_of_field, 
                                                double map_resolution )
{
    return (int)( ceil( (double)( height_of_field ) / map_resolution ) + 0.1 );
}

double map_tools::calculate_distance_between_points( double latitude_1, double longitude_1, 
                                                     double latitude_2, double longitude_2,
                                                     string units )
{    
    double the_distance;
    double d_lat = to_radians( latitude_2 - latitude_1 );
    double d_long = to_radians( longitude_2 - longitude_1 );
    double sin_d_lat=sin( d_lat / 2);
    double sin_d_long=sin( d_long / 2);
    double a = ( sin_d_lat * sin_d_lat +
                 cos( to_radians(latitude_1) ) * cos( to_radians(latitude_2) ) * 
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

void map_tools::calculate_point( double latitude_1, double longitude_1, 
                                 double distance_in_meters, double bearing_in_deg,
                                 double & out_latitude_2, double & out_longitude_2 )
{
  double ang_dist_in_rad = (distance_in_meters / earth_radius);
#ifdef GODDAMMIT
  cout << "ADiR " << ang_dist_in_rad << endl;
#endif
  double bearing_in_rad = to_radians(bearing_in_deg);
#ifdef GODDAMMIT
  cout << "Bear " << bearing_in_rad << endl;
#endif

  latitude_1 = to_radians( latitude_1 );
#ifdef GODDAMMIT
  cout << "Lat 1 " << latitude_1 << endl;
#endif
  longitude_1 = to_radians( longitude_1 );
#ifdef GODDAMMIT
  cout << "Lon 1 " << longitude_1 << endl;
#endif
  out_latitude_2 = to_radians( out_latitude_2 );
#ifdef GODDAMMIT
  cout << "Lat2 " << out_latitude_2 << endl;
#endif
  
  out_latitude_2 = asin( (sin(latitude_1) * cos( ang_dist_in_rad )) + 
                         (cos(latitude_1) * sin( ang_dist_in_rad ) * 
                         cos( bearing_in_rad )) );
#ifdef GODDAMMIT
  cout << "Lat2 " << out_latitude_2<< endl;
#endif
  
  out_longitude_2 = to_radians( out_longitude_2 );
#ifdef GODDAMMIT
  cout << "Lon 2 " << out_longitude_2 << endl;
#endif
  
  out_longitude_2 = longitude_1 +
    atan2( sin(bearing_in_rad) * sin(ang_dist_in_rad) * cos(latitude_1), 
           cos(ang_dist_in_rad)- (sin(latitude_1) * sin(out_latitude_2)) );
#ifdef GODDAMMIT
  cout << "Lon 2" << out_longitude_2 << endl;
#endif

  
  out_longitude_2 *= RADtoDEGREES;
  out_latitude_2 *= RADtoDEGREES;
}

double map_tools::calculateBearing( double latitude_1, double longitude_1, 
                                    double latitude_2, double longitude_2 )
{
  latitude_1 = to_radians( latitude_1 );
  latitude_2 = to_radians( latitude_2 );
  longitude_1 = to_radians( longitude_1 );
  longitude_2 = to_radians( longitude_2 );
  
  double deltalon=longitude_2-longitude_1;
  
  double y = sin(deltalon)*cos(latitude_2);
  double x = cos(latitude_1) * sin(latitude_2) - sin(latitude_1) * cos(latitude_2) * cos(deltalon);
  return atan2(y, x)*RADtoDEGREES;
}

double map_tools::calculate_bearing_in_rad( double latitude_1, double longitude_1, 
                                            double latitude_2, double longitude_2 )
{  
  latitude_1 = to_radians( latitude_1 );
  latitude_2 = to_radians( latitude_2 );
  longitude_1 = to_radians( longitude_1 );
  longitude_2 = to_radians( longitude_2 );
  
  double deltalon = longitude_2 - longitude_1;
  
  double y = sin(deltalon)*cos(latitude_2);
  double x = cos(latitude_1) * sin(latitude_2) - sin(latitude_1) * cos(latitude_2) * cos(deltalon);
  return atan2(y, x);
}

double map_tools::to_radians( double angle_in_degrees )
{
    return ( angle_in_degrees * ( pi/180 ) );
}

#endif