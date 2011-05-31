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
    return (int)( ceil( (double)( width_of_field) / map_resolution ) + 0.1 );
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
    double earth_radius = 6371000.0; // meters, on average
    double d_lat = to_radians( latitude_2 - latitude_1 );
    double d_long = to_radians( longitude_2 - longitude_1 );
    double a = ( sin( d_lat / 2) * sin( d_lat / 2) +
                 cos( to_radians(latitude_1) ) * cos( to_radians(latitude_2) ) * 
                 sin( d_long / 2) * sin( d_long / 2) );
    double c = 2 * atan2( sqrt(a), sqrt(1 - a) );
    
    the_distance = earth_radius * c;
    the_distance = fabs( the_distance ); // make sure it's positive
    
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

double map_tools::to_radians( double angle_in_degrees )
{
    return ( angle_in_degrees * ( pi/180 ) );
}

#endif