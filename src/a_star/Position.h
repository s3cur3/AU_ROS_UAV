//
// Position.h
// Header file for the Position class with use in the Auburn REU program 2011
// By Thomas Crescenzi, with additions from Tyler Young
//

#ifndef POSITION 
#define POSITION

#include <iostream>
#include <fstream>
#include "map_tools.h"
#include <assert.h>

#ifndef EPSILON
#define EPSILON 0.000001
#endif

#ifndef RADIAN_CONSTANTS
#define RADIAN_CONSTANTS
const double PI = 2*acos(0.0);
const double TWO_PI = 2*PI;
const double RADtoDEGREES = 180/PI;//Conversion factor from Radians to Degrees
const double DEGREEStoRAD = PI/180;//Conversion factor from Degrees to Radians
#endif

class Position
{
private:
	//position within the grid
	int x;//long
	int y;//lat
  double decimal_x; // decimal version of the x and y coordinates
  double decimal_y;
	double resolution; // meters in a grid square(meters/square)
	//position on the earth
	double lat;//y
	double lon;//x
	double altidue;//just there for kicks
	//size of the grid or matrix
	int w;// number of squares wide
	int h;// squares high
	//size of the "earth"
	//this part is kinda an issue for now the position class is built around a top left corner of the area it is contained within
	double top_left_lat;//the value of the latitude of the top left corner
	double top_left_long;//same as top_left_lat but for longitude 
  double top_right_lat;
 	double top_right_long;
  double btm_left_lat;
 	double btm_left_long;
	double latWidth;
	double lonWidth;
 	double width_in_meters;
 	double height_in_meters;
  
	//functions for converting between systems
	/**
	A function that converts from latitude and longitude to x and y. It is called in
	setLatLon(). The latitude and longitude must be within the bounds set by setLatLon()
	input:
		@param out_x the x value to be changed. note it is an out parameter.
		@param out_y the y value to be changed. note it is an out parameter.
	**/
  void latLonToXY( int & out_x, int & out_y);
	/**
	A function for converting between latitude and longitude and decimal x and y. Decimal x and y are used by A*.
	A value of (5.5,5.5) would be the middle of the square (5,5). The latitude and longitude must be within the bounds set by setLatLon().
	input:
		@param out_x the x value to be changed. note it is an out parameter.
		@param out_y the y value to be changed. note it is an out parameter.
	**/
  void lat_lon_to_decimal_xy( double & out_x, double & out_y);
    
  
    
  // The constructor's "helper"; used to set the width and height of the grid in
  // which the position exists.
  void set_up_grid_parms();
    	
public:
	/**
	A function used in setXY() to set the lat and longitude values.
	input:
		@param out_lon the latitude value to be changed. note it is an out parameter.
		@param out_lon the longitude value to be changed. note it is an out parameter.
	**/
  void xy_to_latlon( double & out_lat, double & out_lon );
	/**
	A function that sets the x and y values of the position. They MUST be set together, if the Earth is round.
	If it happens that in the future the Earth becomes flat most of this code would be unusable and would have to be 
	replaced with Euclidian equations. After setting the xy the latlon is then set as well to keep everything up to date.
	input:
		@param x1 the x value to be set. It must be >=0 and <w
		@param y1 the y value to be set. It must be >=0 and <h
	**/
  void setXY(int x1, int y1);
	/**
	A function used to set the latitude and longitude. As with setXY() they must be set together unless someone flattens the earth.
	The x and y values are also set with this function
	input: 
		@param latitude the latitude to be set. must be >=top_left_lat and <top_left_lat+latWidth
		@param longitude the latitude to be set. must be >=top_left_lon and <top_left_long+lonWidth
	**/
  void setLatLon( double latitude, double longitude );
  
	//the preceding methods can be used interchangebly as setXY also sets the longitude/latidue and so forth
	//note there will be some error if using x and y mainly as they only can convert to
  //the longitude and latitude of the top left of their block
	
	/** 
	Getter for the latitude. Returns a value in decimal degrees.
	output:
		@return the latitude of the position in decimal degrees.
	**/
	double getLat() const;
	/** 
	Getter for the longitude. Returns a value in decimal degrees.
	output:
		@return the longitude of the position in decimal degrees.
	**/
	double getLon() const;
	/** 
	Getter for the x. Returns a value of a grid position.
	output:
		@return the x.
	**/
	int getX();
	/** 
	Getter for the y. Returns a value of a grid position.
	output:
		@return the y.
	**/
	int getY();
	/** 
	Getter for the decimal x. Returns a value of a decimal grid position.
	output:
		@return the decimal x.
	**/
  double getDecimalX() const;
	/** 
	Getter for the decimal y. Returns a value of a decimal grid position.
	output:
		@return the decimal y.
	**/
  double getDecimalY() const;
	/** 
	Getter for the width in squares aka the number of xs.
	output:
		@return the width of the grid, in grid squares
	**/
	int getWidth();
	/** 
	Getter for the height in squares aka the number of ys.
	output:
		@return the height of the grid, in grid squares
	**/
	int getHeight();
	/** 
	Getter for the top longitude left of the field in which the position exists.
	output:
		@return the top left longitude
	**/
	double getUpperLeftLongitude();
	/** 
	Getter for the top latitude left of the field in which the position exists.
	output:
		@return the top left latitude
	**/
  double getUpperLeftLatitude();
	/**
	An overloaded == operator. Two points are equal if they have the same decimal x and y.
	**/
	bool operator==(Position &equal);

	/**
	A default constructor for whenever a default might be needed.
	**/
  Position(double upperLeftLongitude=0.0, double upperLeftLatitude=0.0,
             double lonwidth=0.0, double latwidth=0.0);
	/**
	A constructor to be used when making a plane with latitude and longitude.
	input:
		@param upperLeftLongitude the upper left longitude of the field to be used in decimal degrees
		@param upperLeftLatitude the upper left latitude of the field to be used in decimal degrees
		@param lonwidth the width of the field in longitude in decimal degrees
		@param latwidth the width of the field in latitude in decimal degrees.
		@param longitude the longitude at which this position is being constructed
		@param latitude the latitude at which this position in being constructed
		@resolution_to_use the resolution to use for the grid in meters per square
	**/
	Position(double upperLeftLongitude, double upperLeftLatitude, double lonwidth, 
             double latwidth, double longitude, double latitude, double resolution_to_use);
						 /**
	A constructor to be used when making a plane with x and y.
	input:
		@param upperLeftLongitude the upper left longitude of the field to be used in decimal degrees
		@param upperLeftLatitude the upper left latitude of the field to be used in decimal degrees
		@param lonwidth the width of the field in longitude in decimal degrees
		@param latwidth the width of the field in latitude in decimal degrees.
		@param x the x at which this position is being constructed
		@param y the y at which this position in being constructed
		@resolution_to_use the resolution to use for the grid in meters per square
	**/
	Position(double upperLeftLongitude, double upperLeftLatitude, double lonwidth, double latwidth, int x, int y, double resolution);
};

bool Position::operator==(Position &rhs)
{
	return ( decimal_y - rhs.getDecimalY() < EPSILON && 
           decimal_y - rhs.getDecimalY() > -EPSILON && 
           decimal_x - rhs.getDecimalX() < EPSILON && 
           decimal_x - rhs.getDecimalX() > -EPSILON );
}

double Position::getLat() const
	{return lat;}

double Position::getLon() const
	{return lon;}

void Position::setLatLon( double latitude, double longitude )
{
  lon = longitude;
  lat = latitude;
  latLonToXY( x, y );
  lat_lon_to_decimal_xy( decimal_x, decimal_y );
}

int Position::getX() 
	{
	return x;
	}

int Position::getY() 
	{
		return y;
	}

double Position::getDecimalX() const
{ 
  return decimal_x;
}

double Position::getDecimalY() const
{
  return decimal_y;
}

void Position::setXY(int x1, int y1)
{
#ifdef DEBUG
  assert( y1 >= 0 && y1 < h );
  assert( x1 >= 0 && x1 < w );
#endif

  x=x1;
  y=y1;
  decimal_x = x1;
  decimal_y = y1;
  
  xy_to_latlon( lat, lon );
  
}
Position::Position(double upperLeftLongitude, double upperLeftLatitude, 
                   double lonwidth, double latwidth)
{
	top_left_long=upperLeftLongitude;
	top_left_lat=upperLeftLatitude;
	lonWidth=lonwidth;
	latWidth=latwidth;
  resolution = -1.0;
    
  set_up_grid_parms();
}

Position::Position(double upperLeftLongitude, double upperLeftLatitude, 
                   double lonwidth, double latwidth, double longitude,
                   double latitude, double resolution_to_use)
{
	top_left_long=upperLeftLongitude;
	top_left_lat=upperLeftLatitude;
	lonWidth=lonwidth;
	latWidth=latwidth;
	
  resolution = resolution_to_use;
    
  set_up_grid_parms();
  
#ifdef DEBUG
  assert( (int)latitude != 0 && (int)longitude != 0 );
#endif
  setLatLon( latitude, longitude );
}

Position::Position(double upperLeftLongitude, double upperLeftLatitude,
                   double lonwidth, double latwidth, int x1, int y1, double res)
{
	top_left_long=upperLeftLongitude;
	top_left_lat=upperLeftLatitude;
	lonWidth=lonwidth;
	latWidth=latwidth;

  resolution = res;
  lat=0;
  lon=0;
  set_up_grid_parms();
  setXY(x1,y1); // changes latitude and longitude
}
	
void Position::xy_to_latlon( double & out_lat, double & out_lon )
{
  double d_from_origin_to_pt = resolution * sqrt( x*x + y*y );
  double bearing_between_pts;
	
  if( d_from_origin_to_pt == 0 )
    bearing_between_pts = 0;
  else
    bearing_between_pts = 90 + (RADtoDEGREES * asin( y*resolution / d_from_origin_to_pt ));
  
  map_tools::calculate_point( top_left_lat, top_left_long, 
                              d_from_origin_to_pt, bearing_between_pts,
                              out_lat, out_lon );
  
  out_lat += (latWidth / (2 * getHeight() ) );
  out_lon += (lonWidth / (2 * getWidth() ) );
#ifdef DEBUG
  assert( d_from_origin_to_pt > -EPSILON ); // non-negative
#endif
}

void Position::latLonToXY( int & out_x, int & out_y)
{
  double d_from_origin = map_tools::calculate_distance_between_points(
                                                  top_left_lat, top_left_long,
                                                  lat, lon, "meters");
  double bearing; // in radians!

  if( d_from_origin > -EPSILON && d_from_origin < EPSILON )
  {
    bearing = 0;
  }
  else
  {
    bearing = map_tools::calculate_bearing_in_rad( top_left_lat, top_left_long,
                                                   lat, lon );
    
    if( bearing > 0 )
    {
      if( bearing < PI/2 )
      {
        bearing -= PI/2;
      }
      else
        bearing = PI/2 - bearing;
    }
    bearing = fmod( bearing, PI/2 );
  }
  
#ifdef DEBUG
  if( bearing > 0.001 )
  {
    cout << "That bearing of " << bearing << "is gonna break things!" << endl;
    cout << "Your point was (" << lat << ", " << lon << ")" << endl;
  }
  assert( bearing < 0.001 );
  assert( bearing > -PI/2 - 0.01 );
#endif
  out_x = (int)( (int)(cos( bearing ) * d_from_origin + 0.5) / resolution );
  out_y = -(int)( (int)(sin( bearing ) * d_from_origin - 0.5) / resolution );
  
#ifdef DEBUG
  if( out_x >= w || out_y >= h )
  {
    cout << "You calculated (x, y) of (" << out_x << ", " << out_y << ") from bearing " << endl;
    cout << bearing*RADtoDEGREES << " and dist from origin " << d_from_origin << endl;
    cout << "Does this surprise you? Your origin is " << top_left_lat << ", " << top_left_long << endl;
  }
  assert( out_x < w );
  assert( out_y < h );
  assert( out_x >= 0 );
  assert( out_y >= 0 );
#endif
}

void Position::lat_lon_to_decimal_xy( double & out_x, double & out_y)
{
  double d_from_origin = map_tools::calculate_distance_between_points(
                                                                      top_left_lat, top_left_long,
                                                                      lat, lon, "meters");
  double bearing; // in radians!
  
  if( d_from_origin > -EPSILON && d_from_origin < EPSILON )
  {
    bearing = 0;
  }
  else
  {
    bearing = map_tools::calculate_bearing_in_rad( top_left_lat, top_left_long,
                                                   lat, lon );
    
    if( bearing > 0 )
    {
      if( bearing < PI/2 )
      {
        bearing -= PI/2;
      }
      else
        bearing = PI/2 - bearing;
    }
    bearing = fmod( bearing, PI/2 );
  }
  
#ifdef DEBUG
  if( bearing > 0.001 )
  {
    cout << "That bearing of " << bearing << "is gonna break things!" << endl;
    cout << "Your point was (" << lat << ", " << lon << ")" << endl;
  }
  assert( bearing < 0.001 );
  assert( bearing > -PI/2 - 0.01 );
#endif
  out_x = (cos( bearing ) * d_from_origin) / resolution;
  out_y = -(sin( bearing ) * d_from_origin) / resolution;
  
#ifdef DEBUG
  assert( (int)out_x < w );
  assert( (int)out_y < h );
  assert( (int)out_x >= 0 );
  assert( (int)out_y >= 0 );
#endif
}

int Position::getWidth()
{
    return w;
}
int Position::getHeight()
{
    return h;
}
double Position::getUpperLeftLongitude()
{
    return top_left_long;
}
double Position::getUpperLeftLatitude()
{
    return top_left_lat;
}

void Position::set_up_grid_parms()
{
  if ( resolution < 1 ) // wasn't set in constructor, so . . .
    resolution = 10.0; // default to a grid with 10 meter squares
  
  top_right_lat = top_left_lat;
  top_right_long = top_left_long + lonWidth;
  btm_left_lat = top_left_lat + latWidth;
  btm_left_long = top_left_long;
  
  width_in_meters = 
  map_tools::calculate_distance_between_points( top_left_lat, top_left_long,
                                               top_right_lat, top_right_long,
                                               "meters" );
  height_in_meters = 
  map_tools::calculate_distance_between_points( top_left_lat, top_left_long,
                                               btm_left_lat, btm_left_long,
                                               "meters" );    
  w = map_tools::find_width_in_squares( width_in_meters, height_in_meters,
                                       resolution );
  h = map_tools::find_height_in_squares( width_in_meters, height_in_meters,
                                        resolution );
}

#endif

