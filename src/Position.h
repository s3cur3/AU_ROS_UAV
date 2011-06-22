//Position.h
//Header file for the Position class with use in the Auburn REU program 2011
//Thomas Crescenzi

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
const double PI = 2*acos(0.0);// pi
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
  void latLonToXY( int & out_x, int & out_y);
	int lonToX();
	int latToY();
    
  double resolution; // meters in a grid square
    
  // The constructor's "helper"; used to set the width and height of the grid in
  // which the position exists.
  void set_up_grid_parms();
    	
public:
  void xy_to_latlon( double & out_lat, double & out_lon );
  void setXY(int x1, int y1);
  void setLatLon( double latitude, double longitude );
  
	//the preceding methods can be used interchangebly as setX also sets the longitude and so forth
	//note there will be some error if using x and y mainly as they only can convert to
  //the longitude and latitude of the top left of their block
	double getLat() const;
	double getLon() const;
	int getX();
	int getY();
	int getWidth();
	int getHeight();
	double getUpperLeftLongitude();
    double getUpperLeftLatitude();
	bool operator==(Position &equal);

    Position(double upperLeftLongitude=0.0, double upperLeftLatitude=0.0,
             double lonwidth=0.0, double latwidth=0.0);
    Position(double upperLeftLongitude, double upperLeftLatitude,
             double lonwidth, double latwidth, double resolution_to_use);
    Position(double upperLeftLongitude, double upperLeftLatitude, double lonwidth, double latwidth, double longitude, double latitude);
    Position(double upperLeftLongitude, double upperLeftLatitude, double lonwidth, 
             double latwidth, double longitude, double latitude, double resolution_to_use);
	Position(double upperLeftLongitude, double upperLeftLatitude, double lonwidth, double latwidth, int x, int y, double resolution);
	Position(double upperLeftLongitude, double upperLeftLatitude, double lonwidth, double latwidth, double longitude, double latitude, int x, int y, double resolution );
};

bool Position::operator==(Position &equal)
{
	return (y==equal.getY()&&x==equal.getX());//a bit vague
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
}

int Position::getX() // TY changed this to return an int instead of a double
	{return x;}

int Position::getY() // TY changed this to return an int instead of a double
{return y;}

void Position::setXY(int x1, int y1)
{
#ifdef DEBUG
  assert( y1 >= 0 && y1 <= h );
  assert( x1 >= 0 && x1 <= w );
#endif
  x=x1;
  y=y1;
//  cout << "You just set (x,y) to (" << x << ", " << y << ")" << endl;
//  cout<<lat<<"    "<<lon<<endl;
  xy_to_latlon( lat, lon );
//  cout<<lat<<"   "<<lon<<endl;
//  cout << "Dist between points " << ( map_tools::calculate_distance_between_points( top_left_lat, top_left_long,
//                                                        lat, lon, "meters") ) << endl;
//  cout << endl << endl << "Pos in Position is " << x << ", " << y;
  
#ifdef GODDAMMIT
  assert( lat > 32.5882 );
  assert( lat < 32.593 );
  assert( lon < -85.484 );
  assert( lon > -85.4915 );
  
  double double_check_lat = getLat();
  assert( double_check_lat > 32.5882 );
  assert( double_check_lat < 32.593 );
  double double_check_lon = getLon();
  assert( double_check_lon < -85.484 );
  assert( double_check_lon > -85.4915 );
#endif
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
                   double lonwidth, double latwidth, double resolution_to_use)
{
	top_left_long=upperLeftLongitude;
	top_left_lat=upperLeftLatitude;
	lonWidth=lonwidth;
	latWidth=latwidth;
  resolution = resolution_to_use;
  
  set_up_grid_parms();
}

Position::Position(double upperLeftLongitude, double upperLeftLatitude,
                   double lonwidth, double latwidth, double longitude, double latitude)
{
	top_left_long=upperLeftLongitude;
	top_left_lat=upperLeftLatitude;
	lonWidth=lonwidth;
	latWidth=latwidth;
	
  resolution = -1.0;
    
  set_up_grid_parms();
  
  /*
  setLon(longitude);
	setLat(latitude); */
  setLatLon( latitude, longitude );
}

Position::Position(double upperLeftLongitude, double upperLeftLatitude, // THOMAS: This is the only constructor I'm using.
                   double lonwidth, double latwidth, double longitude,
                   double latitude, double resolution_to_use)
{
	top_left_long=upperLeftLongitude;
	top_left_lat=upperLeftLatitude;
	lonWidth=lonwidth;
	latWidth=latwidth;
	
  resolution = resolution_to_use;
    
  set_up_grid_parms();
  
  /*
  setLon(longitude);
	setLat(latitude); */
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
//  cout << "In pos, dist is " << d_from_origin_to_pt << endl;

  double bearing_between_pts;
  if( d_from_origin_to_pt == 0 )
    bearing_between_pts = 0;
  else
    bearing_between_pts = 90 + (RADtoDEGREES * asin( y*resolution / d_from_origin_to_pt ));
  
//  cout << "In Pos, bearing is " << bearing_between_pts << endl;
  map_tools::calculate_point( top_left_lat, top_left_long, 
                              d_from_origin_to_pt, bearing_between_pts,
                              out_lat, out_lon );
  
  out_lat += (latWidth / (2 * getHeight() ) );
  out_lon += (lonWidth / (2 * getWidth() ) );
#ifdef GODDAMMIT
  cout << "Calculated lat to be " << out_lat << endl;
  cout << "Calculated lon to be " << out_lon << endl;
#endif 
#ifdef DEBUG
  assert( d_from_origin_to_pt > -EPSILON ); // non-negative
#endif
}

int Position::lonToX()
{
#ifdef DEBUG
  int the_calc = (int)( ( map_tools::
                         calculate_distance_between_points( top_left_lat, top_left_long,
                                                            top_left_lat, lon, "meters") ) /
                       resolution );
  assert( the_calc >= 0 && the_calc < w );
#endif
  return (int)( ( map_tools::
                 calculate_distance_between_points( top_left_lat, top_left_long,
                                                   top_left_lat, lon, "meters") ) /
               resolution );
  /*
	return (int)( ( map_tools::
                    calculate_distance_between_points( top_left_lat, top_left_long,
                                                       lat, lon, "meters") ) /
                resolution );*/
}

int Position::latToY()
{
#ifdef DEBUG
  int the_calc = (int)( ( map_tools::
                         calculate_distance_between_points( top_left_lat, top_left_long,
                                                           lat, top_left_long, "meters") ) /
                       resolution );
  assert( the_calc >= 0 && the_calc < w );
#endif
  return (int)( ( map_tools::
                      calculate_distance_between_points( top_left_lat, top_left_long,
                                                         lat, top_left_long, "meters") ) /
                     resolution );
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
  assert( out_x < w );
  assert( out_y < h );
  assert( out_x >= 0 );
  assert( out_y >= 0 );
#endif
}

// Needed to create a duplicate of a Position object
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

