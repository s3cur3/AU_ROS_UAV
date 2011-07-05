//Plane.h
//Header file for the plane class with use in the Auburn REU program 2011
//Thomas Crescenzi

#ifndef PLANE 
#define PLANE

#include <iostream>
#include "Position.h"
#include <math.h>

#ifndef RADIAN_CONSTANTS
#define RADIAN_CONSTANTS
const double PI = 2*acos(0.0);// pi
const double TWO_PI = 2*PI;
const double RADtoDEGREES = 180/PI;//Conversion factor from Radians to Degrees
const double DEGREEStoRAD = PI/180;//Conversion factor from Degrees to Radians
#endif

class Plane // Just a note, Thomas: Tyler was getting a compile error with
{          // "public" preceding "class Plane"; he removed it.
private:
	int id;
	Position current; // a dummy initialization
	Position destination;
	Position finalDestination;//destination could be an avoidance waypoint
	Position lastPosition;// used to calculate the current baring
	double bearingToDest; // in degrees
	double bearing;
	double speed;
	int direction;//the direction the plane is facing, in the grid, calcuated from the bearing
	//direction is a number from 0 to 7
	//0 is north and the number increases to teh left
	//so 1 is northwest, 2 is west, 3 is southwest and so forth
	void calculateBearings();
  
  bool current_is_virtual;
public:
  /**
   * Used to update a plane's current position without affecting its goal
   * @param current The plane's location, as might be obtained through a telemetry
   *                update
   */
  void update_current( Position current );
  void update_intermediate_wp( Position next );
  void virtual_update_current( Position current );
  void update(Position current, Position destination, double speed);
	void setFinalDestination(double lon, double lat);
	void setFinalDestination(int x, int y);
  void setDestination(int x, int y);
  void setDestination(double lon, double lat);
	double getBearing() const;
  map_tools::bearing_t get_named_bearing() const;
	double getBearingToDest() const;
  map_tools::bearing_t get_named_bearing_to_dest() const;
	double getSpeed();
	int getId();
  bool is_initialized();
  
	//int getDirection();//never actually set
	Position getDestination();
	Position getFinalDestination();
	Position getLocation();
  
  // Constructors
	Plane(int id=-100);
  	Plane(int newid, Position initial, Position goal );

};

void Plane::update_current( Position newcurrent )
{
  // Only if the "current" location came from a telemetry update should it affect
  // our last position
  if( !current_is_virtual )
    lastPosition.setLatLon( current.getLat(), current.getLon() );
  
	current = newcurrent; //.setLatLon( newcurrent.getLat(), newcurrent.getLon() );
  current_is_virtual = false;
  
  // finds both the plane's current bearing and its bearing to its destination,
  // saving them to the proper variables
	calculateBearings();
}

void Plane::virtual_update_current( Position newcurrent )
{
	current = newcurrent; //.setLatLon( newcurrent.getLat(), newcurrent.getLon() );
  current_is_virtual = true;
  
  // Do NOT change bearings, since this is only a "virtual" update
}

void Plane::update_intermediate_wp( Position next )
{
	destination = next;//.setLatLon( newdestination.getLat(), newdestination.getLon() );
  
  // Changing the intermediate waypoint does not change our bearing
}

void Plane::update(Position newcurrent, Position newdestination, double newspeed)
{
  if( !current_is_virtual )
    lastPosition.setLatLon( current.getLat(), current.getLon() );
  
	current = newcurrent; //.setLatLon( newcurrent.getLat(), newcurrent.getLon() );
  current_is_virtual = false;

	destination = newdestination;//.setLatLon( newdestination.getLat(), newdestination.getLon() );
  
	speed=newspeed;

  // finds both the plane's current bearing and its bearing to its destination,
  // saving them to the proper variables
	calculateBearings();
}

void Plane::setFinalDestination(double lon, double lat)
{
	finalDestination.setLatLon(lat, lon);
  
#ifdef GODDAMMIT
  assert( lon < -85.484 );
  assert( lon > -85.4915 );
  assert( lat > 32.5882 );
  assert( lat < 32.593 );
  
  double double_check_lon = finalDestination.getLon();
  double double_check_lat = finalDestination.getLat();
  assert( double_check_lon < -85.484 );
  assert( double_check_lon > -85.4915 );
  assert( double_check_lat > 32.5882 );
  assert( double_check_lat < 32.593 );
#endif
  
  calculateBearings();
}

void Plane::setDestination(double lon, double lat)
{
  destination.setLatLon( lat, lon);

#ifdef GODDAMMIT
  assert( lon < -85.484 );
  assert( lon > -85.4915 );
  assert( lat > 32.5882 );
  assert( lat < 32.593 );
  
  double double_check_lon = destination.getLon();
  double double_check_lat = destination.getLat();
  assert( double_check_lon < -85.484 );
  assert( double_check_lon > -85.4915 );
  assert( double_check_lat > 32.5882 );
  assert( double_check_lat < 32.593 );
#endif
}

void Plane::setFinalDestination(int x, int y)
{
	finalDestination.setXY(x, y);
#ifdef GODDAMMIT
  double double_check_lon = finalDestination.getLon();
  double double_check_lat = finalDestination.getLat();
  assert( double_check_lon < -85.484 );
  assert( double_check_lon > -85.4915 );
  assert( double_check_lat > 32.5882 );
  assert( double_check_lat < 32.593 );
#endif
  
  calculateBearings();
}

Position Plane::getFinalDestination()
{
#ifdef GODDAMMIT
  double double_check_lon = finalDestination.getLon();
  double double_check_lat = finalDestination.getLat();
  assert( double_check_lon < -85.484 );
  assert( double_check_lon > -85.4915 );
  assert( double_check_lat > 32.5882 );
  assert( double_check_lat < 32.593 );
#endif
	return finalDestination;
}

void Plane::setDestination(int x, int y)
{
	destination.setXY(x, y);
  
#ifdef GODDAMMIT
  double double_check_lon = destination.getLon();
  double double_check_lat = destination.getLat();
  assert( double_check_lon < -85.484 );
  assert( double_check_lon > -85.4915 );
  assert( double_check_lat > 32.5882 );
  assert( double_check_lat < 32.593 );
#endif
}

double Plane::getBearing() const
{
	return bearing;
}

map_tools::bearing_t Plane::get_named_bearing() const
{
  return map_tools::name_bearing( bearing );
}

double Plane::getBearingToDest() const
{
	return bearingToDest;
}

map_tools::bearing_t Plane::get_named_bearing_to_dest() const
{
  return map_tools::name_bearing( bearingToDest );
}

void Plane::calculateBearings()
{
  // NOTE: the map_tools::calculateBearing() fn takes lat and long in DEGREES
  double lat1 = current.getLat();
  double lon1 = current.getLon();
  
	if(!(current==lastPosition))
	{
		//uses the same method that the simulator uses to find the planes bearing
		bearing = map_tools::calculateBearing( lastPosition.getLat(), 
                                           lastPosition.getLon(),
                                           lat1, lon1 );
	}
  
  bearingToDest = map_tools::calculateBearing( lat1, lon1,
                                               finalDestination.getLat(),
                                               finalDestination.getLon() );
  
#ifdef DEBUG
//  cout << "Leeeeeeeroy MmmmmmJenkins!!!" << endl;
//  cout << "Plane " << getId() << " has a bearing of " << bearing << endl;
//  printf( " Last pos lat, lon is %f, %f \n", lastPosition.getLat(), 
//         lastPosition.getLon() );
//  printf( " Lat, lon here is %f, %f \n", lat1, lon1 );
#endif
}

double Plane::getSpeed()
{
	return speed;
}

/*int Plane::getDirection()
{
	return direction;
}*/

int Plane::getId()
{
	return id;
}

bool Plane::is_initialized()
{
  if( id == -1 )
    return false;
  return true;
}


Position Plane::getLocation()
{
	return current;
}

Position Plane::getDestination()
{
	return destination;
}

Plane::Plane(int newid)
{
	id=newid;
}

Plane::Plane(int newid, Position initial, Position goal )
{
  id=newid;
  current = Position(initial);
  destination = Position(goal);
  finalDestination = Position(goal);
  lastPosition = Position(initial);
  bearing = 0;
  current_is_virtual = false;
  
  calculateBearings();
}
		
#endif
