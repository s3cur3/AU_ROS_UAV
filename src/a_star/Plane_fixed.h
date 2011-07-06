//
// Plane.h
// Header file for the plane class for use in the Auburn REU program 2011
// By Thomas Crescenzi, with additions from Tyler Young

#ifndef PLANE 
#define PLANE

#include <iostream>
#include <math.h>

#include "Position.h"


#ifndef RADIAN_CONSTANTS
#define RADIAN_CONSTANTS
const double PI = 2*acos(0.0);// pi
const double TWO_PI = 2*PI;
const double RADtoDEGREES = 180/PI;//Conversion factor from Radians to Degrees
const double DEGREEStoRAD = PI/180;//Conversion factor from Degrees to Radians
#endif

class Plane
{
private:
  // The plane's ID number; should be unique
	int id;
  
  // The plane's current location, as might be updated through a telemetry callback
	Position current;
  
  // The plane's "next" destination; this might be a collision avoidance waypoint,
  // or it might be identical to the final destination
	Position destination;
  
  // The plane's true goal; NOT a collision avoidance waypoint
	Position finalDestination;
  
  // The plane's previous location; used to calculate its current baring
	Position lastPosition;
  
  // The bearing from the plane's current location to its FINAL destination 
  // (its goal), in degrees
	double bearingToDest;
  
  // The bearing from the plane's previous location to its current one, in degrees
	double bearing;
  
  // The plane's speed, in whatever units you want to use (we aren't using this)
	double speed;
  
  /**
   * Updates the plane's current bearing and its bearing to its goal.
   * Current bearing is calculated as the bearing from the previous location to
   * the current one, so you should only call this function when you update the
   * current location.
   */
	void calculateBearings();
  
  // Indicates whether your current position is a "virtual" position; see the
  // virtual_update_current() for more
  bool current_is_virtual;
  
public:
  /**
   * Update a plane's current position without affecting its goal. This will
   * automatically update the plane's bearing
   *
   * TODO: Take positions as pointers rather than memory-heavy objects
   * @param current The plane's location, as might be obtained through a telemetry
   *                update
   */
  void update_current( Position current );
  
  /**
   * Update a plane's waypoint on its way to the goal. This will not affect the
   * plane's bearing.
   *
   * TODO: Take positions as pointers rather than memory-heavy objects
   * @param next The plane's intermediate waypoint, possibly a collision avoidance
   *             waypoint
   */
  void update_intermediate_wp( Position next );
  
  /**
   * It's probably a bad idea to use this function. It changes what is stored as the
   * plane's current location without affecting the plane's bearing or bearing to 
   * destination. This might be useful if you wanted other aircraft to "see" this
   * plane as being somewhere else.
   *
   * TODO: Take positions as pointers rather than memory-heavy objects
   *
   * Note that this DOES set the Boolean flag current_is_virtual so that you can
   * always tell whether a plane is ACTUALLY where it says it is.
   * @param virtual_current The position to store as 
   */
  void virtual_update_current( Position virtual_current );
  
  /**
   * Updates a plane's current location, its given intermediate waypoint, and its 
   * speed. The plane will automatically calculate its bearing as the angle between
   * its previous, real (i.e., non-virtual) position and the new current location.
   *
   * TODO: Take positions as pointers rather than memory-heavy objects
   *
   * @param current The plane's current location, as might be obtained through a 
   *                telemetry update
   * @param destination The plane's commanded "next" location; might be the same
   *                    as its final goal, or it might be an intermediate waypoint
   *                    for collision avoidance
   * @param speed The plane's speed (you could use ground speed, indicated airspeed,
   *              or true air speed... we don't actually use it for anything!)
   */
  void update(Position current, Position destination, double speed);
  
  /**
   * Sets the plane's final destination (i.e., its goal) to a given 
   * latitude-longitude
   * 
   * TODO: Swap lat and lon parameter order for the sake of consistency!
   * @param lon The longitude coordinate of the goal
   * @param lat The latitude coordinate of the goal
   */
	void setFinalDestination(double lon, double lat);
  
  /**
   * Sets the plane's final destination (i.e., its goal) to a given (x, y) in your 
   * grid space.
   * @param x The x coordinate of the goal
   * @param y The y coordinate of the goal
   */
	void setFinalDestination(int x, int y);
  
  /**
   * Sets the plane's next destination (i.e., its intermediate waypoint, such as
   * a collision avoidance waypoint) to a given latitude-longitude
   * 
   * TODO: Swap lat and lon parameter order for the sake of consistency!
   * @param lon The longitude coordinate of the "next" location
   * @param lat The latitude coordinate of the "next" location
   */
  void setDestination(double lon, double lat);
  
  /**
   * Sets the plane's next destination (i.e., its intermediate waypoint, such as
   * a collision avoidance waypoint) to a given (x, y) in your grid space.
   * @param x The x coordinate of the "next" location
   * @param y The y coordinate of the "next" location
   */
  void setDestination(int x, int y);
  
  /**
   * @return the plane's current bearing, calculated as the bearing from its previous
   *         location to its current location, with any intervening "virtual" 
   *         position updates ignored
   */
	double getBearing() const;
  
  /**
   * @return a discretized version of the current bearing--this is N, NE, E, &c.
   */
  map_tools::bearing_t get_named_bearing() const;
  
  /**
   * @return the bearing from the plane's current location to its FINAL destination
   *         (i.e., its goal)
   */
	double getBearingToDest() const;
  
  /**
   * @return a discretized version of the bearing to the destination--this is N, NE, E, &c.
   */
  map_tools::bearing_t get_named_bearing_to_dest() const;
  
  /**
   * @return the plane's stored speed. At the moment, this isn't used for anything.
   */
	double getSpeed();
  
  /**
   * @return the plane's ID number--if you're using this right, this number will be
   *         unique in your airspace
   */
	int getId();
  
  /**
   * @return TRUE if this plane was not created with the default, dummy constructor;
   *         FALSE otherwise
   */
  bool is_initialized();
  
  /**
   * TODO: Return a pointer instead of the whole, memory-heavy object
   * 
   * @return The Position object representing plane's NEXT location (may be an
   *         intermediate, collision-avoidance waypoint or may be its final goal)
   */
	Position getDestination();
	
  /**
   * TODO: Return a pointer instead of the whole, memory-heavy object
   * 
   * @return The Position object representing plane's final location (its goal)
   */
  Position getFinalDestination();
  
  /**
   * TODO: Return a pointer instead of the whole, memory-heavy object
   * 
   * @return The Position object representing plane's current location
   */
	Position getLocation();
  
  /**
   * The default constructor for a plane object. Since a plane needs so much 
   * information to actually be useful, you should probably never use this.
   * @param id The plane's unique ID, which defaults to -100
   */
	Plane(int id=-100);
  
  /**
   * The only constructor you should probably use.
   *
   * TODO: Take positions as pointers rather than memory-heavy objects
   * @param newid The unique ID number of the plane
   * @param initial The Position object representing the plane's current location
   * @param goal The Position object representing the plane's final goal
   */
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
  
  // Find both the plane's current bearing and its bearing to its destination,
  // saving them to the proper variables
	calculateBearings();
}

void Plane::virtual_update_current( Position virtual_current )
{
	current = virtual_current;
  current_is_virtual = true;
  
  // Note: We do not change bearings, since this is only a "virtual" update
}

void Plane::update_intermediate_wp( Position next )
{
	destination = next;
  
  // NOTE: Changing the intermediate waypoint does not change our bearing
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
  
  calculateBearings();
}

void Plane::setDestination(double lon, double lat)
{
  destination.setLatLon( lat, lon);
}

void Plane::setFinalDestination(int x, int y)
{
	finalDestination.setXY(x, y);
  
  calculateBearings();
}

Position Plane::getFinalDestination()
{
	return finalDestination;
}

void Plane::setDestination(int x, int y)
{
	destination.setXY(x, y);
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
  
  // If our current location isn't the same as our previous location . . .
	if( !(current==lastPosition) )
	{
		//uses the same method that the simulator uses to find the planes bearing
		bearing = map_tools::calculateBearing( lastPosition.getLat(), 
                                           lastPosition.getLon(),
                                           lat1, lon1 );
	}
  
  bearingToDest = map_tools::calculateBearing( lat1, lon1,
                                               finalDestination.getLat(),
                                               finalDestination.getLon() );
}

double Plane::getSpeed()
{
	return speed;
}

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
