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
	double bearingToDest;
	double bearing;
	double speed;
	int direction;//the direction the plane is facing, in the grid, calcuated from the bearing
	//direction is a number from 0 to 7
	//0 is north and the number increases to teh left
	//so 1 is northwest, 2 is west, 3 is southwest and so forth
	void calculateBearing();
public:
	void update(Position current, Position destination, double bearing, double speed);
	void setFinalDestination(double lon, double lat);
	void setFinalDestination(int x, int y);
  	void setDestination(int x, int y);
	double getBearing();
	double getBearingToDest();
	double getSpeed();
	int getId();
	//int getDirection();//never actually set
	Position getDestination();
	Position getFinalDestination();
	Position getLocation();
  
  // Constructors
	Plane(int id=0);
  	Plane(int newid, Position initial, Position goal );

};

void Plane::update(Position newcurrent, Position newdestination, double bare, double newspeed)
{
	lastPosition.setLon(current.getLon());
	lastPosition.setLat(current.getLat());

	current.setLon(newcurrent.getLon());
  	current.setLat(newcurrent.getLat());

	destination.setLon(newdestination.getLon());
  	destination.setLat(newdestination.getLat());

	bearingToDest=bare;
	speed=newspeed;

	calculateBearing();//finds the planes current bearing and places it in bearing

	/*if((bearing<22.5&&bearing>=0)||(bearing>337.5&&bearing<=360))
		direction=0;//north
	else if(bearing>=22.5&&bearing<=67.5)
		direction=1;//north west
	else if(bearing >67.5&&bearing<112.5)
		direction=2;//west
	else if(bearing >=112.5&&bearing<=157.5)
		direction=3;//southwest
	else if(bearing>157.5&&bearing<202.5)
		direction=4;//south
	else if(bearing>=202.5&&bearing<=247.5)
		direction=5;//southeast
	else if(bearing>247.5&&bearing<292.5)
		direction=6;//east
	else if(bearing>=292.5&&bearing<=337.5)
		direction=7;//northeast*///could be useful for refrence later
}

void Plane::setFinalDestination(double lon, double lat)
{
	finalDestination.setLon(lon);
	finalDestination.setLat(lat);
}

void Plane::setFinalDestination(int x, int y)
{
	finalDestination.setXY(x, y);
}

Position Plane::getFinalDestination()
{
	return finalDestination;
}

void Plane::setDestination(int x, int y)
{
	destination.setXY(x, y);
}

double Plane::getBearing()
{
	return bearing;
}

void Plane::calculateBearing()
{
	if(!(current==lastPosition))
	{
		//uses the same method that the simulator uses to find the planes bearing
		double lon0=lastPosition.getLon()*DEGREEStoRAD;
		double lat0=lastPosition.getLat()*DEGREEStoRAD;
		double lon1=current.getLon()*DEGREEStoRAD;
		double lat1=current.getLat()*DEGREEStoRAD;
	
		double deltalon=lon1-lon0;
		// double deltalat=lat1-lat0; // don't need this!
	
		double y = sin(deltalon)*cos(lat1);
		double x = cos(lat0)*sin(lat1) - sin(lat0)*cos(lat1)*cos(deltalon);
		bearing = atan2(y, x)*RADtoDEGREES;
	}
	else
	bearing=0;
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
}

//locationIn takes a time,in seconds, 
//returns the location where the plane will be at that time, assuming a straight line
//the default time is 1 so a call of locationIn() will return its next location
/*Position Plane::locationIn(double seconds)
{
    // TY *thinks* this is right . . .
	Position moving( current.getWidth(), current.getHeight(),
                     current.getUpperLeftLongitude(), 
                     current.getUpperLeftLatitude(), current.getLon(),
                     current.getLat(), current.getX(),current.getY());
	int y=moving.getY();
	int x=moving.getX();
	for(int i=0; i<seconds; i++)
	{
		switch(direction)
		{
			case 0://headed north
				if(y-1>-1)//make sure it is not about to move off of the grid
				moving.setY(y-1);
				
				else
				return moving;//return it at the edge of the grid
				
				break;
				
			case 1://headed northwest
				if(y-1>-1&&x-1>-1)
				{moving.setY(y-1);moving.setX(x-1);}
				
				else
				return moving;
				
				break;
			
			case 2://headed west
				if(x-1>-1)
				moving.setX(x-1);
				
				else
				return moving;
				
				break;
			
			case 3://headed southwest
				if(y+1<32&&x-1>-1)
				{moving.setY(y+1);moving.setX(x-1);}
				
				else
				return moving;
				
				break;
			
			case 4://headed south
				if(y+1<32)
				moving.setY(y+1);
				
				else
				return moving;
				
				break;
			
			case 5://headed southeast
				if(y+1<32&&x+1<32)
				{moving.setY(y+1);moving.setX(x+1);}
				
				else
				return moving;
				
				break;
				
			case 6://headed east
				if(x+1<32)
				moving.setX(x+1);
				
				else
				return moving;
				
				break;
			
			case 7://headed northease
				if(x+1<32&&y-1>-1)
				{moving.setY(y-1);moving.setX(x+1);}
				
				else
				return moving;
				
				break;
			
			default:
				cout<<"something has gone wrong";
		}
		x=moving.getX();
		y=moving.getY();
	}
	return moving;
}*/
		
#endif
