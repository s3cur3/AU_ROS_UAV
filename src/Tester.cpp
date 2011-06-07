#include <cstdlib>
#include <string>
#include <math.h>
#include <vector>
#include "Position.h"
#include "Plane.h"
#include "estimate.h"
#include <time.h>

using namespace std;
const double PI = 2*acos(0.0);//PI
const double RADtoDEGREES = 180/PI;//Conversion factor from Radians to Degrees
const double DEGREEStoRAD = PI/180;//Conversion factor from Degrees to Radians
//int planesMap[];
double upperLeftLon;
	double upperLeftLat;
	double lonWidth;
	double latWidth;
	double res;
	int planeNum;	
	vector<Plane> planes();

void neighoboringAngles(double angle, double &first, double &second)
{
		if(angle>0)
	{
		if(angle<45)
			{first=0; second = 45;}
		else if(angle<90)
			{first=45; second = 90;}
		else if(angle<135)
			{first=90; second = 135;}
		else if(angle<=180)
			{first=135; second = 180;}
	}
	
	else
	{
		if(angle>-45)
			{first=0; second = -45;}
		else if(angle>-90)
			{first=-45; second = -90;}
		else if(angle>-135)
			{first=-90; second = -135;}
		else if(angle>=-180)
			{first=-135; second = -180;}
	}
}
void placeDanger(double angle, vector<estimate> &e, double closest, double other, int x, int y, double danger)
{
	if(angle>0)//to the right
	{
		if(closest==0)//north && northeast
		{
			e.push_back(estimate(x,y-1,danger));//majority in N
			e.push_back(estimate(x+1,y-1,1-danger));//remainder in NE
		}
		else if(closest == 45 && other == 0)//northeast && north
		{
			e.push_back(estimate(x+1,y-1,danger));//majority in NE
			e.push_back(estimate(x,y-1,1-danger));//remainder in N
		}
		
		else if(closest == 45)//northeast && east
		{
			e.push_back(estimate(x+1,y-1,danger));//majority in NE
			e.push_back(estimate(x+1,y,1-danger));//remainder in east
		}
		
		else if(closest == 90 && other == 45)//east && northeast
		{
			e.push_back(estimate(x+1,y,danger));//majority in east
			e.push_back(estimate(x+1,y-1,1-danger));//remainder in NE
		}
		
		else if(closest == 90)//east && southeast
		{
			e.push_back(estimate(x+1,y,danger));//majority in east
			e.push_back(estimate(x+1,y+1,1-danger));//remainder in SE
		}
			
		else if(closest == 135 && other == 90)//southeast && east
		{	
			e.push_back(estimate(x+1,y+1,danger));//majority in SE
			e.push_back(estimate(x+1,y,1-danger));//remainder in east
		}
		
		else if(closest == 135)//southeast && south
		{
			e.push_back(estimate(x+1,y+1,danger));//majority in SE
			e.push_back(estimate(x,y+1,1-danger));//remainder in south
		}
		
		else//south && southeast
		{
			e.push_back(estimate(x,y+1,danger));//majority in south
			e.push_back(estimate(x+1,y+1,1-danger));//majority in SE
		}
	}
	else//to the left
	{
			if(closest==0)//north && northwest
		{
			e.push_back(estimate(x,y-1,danger));//majority in N
			e.push_back(estimate(x-1,y-1,1-danger));//remainder in NW
		}
		else if(closest == -45 && other == 0)//northwest && north
		{
			e.push_back(estimate(x-1,y-1,danger));//majority in NW
			e.push_back(estimate(x,y-1,1-danger));//remainder in N
		}
		
		else if(closest == -45)//northwest && west
		{
			e.push_back(estimate(x-1,y-1,danger));//majority in NW
			e.push_back(estimate(x-1,y,1-danger));//remainder in west
		}
		
		else if(closest == -90 && other == -45)//west && northwest
		{
			e.push_back(estimate(x-1,y,danger));//majority in west
			e.push_back(estimate(x-1,y-1,1-danger));//remainder in NE
		}
		
		else if(closest == -90)//west && southwest
		{
			e.push_back(estimate(x-1,y,danger));//majority in west
			e.push_back(estimate(x-1,y+1,1-danger));//remainder in SW
		}
			
		else if(closest == -135 && other == -90)//southwest && west
		{	
			
			e.push_back(estimate(x-1,y+1,danger));//majority in SW
			e.push_back(estimate(x-1,y,1-danger));//remainder in west
		}
		
		else if(closest == -135)//southwest && south
		{
			e.push_back(estimate(x-1,y+1,danger));//majority in SW
			e.push_back(estimate(x,y+1,1-danger));//remainder in south
		}
		
		else//south && southwest
		{
			e.push_back(estimate(x,y+1,danger));//majority in south
			e.push_back(estimate(x-1,y+1,1-danger));//majority in SW
		}
	}
}
void dangerRecurse(estimate e, int destination[], vector<estimate> &theFuture,double epislon)
{
	int x1=e.x;
	int y1=e.y;
	int x2=destination[0];
	int y2=destination[1];
	int dest[2]={x2,y2};
	
	double xDistance=fabs((double)x2-x1) , yDistance=fabs((double)y2-y1);
	if(xDistance==0&&yDistance==0)//your there!!!!!!!(hopefully)
	{return;}
	double distance = sqrt((double)(xDistance*xDistance)+(yDistance*yDistance));
	
	//find the angle to the waypoint
	double angle=(180-RADtoDEGREES*(asin((double)xDistance/distance)));
	if(y2<y1)
			angle=(RADtoDEGREES*(asin((double)xDistance/(double)distance)));
	if((x2-x1)<0)//positive means that the plane is headed to the left aka west
		angle=(-1)*angle;//the plane goes from -180 to +180

	//find closest straight line
	double neighbors[2];
	neighoboringAngles(angle, neighbors[0], neighbors[1]);
	double closestAngle=0,otherAngle=0;
	if(fabs(angle-neighbors[0])>=fabs(angle-neighbors[1]))//distance
		{closestAngle=neighbors[1]; otherAngle=neighbors[0];}
	else
		{closestAngle=neighbors[0]; otherAngle=neighbors[1];}

	//find displacement percentage
	double danger;
	if((fabs(angle)>fabs(closestAngle))&&closestAngle!=0)
		danger=(closestAngle/angle);
	else if(closestAngle!=0)
		danger=(angle/closestAngle);
	else//i hate 0
		danger=1-(angle/otherAngle);//because you can't use 0 find the inverse of the displacement to the other angle.
	//now add the new danger to theFuture
	placeDanger(angle, theFuture, closestAngle, otherAngle, x1, y1, danger);
	if(theFuture.back().danger>epislon)
		dangerRecurse(theFuture.back(), dest, theFuture, epislon);
	theFuture.push_back(estimate(-1,-1,-1));
	dangerRecurse(theFuture[theFuture.size()-3],dest,theFuture,epislon);


}


vector< estimate > calculate_future_pos_it(/*Plane & plane*/int x1,int y1,int x2, int y2, double epislon)
{
    vector< estimate > theFuture;
	
   // Position current=plane.getLocation();
//	Position destination=plane.getDestination();
	
	//distance formula: line to destination
	//int x1=current.getX(),x2=destination.getX(),y1=current.getY(),y2=destination.getY();
	int xDistance=fabs((double)x2-x1),yDistance=fabs((double)y2-y1);
	cout<<xDistance<<yDistance;
	while((xDistance!=0)||(yDistance!=0))
	{
		double distance = sqrt((double)(xDistance*xDistance)+(yDistance*yDistance));
		//find the angle to the waypoint
		double angle=(180-RADtoDEGREES*(asin((double)xDistance/(double)distance)));
		if(y2<y1)
				angle=(RADtoDEGREES*(asin((double)xDistance/(double)distance)));
		if((x2-x1)<0)//positive means that the plane is headed to the left aka west
			angle=(-1)*angle;//the plane goes from -180 to +180

		//find closest straight line
		double neighbors[2];
		neighoboringAngles(angle, neighbors[0], neighbors[1]);
		double closestAngle=0,otherAngle=0;
		if(fabs(angle-neighbors[0])>=fabs(angle-neighbors[1]))//distance
			{closestAngle=neighbors[1]; otherAngle=neighbors[0];}
		else
			{closestAngle=neighbors[0]; otherAngle=neighbors[1];}
		
		//find displacement percentage
		double danger;
		if((fabs(angle)>fabs(closestAngle))&&closestAngle!=0)
			danger=(closestAngle/angle);
		else if(closestAngle!=0)
			danger=(angle/closestAngle);
		else//i hate 0
		danger=1-(angle/otherAngle);//because you can't use 0 find the inverse of the displacement to the other angel.
		
		//place displacement percentage in closest square and then place the remainder in the other square
		placeDanger(angle, theFuture, closestAngle, otherAngle, x1, y1, danger);
		//start the branching NOT ANY MORE THIS SHIT IS ITERATIVE!
		//int dest[2]={x2,y2};//can't pass it without a name :(
		//if(theFuture.back().danger>epislon)//not sure what to do with this guy
			//dangerRecurse(theFuture.back(), dest, theFuture,epislon);
		theFuture.push_back(estimate(-1,-1,-1));
		//dangerRecurse(theFuture[theFuture.size()-3],dest,theFuture,epislon);
		x1=theFuture[theFuture.size()-3].x;
		y1=theFuture[theFuture.size()-3].y;
		xDistance=fabs((double)x2-x1),yDistance=fabs((double)y2-y1);
		cout<<"I'm iterative bitches!"<<endl;
	}
	return theFuture;
}

vector< estimate > calculate_future_pos(/*Plane & plane*/int x1,int y1,int x2, int y2, double epislon)
{
    // Hi Thomas!  :)
    
    //estimate test( 0, 0, 0 );
    vector< estimate > theFuture;
	
	
   // Position current=plane.getLocation();
//	Position destination=plane.getDestination();
	
	//distance formula: line to destination
	//int x1=current.getX(),x2=destination.getX(),y1=current.getY(),y2=destination.getY();
	int xDistance=fabs((double)x2-x1),yDistance=fabs((double)y2-y1);
	
	double distance = sqrt((double)(xDistance*xDistance)+(yDistance*yDistance));
	//find the angle to the waypoint
	double angle=(180-RADtoDEGREES*(asin((double)xDistance/(double)distance)));
	if(y2<y1)
			angle=(RADtoDEGREES*(asin((double)xDistance/(double)distance)));
	if((x2-x1)<0)//positive means that the plane is headed to the left aka west
		angle=(-1)*angle;//the plane goes from -180 to +180

	//find closest straight line
	double neighbors[2];
	neighoboringAngles(angle, neighbors[0], neighbors[1]);
	double closestAngle=0,otherAngle=0;
	if(fabs(angle-neighbors[0])>=fabs(angle-neighbors[1]))//distance
		{closestAngle=neighbors[1]; otherAngle=neighbors[0];}
	else
		{closestAngle=neighbors[0]; otherAngle=neighbors[1];}
		
	//find displacement percentage
	double danger;
	if((fabs(angle)>fabs(closestAngle))&&closestAngle!=0)
		danger=(closestAngle/angle);
	else if(closestAngle!=0)
		danger=(angle/closestAngle);
	else//i hate 0
		danger=1-(angle/otherAngle);//because you can't use 0 find the inverse of the displacement to the other angel.
		
	//place displacement percentage in closest square and then place the remainder in the other square
		placeDanger(angle, theFuture, closestAngle, otherAngle, x1, y1, danger);
		//start the branching
		int dest[2]={x2,y2};//can't pass it without a name :(
		if(theFuture.back().danger>epislon)
			dangerRecurse(theFuture.back(), dest, theFuture,epislon);
		theFuture.push_back(estimate(-1,-1,-1));
		dangerRecurse(theFuture[theFuture.size()-3],dest,theFuture,epislon);
		
	return theFuture;
}



int main() 
{
	//code for testing estimates
	//vector<estimate> theFuture=calculate_future_pos(18,2,5,15,.4);
	
	/*const double upper_left_longitude = 85.490363;
	const double upper_left_latitude = 32.592425;
	const double width_in_degrees_longitude = 0.005002;
	const double height_in_degrees_latitude = 0.003808;
	srand(21);
	double lon = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
	double lat = ( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;

	double lon2 = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
	double lat2 = ( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;

	Position pos = Position(upper_left_longitude, upper_left_latitude, width_in_degrees_longitude, height_in_degrees_latitude, lon, lat, 10);
	Position pos2 = Position(upper_left_longitude, upper_left_latitude,width_in_degrees_longitude,height_in_degrees_latitude,lon2,lat2,10);
	Plane p1=Plane(0);
	Plane p2=Plane(1);
	Plane p3=Plane(2);
	Plane p4=Plane(3);
	Plane p5=Plane(4);

	p1.update(pos,pos2,120,25);
	
	 lon = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
	 lat = ( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;
	 lon2 = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
	 lat2 = ( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;
	
  pos = Position(upper_left_longitude, upper_left_latitude, width_in_degrees_longitude, height_in_degrees_latitude, lon, lat, 10);
	pos2 = Position(upper_left_longitude, upper_left_latitude,width_in_degrees_longitude,height_in_degrees_latitude,lon2,lat2,10);
	p2.update(pos,pos2,120,25);
	 lon = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
	 lat = ( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;
	 lon2 = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
	 lat2 = ( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;
  
	pos = Position(upper_left_longitude, upper_left_latitude, width_in_degrees_longitude, height_in_degrees_latitude, lon, lat, 10);
	pos2 = Position(upper_left_longitude, upper_left_latitude,width_in_degrees_longitude,height_in_degrees_latitude,lon2,lat2,10);
  
	p3.update(pos,pos2,
					120,25);
	lon = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
	 lat = ( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;
	 lon2 = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
	 lat2 = ( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;
  
	pos = Position(upper_left_longitude, upper_left_latitude, width_in_degrees_longitude, height_in_degrees_latitude, lon, lat, 10);
	pos2 = Position(upper_left_longitude, upper_left_latitude,width_in_degrees_longitude,height_in_degrees_latitude,lon2,lat2,10);
  
	p4.update(pos,pos2,
					120,25);
	lon = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
	 lat = ( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;
	 lon2 = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
	 lat2 = ( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;
	
  pos = Position(upper_left_longitude, upper_left_latitude, width_in_degrees_longitude, height_in_degrees_latitude, lon, lat, 10);
	pos2 = Position(upper_left_longitude, upper_left_latitude,width_in_degrees_longitude,height_in_degrees_latitude,lon2,lat2,10);
  
	p5.update(pos,pos2,
					120,25);
  
	vector<estimate> theFuture=calculate_future_pos(p1,.3);
	vector<estimate> theFuture2=calculate_future_pos(p2,.3);
  vector<estimate> theFuture3=calculate_future_pos(p3,.3);
	vector<estimate> theFuture4=calculate_future_pos(p4,.3);
  vector<estimate> theFuture5=calculate_future_pos(p5,.3);*/
	//time_t start;
	//start=time(NULL);
	vector<estimate> theFuture=calculate_future_pos/*_it*/(0,0,35,24,.3);
	//printf("Time:%0.7f",start-time(NULL));
	/*int grid[46][42];
	for(int i=0; i<42; i++)
		for(int j=0; j<46; j++)
			grid[j][i]=0;
	int seconds=1;
	for(int i=0; i<theFuture.size(); i++)
	{
		if(theFuture[i].danger!=-1)
		{
			//cout<<"Time:"<<seconds<<" X:"<<theFuture[i].x<<" Y:"<<theFuture[i].y<<" Danger:"<<theFuture[i].danger<<endl;
			if(grid[theFuture[i].x][theFuture[i].y]==0)
			grid[theFuture[i].x][theFuture[i].y]=(int)(10*theFuture[i].danger);
		}
		else
			seconds++;
	}
  
	/*for(int i=0; i<theFuture2.size(); i++)
	{
		if(theFuture2[i].danger!=-1)
		{
			//cout<<"Time:"<<seconds<<" X:"<<theFuture[i].x<<" Y:"<<theFuture[i].y<<" Danger:"<<theFuture[i].danger<<endl;
			if(grid[theFuture2[i].x][theFuture2[i].y]==0)
			grid[theFuture2[i].x][theFuture2[i].y]=(int)(10*theFuture2[i].danger);
		}
		else
			seconds++;
	}
  for(int i=0; i<theFuture3.size(); i++)
	{
		if(theFuture3[i].danger!=-1)
		{
			//cout<<"Time:"<<seconds<<" X:"<<theFuture[i].x<<" Y:"<<theFuture[i].y<<" Danger:"<<theFuture[i].danger<<endl;
			if(grid[theFuture3[i].x][theFuture3[i].y]==0)
        grid[theFuture3[i].x][theFuture3[i].y]=(int)(10*theFuture3[i].danger);
		}
		else
			seconds++;
	}
  
  for(int i=0; i<theFuture4.size(); i++)
	{
		if(theFuture4[i].danger!=-1)
		{
			//cout<<"Time:"<<seconds<<" X:"<<theFuture[i].x<<" Y:"<<theFuture[i].y<<" Danger:"<<theFuture[i].danger<<endl;
			if(grid[theFuture4[i].x][theFuture4[i].y]==0)
        grid[theFuture4[i].x][theFuture4[i].y]=(int)(10*theFuture4[i].danger);
		}
		else
			seconds++;
	}
  
  for(int i=0; i<theFuture5.size(); i++)
	{
		if(theFuture5[i].danger!=-1)
		{
			//cout<<"Time:"<<seconds<<" X:"<<theFuture[i].x<<" Y:"<<theFuture[i].y<<" Danger:"<<theFuture[i].danger<<endl;
			if(grid[theFuture5[i].x][theFuture5[i].y]==0)
        grid[theFuture5[i].x][theFuture5[i].y]=(int)(10*theFuture5[i].danger);
		}
		else
			seconds++;
	}
  */
	/*for(int i=0; i<42; i++)
	{	for (int j=0; j<46; j++)
			printf( "%1d ", grid[j][i]);
	cout<<endl;}
	cout<<endl;

	//testing of position class

	/*double lon, lat;
	int x, y;
	double resolution = 10; // meters per grid square
	const double upper_left_longitude = 85.490363;
	const double upper_left_latitude = 32.592425;
	const double width_in_degrees_longitude = 0.005002;
	const double height_in_degrees_latitude = 0.003808;
	int random=(rand()%100)+1;
	//cout<<random;
	for(int i=0; i<100; i++)
	{
	 double lon = ( (double)( rand() % 5002 ) / 1000000 ) + upper_left_longitude;
		double lat = ( (double)( rand() % 3808 ) / 1000000 ) + upper_left_latitude;
	Position p = Position(upper_left_longitude,upper_left_latitude,width_in_degrees_longitude,height_in_degrees_latitude,lon,lat,10);
	//cout<<"Lon:"<<p.getLon()<<" Lat:"<<p.getLat()<<" X:"<<p.getX()<<" Y:"<<p.getY()<<endl;
	if(p.getX()>46||p.getY()>46||p.getX()<0||p.getY()<0)
		cout<<"Lon:"<<p.getLon()<<" Lat:"<<p.getLat()<<" X:"<<p.getX()<<" Y:"<<p.getY()<<endl;
	}
	cout<<"hey";*/




	//testing of logic in ros cross through
	/*
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

	if(planesMap[msg->planeID]==-1)
		{
			cout<<"hey";
		}
		*/
//	cin.get();
}