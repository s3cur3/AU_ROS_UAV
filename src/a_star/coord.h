//
//  coord.h
//  AU_UAV_ROS
//
//  Created by Tyler Young on 6/27/11.
//
// A simplified position class

#ifndef COORD
#define COORD

struct coord
{
  unsigned int x;
  unsigned int y;
  unsigned int t;
  char tag;
  
  coord()
  {
    x = 0;
    y = 0;
    t = 0;
  }
  
  coord( unsigned int start_x, unsigned int start_y )
  {
    x = start_x;
    y = start_y;
    t = 0;
    tag = NULL;
  }
  
  coord( unsigned int start_x, unsigned int start_y, unsigned int time )
  {
    x = start_x;
    y = start_y;
    t = time;
    tag = NULL;
  }
  
  coord( unsigned int start_x, unsigned int start_y, unsigned int time, char label )
  {
    x = start_x;
    y = start_y;
    t = time;
    tag = label;
  }
};
#endif