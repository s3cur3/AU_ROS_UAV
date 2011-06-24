//
//  estimate.h
//  AU_UAV_ROS
//
//  Created by Tyler Young on 5/23/11.
//  Ruined by misuse by Thomas Crescenzi on 5/25/11.(see note)
// This struct is used to coordinate between a plane and the danger grid
// when the danger grid is being created.
// The danger grid asks where the plane will be at a given time, and the plane
// returns a vector of estimates corresponding to the plane's probability of being
// in a given location.


#ifndef ESTIMATE
#define ESTIMATE

using namespace std;

struct estimate
{
    int x;
    int y;
    double danger;
    
    estimate( int x_pos, int y_pos, double the_danger )
    {
        x = x_pos;
        y = y_pos;
        danger = the_danger;
    }

    estimate( )
    {
        danger = 0.0;
    }
};

#endif