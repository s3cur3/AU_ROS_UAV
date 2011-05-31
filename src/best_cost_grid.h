//
//  best_cost_grid.h
//  AU_UAV_ROS
//
//  Created by Tyler Young on 5/30/11.
//
// This class takes the "danger" rating associated with each square in a map of an
// airspace and adds to it the cost of traversing the distance left to the goal.
// This effectively comprises a heuristic for the best one can hope to do when 
// attempting to reach a goal from a given square. The heuristic is:
//     danger associated with a square + straight line dist. from square to goal,
// with a weighting factor applied to the danger rating to prevent the algorithm from
// taking very risky squares at the promise of short distance.

#ifndef BEST_COST_GRID
#define BEST_COST_GRID

#define natural unsigned int

#include <vector>
#include "danger_grid.h"
#include "Position.h"
#include <math.h>

using namespace std;

class best_cost_grid
{
public:
  /**
   * The constructor for the best cost grid. It will set up a danger grid using the 
   * parameters given, then automatically calculate the best cost for each square.
   *
   * Note that the width, height, and resolution may be in any units, but the units
   * must be consistent across all measurements.
   * @param set_of_aircraft A vector array containing the aircraft that need to
   * be considered
   * @param width The width of the airspace (our x dimension)
   * @param height The height of the airspace (our y dimension)
   * @param resolution The resolution to be used in the map
   * @param plane_id The index of the plane for which we are generating the best 
   *                 cost grid
   */
  best_cost_grid( vector< Plane > & set_of_aircraft, double width, double height,
                  double resolution, unsigned int plane_id);
  
  /**
   * The overloaded ( ) operator. Allows simple access to the cost of a
   * given square at a specified number of seconds in the future.
   * @param x The x location of the square in question
   * @param y The y location of the square in question
   * @param time The number of seconds in the future for which we need the danger
   * rating.
   * @return a double containing the cost of the square
   */
  double operator()( unsigned int x, unsigned int y, int time ) const;
  
  /**
   * Output the map at a given time; for troubleshooting only
   * @param time The time, in seconds, whose map should be output
   */
  void dump( int time ) const;
  
private:
  /**
   * The workhorse for the class. Modified the grid to account for distances to the
   * goal.
   * @param goal_x The x coordinate for the goal
   * @param goal_y The y coordinate for the goal
   */
  void calculate_costs( unsigned int goal_x, unsigned int goal_y );

  danger_grid * the_grid;
  Position the_goal;
  double danger_adjust; // the number by which we will multiply danger ratings
};

best_cost_grid::best_cost_grid( vector< Plane > & set_of_aircraft, double width, double height,
                  double resolution, unsigned int plane_id)
{
  assert( plane_id < set_of_aircraft.size() );
  the_grid = new danger_grid( set_of_aircraft, width, height, resolution );
  
  the_goal = set_of_aircraft[ plane_id ].getFinalDestination();
  
  danger_adjust = ( width + height ) / 4;

  calculate_costs( the_goal.getX(), the_goal.getY() );
}

void best_cost_grid::calculate_costs( unsigned int goal_x, unsigned int goal_y )
{  // This calculation clearly has room for speed improvements
  for( natural crnt_x = 0; crnt_x < the_grid->get_width_in_squares(); crnt_x++ )
  {
    for( natural crnt_y = 0; crnt_y < the_grid->get_height_in_squares(); crnt_y++ )
    {
      for( natural crnt_t = 0; crnt_t < the_grid->get_time_in_secs(); crnt_t++ )
      {
        double crnt_danger = (*the_grid)( crnt_x, crnt_y, crnt_t );
        double dist = sqrt( (crnt_x - goal_x)*(crnt_x - goal_x) + 
                            (crnt_y - goal_y)*(crnt_y - goal_y) );
        the_grid->set_danger_at( crnt_x, crnt_y, crnt_t,
                                 danger_adjust * crnt_danger + dist );
      }
    }
  }
}

double best_cost_grid::operator()( unsigned int x, unsigned int y, int time ) const
{
  return the_grid->get_danger_at( x, y, time );
}

void best_cost_grid::dump( int time ) const
{
  the_grid->dump( time );
}

#endif