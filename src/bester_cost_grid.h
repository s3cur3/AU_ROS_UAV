//
//  bester_cost_grid.h
//  AU_UAV_ROS
//
//  Created by Tyler Young on 5/31/11.
//
// This class takes the "danger" rating associated with each square in a map of an
// airspace and adds to it the cost of traversing the distance left to the goal.
// This effectively comprises a heuristic for the best one can hope to do when 
// attempting to reach a goal from a given square. The heuristic is borrowed from 
// P. N. Stiles and I. S. Glickstein's "Highly parallelizable route planner based
// on cellular automata algorithms."
//
// This should provide a much more precise (yet still consistently underestimating)
// heuristic than the heuristic:
//     weighing factor * danger at node n + straight line distance from n to goal,
// as used in the best_cost_grid class.

#ifndef BEST_COST_GRID
#define BEST_COST_GRID

#define natural unsigned int

#include <vector>
#include "danger_grid.h"
#include "Position.h"
#include <math.h>

// used to tell the map class that it's okay to have costs greater than 1 associated
// with squares
#define LARGE_COSTS 1

using namespace std;

struct coord
{
  unsigned int x;
  unsigned int y;
  
  coord()
  {
    x = 0;
    y = 0;
  }
};

class bester_cost_grid
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
  bester_cost_grid( vector< Plane > & set_of_aircraft, double width, double height,
                    double resolution, unsigned int plane_id );
  
  /**
   * Output the map at a given time; for troubleshooting only
   * @param time The time, in seconds, whose map should be output
   */
  void dump( int time ) const;
  
private:
  void initial_path();
  void cost_min();
  
  danger_grid * mc;
  danger_grid * bc;
  vector< coord > to_do;
  Position the_goal;
};

bester_cost_grid::bester_cost_grid( vector< Plane > & set_of_aircraft, double width,
                                    double height, double resolution, 
                                    unsigned int plane_id)
{
  assert( plane_id < set_of_aircraft.size() );

  the_goal = set_of_aircraft[ plane_id ].getFinalDestination();
  
  mc = new danger_grid( set_of_aircraft, width, height, resolution );
  
  vector< Plane > no_planes;
  bc = new danger_grid( no_planes, width, height, resolution );
  
  // Initialization of BC grid to infinity
  for( natural crnt_x = 0; crnt_x < bc->get_width_in_squares(); crnt_x++ )
  {
    for( natural crnt_y = 0; crnt_y < bc->get_height_in_squares(); crnt_y++ )
    {
      for( natural crnt_t = 0; crnt_t < bc->get_time_in_secs(); crnt_t++ )
      {
        bc->set_danger_at( crnt_x, crnt_y, crnt_t,
                                numeric_limits<double>::max( ) );
      }
    }
  }
}

void bester_cost_grid::initial_path( )
{
  
}

void bester_cost_grid::cost_min()
{
  
}

void bester_cost_grid::dump( int time ) const
{
  bc->dump( time );
}

#endif