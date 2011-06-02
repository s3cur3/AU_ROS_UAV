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

#ifndef BESTER_COST_GRID
#define BESTER_COST_GRID

#define natural unsigned int
#define SQRT_2 1.41421356

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
  /**
   * Calculates a straight path from the starting position to the goal,
   * and calculates the best cost for the grid squares along that path as the 
   * starting point for the cost minimization step.
   */
  void initialize_path();
  
  /**
   * Calculate the minimum cost for each square of the grid at all times; loop until
   * no more squares may update the cost of their neighbors.
   */
  void minimize_cost();
  
  /**
   * Search the to-do list for a given (x,y) coordinate. If it is found, return a
   * pointer to it.
   * @param find_me The (x,y) coordinates which are sought
   * @return True if the coordinate is in the to-do list, false otherwise
   */
  bool is_in_to_do( coord find_me );
  
  /**
   * Sets up the in_to_do 3-D vector array to be of the same size as the mc and bc
   * grids
   */
  void initialize_to_do_index();
  
  danger_grid * mc; // the map cost (MC) grid (a.k.a., the danger grid)
  danger_grid * bc; // the best cost grid; the heart of this class
  vector< coord > to_do;
  vector< vector< vector< bool > > > in_to_do;
  
  coord goal;
  coord start;
  
  int n_secs; // number of seconds in the danger grid
  unsigned int n_sqrs_h; // height of the danger grid in squares
  unsigned int n_sqrs_w; // width of the danger grid in squares
  double res;
};

bester_cost_grid::bester_cost_grid( vector< Plane > & set_of_aircraft, double width,
                                    double height, double resolution, 
                                    unsigned int plane_id)
{
  assert( plane_id < set_of_aircraft.size() );

  res = resolution;
  
  start.x = set_of_aircraft[ plane_id ].getLocation().getX();
  start.y = set_of_aircraft[ plane_id ].getLocation().getY();
  
  goal.x = set_of_aircraft[ plane_id ].getFinalDestination().getX();
  goal.y = set_of_aircraft[ plane_id ].getFinalDestination().getY();
  
  
  mc = new danger_grid( set_of_aircraft, width, height, resolution );
  
  vector< Plane > no_planes;
  bc = new danger_grid( no_planes, width, height, resolution );
  
  // Initialization of BC grid to infinity
  n_secs = bc->get_time_in_secs();
  n_sqrs_w = bc->get_width_in_squares();
  n_sqrs_h = bc->get_height_in_squares();
  for( natural crnt_x = 0; crnt_x < n_sqrs_w; crnt_x++ )
  {
    for( natural crnt_y = 0; crnt_y < n_sqrs_h; crnt_y++ )
    {
      for( natural crnt_t = 0; crnt_t < n_secs; crnt_t++ )
      {
        bc->set_danger_at( crnt_x, crnt_y, crnt_t,
                                numeric_limits<double>::max( ) );
      }
    }
  }
  
//  cout << "Start: (" << start.x << ", " << start.y << ")" << endl;
//  cout << "Goal: (" << goal.x << ", " << goal.y << ")" << endl;
  
  initialize_to_do_index();
  initialize_path();
  minimize_cost();
}

void bester_cost_grid::initialize_path( )
{
  // Initialize the best cost at the goal to 0 for all times
  for( int t = 0; t < n_secs; t++ )
    bc->set_danger_at( goal.x, goal.y, t, 0.0);
  
  // Find the squares which lie in the straight line between BC_start and BC_goal
  int dx = goal.x - start.x;
  int dy = start.y - goal.y; // since origin is at top left
  int x_moves_rem = abs( dx ); // remaining x moves until we reach goal's x
  int y_moves_rem = abs( dy );
  bool move_up = ( dy > 0 ? true : false );
  bool move_right = ( dx > 0 ? true : false );
  
  int x_to_set, y_to_set;
  double danger_adjust = ( n_sqrs_w + n_sqrs_h ) / 4;
  while( x_moves_rem > 0 || y_moves_rem > 0 ) // 1 or more moves req'd to reach goal
  {
    if( x_moves_rem != 0 ) // there are one or more x moves remaining, so . . .
      --x_moves_rem;      // we'll use one
    if( y_moves_rem != 0 )
      --y_moves_rem;
    
    // Values depend on direction of movement
    x_to_set = ( move_right ? start.x + x_moves_rem : start.x - x_moves_rem );
    y_to_set = ( move_up ? start.y - y_moves_rem : start.y + y_moves_rem );
    
    // Calculate starting-place heuristic for this square using the MC (danger) grid
    // and the straight-line distance to the goal
    for( unsigned int t = 0; t <= n_secs; t++ )
    {
      double danger = (*mc)( x_to_set, y_to_set, t );
      double dist = sqrt( (x_to_set - goal.x)*(x_to_set - goal.x) + 
                         (y_to_set - goal.y)*(y_to_set - goal.y) );
      bc->set_danger_at( x_to_set, y_to_set, t,
                         danger_adjust * danger + dist );
      // This (x, y, t) coordinate can change the best cost of its neighbors; check later
      to_do.push_back( coord( x_to_set, y_to_set, t ) );
      in_to_do[ x_to_set ][ y_to_set ][ t ] = true;
    }
  }

  //bc->dump_big_numbers( 0 );
  //bc->dump_big_numbers( 3 );
}

void bester_cost_grid::minimize_cost()
{
  // increase travel cost to search a smaller area
  // 0.05 gives almost no penalty to added distance
  // 0.1 looks to be about right when you're close to the goal, but it causes an
  // explosion with random seed 20 in the tester class
  const double travel_cost = res * 1;
  
  while( to_do.size() != 0 )
  {
    // Note that variable names i and j come from "Highly parallelizable . . ."
    coord i = to_do.back();
    to_do.pop_back();
    in_to_do[ i.x ][ i.y ][ i.t ] = false;
    
    vector< coord > neighbors;
    // Per "Highly parallelizable . . . ", we may be able to get away with
    // considering only the up, down, left, and right neighbors (ignoring diagonals)
    if( i.x + 1 < n_sqrs_w ) // Only add neighbor if it is a legal square
    {
      neighbors.push_back( coord( i.x + 1, i.y, i.t ) ); // right neighbor
      if( i.y + 1 < n_sqrs_h )
       neighbors.push_back( coord( i.x + 1, i.y + 1, i.t, 'd' ) ); // down-right neighbor
       if( i.y > 0 )
       neighbors.push_back( coord( i.x + 1, i.y - 1, i.t, 'd' ) ); // up-right neighbor
    }
    if( i.x > 0 )
    {
      neighbors.push_back( coord( i.x - 1, i.y, i.t ) ); // left neighbor
      if( i.y + 1 < n_sqrs_h )
       neighbors.push_back( coord( i.x - 1, i.y + 1, i.t, 'd' ) ); // down-left neighbor
       if( i.y > 0 )
       neighbors.push_back( coord( i.x - 1, i.y - 1, i.t, 'd' ) ); // down-right neighbor
    }
    if( i.y + 1 < n_sqrs_h )
      neighbors.push_back( coord( i.x, i.y + 1, i.t ) ); // down neighbor
    if( i.y > 0 )
      neighbors.push_back( coord( i.x, i.y - 1, i.t ) ); // up neighbor
    
    // the scale factor for the "danger" rating;
    // the cost of conflicting with another aircraft
    const double map_weight = 20.0;
    
    // for each neighbor j of i . . .
    for( vector< coord >::const_iterator j = neighbors.begin(); j != neighbors.end(); ++j )
    {
      double cost = (*bc)( i.x, i.y, i.t ) + map_weight * (*mc)( j->x, j->y, j->t ) +
                    ( j->tag == 'd' ? travel_cost * SQRT_2 : travel_cost ); // higher travel cost for diagonals
      // IS THIS THE CORRECT TIME TO USE ON THE START COMPARISON?? //////////////////////////////////////////////////////
      if( cost < (*bc)( j->x, j->y, j->t ) && cost < (*bc)( start.x, start.y, start.t ) )
      {
        (*bc).set_danger_at( j->x, j->y, j->t, cost );
                
        // If the neighbor isn't in the to-do list . . .
        if( !in_to_do[ j->x ][ j->y ][ j->t ] )
        {
          // . . . add it.
          to_do.push_back( *j );
          in_to_do[ j->x ][ j->y ][ j->t ] = true;
        }
      }
    }
  }
}

bool bester_cost_grid::is_in_to_do( coord find_me )
{
  for( vector< coord >::iterator it = to_do.begin(); it != to_do.end(); ++it )
  {
    if( it->x == find_me.x && it->y == find_me.y && it->t == find_me.t )
    {
      return true;
    }
  }
  return false;
}

void bester_cost_grid::initialize_to_do_index()
{  
  in_to_do.resize( n_sqrs_w );
  for( unsigned int x = 0; x < n_sqrs_w; x++ )
  {
    in_to_do[ x ].resize( n_sqrs_h );
    for( unsigned int y = 0; y < n_sqrs_h; y++ )
    {
      in_to_do[ x ][ y ].resize( n_secs, false );
    }
  }
}

void bester_cost_grid::dump( int time ) const
{  
  cout << endl << "Your plane begins at (" << start.x << ", " << start.y << ")" << endl;
  
  cout << "The MC grid for " << time << endl;
  mc->dump_big_numbers( time );
  
  cout << endl << "The BC grid for " << time << endl;
  bc->dump_big_numbers( time );
}

#endif