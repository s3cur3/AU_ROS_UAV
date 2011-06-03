//
//  best_cost.h
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
// heuristic than the baseline heuristic:
//     weighing factor * danger at node n + straight line distance from n to goal,
// as used in the old best_cost_grid class.

#ifndef BC_GRID_EXP
#define BC_GRID_EXP

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

class bc_grid_experiment
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
  bc_grid_experiment( vector< Plane > & set_of_aircraft, double width, double height,
                      double resolution, unsigned int plane_id );
   
  /**
   * Output the map at a given time; for troubleshooting only
   * @param time The time, in seconds, whose map should be output
   */
  void dump( int time ) const;
  
private:  
  /**
   * Calculate the minimum cost for each square of the grid at all times; loop until
   * no more squares may update the cost of their neighbors.
   */
  void minimize_cost();
  
  /**
   * Sets up the in_to_do 3-D vector array to be of the same size as the mc and bc
   * grids. This is the in_to_do variable, which is used to decide (in constant time)
   * whether a particular (x, y, t) coordinate is in the to-do list.
   */
  void initialize_to_do_index();
  
  danger_grid * mc; // the map cost (MC) grid (a.k.a., the danger grid)
  danger_grid * bc; // the best cost grid; the heart of this class
    
  // A sort of indexer for the to-do list. Allows a constant-time check to see if
  // a given (x, y, t) coordinate is in the to-do list.
  vector< vector< vector< bool > > > in_to_do;
  // stores a list of the nodes which could modify the best cost of their neighbors
  vector< coord > to_do;
  
  coord goal; // the x and y coordinates of the goal
  coord start;

  double res;                          // resolution of the danger grid in meters
  int n_secs;                         // number of seconds in the danger grid
  unsigned int n_sqrs_h;             // height of the danger grid in squares
  unsigned int n_sqrs_w;            // width of the danger grid in squares
  
  // This gets set to the plane's initial square's intial danger rating. If a
  // potential update to a square's best cost is ABOVE this value, the update isn't
  // worth making, so it is ignored.
  double danger_threshold;
};

bc_grid_experiment::bc_grid_experiment( vector< Plane > & set_of_aircraft,
                                      double width, double height, double resolution, 
                                      unsigned int plane_id)
{
  assert( plane_id < set_of_aircraft.size() );

  // Set up all the variables relating to the characteristics of our airspace //
  res = resolution;
  
  start.x = set_of_aircraft[ plane_id ].getLocation().getX();
  start.y = set_of_aircraft[ plane_id ].getLocation().getY();
  
  goal.x = set_of_aircraft[ plane_id ].getFinalDestination().getX();
  goal.y = set_of_aircraft[ plane_id ].getFinalDestination().getY();
  
  n_secs = bc->get_time_in_secs();
  n_sqrs_w = bc->get_width_in_squares();
  n_sqrs_h = bc->get_height_in_squares();
  
  // The "map cost" array, a very sparse representation of our airspace which notes
  // the likelihood of encountering an aircraft at each square at each time.
  // This is consulted when calculating the best cost from a given square.
  mc = new danger_grid( set_of_aircraft, width, height, resolution );
  
  // The real meat of this class; stores the cost of the best possible path from each
  // square at each time to the goal square. Initializes each square with the 
  // following simple heuristic:
  //      cost( node n ) = mc( n ) + (weighing factor) * distance( from n to goal )
  bc = new danger_grid( set_of_aircraft, width, height, resolution, goal.x, goal.y );
  
  initialize_to_do_index();

  // Do the real work of the class //
  minimize_cost();
}


void bc_grid_experiment::minimize_cost()
{
  // Increase the travel cost to search a smaller area
  // 0.05 gives almost no penalty to added distance
  // 1.0 seems to work best for all ranges.
  const double travel_cost = res * 1;
  
  while( to_do.size() != 0 )
  {
    // Note that variable names i and j come from
    // "Highly parallelizable route planner based on cellular automata algorithms"
    coord i = to_do.back();
    to_do.pop_back();
    in_to_do[ i.x ][ i.y ][ i.t ] = false;
    
    // The squares whose best cost could change based on the value of i
    vector< coord > neighbors;
    
    // Unlike the situation in "Highly parallelizable . . . ", we can't get away with
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
    
    // The scaling factor for the "danger" rating; effectively, the cost of 
    // conflicting with another aircraft
    const double map_weight = 20.0;
    
    // for each neighbor j of the node i . . .
    for( vector< coord >::const_iterator j = neighbors.begin(); j != neighbors.end(); ++j )
    {
      // The cost to check is the (current) best cost of i plus the danger cost of j
      // plus the cost of traversing the distance; note the higher travel cost for
      // squares tagged 'd' (indicating they are 'd'iagonals to the node i).
      double cost = (*bc)( i.x, i.y, i.t ) + (map_weight * (*mc)( j->x, j->y, j->t ))
                     + ( j->tag == 'd' ? travel_cost * SQRT_2 : travel_cost );

      if( cost < (*bc)( j->x, j->y, j->t ) && cost < danger_threshold )
      {
        (*bc).set_danger_at( j->x, j->y, j->t, cost );
                
        // If the neighbor isn't in the to-do list . . .
        if( !in_to_do[ j->x ][ j->y ][ j->t ] )
        {
          // . . . add it . . .
          to_do.push_back( *j );
          // . . . and make a note in the to-do list's index.
          in_to_do[ j->x ][ j->y ][ j->t ] = true;
        }
      }
    }
  }
}

void bc_grid_experiment::initialize_to_do_index()
{  
  in_to_do.resize( n_sqrs_w );
  for( unsigned int x = 0; x < n_sqrs_w; x++ )
  {
    in_to_do[ x ].resize( n_sqrs_h );
    for( unsigned int y = 0; y < n_sqrs_h; y++ )
    {
      // Initialize everything to false; intially, no squares are present in the
      // to-do list.
      in_to_do[ x ][ y ].resize( n_secs, false );
    }
  }
}

void bc_grid_experiment::dump( int time ) const
{  
  cout << endl << "Your plane begins at (" << start.x << ", " << start.y << ")" << endl;
  
  cout << "The MC grid for " << time << endl;
  mc->dump_big_numbers( time );
  
  cout << endl << "The BC grid for " << time << endl;
  bc->dump( time );
}

#endif