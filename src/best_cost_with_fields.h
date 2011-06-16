//
//  best_cost_with_fields.h
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
//
// Unlike best_cost.h, this class adds a "field" around each aircraft, extending
// in the direction of the aircraft's travel. This means that when A* works with the
// grid, there is some cost associated with coming close to another aircraft.


#ifndef BC_GRID_EXP
#define BC_GRID_EXP

#define natural unsigned int
#define SQRT_2 1.41421356

#include <vector>
#include "danger_grid_with_fields.h"
#include "Position.h"
#include <math.h>
#include <cstdlib>
// used to tell the map class that it's okay to have costs greater than 1 associated
// with squares
#define LARGE_COSTS 1

using namespace std;

// The scaling factor for the "danger" rating; effectively, the cost of 
// conflicting with another aircraft
static const double map_weight = 40.0;

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

class best_cost
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
  best_cost( vector< Plane > * set_of_aircraft, double width, double height,
             double resolution, unsigned int plane_id );
   
  /**
   * The overloaded ( ) operator. Allows simple access to the cost rating of a
   * given square at a specified number of seconds in the future.
   * @param x The x location of the square in question
   * @param y The y location of the square in question
   * @param time The number of seconds in the future for which we need the danger
   *             rating.
   * @return the cost of the (estimated) best path from the square to the goal
   */
  double operator()( unsigned int x, unsigned int y, int time ) const;
  
  /**
   * Same function as ()
   */
  double get_pos(unsigned int x, unsigned int y, int time) const;
    
  /**
   * Output the map at a given time; for troubleshooting only
   * @param time The time, in seconds, whose map should be output
   */
  void dump( int time ) const;
  
  // AK: Destructor, combats memory leak issues
  ~best_cost();
  
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

best_cost::best_cost( vector< Plane > * set_of_aircraft,
                      double width, double height, double resolution, 
                      unsigned int plane_id)
{
#ifdef DEBUG
  assert( plane_id < set_of_aircraft->size() );
  assert( set_of_aircraft->size() != 0 );
  assert( set_of_aircraft->size() < 1000000000 );
  assert( resolution > EPSILON );
  assert( resolution < height && resolution < width );
  assert( height / resolution < 1000000 );
  assert( width / resolution < 1000000 );
#endif
  
  // Set up all the variables relating to the characteristics of our airspace //
  res = resolution;
  
  start.x = (*set_of_aircraft)[ plane_id ].getLocation().getX();
  start.y = (*set_of_aircraft)[ plane_id ].getLocation().getY();
  
  goal.x = (*set_of_aircraft)[ plane_id ].getFinalDestination().getX();
  goal.y = (*set_of_aircraft)[ plane_id ].getFinalDestination().getY();
  
  // The "map cost" array, a very sparse representation of our airspace which notes
  // the likelihood of encountering an aircraft at each square at each time.
  // This is consulted when calculating the best cost from a given square.
  mc = new danger_grid( set_of_aircraft, width, height, resolution );
    
  // The real meat of this class; stores the cost of the best possible path from each
  // square at each time to the goal square. Initializes each square with the 
  // following simple heuristic:
  //      cost( node n ) = mc( n ) + (weighing factor) * distance( from n to goal )
  bc = new danger_grid( mc );
  bc->calculate_distance_costs( goal.x, goal.y );
  
  n_secs = bc->get_time_in_secs();
  n_sqrs_w = bc->get_width_in_squares();
  n_sqrs_h = bc->get_height_in_squares();
  
  initialize_to_do_index();

  // Create a starting point for the minimization step
  initialize_path();
  
  // Do the real work of the class //
  minimize_cost();
}

// AK: DESTRUCTOR -- DESTROY mc, bc to release their memory
best_cost::~best_cost(){
  delete mc;
  delete bc;
}


void best_cost::initialize_path( )
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
  
  unsigned int moves_since_dag = 0;
  const bool more_x = x_moves_rem > y_moves_rem;
  unsigned int dag_to_straight_ratio;
  if( x_moves_rem == 0 )
    dag_to_straight_ratio = y_moves_rem;
  else if( y_moves_rem == 0 )
    dag_to_straight_ratio = x_moves_rem;
  else
    dag_to_straight_ratio = more_x ? x_moves_rem/y_moves_rem : y_moves_rem/x_moves_rem;
  
  double danger_adjust = ( n_sqrs_w + n_sqrs_h ) / 4;
  while( x_moves_rem > 0 || y_moves_rem > 0 ) // 1 or more moves req'd to reach goal
  {
    if( moves_since_dag >= dag_to_straight_ratio )
    {
      // make a diagonal move
      if( x_moves_rem != 0 ) // there are one or more x moves remaining, so . . .
        --x_moves_rem;      // we'll use one
      if( y_moves_rem != 0 )
        --y_moves_rem;
      moves_since_dag = 0;
    }
    else if( more_x )
    {
      if( x_moves_rem != 0 ) // there are one or more x moves remaining, so . . .
        --x_moves_rem;      // we'll use one
      moves_since_dag++;
    }
    else
    {
      if( y_moves_rem != 0 )
        --y_moves_rem;
      moves_since_dag++;
    }
    
    // Values depend on direction of movement
    int x_to_set, y_to_set;
    x_to_set = ( move_right ? start.x + x_moves_rem : start.x - x_moves_rem );
    y_to_set = ( move_up ? start.y - y_moves_rem : start.y + y_moves_rem );
    
    // Calculate starting-place heuristic for this square using the MC (danger) grid
    // and the straight-line distance to the goal
    for( int t = n_secs; t >= 0; t-- )
    { //              This isn't quite right . . . FIX ME! /////////////////////////////////////////////////////////////////////////////////////////
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
  
  danger_threshold = (*bc)( start.x, start.y, start.t );
}

void best_cost::minimize_cost()
{
  // Increase the travel cost to search a smaller area
  // 0.05 gives almost no penalty to added distance
  // 1.0 seems to work best for all ranges.
  const double travel_cost = res * 1;
  // the cost of traversing a diagonal
  const double dag_travel_cost = SQRT_2 * travel_cost;
  
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
    
    // for each neighbor j of the node i . . .
    for( vector< coord >::const_iterator j = neighbors.begin(); j != neighbors.end(); ++j )
    {         // This could be threaded ////////////////////////////////////////////////////////////////
      // The cost to check is the (current) best cost of i plus the danger cost of j
      // plus the cost of traversing the distance; note the higher travel cost for
      // squares tagged 'd' (indicating they are 'd'iagonal to the node i).
      double cost = (*bc)( i.x, i.y, i.t ) + (map_weight * (*mc)( j->x, j->y, j->t ))
                     + ( j->tag == 'd' ? dag_travel_cost : travel_cost );

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

void best_cost::initialize_to_do_index()
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

double best_cost::operator()( unsigned int x, unsigned int y, int time ) const
{
  return bc->get_danger_at( x, y, time );
}

// AK: Added so A-Star would accept
double best_cost::get_pos(unsigned int x, unsigned int y, int time) const
{
  return bc->get_danger_at(x, y, time);
}


void best_cost::dump( int time ) const
{  
  cout << endl << "Your plane begins at (" << start.x << ", " << start.y << ")" << endl;
  
  cout << "The MC grid for " << time << endl;
  mc->dump( time );
  
  cout << endl << "The BC grid for " << time << endl;
  bc->dump_big_numbers( time );
}

#endif