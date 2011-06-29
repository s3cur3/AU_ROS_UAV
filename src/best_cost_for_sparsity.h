//
//  best_cost_final.h
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
// Additionally, we've fixed a (rather important) error in the compuation of the
// initial path.


#define DEBUG
#define DEBUG_BC

#ifndef BC_GRID_EXP
#define BC_GRID_EXP

#define natural unsigned int
#define SQRT_2 1.41421356

#include <vector>
#include "danger_grid_with_fields.h"
#include "Position.h"
#include <math.h>
#include <cstdlib>
#include "write_to_log.h"
#include "map_tools.h"
#include "Plane_fixed.h"
#include "coord.h"
#include <climits>

// used to tell the map class that it's okay to have costs greater than 1 associated
// with squares
#define LARGE_COSTS 1

#ifndef EPSILON
#define EPSILON 0.000001
#endif

using namespace std;
using namespace map_tools;

// The scaling factor for the "danger" rating; effectively, the cost of 
// conflicting with another aircraft
static const double map_weight = 1.0;

// The number of squares that we "branch" to during the path initialization
static const int branch_width = 10;

const string log_path = "/mnt/hgfs/Dropbox/school/Auburn/Code/AU_UAV_stack/AU_UAV_ROS/log/";

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
   * Gets the distance from the specifyed (x, y) position to the goal of this
   * BC grid's owner
   * @param x_pos The x coordinate of the position in question
   * @param y_pos The y coordinate of the position in question
   * @return the straight-line distance from (x, y) to the goal
   */
  double get_dist_cost_at( unsigned int x_pos, unsigned int y_pos ) const;
  
  /**
   * Same function as overloaded operator ()
   */
  double get_pos(unsigned int x, unsigned int y, int time) const;
  
  /**
   * @return the width of the best cost grid in squares
   */
  unsigned int get_width_in_squares() const;
  /**
   * @return the height of the best cost grid in squares
   */
  unsigned int get_height_in_squares() const;
  
  /**
   * Output the map at a given time; for troubleshooting only
   * @param time The time, in seconds, whose map should be output
   */
  void dump( int time ) const;
  
  /**
   * Output the map to a CSV file, whose path is specified in the map class.
   * For troubleshooting only
   * @param time The time, in seconds, whose map should be output
   */
  void dump_csv( int time ) const;
  void dump_csv( int time, string prefix, string name ) const;
  
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
  
  /**
   * Calculates the Euclidean distance from the starting square to each of the
   * squares in the list, and returns a reference to the one that is closest.
   * 
   * @param starting_sqr The square to which all other squares' distance is calculated
   * @param list_of_sqrs A reference to a vector containing some number of squares
   *                     whose distance to the starting square will be calculated
   * @param bearing The bearing from the goal to the plane
   * @return The address of the coordinate in the list_of_sqrs with minimum distance
   *         from the starting square
   */
  coord get_branching_sqr( const coord starting_sqr, 
                           vector< coord > * list_of_sqrs,
                           bearing_t bearing );
  
  /**
   * Returns an array of the squares which will update their neighbors 
   * @param branching_sqr The goal, or the square closest to the plane which was
   *                      previously updated
   * @param branching_to_plane The (named) bearing from the plane to the branching
   *                           square
   * @param t the time we're working with here
   * @param out_updating_cells The vector to be modified which will contain the 
   *                           squares that can update their neighbors
   */
  void find_updating_sqrs( const coord branching_sqr, 
                           const bearing_t branching_to_plane,
                           const int t, vector< coord > * out_updating_cells );
  
  // The "owner" of this BC grid, for whom we will calculate distance costs &c.
  Plane * owner;
  
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
  
  owner = &( (*set_of_aircraft)[ plane_id ] );
  
#ifdef DEBUG
  create_log( "Creating BC grid for plane " + to_string( plane_id ), log_path );
  add_to_log( "Plane's starting loc: (" + to_string( start.x ) + ", " + to_string( start.y ) + ")", log_path );
  add_to_log( "Plane's goal loc:     (" + to_string( goal.x ) + ", " + to_string( goal.y ) + ")", log_path );
#endif
  
  // The "map cost" array, a very sparse representation of our airspace which notes
  // the likelihood of encountering an aircraft at each square at each time.
  // This is consulted when calculating the best cost from a given square.
  mc = new danger_grid( set_of_aircraft, width, height, resolution, plane_id );
    
  // The real meat of this class; stores the cost of the best possible path from each
  // square at each time to the goal square. Initializes each square with the 
  // maximum double value ("infinity")
  bc = new danger_grid( mc, "infinity" );
  bc->calculate_distance_costs( goal.x, goal.y, 1.0 );
  
  n_secs = bc->get_time_in_secs();
  n_sqrs_w = bc->get_width_in_squares();
  n_sqrs_h = bc->get_height_in_squares();
  
#ifdef DEBUG
  assert( mc->get_time_in_secs() < 100000 );
  assert( mc->get_width_in_squares() < 100000 ); 
  assert( mc->get_height_in_squares() < 100000 ); 
  
  assert( n_secs < 100000 );    // sizes larger than this can't be searched in
  assert( n_sqrs_h < 100000 ); // anything resembling a reasonable amount of time
  assert( n_sqrs_w < 100000 ); 
  
  assert( goal.x < n_sqrs_w );
  assert( goal.y < n_sqrs_h );
  assert( start.x < n_sqrs_w );
  assert( start.y < n_sqrs_h );
#endif
  
  initialize_to_do_index();
  
  // Create a starting point for the minimization step
  initialize_path();
  
  // Do the real work of the class //
  minimize_cost();
}

// AK: DESTRUCTOR -- DESTROY mc, bc to release their memory
best_cost::~best_cost()
{
  delete mc;
  delete bc;
}


void best_cost::initialize_path( )
{
  // Increase the travel cost to search a smaller area
  const double travel_cost = 1;
  // the cost of traversing a diagonal
  const double dag_travel_cost = SQRT_2 * travel_cost;
  
  // Initialize the best cost at the goal to the map cost for all times
  for( int t = 0; t < n_secs; t++ )
    bc->set_danger_at( goal.x, goal.y, t, (*mc)(goal.x, goal.y, t) );
  
  // The squares which will update their neighbors
  vector< coord > updating_cells;
  
#ifdef DEBUG_BC
  cout << "Goal: (" << goal.x << ", " << goal.y << ")" << endl;
  cout << "Starting pos: (" << start.x << ", " << start.y << ")" << endl;
  cout << "Plane says its bearing to the dest is " << (*owner).getBearingToDest() << endl;
  cout << "We would name that " << bearing_to_string( name_bearing( (*owner).getBearingToDest() ) )<< endl;
#endif

  // For each time in prediction space . . .
  for( int t = 0; t < n_secs; t++ )
  {
    bool start_has_been_updated = false;
    
    // Bearing from the branching square (here, the goal) to the plane is the
    // opposite of the bearing from the plane to the goal
    bearing_t branching_to_plane = reverse_bearing( (*owner).get_named_bearing_to_dest() );  
    
    cout << "                                        Setting sqrs close to start " << endl;
    // The first time we run, the squares that will update their neighbors' true best
    // cost is simply the branch_width squares adjacent to the goal which are closest
    // to the plane
    find_updating_sqrs( goal, branching_to_plane, t, &updating_cells );
    for( vector< coord >::iterator init_sqr = updating_cells.begin(); 
        init_sqr != updating_cells.end(); ++init_sqr )
    {
      bc->set_danger_at( (*init_sqr).x, (*init_sqr).y, t, 
                         get_euclidean_dist_between( goal.x, goal.y, (*init_sqr).x, (*init_sqr).y ) + 
                         map_weight * ((*mc)( (*init_sqr).x, (*init_sqr).y, t ) + 
                                       (*mc)( goal.x, goal.y, t )) );
    }
    
    cout << "First of updating cells is (" << updating_cells.front().x << ", " << updating_cells.front().y << ")" << endl;
    
    coord * branching_sqr;
    
    // The squares we are changing in each round of the while loop
    vector< coord > cells_to_change;
    
    cout << "                                        Entering while " << endl;
    while( !start_has_been_updated && updating_cells.size() > 0 )
    {
#ifdef DEBUG
      assert( (int)updating_cells.size() <= branch_width );
      assert( updating_cells.size() > 0 );
#endif
      
      branching_sqr = new coord( get_branching_sqr( start, &updating_cells, branching_to_plane ) );
      cout << "Branching sqr is (" << (*branching_sqr).x << ", " << (*branching_sqr).y << ")" << endl;
      
      branching_to_plane = name_bearing( calculate_euclidean_bearing(
        (*branching_sqr).x, (*branching_sqr).y, start.x, start.y ) );

      find_updating_sqrs( (*branching_sqr), branching_to_plane, t, &cells_to_change );
#ifdef DEBUG
      assert( (int)cells_to_change.size() <= branch_width );
      assert( cells_to_change.size() > 0 );
#endif
      
      // For each square that we're going to change . . .
      for( vector< coord >::iterator changing_sqr = cells_to_change.begin();
           changing_sqr != cells_to_change.end(); ++changing_sqr )
      {
        // Cost of this square is the cost from the minimum-cost reachable square to
        // the goal + the cost of this square + the distance from this square to the
        // min reachable square
        double danger_of_changing_sqr = (*mc).get_danger_at( (*changing_sqr).x, (*changing_sqr).y, t );
        
        double lowest_cost_so_far = numeric_limits<double>::max( );
        
        vector< coord > adj_cells;
        adj_cells.reserve( 8 );
        if( (*changing_sqr).x + 1 < n_sqrs_w )
        {
          adj_cells.push_back( coord((*changing_sqr).x + 1, (*changing_sqr).y) );
          
          if( (*changing_sqr).y + 1 < n_sqrs_h )
            adj_cells.push_back( coord((*changing_sqr).x + 1, (*changing_sqr).y + 1, 'd') );
          
          if( (*changing_sqr).y > 0 )
            adj_cells.push_back( coord((*changing_sqr).x + 1, (*changing_sqr).y - 1, 'd') );
        }
        
        if( (*changing_sqr).y + 1 < n_sqrs_h )
          adj_cells.push_back( coord((*changing_sqr).x, (*changing_sqr).y + 1) );
        
        if( (*changing_sqr).y > 0 )
          adj_cells.push_back( coord((*changing_sqr).x, (*changing_sqr).y - 1) );
        
        if( (*changing_sqr).x > 0 )
        {
          adj_cells.push_back( coord((*changing_sqr).x - 1, (*changing_sqr).y) );
          
          if( (*changing_sqr).y + 1 < n_sqrs_h )
            adj_cells.push_back( coord((*changing_sqr).x - 1, (*changing_sqr).y + 1, 'd') );
          
          if( (*changing_sqr).y > 0 )
            adj_cells.push_back( coord((*changing_sqr).x - 1, (*changing_sqr).y - 1, 'd') );
        }
        
        // Each of the adjacent cells is potentially the min-cost reachable sqr
        for( vector< coord >::iterator p_min = adj_cells.begin();
            p_min != adj_cells.end(); ++p_min )
        {
          double distance = ( (*p_min).tag == 'd' ? dag_travel_cost : travel_cost );
          
          double bc_with_p_min = (*bc)( (*p_min).x, (*p_min).y, t ) + distance +
            map_weight * danger_of_changing_sqr;
          
          // If this is the lowest cost we've seen so far, and the BC at this 
          // square doesn't incorporate a plane danger . . .
          if( bc_with_p_min < lowest_cost_so_far )
          {
            cout << "Changed bc at (" << (*changing_sqr).x << ", " <<
            (*changing_sqr).y << ") to " << bc_with_p_min << " because bc with pmin is " << bc_with_p_min << endl;
            
            bc->set_danger_at( (*changing_sqr).x, (*changing_sqr).y, t, 
                              bc_with_p_min  );
            
            lowest_cost_so_far = bc_with_p_min;
          }
          
          /*
          else
          {            
            cout << "Didn't change bc at (" << (*changing_sqr).x << ", " <<
            (*changing_sqr).y << ") to " << bc_with_p_min << " because bc with pmin is " << bc_with_p_min << endl;
            
          }
          
           */
        } // end for each square in updating_cells
      
        if( (*changing_sqr).x == start.x && (*changing_sqr).y == start.y )
        {
          start_has_been_updated = true;
#ifdef DEBUG_BC
          cout << "BC at start is " << (*bc)( start.x, start.y, t ) << endl << endl;
#endif
        } // end if start has been updated
        
#ifdef DEBUG_BC
        cout << "    Changed sqr: (" << (*changing_sqr).x << ", " << (*changing_sqr).y << ")" << endl;
#endif
        
        to_do.push_back( (*changing_sqr) );
        in_to_do[ (*changing_sqr).x ][ (*changing_sqr).y ][ t ] = true;
      } // end for each changing_square
      
      // The squares we just changed are now going to update their neighbors
      updating_cells = cells_to_change;
      
      cells_to_change.clear();
      
      delete branching_sqr;
      branching_sqr = NULL;
    } // end while start has not been updated and there are updating cells
    
    if( branching_sqr != NULL )
    {
      delete branching_sqr;
      branching_sqr = NULL;
    }
    
    updating_cells.clear();
    
    cout << "                                      Finished a time step " << endl;
  } // end for each time step
  
  danger_threshold = (*bc)( start.x, start.y, start.t );
}

void best_cost::minimize_cost()
{
  // Increase the travel cost to search a smaller area
  const double travel_cost = 1.0;
  // the cost of traversing a diagonal
  const double dag_travel_cost = SQRT_2 * 1.0;
  
  cout << "                                      Beginning min cost" << endl;
  while( to_do.size() != 0 )
  {
    // Note that variable names i and j come from
    // "Highly parallelizable route planner based on cellular automata algorithms"
    coord i = to_do.back();
    to_do.pop_back();
    in_to_do[ i.x ][ i.y ][ i.t ] = false;
    
    // The squares whose best cost could change based on the value of i
    vector< coord > neighbors;
    
    // Since we're using square buffers around planes, we can get away with 
    // considering only the up, down, left, and right neighbors (ignoring diagonals),
    // as in "Highly parallelizable . . . "
    if( i.x + 1 < n_sqrs_w ) // Only add neighbor if it is a legal square
    {
      neighbors.push_back( coord( i.x + 1, i.y, i.t ) ); // right neighbor
    }
    
    if( i.x > 0 )
    {
      neighbors.push_back( coord( i.x - 1, i.y, i.t ) ); // left neighbor
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
#ifdef DEBUG
        if( j->x == goal.x && j->y == goal.y )
          cout << "Why are you trying to change the goal in the min cost fn??" << endl;
#endif
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
    } // end for each neighbor j of i
  } // end while to_do.size != 0
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

double best_cost::get_dist_cost_at( unsigned int x_pos, unsigned int y_pos ) const
{
  return bc->get_dist_cost_at(x_pos, y_pos);
}


unsigned int best_cost::get_width_in_squares() const
{
  return (*bc).get_width_in_squares();
}

unsigned int best_cost::get_height_in_squares() const
{
  return (*bc).get_height_in_squares();
}


coord best_cost::get_branching_sqr( const coord starting_sqr, 
                                    vector< coord > * list_of_sqrs,
                                    bearing_t bearing )
{
#ifdef DEBUG
  if( (*list_of_sqrs).size() == 0 )
    assert( false );
#endif
  
  if( bearing == N || bearing == S || bearing == E || bearing == W )
  { // return the closest square
    double min_d = 10000000;
    double d_to_crnt;
    coord * closest_sqr;
    
    for( vector< coord >::iterator crnt_sqr = (*list_of_sqrs).begin(); 
        crnt_sqr != (*list_of_sqrs).end(); ++crnt_sqr )
    {
      d_to_crnt = sqrt( ((*crnt_sqr).x - start.x)*((*crnt_sqr).x - start.x) + 
                       ((*crnt_sqr).y - start.y)*((*crnt_sqr).y - start.y) );
      if( d_to_crnt < min_d )
      {
        min_d = d_to_crnt;
        closest_sqr = &(*crnt_sqr);
      }
    }
    
#ifdef DEBUG_BC
    cout << "Given the choices ";
    for( vector< coord >::iterator crnt_sqr = (*list_of_sqrs).begin(); 
        crnt_sqr != (*list_of_sqrs).end(); ++crnt_sqr )
    {
      cout << "(" << (*crnt_sqr).x << ", " << (*crnt_sqr).y << "), ";
    }
    cout << endl << "...we chose ("<< (*closest_sqr).x << ", " << (*closest_sqr).y << ") " << endl;
#endif
    
#ifdef DEBUG
    assert( (*closest_sqr).x < n_sqrs_w );
    assert( (*closest_sqr).y < n_sqrs_h );
#endif
    
    return (*closest_sqr);
  }
  
  vector< double > d_to_sqr;
  d_to_sqr.resize( (*list_of_sqrs).size() );
  
  coord * branching_sqr;
  
  // find distances
  for( natural crnt_sqr = 0; crnt_sqr < (*list_of_sqrs).size(); crnt_sqr++ )
  {
    d_to_sqr[ crnt_sqr ] = 
    sqrt( ((*list_of_sqrs)[ crnt_sqr ].x - start.x)*((*list_of_sqrs)[ crnt_sqr ].x - start.x) + 
          ((*list_of_sqrs)[ crnt_sqr ].y - start.y)*((*list_of_sqrs)[ crnt_sqr ].y - start.y) );
  }
  
  vector< double > original = d_to_sqr;
  
  sort( d_to_sqr.begin(), d_to_sqr.end() );
  
  // find which of the squares had the middlest distance
  for( natural crnt_sqr = 0; crnt_sqr < (*list_of_sqrs).size(); crnt_sqr++ )
  {
    if( d_to_sqr[ (*list_of_sqrs).size() / 2 ] - original[ crnt_sqr ] < EPSILON && 
       d_to_sqr[ (*list_of_sqrs).size() / 2 ] - original[ crnt_sqr ] > -EPSILON )
    {
      branching_sqr = &( (*list_of_sqrs)[ crnt_sqr ] );
#ifdef DEBUG_BC
      cout << "Given the following " << (*list_of_sqrs).size() << " choices " << endl;
      for( vector< coord >::iterator crnt_sqr = (*list_of_sqrs).begin(); 
          crnt_sqr != (*list_of_sqrs).end(); ++crnt_sqr )
      {
        cout << "(" << (*crnt_sqr).x << ", " << (*crnt_sqr).y << "), ";
      }
      cout << endl << "...we chose ("<< (*branching_sqr).x << ", " << (*branching_sqr).y << ") " << endl;
#endif
      
#ifdef DEBUG
      assert( (*branching_sqr).x < n_sqrs_w );
      assert( (*branching_sqr).y < n_sqrs_h );
#endif
      
      return (*branching_sqr);
    }
  }
  assert( branching_sqr != NULL );
}

void best_cost::find_updating_sqrs( const coord branching_sqr,
                                    const bearing_t branching_to_plane,
                                    const int t, vector< coord > * out_updating_cells )
{  
#ifdef DEBUG
  assert( (*out_updating_cells).size() == 0 );
#endif
  
  (*out_updating_cells).reserve( branch_width ); // save a bit of time later, maybe
  
  if( branch_width == 3 )
  {
    if( branching_to_plane == N )
    {
      if( branching_sqr.y > 0 ) // safe to add squares that are up 1
      {
        if( branching_sqr.x > 0 ) // safe to add squares that are 1 left
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y - 1, t) ); // up and left
        
        if( branching_sqr.x + 1 < n_sqrs_w ) // safe to add squares that are 1 right
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y - 1, t) ); // up and right
        
        (*out_updating_cells).push_back( coord( branching_sqr.x, branching_sqr.y - 1, t) ); // straight up
      }
    }
    else if( branching_to_plane == NE )
    {
      if( branching_sqr.x + 1 < n_sqrs_w ) // safe to add squares that are 1 right
        (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y, t) ); // straight right
      
      if( branching_sqr.y > 0 ) // safe to add squares that are up 1
      {
        (*out_updating_cells).push_back( coord( branching_sqr.x, branching_sqr.y - 1, t) ); // straight up
        
        if( branching_sqr.x + 1 < n_sqrs_w ) // safe to add squares that are 1 right
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y - 1, t) ); // up and right
      }
    }
    else if( branching_to_plane == E )
    {
      if( branching_sqr.x + 1 < n_sqrs_w ) // safe to add squares that are 1 right
      {
        (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y, t) ); // straight right
        
        if( branching_sqr.y > 0 ) // safe to add squares that are up 1
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y - 1, t) ); // up and right
        
        if( branching_sqr.y + 1 < n_sqrs_h ) // safe to add squares that are down 1
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y + 1, t) ); // down and right
      }
    }
    else if( branching_to_plane == SE )
    {
      if( branching_sqr.x + 1 < n_sqrs_w ) // safe to add squares that are 1 right
      {
        (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y, t) ); // straight right
        
        if( branching_sqr.y + 1 < n_sqrs_h ) // safe to add squares that are down 1
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y + 1, t) ); // down and right
      }
      
      if( branching_sqr.y + 1 < n_sqrs_h ) // safe to add squares that are down 1
        (*out_updating_cells).push_back( coord(branching_sqr.x, branching_sqr.y + 1, t) ); // straight down
    }
    else if( branching_to_plane == S )
    {
      if( branching_sqr.y + 1 < n_sqrs_h ) // safe to add squares that are down 1
      {
        if( branching_sqr.x > 0 ) // safe to add squares that are 1 left
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y + 1, t) ); // down and left
        
        if( branching_sqr.x + 1 < n_sqrs_w ) // safe to add squares that are 1 right
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y + 1, t) ); // down and right
        
        (*out_updating_cells).push_back( coord( branching_sqr.x, branching_sqr.y + 1, t) ); // straight down
      }
    }
    else if( branching_to_plane == SW )
    {
      if( branching_sqr.x > 0 ) // safe to add squares that are 1 left
      {
        (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y, t) ); // straight left
        
        if( branching_sqr.y + 1 < n_sqrs_h ) // safe to add squares that are down 1
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y + 1, t) ); // down and left
      }
      
      if( branching_sqr.y + 1 < n_sqrs_h ) // safe to add squares that are down 1
        (*out_updating_cells).push_back( coord(branching_sqr.x, branching_sqr.y + 1, t) ); // straight down
    }
    else if( branching_to_plane == W )
    {
      if( branching_sqr.x > 0 ) // safe to add squares that are 1 left
      {
        (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y, t) ); // straight left
        
        if( branching_sqr.y > 0 ) // safe to add squares that are up 1
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y - 1, t) ); // up and left
        
        if( branching_sqr.y + 1 < n_sqrs_h ) // safe to add squares that are down 1
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y + 1, t) ); // down and left
      }
    }
    else if( branching_to_plane == NW )
    {
      if( branching_sqr.x > 0 ) // safe to add squares that are 1 left
      {
        (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y, t) ); // straight left
        
        if( branching_sqr.y > 0 ) // safe to add squares that are up 1
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y - 1, t) ); // up and left
      }
      
      if( branching_sqr.y > 0 ) // safe to add squares that are up 1
        (*out_updating_cells).push_back( coord(branching_sqr.x, branching_sqr.y - 1, t) ); // straight up
    }
  }
  else if( branch_width == 10 )
  {
    if( branching_to_plane == N )
    {
      if( branching_sqr.y > 0 ) // safe to add squares that are up 1
      {
        if( branching_sqr.x > 4 ) // safe to add squares that are 5 left
          (*out_updating_cells).push_back( coord(branching_sqr.x - 5, branching_sqr.y - 1, t) );
        
        if( branching_sqr.x > 3 ) // safe to add squares that are 4 left
          (*out_updating_cells).push_back( coord(branching_sqr.x - 4, branching_sqr.y - 1, t) );
        
        if( branching_sqr.x > 2 ) // safe to add squares that are 3 left
          (*out_updating_cells).push_back( coord(branching_sqr.x - 3, branching_sqr.y - 1, t) );
        
        if( branching_sqr.x > 1 ) // safe to add squares that are 2 left
          (*out_updating_cells).push_back( coord(branching_sqr.x - 2, branching_sqr.y - 1, t) );
        
        if( branching_sqr.x > 0 ) // safe to add squares that are 1 left
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y - 1, t) ); // up and left
        
        (*out_updating_cells).push_back( coord( branching_sqr.x, branching_sqr.y - 1, t) ); // straight up
        
        if( branching_sqr.x + 1 < n_sqrs_w ) // safe to add squares that are 1 right
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y - 1, t) ); // up and right
        
        if( branching_sqr.x + 2 < n_sqrs_w ) // safe to add squares that are 2 right
          (*out_updating_cells).push_back( coord(branching_sqr.x + 2, branching_sqr.y - 1, t) );
        
        if( branching_sqr.x + 3 < n_sqrs_w ) // safe to add squares that are 3 right
          (*out_updating_cells).push_back( coord(branching_sqr.x + 3, branching_sqr.y - 1, t) );
        
        if( branching_sqr.x + 4 < n_sqrs_w ) // safe to add squares that are 4 right
          (*out_updating_cells).push_back( coord(branching_sqr.x + 4, branching_sqr.y - 1, t) );
      }
    }
    else if( branching_to_plane == NE )
    {
      if( branching_sqr.x > 0 )
      {
        if( branching_sqr.x > 1 )
        {
          if( branching_sqr.y > 2 )
            (*out_updating_cells).push_back( coord(branching_sqr.x - 2, branching_sqr.y - 3, t) );
        }
        if( branching_sqr.y > 1 )
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y - 2, t) );
      }
      
      if( branching_sqr.y > 0 )
      {
        if( branching_sqr.y > 1 )
          (*out_updating_cells).push_back( coord(  branching_sqr.x  , branching_sqr.y - 2, t) );
        
        (*out_updating_cells).push_back( coord(  branching_sqr.x  , branching_sqr.y - 1, t) );
      }
      
      if( branching_sqr.x + 1 < n_sqrs_w )
      {
        if( branching_sqr.y > 0 )
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y - 1, t) );
        
        if( branching_sqr.y + 1 < n_sqrs_h )
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1,   branching_sqr.y  , t) );
      }
      
      
      if( branching_sqr.x + 2 < n_sqrs_w )
      {
        (*out_updating_cells).push_back( coord(branching_sqr.x + 2,   branching_sqr.y  , t) );
        
        if( branching_sqr.y + 1 < n_sqrs_h )
        {
          (*out_updating_cells).push_back( coord(branching_sqr.x + 2, branching_sqr.y + 1, t) );
        }
      }
      
      if( branching_sqr.x + 3 < n_sqrs_w )
      {
        if( branching_sqr.y + 1 < n_sqrs_h )
        {
          (*out_updating_cells).push_back( coord(branching_sqr.x + 3, branching_sqr.y + 1, t) );
        
          if( branching_sqr.y + 2 < n_sqrs_h )
            (*out_updating_cells).push_back( coord(branching_sqr.x + 3, branching_sqr.y + 2, t) );
        }
      }
    } // end if NE
    else if( branching_to_plane == E )
    {
      if( branching_sqr.x + 1 < n_sqrs_w ) // safe to add squares that are 1 left
      {
        if( branching_sqr.y > 0 )
        {
          if( branching_sqr.y > 1 )
          {
            if( branching_sqr.y > 2 )
            {
              if( branching_sqr.y > 3 )
              {
                if( branching_sqr.y > 4 )
                  (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y - 5, t) );
                
                (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y - 4, t) );
              }
              
              (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y - 3, t) );
            }
            
            (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y - 2, t) );
          }
          
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y - 1, t) );
        }
        
        (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y, t) );
        
        if( branching_sqr.y + 1 < n_sqrs_h )
        {
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y + 1, t) );
          
          if( branching_sqr.y + 2 < n_sqrs_h )
          {
            (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y + 2, t) );
            
            if( branching_sqr.y + 3 < n_sqrs_h )
            {
              (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y + 3, t) );
              
              if( branching_sqr.y + 4 < n_sqrs_h )
                (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y + 4, t) );
            }
          }
        }
      } // end if x > 0
    } // end if E
    else if( branching_to_plane == SE )
    {
      if( branching_sqr.x > 0 )
      {
        if( branching_sqr.x > 1 )
        {
          if( branching_sqr.y + 3 < n_sqrs_h )
            (*out_updating_cells).push_back( coord(branching_sqr.x - 2, branching_sqr.y + 3, t) );
        }
        if( branching_sqr.y + 2 < n_sqrs_h )
        {
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y + 2, t) );
          
          if( branching_sqr.y + 3 < n_sqrs_h )
          {
            (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y + 3, t) );
          }
        }
      }
      
      if( branching_sqr.y + 1 < n_sqrs_h )
      {
        if( branching_sqr.y + 2 < n_sqrs_h )
          (*out_updating_cells).push_back( coord(branching_sqr.x, branching_sqr.y + 2, t) );
        
        (*out_updating_cells).push_back( coord(branching_sqr.x, branching_sqr.y + 1, t) );
      }
      
      if( branching_sqr.x + 1 < n_sqrs_w )
      {
        (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y, t) );
        
        if( branching_sqr.y + 1 < n_sqrs_h )
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y + 1, t) );
      }
      
      
      if( branching_sqr.x + 2 < n_sqrs_w )
      {
        (*out_updating_cells).push_back( coord(branching_sqr.x + 2,   branching_sqr.y  , t) );
        
        if( branching_sqr.y > 0 )
        {
          (*out_updating_cells).push_back( coord(branching_sqr.x + 2, branching_sqr.y - 1, t) );
        }
      }
      
      if( branching_sqr.x + 3 < n_sqrs_w )
      {
        if( branching_sqr.y > 1 )
          (*out_updating_cells).push_back( coord(branching_sqr.x + 3, branching_sqr.y - 2, t) );
      }
    } // end if SE
    else if( branching_to_plane == S )
    {
      if( branching_sqr.y > 0 ) // safe to add squares that are 1 down
      {
        if( branching_sqr.x > 0 )
        {
          if( branching_sqr.x > 1 )
          {
            if( branching_sqr.x > 2 )
            {
              if( branching_sqr.x > 3 )
              {
                if( branching_sqr.x > 4 )
                  (*out_updating_cells).push_back( coord(branching_sqr.x - 5, branching_sqr.y + 1, t) );
                
                (*out_updating_cells).push_back( coord(branching_sqr.x - 4, branching_sqr.y + 1, t) );
              }
              
              (*out_updating_cells).push_back( coord(branching_sqr.x - 3, branching_sqr.y + 1, t) );
            }
            
            (*out_updating_cells).push_back( coord(branching_sqr.x - 3, branching_sqr.y + 1, t) );
          }
          
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y + 1, t) );
        }
        
        (*out_updating_cells).push_back( coord(branching_sqr.x, branching_sqr.y + 1, t) );
        
        if( branching_sqr.x + 1 < n_sqrs_w )
        {
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y + 1, t) );
          
          if( branching_sqr.x + 2 < n_sqrs_w )
          {
            (*out_updating_cells).push_back( coord(branching_sqr.x + 2, branching_sqr.y + 1, t) );
            
            if( branching_sqr.x + 3 < n_sqrs_w )
            {
              (*out_updating_cells).push_back( coord(branching_sqr.x + 3, branching_sqr.y + 1, t) );
              
              if( branching_sqr.x + 4 < n_sqrs_w )
                (*out_updating_cells).push_back( coord(branching_sqr.x + 4, branching_sqr.y + 1, t) );
            }
          }
        }
      } // end if y > 0
    } // end if S
    else if( branching_to_plane == SW )
    {
      if( branching_sqr.x > 0 )
      {
        if( branching_sqr.x > 1 )
        {
          if( branching_sqr.x > 2 )
          {
            if( branching_sqr.y > 0 )
            {
              if( branching_sqr.y > 1 )
                (*out_updating_cells).push_back( coord(branching_sqr.x - 3, branching_sqr.y - 2, t) );
                
              (*out_updating_cells).push_back( coord(branching_sqr.x - 3, branching_sqr.y - 1, t) );
            }
          }
          if( branching_sqr.y > 0 )
            (*out_updating_cells).push_back( coord(branching_sqr.x - 2, branching_sqr.y - 1, t) );
          
          (*out_updating_cells).push_back( coord(branching_sqr.x - 2, branching_sqr.y, t) );
        }
        if( branching_sqr.y + 1 < n_sqrs_h )
        {
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y + 1, t) );
        }
        (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y, t) );

      }
      
      if( branching_sqr.y + 1 < n_sqrs_h )
      {
        (*out_updating_cells).push_back( coord(branching_sqr.x, branching_sqr.y + 1, t) );
        
        if( branching_sqr.y + 2 < n_sqrs_h )
          (*out_updating_cells).push_back( coord(branching_sqr.x, branching_sqr.y + 2, t) );
      }
      
      if( branching_sqr.x + 1 < n_sqrs_w )
      {        
        if( branching_sqr.y + 2 < n_sqrs_h )
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y + 2, t) );
        
        if( branching_sqr.x + 2 < n_sqrs_w )
        {          
          if( branching_sqr.y > 2 )
          {
            (*out_updating_cells).push_back( coord(branching_sqr.x + 2, branching_sqr.y - 3, t) );
          }
        }
      }
    }
    else if( branching_to_plane == W )
    {
      if( branching_sqr.x > 0 ) // safe to add squares that are 1 left
      {
        if( branching_sqr.y > 0 )
        {
          if( branching_sqr.y > 1 )
          {
            if( branching_sqr.y > 2 )
            {
              if( branching_sqr.y > 3 )
              {
                if( branching_sqr.y > 4 )
                  (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y - 5, t) );
                
                (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y - 4, t) );
              }
              
              (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y - 3, t) );
            }
            
            (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y - 2, t) );
          }
          
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y - 1, t) );
        }
        
        (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y, t) );
        
        if( branching_sqr.y + 1 < n_sqrs_h )
        {
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y + 1, t) );
          
          if( branching_sqr.y + 2 < n_sqrs_h )
          {
            (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y + 2, t) );
            
            if( branching_sqr.y + 3 < n_sqrs_h )
            {
              (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y + 3, t) );
              
              if( branching_sqr.y + 4 < n_sqrs_h )
                (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y + 4, t) );
            }
          }
        }
      } // end if x > 0
    } // end if W
    else if( branching_to_plane == NW )
    {
      if( branching_sqr.x > 0 )
      {
        if( branching_sqr.x > 2 )
        {
          if( branching_sqr.y + 1 < n_sqrs_h )
          {
            if( branching_sqr.y + 2 < n_sqrs_h )
              (*out_updating_cells).push_back( coord(branching_sqr.x - 3, branching_sqr.y + 2, t) );
            
            (*out_updating_cells).push_back( coord(branching_sqr.x - 3, branching_sqr.y + 1, t) );
          }
        }
        
        if( branching_sqr.x > 1 )
        {
          (*out_updating_cells).push_back( coord(branching_sqr.x - 2, branching_sqr.y, t) );
          
          if( branching_sqr.y + 1 < n_sqrs_h )
            (*out_updating_cells).push_back( coord(branching_sqr.x - 2, branching_sqr.y + 1, t) );
        }
        
        if( branching_sqr.y > 0 )
          (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y - 1, t) );
        
        (*out_updating_cells).push_back( coord(branching_sqr.x - 1, branching_sqr.y, t) );
      }
      
      if( branching_sqr.y > 0 )
      {
        if( branching_sqr.y > 1 )
          (*out_updating_cells).push_back( coord(  branching_sqr.x  , branching_sqr.y - 2, t) );
        
        (*out_updating_cells).push_back( coord(  branching_sqr.x  , branching_sqr.y - 1, t) );
      }
      
      if( branching_sqr.x + 1 < n_sqrs_w )
      {
        if( branching_sqr.y > 1 )
          (*out_updating_cells).push_back( coord(branching_sqr.x + 1, branching_sqr.y - 2, t) );
      }
      
      if( branching_sqr.x + 2 < n_sqrs_w )
      {        
        if( branching_sqr.y > 2 )
        {
          (*out_updating_cells).push_back( coord(branching_sqr.x + 2, branching_sqr.y - 3, t) );
        }
      }
    } // end if NW
  } // end if branch_width == 10
#ifdef DEBUG
  cout << endl << endl << "Gonna break cause size of updating cells is " << (*out_updating_cells).size() << endl;
  assert( (int)(*out_updating_cells).size() <= branch_width );
#endif
}

void best_cost::dump( int time ) const
{  
  cout << endl << "Your plane begins at (" << start.x << ", " << start.y << ")" << endl;
  
  cout << "The MC grid for " << time << endl;
  mc->dump( time );
  
  cout << endl << "The BC grid for " << time << endl;
  bc->dump( time );
}

void best_cost::dump_csv( int time, string prefix, string name ) const
{
  mc->dump_csv( time, prefix, name );
  bc->dump_csv( time, prefix, name );
}

void best_cost::dump_csv( int time ) const
{
  mc->dump_csv( time, "", "" );
  bc->dump_csv( time, "", "" );
}


#endif
