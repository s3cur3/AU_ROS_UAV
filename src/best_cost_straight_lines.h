//
//  best_cost_straight_lines.h
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
// The following heuristic is used to estimate the cost of traveling from a node
// n to the goal:
//     weighing factor * danger at node n + weighing factor * danger at goal +
//        straight line distance from n to goal
//
// Unlike best_cost.h, this class adds a "field" around each aircraft, extending
// in the direction of the aircraft's travel. This means that when A* works with the
// grid, there is some cost associated with coming close to another aircraft.
// 


#define DEBUG
//#define DEBUG_BC

#ifndef BC_GRID_EXP
#define BC_GRID_EXP

#define natural unsigned int
#define SQRT_2 1.41421356

#include <vector>
#include <map>
#include "danger_grid_with_turns.h"
#include "Position.h"
#include <math.h>
#include <cstdlib>
#include "write_to_log.h"
#include "map_tools.h"
#include "Plane_fixed.h"
#include "coord.h"

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
   * @param set_of_aircraft A std::map containing the aircraft that need to
   * be considered
   * @param width The width of the airspace (our x dimension)
   * @param height The height of the airspace (our y dimension)
   * @param resolution The resolution to be used in the map
   * @param plane_id The index of the plane for which we are generating the best 
   *                 cost grid
   */
  best_cost( std::map< int, Plane > * set_of_aircraft, double width, double height,
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
   * Gives the "threshold" value indicating that there is a plane in the square.
   * Requires a time because the threshold may change depending on the danger
   * rating at the goal.
   *
   * NOTE: Returns -1 if this has not been initialized.
   * @param time The number of seconds in the future for which you're inquiring
   * @return the danger value indicating we are as certain as we possibly can be that 
   * there is/will be an aircraft in this square at some time
   */
  double get_plane_danger( int time ) const;
  
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
  // The "owner" of this BC grid, for whom we will calculate distance costs &c.
  Plane * owner;
  
  danger_grid * mc; // the map cost (MC) grid (a.k.a., the danger grid)
  danger_grid * bc; // the best cost grid; the heart of this class
  
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

best_cost::best_cost( std::map< int, Plane > * set_of_aircraft,
                      double width, double height, double resolution, 
                      unsigned int plane_id)
{
#ifdef DEBUG
  assert( (*set_of_aircraft).find( plane_id ) != (*set_of_aircraft).end() );
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
  // following simple heuristic:
  //      cost( node n ) = mc( n ) + (weighing factor) * distance( from n to goal )
  bc = new danger_grid( mc, set_of_aircraft, plane_id, "heuristic" );
  
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
}


// AK: DESTRUCTOR -- DESTROY mc, bc to release their memory
best_cost::~best_cost()
{
  delete mc;
  delete bc;
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

double best_cost::get_plane_danger( int time ) const
{
  return bc->get_plane_danger( time );
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
  mc->dump_csv( time, prefix, name + "mc" );
  bc->dump_csv( time, prefix, name + "bc" );
}

void best_cost::dump_csv( int time ) const
{
  mc->dump_csv( time, "", "mc" );
  bc->dump_csv( time, "", "bc" );
}


#endif
