//
// danger_grid.h
// AU_UAV_ROS
//
// Created by Tyler Young on 5/23/11.
// Added to by Thomas Crescinzi.
//
// A class to generate a grid with a "danger" rating associated with
// each block in the flyable area (which is of our "map" type).
//
// The danger grid is three dimensional: it has x and y dimensions,
// corresponding to the size of the flyable physical area. Additionally, it has a
// time dimension.
//
// Thus, to find the danger associated with the (x, y) position (10, 7) at 4 seconds
// in the future, you would call your_danger_grid_variable( 10, 7, 4 ) (which works 
// in virtue of the overloaded parenthesis operator), which would return a double 
// with the "danger rating" at that square.

#ifndef DANGER_GRID
#define DANGER_GRID

#ifndef DEBUG_TURN
#define DEBUG_TURN
#endif

#include <vector>
#include <map>
#include "map.h"
#include "estimate.h"
#include "Plane_fixed.h"
#include <math.h>
#include <climits>
#include "map_tools.h"
#include "coord.h"

using namespace std;

#ifndef natural
#define natural unsigned int
#endif

// The default amount of time in the future to "look ahead" when generating the grid;
// If the aircraft that you're working with haven't hit their goal by this time, the
// calculation stops anyway
static const unsigned int look_ahead = 20;
// the number of seconds to consider in the past
static const unsigned int look_behind = 2;

// This is defined in the constructor to be a bit greater than:
// sqrt( (width in squares)^2 + (height in squares) ^2) 
static double default_plane_danger;
static double default_scaling;
static double inverse_default_scaling;

#ifndef RADIAN_CONSTANTS
#define RADIAN_CONSTANTS
const double PI = 2*acos(0.0);// pi
const double TWO_PI = 2*PI;
const double RADtoDEGREES = 180/PI;//Conversion factor from Radians to Degrees
const double DEGREEStoRAD = PI/180;//Conversion factor from Degrees to Radians
#endif

// the amount we'll multiply danger values by when adding the buffer zones (the 
// danger around the predicted squares, to keep other aircraft from coming too
// close)
static const double field_weight = 0.6;

class danger_grid
{
public:
  /**
   * The constructor for the danger grid. It will set up a map per the parameters
   * given and then automatically calculate the danger associated with each square
   * in the map up to the default number of seconds in the future.
   *
   * Note that the width, height, and resolution may be in any units, but the units
   * must be consistent across all measurements.
   * @param set_of_aircraft A vector array containing the aircraft that need to
   * be considered
   * @param width The width of the airspace (our x dimension)
   * @param height The height of the airspace (our y dimension)
   * @param resolution The resolution to be used in the map
   * @param plane_id The ID of this danger grid's "owner" (should be its position
   *                 in the set_of_aircraft vector
   */
  danger_grid( std::map< int, Plane > * set_of_aircraft, const double width,
              const double height, const double resolution, const natural plane_id );
  
  /**
   * The heuristic generation constructor; takes a reference to a danger grid
   *  and makes this object a best cost grid.
   * @param dg A reference to another danger grid
   * @param set_of_aircraft A vector array containing the aircraft that need to
   *                        be considered
   * @param plane_id The ID of this danger grid's "owner" (should be its position
   *                 in the set_of_aircraft vector
   * @param flag The flag -- if this is set to "heuristic", we will initialize all
   *             squares to the straight-line cost to the goal
   */
  danger_grid( const danger_grid * dg, std::map< int, Plane > * set_of_aircraft,
              const natural plane_id, string flag );
  
  /**
   * The destructor for the danger_grid object
   */
  ~danger_grid();
  
  /**
   * Return the danger rating of a square
   * @param x_pos the x position of the square in question
   * @param y_pos the y position of the square in question
   * @param seconds The number of seconds in the future
   * @return a double containing the square's "danger" rating
   */
  double get_danger_at( unsigned int x_pos, unsigned int y_pos, int seconds ) const;
  
  /**
   * Return the danger cost + distance to goal from a given square
   * @param x_pos the x position of the square in question
   * @param y_pos the y position of the square in question
   * @return a double containing the square's cost (danger + dist. to goal)
   */
  double get_dist_cost_at( unsigned int x_pos, unsigned int y_pos ) const;
  
  /**
   * Adds to the danger rating of a square
   * @param x_pos the x position of the square in question
   * @param y_pos the y position of the square in question
   * @param seconds The number of seconds in the future
   * @param danger The danger rating to be assigned
   */
  void add_danger_at( unsigned int x_pos, unsigned int y_pos, int seconds,
                     double danger );
  
  /**
   * Sets the danger rating of a square
   * @param x_pos the x position of the square in question
   * @param y_pos the y position of the square in question
   * @param seconds The number of seconds in the future
   * @param danger The danger rating to be assigned
   */
  void set_danger_at( unsigned int x_pos, unsigned int y_pos, int seconds,
                     double danger );
  
  /**
   * The overloaded ( ) operator. Allows simple access to the danger rating of a
   * given square at a specified number of seconds in the future.
   * This performs the same function as get_danger_at()
   * @param x The x location of the square in question
   * @param y The y location of the square in question
   * @param time The number of seconds in the future for which we need the danger
   * rating.
   * @return a double containing the square's "danger" rating
   */
  double operator()( unsigned int x, unsigned int y, int time ) const;
  
  unsigned int get_width_in_squares() const;
  unsigned int get_height_in_squares() const;
  unsigned int get_time_in_secs() const;
  
  /**
   * This is only for copying the prediction space from another danger grid, and
   * even then, the only reason to use it over get_time_in_secs() is to maintain
   * compatibility with future updates which predict plane locations in the PAST.
   */
  unsigned int get_pred_space_time_in_secs() const;
  vector< bc::map > get_danger_space() const;
  double get_res() const;
  
  /**
   * @return a reference to the "owner" of this danger grid (the plane for which
   * it was created)
   */
  Plane * get_owner();
  
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
   * Modifies the map to store the cost of the path which begins at each square and
   * takes a straight line to the goal, effectively creating a simplified version of
   * a best cost grid. Adjusts all existing danger ratings by the danger_adjust 
   * passed in.
   * Tyler is adding this here to avoid using a "wrapper" for the straight-line
   * heuristic.
   * @param goal_x The x coordinate for the goal
   * @param goal_y The y coordinate for the goal
   * @param danger_adjust The amount we multiply a danger rating by
   */
  void calculate_distance_costs( unsigned int goal_x, unsigned int goal_y, 
                                const danger_grid * dg, double danger_adjust );
  
  /**
   * Adds cost to grid squares on the left of the aircraft, effectively implementing
   * a preference for right-hand turns when path planning using A*.
   * @param plane The "owner" of this BC grid, from whose starting location and
   *              toward whose goal we will be adding cost.
   */
  void encourage_right( );
  
  /**
   * Output the map at a given time; for troubleshooting only
   * @param time The time, in seconds, whose map should be output
   */
  void dump( int time ) const;
  
  /**
   * Output the map to a CSV file.
   * For troubleshooting the best cost grid
   * @param time The time, in seconds, whose map should be output
   */
  void dump_big_numbers( int time ) const;
  
  void dump_csv( int time, string prefix, string name ) const;
  
private:
  enum bearing_t { N, NE, E, SE, S, SW, W, NW };
  
  /**
   * The method that does virtually all the important work for the class.
   * Calculates danger ratings for all squares in all maps of the danger_space
   * (where the danger_space is the set of maps corresponding to each second we are
   * looking ahead and behind).
   * @param plane_id The ID number of the plane to ignore (that is, the ID of the
   *                 plane that this DG will be used for; we don't want it avoiding
   *                 itself!)
   */
  void fill_danger_space( const natural plane_id );
  
  /**
   * Set up the weighting scheme for danger ratings in the future.
   * At the moment, this simply decreases the danger linearly as you go farther
   * into the future.
   */
  void set_danger_scale( );
  
  /**
   * Return the danger constant for a given time in the future; multiply a "raw"
   * danger rating by this to scale the rating down based on the uncertainty
   * inherent in predicting the other aircraft's future paths.
   * @param seconds The number of seconds in the future
   * @return a double containing the time's danger constant
   */
  double adjust_danger( int seconds ) const;
  
  /**
   * A function to calculate an aircraft's future positions. The plane's position is
   * extrapolated until it is estimated to reach its goal waypoint.
   * Each second's-worth of estimate is separated by an estimate with danger = -1.
   * @param the_plane The plane object whose future position is being calculated
   * @return A vector of estimates, which are (x, y, danger) triples; these
   * estimates represent a "spread" of probable locations where the
   * aircraft may be at a given second in the future.
   */
  vector< estimate > calculate_future_pos( Plane & the_plane, int & time );
  
  /**
   * Converts a bearing in degrees to a "named" version, for use in deciding which
   * nearby squares are in the path of the aircraft
   * @param the_bearing Bearing of the aircraft in degrees (0 is due north, 
   *                    90 due east, and so on)
   * @return A named version of the direction (N for bearings -22.5 to 22.5 deg,
   *         NE for bearings 22.5 to 67.5 deg, and so on)
   */
  bearing_t name_bearing( double the_bearing );
  
  /**
   * Having calculated a danger rating for square (x, y), call this function to
   * fill in the surrounding squares with a "field" of somewhat lesser danger values,
   * whose ultimate purpose is to keep planes a minimum distance apart.
   * @param bearing The bearing of the aircraft, in degrees
   * @param unweighted_danger The danger rating that was just given to square (x, y)
   *                          (the plane's actual location)
   * @param x The x coordinate of the plane's actual location
   * @param y The y coordinate of the plane's actual location
   * @param time The number of seconds in the future for which the plane's danger was
   *             just set
   */
  void set_danger_buffer( double bearing, double unweighted_danger,
                         natural x, natural y, int time );
  
  /**
   * Outputs the contents of an "estimate" vector array
   * Useful only for troubleshooting
   * @param dump_me The vector of estimate to be output
   */
  void dump_est( vector< estimate > dump_me );
  
  // Functions by Thomas
  void neighoboringAngles(double angle, double &first, double &second);
  void placeDanger(double angle, vector<estimate> &e, double closest, double other,
                   int x, int y, double danger);//places the data into the estimate struct
  void dangerRecurse(estimate e, int destination[], vector<estimate> &theFuture, int & time);
  void turn(double startingAngle, double endAngle, vector<estimate> &e, int &x, int &y);
  
  // the weighting applied to danger estimates in the future
  vector< double > danger_ratings;
  
  vector< double > plane_danger;
  
  // the set of aircraft with which we are concerned
  std::map< int, Plane > * aircraft;
  
  // The danger space is a bit strange due to the fact that it's an array of maps,
  // where each position in the array corresponds to a time.
  vector< bc::map > * danger_space;
  
  // The "owner" of this danger grid, for whom we will calculate distance costs &c.
  Plane * owner;
  
#ifdef OVERLAYED
  vector< bc::map > overlayed; // Used only when dumping output
#endif
  
  double map_res;
  
  bc::map * dist_map;
  bc::map * encouraged_right;
  bool distance_costs_initialized;
};

danger_grid::danger_grid( std::map< int, Plane > * set_of_aircraft, const double width,
                         const double height, const double resolution,
                         const natural plane_id )
{
  aircraft = set_of_aircraft;
  map_res = resolution;
  distance_costs_initialized = false;
  owner = &( (*set_of_aircraft)[ plane_id ] );
  
#ifdef DEBUG
  assert( set_of_aircraft->size() != 0 );
  assert( resolution > EPSILON );
  assert( resolution < height && resolution < width );
  assert( height / resolution < 1000000 );
  assert( width / resolution < 1000000 );
#endif
  
  natural sqrs_wide = map_tools::find_width_in_squares( width, height, resolution );
  natural sqrs_high = map_tools::find_height_in_squares( width, height, resolution );
  default_scaling = 10;
  inverse_default_scaling = 1/default_scaling;
  default_plane_danger = sqrt( sqrs_wide * sqrs_wide + sqrs_high * sqrs_high ) * default_scaling;
  
#ifdef OVERLAYED
  overlayed.push_back( map( width, height, resolution ) );
#endif
  
  bc::map set_up( width, height, resolution );
  // Make danger_space a set of maps, with one map for each second in time that
  // we will work with.
  danger_space = new vector< bc::map >( look_ahead + look_behind + 1, set_up );
  
  // Set up the danger ratings
  set_danger_scale( );
  
  // Do all the work -- calculate the danger rating for all squares at all times
  fill_danger_space( plane_id );
}

danger_grid::danger_grid( const danger_grid * dg, std::map< int, Plane > * set_of_aircraft,
                         const natural plane_id,  string flag )
{
  owner = &( (*set_of_aircraft)[ plane_id ] );
  if( flag != "heuristic" )
    cout << "You're using the wrong constructor." << endl;
  assert( flag == "heuristic" );
  
  calculate_distance_costs( owner->getFinalDestination().getX(),
                           owner->getFinalDestination().getY(),
                           dg, 1.0 );
}

danger_grid::~danger_grid()
{
  if( distance_costs_initialized )
  {
    delete dist_map;
    //delete encouraged_right;
  }
  delete danger_space;
}

void danger_grid::fill_danger_space( const natural plane_id )
{  
  // This will store the list of predicted plane locations from the plane's
  // current location to its avoidance waypoint; if there is no avoidance waypoint,
  // it will contain predictions all the way to the plane's goal.
  vector< estimate > est_to_avoid;
  // If the plane has an avoidance waypoint, this will store the predicted plane
  // locations from the avoidance waypoint to the goal; else, it will be empty.
  vector< estimate > est_to_goal;
  
  // For each plane . . .
  for( map< int, Plane >::iterator plane_pair = aircraft->begin(); 
      plane_pair != aircraft->end(); ++plane_pair )
  {
    Plane * current_plane = &( (*plane_pair).second );
    
    // If this is not the "owner" of the danger grid . . . 
    if( (*current_plane).getId() != (int)plane_id )
    {
      // Set the danger at the plane's starting location
      (*danger_space)[0 + look_behind].
      add_danger_at( (*current_plane).getLocation().getX(),
                    (*current_plane).getLocation().getY(), default_plane_danger);
#ifdef OVERLAYED
      overlayed[0].add_danger_at((*current_plane).getLocation().getX(),
                                 (*current_plane).getLocation().getY(), 1.0);
#endif
      
      // Get the estimated danger for relevant squares in the map at this time
      int dummy = 0;
      est_to_avoid = calculate_future_pos( *current_plane, dummy );
      int est_break = dummy;
      est_to_goal = calculate_future_pos( *current_plane, dummy );
      /*
       cout << "Here's the est to avoidance wypt " << endl;
       dump_est( est_to_avoid );
       cout << "Here's the est to da goal " << endl;
       dump_est( est_to_goal );
       */
      
      double bearing = (*current_plane).getBearing();
      
      int t = 1; // initialize the counter for steps in time (seconds)
      int counter = 0;
      
      // For each estimated (x, y, danger) triple . . .
      for( vector< estimate >::iterator current_est = est_to_avoid.begin();
          current_est != est_to_avoid.end(); ++current_est )
      {
        ++counter; // we got a legal estimate
        
        if( t <= (int)look_ahead ) // if this estimate is close enough to plan for it . . .
        {
          // If these are legal xs and ys, and if the danger is not a "timestamp" divider
          if( (*current_est).x >= 0 && (*current_est).x <
             (int)( (*danger_space)[0].get_width_in_squares() ) &&
             (*current_est).y >= 0 &&
             (*current_est).y < (int)( (*danger_space)[0].get_height_in_squares() ) &&
             (*current_est).danger > -(EPSILON) )
          {
            // Set the danger of the square based on what
            // calculate_future_pos() found, but scale it according to how
            // far back in time we're predicting
            natural time = t + look_behind;
            natural x = (*current_est).x;
            natural y = (*current_est).y;
            double d = (*current_est).danger * adjust_danger( t );
            
            (*danger_space)[ time ].add_danger_at( x, y, d );
            
            // . . . and then add a bit of "fuzziness" (danger around the predicted
            // square, so that other planes don't come too close)
            set_danger_buffer( bearing, d, x, y, time );
            
#ifdef OVERLAYED
            overlayed[0].add_danger_at((*current_est).x,
                                       (*current_est).y,
                                       (*current_est).danger * adjust_danger(t) );
#endif
          } // end if these are legal xs and ys, and if this is not a "timestamp" divider
          else // this estimate is only a timestamp marker
          {
#ifdef DEBUG
            assert( counter > 0 ); // we got at least one estimate
#endif
            ++t;
          }
        } // end if t < look_ahead
      } // end for each estimated (x, y, danger) triple
      
      // Now do the same thing with the list of predicted plane locations from the
      // avoidance waypoint to the goal, where applicable
      t = est_break + 1;
      for( vector< estimate >::iterator current_est = est_to_goal.begin();
          current_est != est_to_goal.end(); ++current_est )
      {
        if( t <= (int)look_ahead ) // if this estimate is close enough to plan for it . . .
        {
          // If these are legal xs and ys, and if the danger is not a "timestamp" divider
          if( (*current_est).x >= 0 && (*current_est).x <
             (int)( (*danger_space)[0].get_width_in_squares() ) &&
             (*current_est).y >= 0 &&
             (*current_est).y < (int)( (*danger_space)[0].get_height_in_squares() ) &&
             (*current_est).danger > -(EPSILON) )
          {
            // Set the danger of the square based on what
            // calculate_future_pos() found, but scale it according to how
            // far back in time we're predicting
            natural time = t + look_behind;
            natural x = (*current_est).x;
            natural y = (*current_est).y;
            double d = (*current_est).danger * adjust_danger( t );
            
            (*danger_space)[ time ].add_danger_at( x, y, d );
            
            // . . . and then add a bit of "fuzziness" (danger around the predicted
            // square, so that other planes don't come too close)
            set_danger_buffer( bearing, d, x, y, time );
            
#ifdef OVERLAYED
            overlayed[0].add_danger_at((*current_est).x,
                                       (*current_est).y,
                                       (*current_est).danger * adjust_danger(t) );
#endif
          } // end if these are legal xs and ys, and if this is not a "timestamp" divider
          else // this estimate is only a timestamp marker
          {
            ++t;
          }
        } // end if t < look_ahead
      } // end for each estimated (x, y, danger) triple
      
    } // end if this is not the "owner" of the danger grid
  } // end for each plane in the list
}

void danger_grid::set_danger_buffer( double bearing, double unweighted_danger,
                                    natural x, natural y, int time )
{
  double d = unweighted_danger * field_weight;
  
  // These buffer zones have been made wider in light of A*'s propensity for taking
  // diagonals when we allow it.
  
  // dag left+down
  (*danger_space)[ time ].safely_add_danger_at( x - 1, y + 1, d );
  // straight left
  (*danger_space)[ time ].safely_add_danger_at( x - 1,   y  , d );
  // dag left+up
  (*danger_space)[ time ].safely_add_danger_at( x - 1, y - 1, d );
  // straight up
  (*danger_space)[ time ].safely_add_danger_at(   x  , y - 1, d );
  // dag right+up
  (*danger_space)[ time ].safely_add_danger_at( x + 1, y - 1, d );
  // straight right
  (*danger_space)[ time ].safely_add_danger_at( x + 1,   y  , d );
  // dag right+down
  (*danger_space)[ time ].safely_add_danger_at( x + 1, y + 1, d );
  // straight down
  (*danger_space)[ time ].safely_add_danger_at(   x ,  y + 1, d);
  
  
  // Scale the danger down slightly so A* will not treat collision distances
  // the same as conflict distance
  d *= field_weight;
  
  // Begin squares that are 2 away from current location
  // dag less left+down
  (*danger_space)[ time ].safely_add_danger_at( x - 1, y + 2, d );
  // dag left+down
  (*danger_space)[ time ].safely_add_danger_at( x - 2, y + 2, d );
  // dag left+less down
  (*danger_space)[ time ].safely_add_danger_at( x - 2, y + 1, d );
  // straight left
  (*danger_space)[ time ].safely_add_danger_at( x - 2,   y  , d );
  // dag left+up
  (*danger_space)[ time ].safely_add_danger_at( x - 2, y - 2, d );
  // dag left+less up
  (*danger_space)[ time ].safely_add_danger_at( x - 2, y - 1, d );
  // dag less left+up
  (*danger_space)[ time ].safely_add_danger_at( x - 1, y - 2, d );
  // straight up
  (*danger_space)[ time ].safely_add_danger_at(   x  , y - 2, d );
  // dag less right+up
  (*danger_space)[ time ].safely_add_danger_at( x + 1, y - 2, d );
  // dag right+up
  (*danger_space)[ time ].safely_add_danger_at( x + 2, y - 2, d );
  // dag right+less up
  (*danger_space)[ time ].safely_add_danger_at( x + 2, y - 1, d );
  // straight right
  (*danger_space)[ time ].safely_add_danger_at( x + 2,   y  , d );
  // dag right+less down
  (*danger_space)[ time ].safely_add_danger_at( x + 2, y + 1, d );
  // dag right+down
  (*danger_space)[ time ].safely_add_danger_at( x + 2, y + 2, d );
  // dag less right+down
  (*danger_space)[ time ].safely_add_danger_at( x + 1, y + 2, d );
  // straight down
  (*danger_space)[ time ].safely_add_danger_at(   x ,  y + 2, d );
  
  
}

void danger_grid::set_danger_scale( )
{
  // For now, we aren't scaling anything down
  danger_ratings.resize( look_behind + look_ahead + 1, default_plane_danger );
}

double danger_grid::get_danger_at( unsigned int x_pos, unsigned int y_pos,
                                  int seconds ) const
{
#ifdef DEBUG
  if( seconds > (int)( danger_space->size() - look_behind ) )
    cout << "Time " << seconds << " doesn't exist.";
  assert( seconds < (int)( danger_space->size() - look_behind ) );
  assert( seconds >= -(int)look_behind );
  assert( x_pos < UINT_MAX && y_pos < UINT_MAX );
#endif
  return (*danger_space)[ seconds + look_behind ].get_danger_at( x_pos, y_pos );
}

void danger_grid::add_danger_at( unsigned int x_pos, unsigned int y_pos, int seconds,
                                double danger )
{
#ifdef DEBUG
  assert( seconds < (int)( danger_space->size() - look_behind ) );
  assert( seconds >= -(int)look_behind );
  assert( x_pos < UINT_MAX && y_pos < UINT_MAX );
  assert( danger > -1.0 );
#endif
  (*danger_space)[ seconds + look_behind ].add_danger_at( x_pos, y_pos, danger );
}

void danger_grid::set_danger_at( unsigned int x_pos, unsigned int y_pos, int seconds,
                                double danger )
{
#ifdef DEBUG
  assert( seconds < (int)( danger_space->size() - look_behind ) );
  assert( seconds >= -(int)look_behind );
  assert( x_pos < UINT_MAX && y_pos < UINT_MAX );
  assert( danger > -1.0 );
#endif
  (*danger_space)[ seconds + look_behind ].set_danger_at( x_pos, y_pos, danger );
}

double danger_grid::adjust_danger( int seconds ) const
{
  return danger_ratings[ seconds + look_behind ];
}

double danger_grid::operator()( unsigned int x, unsigned int y, int time ) const
{
  return get_danger_at( x, y, time );
}

unsigned int danger_grid::get_width_in_squares() const
{
  return (*danger_space)[ 0 ].get_width_in_squares();
}

unsigned int danger_grid::get_height_in_squares() const
{
  return (*danger_space)[ 0 ].get_height_in_squares();
}

unsigned int danger_grid::get_time_in_secs() const
{
  return look_ahead;
}

unsigned int danger_grid::get_pred_space_time_in_secs() const
{
  return look_ahead + look_behind + 1;
}

double danger_grid::get_res() const
{
  return (*danger_space)[ 0 ].get_resolution();
}

Plane * danger_grid::get_owner()
{
  return owner;
}

double danger_grid::get_plane_danger( int time ) const
{
  if( plane_danger[ time ] < EPSILON )
    return -1.0;
  
  return plane_danger[ time ];
}

vector< bc::map > danger_grid::get_danger_space() const
{
  return (*danger_space);
}

/** 
	a function for predicting planes. most of the work is done in the recursive calling of 
	danger recurse but this guy starts the whole process. relies on dangerRecurse(), neighoboringAngles(),
	and placeDanger(). it is built to be called 2 times. use the same variable for time in each instances.
	it decides which point, the final destination or just the goal, to fly to based on the value in time.
	if time is =0 it assumes the destination is where it is going otherwise it assumes it is flying from the 
	destination to the goal. if there is only a goal then the destination will be the goal based on the way that 
	planes and the telemetry data works.note this prediction is based on 2 assumptions: a flat cartesian grid in
	which the planes exist and that the planes can point turn. in later editions the point turning assumption
	will, hopefully, be dropped. the cartesian grid will always be an assumption as A* only works in a cartesian grid
	so there is no reason to do anything else. 
	input:(yes there is more to this function than just a description)
		plane: the plane whose path you are predicting
		time: the time from which you are starting prediction, must be >=0
	output:
		a vector that contains estimates of the planes path. as the plane traves through time a (0,0,-1) estimate is inserter
		as a time marker.
**/
vector< estimate > danger_grid::calculate_future_pos( Plane & plane, int &time )
{
  bool turned=false;
  vector< estimate > theFuture;

  Position current, destination;
  if(time==0)//meaning that the plane is now moving towards it's next goal be it an avoidance point or a final destination
	{
  	current=plane.getLocation();
  	destination=plane.getDestination();
  }
	else//will always be called twice. if it moving to its final goal the distance will be 0 and it will break out immediatly
	{
		destination=plane.getFinalDestination();
		current=plane.getDestination();  
	}

  //distance formula: line to destination
  int x1=current.getX(),x2=destination.getX(),y1=current.getY(),y2=destination.getY();
  double xDistance=( fabs((double)x2-x1) ),yDistance=( fabs((double)y2-y1) );
  double distance = sqrt((double)(xDistance*xDistance)+(yDistance*yDistance));

  if(xDistance==0&&yDistance==0)//your there!!!!!!!(hopefully) or your next destination was your goal
  {return theFuture;}

  //find the angle to the waypoint
  double angle=(180-RADtoDEGREES*(asin((double)xDistance/(double)distance)));
  if(y2<y1)
    angle=(RADtoDEGREES*(asin((double)xDistance/(double)distance)));
  
  if((x2-x1)<0)//positive means that the plane is headed to the left aka west
    angle=(-1)*angle;//the plane goes from -180 to +180

	//if a turn needs to be made to correct for the bearing
	if(time==0)
	{
	double bearing = plane.getBearing();
	map_tools::bearing_t bearingNamed = plane.get_named_bearing();
	cout<<"My names bearing(before shit happens) is:"<<map_tools::bearing_to_string(bearingNamed)<<endl;
	if( bearingNamed == map_tools::N ) //headed north
    {
      bearing=0;
    }
    else if( bearingNamed == map_tools::NW ) //headed northwest
    {
      bearing=-45;
    }
    else if( bearingNamed == map_tools::W ) //headed west
    {
      bearing=-90;
    }        
    else if( bearingNamed == map_tools::SW ) //headed southwest
    {
      bearing=-135;
    }
    else if( bearingNamed == map_tools::S ) //headed south
    {
    	bearing=180;
    }
    else if( bearingNamed == map_tools::SE ) //headed southeast
    {
      bearing=135;
    }
    else if( bearingNamed == map_tools::E ) //headed east
    {
      bearing=90;
    }
    else //headed northeast
    {
      bearing=45;
    }
  if(fabs(fabs(angle)-fabs(bearing))>22.5)
	{
		turned=true;
		printf("Turning check out how I do!\n");
		printf("Oh heres my info :) %f  %f  %f\n", angle,bearing, plane.getBearing());
		turn(bearing,angle,theFuture,x1,y1);
		x1=theFuture[theFuture.size()-2].x;
		y1=theFuture[theFuture.size()-2].y;
		printf("My starting x,y after the turn: %d,%d\n",x1,y1);
		xDistance=fabs((double)x2-x1),yDistance=fabs((double)y2-y1);
		distance = sqrt((double)(xDistance*xDistance)+(yDistance*yDistance));
		angle=(180-RADtoDEGREES*(asin((double)xDistance/(double)distance)));
		if(y2<y1)
				angle=(RADtoDEGREES*(asin((double)xDistance/(double)distance)));
		if((x2-x1)<0)//positive means that the plane is headed to the left aka west
			angle=(-1)*angle;//the plane goes from -180 to +180
		printf("Oh heres my new angle %f",angle);

		////printing the turn
		/*int x=plane.getLocation().getWidth()+1, y=plane.getLocation().getHeight()+1;
	int grid[x][y];
	for(int i=0; i<plane.getLocation().getHeight()+1; i++)
		for(int j=0; j<plane.getLocation().getWidth()+1; j++)
		{
			if(j==current.getX() && i==current.getY())
			grid[j][i]=5;
			else
			grid[j][i]=0;
		}

	for(int i=0; i<y; i++)
		grid[0][i]=i;
	for(int i=0; i<x; i++)
		grid[i][0]=i;
	
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
		{		seconds++; }
		
	}

	for(int i=0; i<plane.getLocation().getHeight(); i++)
	{	
	for (int j=0; j<plane.getLocation().getWidth(); j++)
	{
			if(grid[j][i]!=0)
			printf( " \033[22;32m%1d\e[0m ", grid[j][i]);
			else
			printf( " %1d ", grid[j][i]);
	}			
	cout<<endl;

	}
	cout<<endl;

*/

	}
  }
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
	if(!turned)
	theFuture.push_back(estimate(0,0,-1));
	time++;
  if(theFuture[1].danger>.3)
    dangerRecurse(theFuture[1], dest, theFuture, time);
	else
  dangerRecurse(theFuture[0],dest,theFuture,time);
  
	//add prediction to one square ahead of goal
	//find closest straight line from target bearing
	angle=plane.getBearingToDest();
  neighoboringAngles(angle, neighbors[0], neighbors[1]);
  if(fabs(angle-neighbors[0])>=fabs(angle-neighbors[1]))//distance
  {closestAngle=neighbors[1]; otherAngle=neighbors[0];}
  else
  {closestAngle=neighbors[0]; otherAngle=neighbors[1];}
	
	danger=1;
	
  //now add the danger ahead of the goal to theFuture
  placeDanger(angle, theFuture, closestAngle, otherAngle, x2, y2, danger);	

	//2 seconds ho!!!!(like land ho! not the street-corner kind)
	theFuture.push_back(estimate(0,0,-1));
  //now add the danger ahead of the goal to theFuture(btw this is all you need since were assuming a line at this point)
  placeDanger(angle, theFuture, closestAngle, otherAngle, theFuture[theFuture.size()-3].x, theFuture[theFuture.size()-3].y, danger);	

	//3 seconds ho!!!!(this time I need a good night)
	theFuture.push_back(estimate(0,0,-1));
  //now add the danger ahead of the goal to theFuture(btw this is all you need since were assuming a line at this point)
  placeDanger(angle, theFuture, closestAngle, otherAngle,  theFuture[theFuture.size()-3].x, theFuture[theFuture.size()-3].y, danger);	
	
#ifdef DEBUG_TURN
	//here is where we find out what this shit looks like
	cout<<"The path predicted for plane "<<plane.getId()<<" from "<<current.getX()<<","<<current.getY()<<" to "<<destination.getX()<<","<<destination.getY()<<endl;
	cout<<"The planes bearing is:"<<map_tools::bearing_to_string(plane.get_named_bearing())<<endl;
	int x=plane.getLocation().getWidth()+1, y=plane.getLocation().getHeight()+1;
	int grid[x][y];
	for(int i=0; i<plane.getLocation().getHeight()+1; i++)
		for(int j=0; j<plane.getLocation().getWidth()+1; j++)
		{
			if(j==current.getX() && i==current.getY())
			grid[j][i]=5;
			else
			grid[j][i]=0;
		}

	for(int i=0; i<y; i++)
		grid[0][i]=i;
	for(int i=0; i<x; i++)
		grid[i][0]=i;
	
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
		{		seconds++; }
		
	}

	for(int i=0; i<plane.getLocation().getHeight(); i++)
	{	
	for (int j=0; j<plane.getLocation().getWidth(); j++)
	{
			if(grid[j][i]!=0)
			printf( " \033[22;32m%1d\e[0m ", grid[j][i]);
			else
			printf( " %1d ", grid[j][i]);
	}			
	cout<<endl;

	}
	cout<<endl;
	
	for(int i=0; i<theFuture.size(); i++)
		cout<<theFuture[i].x<<","<<theFuture[i].y;
#endif
  
  return theFuture;
}

/**
	the recursive function that calculates the planes path
	input:
		e: an "estimate"
		destination[]: a pair of x,y that represents where the plane is going
		theFuture: a vector of estimates
		time: an int used for calculating the total time the plane is predicted for
	output:
		it returns both a vector of extimates and the total time value
**/
void danger_grid::dangerRecurse(estimate e, int destination[], vector<estimate> &theFuture,int &time)
{
  
  int x1=e.x;
  int y1=e.y;
  int x2=destination[0];
  int y2=destination[1];
  int dest[2]={x2,y2};
  
  double xDistance=( fabs((double)x2-x1) ), yDistance=( fabs((double)y2-y1) );
  if(xDistance==0&&yDistance==0)//your there!!!!!!!(hopefully)
		{/*theFuture.push_back(estimate(0,0,-1));*/return;}
  double distance = sqrt((double)(xDistance*xDistance)+(yDistance*yDistance));
  
  //find the angle to the waypoint
  double angle=(180-RADtoDEGREES*(asin((double)xDistance/(double)distance)));
  if(y2<y1)
    angle=(RADtoDEGREES*(asin((double)xDistance/(double)distance)));
  if((x2-x1)<0)//negative means that the plane is headed to the left aka west
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
  int nextPos = (int)theFuture.size()-2;
  //branch it up now
	theFuture.push_back(estimate(0,0,-1));
	time++;
  if(theFuture[nextPos+2].danger>.3)
    dangerRecurse(theFuture[nextPos+2], dest, theFuture, time);
  //default branch
	else
  dangerRecurse(theFuture[nextPos],dest,theFuture, time);
  
  return;
}

/**
	a function that finds the neighbors of a given angle
	input:
		angle: the angle you are finding the neighbors of
		first and second: both are out parameters only 0s are expected in
	output:
		first: the closest angle
		second: the next closest angle
**/

void danger_grid::neighoboringAngles(double angle, double &first, double &second)
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

/**
	input:
		angle: the bearing from the location to the goal location
		e: a vector of "estimates" 
		closest: the angle that is closest to angle that bisects a neighboring square
		other: the next closest angle
		x: the x location in the grid(of the current location not the one that you place the danger in)
		y: the y location in the grid
	output:
		e is changed to contain the the estimated danger in the new location
	**/
void danger_grid::placeDanger(double angle, vector<estimate> &e, double closest, double other, int x, int y, double danger)
{
	double dangerCeiling=.4;
	double remainingDanger=1-danger;
	if(danger>=dangerCeiling)
		danger=dangerCeiling;
	if(remainingDanger>=dangerCeiling)
		remainingDanger=dangerCeiling;
		
	
  if(angle>0)//to the right
  {
    if(closest==0)//north && northeast
    {
      e.push_back(estimate(x,y-1,danger));//majority in N
      e.push_back(estimate(x+1,y-1,remainingDanger));//remainder in NE
    }
    else if(closest == 45 && other == 0)//northeast && north
    {
      e.push_back(estimate(x+1,y-1,danger));//majority in NE
      e.push_back(estimate(x,y-1,remainingDanger));//remainder in N
    }
    
    else if(closest == 45)//northeast && east
    {
      e.push_back(estimate(x+1,y-1,danger));//majority in NE
      e.push_back(estimate(x+1,y,remainingDanger));//remainder in east
    }
    
    else if(closest == 90 && other == 45)//east && northeast
    {
      e.push_back(estimate(x+1,y,danger));//majority in east
      e.push_back(estimate(x+1,y-1,remainingDanger));//remainder in NE
    }
    
    else if(closest == 90)//east && southeast
    {
      e.push_back(estimate(x+1,y,danger));//majority in east
      e.push_back(estimate(x+1,y+1,remainingDanger));//remainder in SE
    }
    
    else if(closest == 135 && other == 90)//southeast && east
    {
      e.push_back(estimate(x+1,y+1,danger));//majority in SE
      e.push_back(estimate(x+1,y,remainingDanger));//remainder in east
    }
    
    else if(closest == 135)//southeast && south
    {
      e.push_back(estimate(x+1,y+1,danger));//majority in SE
      e.push_back(estimate(x,y+1,remainingDanger));//remainder in south
    }
    
    else//south && southeast
    {
      e.push_back(estimate(x,y+1,danger));//majority in south
      e.push_back(estimate(x+1,y+1,remainingDanger));//majority in SE
    }
  }
  else//to the left
  {
    if(closest==0)//north && northwest
    {
      e.push_back(estimate(x,y-1,danger));//majority in N
      e.push_back(estimate(x-1,y-1,remainingDanger));//remainder in NW
    }
    else if(closest == -45 && other == 0)//northwest && north
    {
      e.push_back(estimate(x-1,y-1,danger));//majority in NW
      e.push_back(estimate(x,y-1,remainingDanger));//remainder in N
    }
    
    else if(closest == -45)//northwest && west
    {
      e.push_back(estimate(x-1,y-1,danger));//majority in NW
      e.push_back(estimate(x-1,y,remainingDanger));//remainder in west
    }
    
    else if(closest == -90 && other == -45)//west && northwest
    {
      e.push_back(estimate(x-1,y,danger));//majority in west
      e.push_back(estimate(x-1,y-1,remainingDanger));//remainder in NE
    }
    
    else if(closest == -90)//west && southwest
    {
      e.push_back(estimate(x-1,y,danger));//majority in west
      e.push_back(estimate(x-1,y+1,remainingDanger));//remainder in SW
    }
    
    else if(closest == -135 && other == -90)//southwest && west
    {
      e.push_back(estimate(x-1,y+1,danger));//majority in SW
      e.push_back(estimate(x-1,y,remainingDanger));//remainder in west
    }
    
    else if(closest == -135)//southwest && south
    {
      e.push_back(estimate(x-1,y+1,danger));//majority in SW
      e.push_back(estimate(x,y+1,remainingDanger));//remainder in south
    }
    
    else//south && southwest
    {
      e.push_back(estimate(x,y+1,danger));//majority in south
      e.push_back(estimate(x-1,y+1,remainingDanger));//majority in SW
    }
  }
}

void danger_grid::turn(double startingAngle, double endAngle, vector<estimate> &e, int &x, int &y)
{
	bool negative=false;
	if(startingAngle<0)
		negative=true;

	if(fabs((double)endAngle)-fabs((double)startingAngle) <22.5)
		return;//your done turning
	
	if(negative)
	{
		placeDanger(-22.5,e,0,-45,x,y,.5);
		e.push_back(estimate(-1,-1,-1));
		turn(startingAngle-22.5,endAngle, e, e[e.size()-2].x,e[e.size()-2].y);
	}
		
	else
	{
		placeDanger(22.5,e,0,45,x,y,.5);
		e.push_back(estimate(-1,-1,-1));
		turn(startingAngle+22.5,endAngle, e, e[e.size()-2].x,e[e.size()-2].y);
	}
	
	return;
}

danger_grid::bearing_t danger_grid::name_bearing( double the_bearing )
{
  the_bearing = fmod(the_bearing, 360); // modular division for floats
  
  if( the_bearing > -22.5 && the_bearing <= 22.5 )
    return N;
  else if( the_bearing > 22.5 && the_bearing <= 67.5 )
    return NE;
  else if( the_bearing > 67.5 && the_bearing <= 112.5 )
    return E;
  else if( the_bearing > 112.5 && the_bearing <= 157.5 )
    return SE;
  else if( the_bearing > 157.5 && the_bearing <= 202.5 )
    return S;
  else if( the_bearing > 202.5 && the_bearing <= 247.5 )
    return SW;
  else if( the_bearing > 247.5 && the_bearing <= 292.5 )
    return W;
  else if( the_bearing > 292.5 && the_bearing <= 337.5 )
    return NW;
  else if( the_bearing > -67.5 && the_bearing <= -22.5 )
    return NW;
  else if( the_bearing > -112.5 && the_bearing <= -67.5 )
    return W;
  else if( the_bearing > -157.5 && the_bearing <= -112.5 )
    return SW;
  else if( the_bearing > -202.5 && the_bearing <= -157.5 )
    return S;
  else if( the_bearing > -247.5 && the_bearing <= -202.5 )
    return SE;
  else if( the_bearing > -292.5 && the_bearing <= -247.5 )
    return E;
  else if( the_bearing > -337.5 && the_bearing <= -292.5 )
    return NW;
  else
  {
#ifdef DEBUG
    assert( the_bearing > -361 && the_bearing < 361 );
#endif
    return N;
  }
}


void danger_grid::dump_est( vector< estimate > dump_me )
{
  unsigned int i = 0;
  for( vector< estimate >::iterator crnt_est = dump_me.begin();
      crnt_est != dump_me.end(); ++crnt_est )
  {
    cout << endl << " Estimate " << i << "'s data: " << endl;
    cout << " x = " << (*crnt_est).x << endl;
    cout << " y = " << (*crnt_est).y << endl;
    cout << " d = " << (*crnt_est).danger << endl;
    ++i;
  }
}


void danger_grid::calculate_distance_costs( unsigned int goal_x, unsigned int goal_y,
                                           const danger_grid * dg, double danger_adjust )
{
  // This will store the cost of travelling from each square to the goal
  dist_map = new bc::map( dg->get_width_in_squares() * dg->get_res(), 
                         dg->get_height_in_squares() * dg->get_res(),
                         dg->get_res() );
  
  for( unsigned int crnt_x = 0; crnt_x <  dg->get_width_in_squares(); crnt_x++ )
  {
    for( unsigned int crnt_y = 0; crnt_y < dg->get_height_in_squares(); crnt_y++ )
    {
      dist_map->set_danger_at(crnt_x, crnt_y, sqrt( (crnt_x - goal_x)*(crnt_x - goal_x) + 
                                                   (crnt_y - goal_y)*(crnt_y - goal_y) ) );
    }
  }
  
  //encouraged_right = new map(*dist_map);
  
  // Add a bit of cost to the left side of the aircraft to encourage it to
  // take avoidance maneuvers to the right
  //encourage_right( );
  
  distance_costs_initialized = true;
  
  danger_space = new vector< bc::map >( look_ahead + look_behind + 1, (*dist_map) );
  
  double d_at_goal;
  
  for( unsigned int crnt_t = 0; crnt_t <= look_ahead; crnt_t++ )
  {
    d_at_goal = (*dg).get_danger_at(goal_x, goal_y, crnt_t);
    
    plane_danger.push_back( d_at_goal + (default_plane_danger*inverse_default_scaling) );
    
    for( unsigned int crnt_x = 0; crnt_x <  dg->get_width_in_squares(); crnt_x++ )
    {
      for( unsigned int crnt_y = 0; crnt_y < dg->get_height_in_squares(); crnt_y++ )
      {
        double crnt_danger = (*dg).get_danger_at( crnt_x, crnt_y, crnt_t );
        
        if( crnt_danger > EPSILON || d_at_goal > EPSILON )
        {
          (*danger_space)[crnt_t + look_behind].set_danger_at( crnt_x, crnt_y,
                                                              danger_adjust * crnt_danger + d_at_goal +
                                                              dist_map->get_danger_at( crnt_x, crnt_y ) );
        }
      }
    }
  }
}


void danger_grid::encourage_right( )
{
  bool reached_edge_of_grid;
  vector< vector< bool> > in_list_of_pts;
  vector< coord > pts;
  natural deviation_min = 12;
  natural deg_per_deviation = 6; // how far left we deviate each time
  
  
  // Intially, no squares are present in the list of points; intialize it to false.
  in_list_of_pts.resize( get_width_in_squares() ); 
  for( unsigned int x = 0; x < get_width_in_squares(); x++ )
  {
    in_list_of_pts[ x ].resize( get_height_in_squares(), false );
  }
  
  Position start_pos = (*owner).getLocation();
  
  // For each deviation from a straight line to the goal that we're adding cost to . . .
  for( natural i = 0; i < 4; i++ )
  {
    reached_edge_of_grid = false;
    
    // Starting adding cost to squares which are four times the
    // "resolution" distance from the plane start; get closer to the plane as the
    // angle widens
    natural dist = 4 * (natural)get_res();
    natural deviation = i*deg_per_deviation + deviation_min;
    double plane_bearing = (*owner).getBearing();
    
#ifdef DEBUG_DG
    cout << "Plane says its bearing is " << plane_bearing;
    cout << ", which gets named " << map_tools::bearing_to_string( (*owner).get_named_bearing() ) << endl;
    cout << "At round " << i << " we're working with deviation " << deviation << endl;
#endif
    
    // Until we have added squares all the way to the edge . . .
    while( !reached_edge_of_grid )
    {
      natural the_x, the_y;
      
      // Get the grid square which is one resolution-length farther away than the last
      
      double end_lat, end_lon;
      // Get the ending point in latitude and longitude
      map_tools::calculate_point( start_pos.getLat(), start_pos.getLon(), 
                                  dist, plane_bearing - deviation,
                                  end_lat, end_lon );
      double d_from_origin = 
        map_tools::calculate_distance_between_points( start_pos.getUpperLeftLatitude(), 
                                                      start_pos.getUpperLeftLongitude(),
                                                      end_lat, end_lon, "meters");
      
      double bearing; // in radians!
      
      if( d_from_origin > -EPSILON && d_from_origin < EPSILON ) // d == 0
      {
        bearing = 0;
      }
      else
      {
        bearing = map_tools::calculate_bearing_in_rad( start_pos.getUpperLeftLatitude(), 
                                                       start_pos.getUpperLeftLongitude(),
                                                       end_lat, end_lon );
        
        if( bearing > 0 )
        {
          if( bearing < PI/2 )
          {
            bearing -= PI/2;
          }
          else
            bearing = PI/2 - bearing;
        }
        bearing = fmod( bearing, PI/2 );
      }
      
#ifdef DEBUG_DG
      if( bearing > 0.001 )
      {
        cout << "That bearing of " << bearing << "is gonna break things!" << endl;
        cout << "Your point was (" << end_lat << ", " << end_lon << ")" << endl;
      }
      assert( bearing < 0.001 );
      assert( bearing > -PI/2 - 0.01 );
#endif
      
      the_x = (int)( (int)(cos( bearing ) * d_from_origin + 0.5) / get_res() );
      the_y = -(int)( (int)(sin( bearing ) * d_from_origin - 0.5) / get_res() );
      
#ifdef DEBUG_DG
      assert( (int)the_x < start_pos.getWidth() + 5 );
      assert( (int)the_y < start_pos.getHeight() + 5 );
      assert( the_x >= 0 );
      assert( the_y >= 0 );
#endif
      
      // If we've reached the edge of the grid, the thing we just pushed back will
      // serve as the "marker" indicating we've moved to the next deviation width
      if( the_x >= get_width_in_squares() || the_y >= get_height_in_squares() )
      {
        reached_edge_of_grid = true;
        
        // Add this to the list of points anyway, as a marker that we're moving to
        // the next deviation
        pts.push_back( coord(the_x, the_y) );
      }
      else if( the_x == 0 || the_y == 0 ) // these are unsigned, so they'll never be 
      {                                  // less than zero
        reached_edge_of_grid = true;
        
        // This point is perfectly fine
        if( !in_list_of_pts[ the_x ][ the_y ] )
        {
          pts.push_back( coord(the_x, the_y) );
          in_list_of_pts[ the_x ][ the_y ] = true;
#ifdef DEBUG_DG
          cout << "Added to list: (" << the_x << ", " << the_y << ")" << endl;
#endif
        }
        
        // Add to the list a marker indicating we're moving to the next deviation
        pts.push_back( coord( start_pos.getWidth() + 100, start_pos.getHeight() + 100 ) );
      }
      else
      {
        // If we haven't already added this (x, y) to the list of things to which
        // we will add cost . . .
        if( !in_list_of_pts[ the_x ][ the_y ] )
        {
          pts.push_back( coord(the_x, the_y) ); // add it.
          in_list_of_pts[ the_x ][ the_y ] = true;
#ifdef DEBUG_DG
          cout << "Added to list: (" << the_x << ", " << the_y << ")" << endl;
#endif
        }
      }

      dist += (natural)get_res(); // increase the distance for the next round
    } // end while we haven't reached the edge
  } // end for each deviation from a straight line
  
  double added_cost = default_plane_danger * 0.1; // begin with the highest cost we will add
  
  pts.pop_back(); // the last thing we added was a marker
  
  while( !pts.empty() )
  {
    coord back = pts.back();
    
    // If this is not a marker . . .
    if( back.x < get_width_in_squares() && back.y < get_height_in_squares() )
    {
      // Add cost at this square
      encouraged_right->add_danger_at( back.x, back.y, added_cost);

      pts.pop_back(); // done with this point
    }
    else // this was a marker
    {
      pts.pop_back(); // nuke it!
      added_cost *= 0.8; // decrease the cost we're adding for the next round
    }
  }
}

double danger_grid::get_dist_cost_at( unsigned int x_pos, unsigned int y_pos ) const
{
  if( distance_costs_initialized )
  {
    return dist_map->get_danger_at(x_pos, y_pos);
  }
#ifdef DEBUG
  else
  {
    assert( false );
  }
#endif
}

void danger_grid::dump( int time ) const
{
#ifdef DEBUG
  assert( time + (int)look_behind < (int)( danger_space->size() ) || time == 10000 );
#endif
  
  if( time == 10000 )
  {
#ifdef OVERLAYED
    overlayed[0].dump();
#endif
  }
  else
  {
    // the meat of the dump is performed by the map class
    (*danger_space)[ time + look_behind ].dump();
  }
}

void danger_grid::dump_big_numbers( int time ) const
{
#ifdef DEBUG
  assert( time + (int)look_behind < (int)( danger_space->size() ) || time == 10000 );
#endif
  
  if( time == 10000 )
  {
#ifdef OVERLAYED
    overlayed[0].dump_big_numbers();
#endif
  }
  else
  {
    // the meat of the dump is performed by the map class
    (*danger_space)[ time + look_behind ].dump_big_numbers();
  }
}

void danger_grid::dump_csv( int time, string prefix, string name ) const
{
#ifdef DEBUG
  assert( time + (int)look_behind < (int)( danger_space->size() ) || time == 10000 );
#endif
  (*danger_space)[ time + look_behind ].dump_csv( prefix, name );
}

#endif

