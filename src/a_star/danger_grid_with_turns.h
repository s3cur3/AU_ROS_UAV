//
//  danger_grid.h
//  AU_UAV_ROS
//
//  Created by Tyler Young on 5/23/11.
//  Plane prediction by Thomas Crescenzi.
//
// A class to generate a grid with a "danger" rating associated with
// each block in the flyable area (which is of our bc::map type).
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

#include <vector>
#include <map> // std::map, which stores our set of aircraft
#include <math.h>
#include <climits>

#include "map_cleaner.h"
#include "estimate.h"
#include "Plane_fixed.h"
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
   * Adds a small cost to grid squares on the left of the aircraft, effectively 
   * implementing a preference for right-hand turns when path planning using A*.
   * @param plane The "owner" of this BC grid, from whose starting location and
   *              toward whose goal we will be adding cost.
   */
  void encourage_right( natural width_in_sqrs, natural height_in_sqrs, double resolution );
  
  /**
   * Output the map at a given time to the standard output; for troubleshooting only
   * @param time The time, in seconds, whose map should be output
   */
  void dump( int time ) const;
  
  /**
   * Output the map to a CSV file.
   * For troubleshooting the best cost grid
   * @param time The time, in seconds, whose map should be output
   */
  void dump_big_numbers( int time ) const;
  
  /**
   * Output the map to a CSV file, whose path is specified in the map class.
   * For troubleshooting only
   * @param prefix A string to be printed as the first thing in this CSV file;
   *               use it to add the plane's goal, the time this file is created,
   *               your fortune cookie's lucky numbers, &c.
   * @param name The file name (output will be [name].csv)
   * @param time The time, in seconds, whose map should be output
   */
  void dump_csv( int time, string prefix, string name ) const;
  
private:  
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
  // A Little Side Note by Thomas
  /*
	The domain of the bearing used in all of the functions by me here is diffrent than the one used elsewhere in the project.
	This is because I was originally told that the bearing went from -180 to 180. Every angle is covered in this domain and 
	it works rather well with the other stuff in the project. It wouldn't be too hard to change the domain but it does
	mean changing a ton of ifs. You have been warned :).
  */

  /** 
   a function for predicting planes. most of the work is done in the recursive calling of 
   danger recurse but this guy starts the whole process. relies on dangerRecurse(), neighoboringAngles(),
   and placeDanger(). it is built to be called 2 times. use the same variable for time in each instances.
   it decides which point, the final destination or just the goal, to fly to based on the value in time.
   if time is =0 it assumes the destination is where it is going otherwise it assumes it is flying from the 
   destination to the goal. if there is only a goal then the destination will be the goal based on the way that 
   planes and the telemetry data works. the prediction uses the function turn() to predict turns as
   best as it can. hopefully the turns can eventually be predicted better however as of now the prediction
   works well enough for most of our purposes. turns are predicted both from the start to the avoidance point
   and from the avoidance point to the goal. note: this function assumes a basic cartesian grid. p.s. this is by thomas
   input:(yes there is more to this function than just a description)
   @param plane the plane whose path you are predicting
   @param time the time from which you are starting prediction, must be >=0
   output:
   @return a vector that contains estimates of the planes path. as the plane traves through time a (0,0,-1) estimate is inserter
   as a time marker.
   **/
  vector< estimate > calculate_future_pos( Plane & the_plane, int & time );
  
	/**
	a function that finds the neighbors of a given angle. a neighboring angle is one of the angles
	that perfectly bisects a neighboring square and is within 45 degrees of @param angle.
	input:
		@param angle the angle you are finding the neighbors of
		@param first 
		@param second both are out parameters only 0s are expected in
	output:
		first: the closest angle
		second: the next closest angle
	**/
  void neighoboringAngles(double angle, double &first, double &second);
	/**
	a function that "places" danger into the squares that neighbor the current x,y location
	input:
		@param angle the bearing from the location to the goal location
		@param e a vector of "estimates" 
		@param closest the angle that is closest to angle that bisects a neighboring square
		@param other the next closest angle
		@param x the x location in the grid(of the current location not the one that you place the danger in)
		@param y the y location in the grid
	output:
		e is changed to contain the the estimated danger in the new location
	**/
  void placeDanger(double angle, vector<estimate> &e, double closest, double other, int x, int y, double danger);
	/**
	the recursive function that calculates the plane's path
	input:
		@param e an "estimate"
		@param destination[] a pair of x,y that represents where the plane is going
		@param theFuture a vector of estimates
		@param time an int used for calculating the total time the plane is predicted for
	output:
		it returns both a vector of extimates and the total time value as outparameters
	**/
  void dangerRecurse(estimate e, int destination[], vector<estimate> &theFuture, int & time);
/**
	a function that predicts turns made by planes. it bases this turn off of an assumption of a 22.5 turning angle.
	the direction of the turn depends on wether the goal(or aovidance point) is closer on the right or left. if the goal is equally close 
	it is assumed that a right hand turn will be made.
	input:
		@param startingAngle the bearing from which the plane is starting to make the turn
		@param endAngle the bearing at which the plane plans to end the turn within +/-22.5 degrees of
		@param e a vector that contains estimates
		@param x the starting x location in the grid
		@param y the starting y location in the grid
		@param x2 the x location of the goal
		@param y2 the y location of the goal
	output:
		e is changed to contain the estimated turn that will be made
		x is set to be the x location of the plane at the end of the turn
		y is set to be the y location of the plane at the end of the turn
**/
  void turn(double startingAngle, double endAngle, vector<estimate> &e, int &x, int &y, int x2, int y2);
  
  // Back to Tyler's stuff
  
  // the weighting applied to danger estimates in the future
  vector< double > danger_ratings;
  
  vector< double > plane_danger; // in case the plane danger were to change over time
  
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
  
  // a value calculated by plane prediction for use of calculating the turn made
  // after an avoidance point
  double bearingAfterAvoid;
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
  assert( (*owner).getId() != -100 );
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
    // We're currently not using the right-hand encouragement
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
    Plane * current_plane = &( (*plane_pair).second ); // avoid the double indirection
    
#ifdef DEBUG
    assert( (*current_plane).getId() != -100 );
    
    if( (*current_plane).getLocation().getY() > 10000 )
      cout << " plane " << (*current_plane).getId() << endl;
    assert( (*current_plane).getLocation().getX() < 10000 );
    assert( (*current_plane).getLocation().getY() < 10000 );
#endif
    
    // If this is not the "owner" of the danger grid . . . 
    if( (*current_plane).getId() != (int)plane_id )
    {
      // Set the danger at the plane's starting location
      (*danger_space)[0 + look_behind].
        add_danger_at( (*current_plane).getLocation().getX(),
                      (*current_plane).getLocation().getY(), 
                      default_plane_danger);
#ifdef OVERLAYED
      overlayed[0].add_danger_at((*current_plane).getLocation().getX(),
                                 (*current_plane).getLocation().getY(), 1.0);
#endif
      
      // Get the estimated danger for relevant squares in the map at this time
      int dummy = 0;
      est_to_avoid = calculate_future_pos( *current_plane, dummy );
      est_to_goal = calculate_future_pos( *current_plane, dummy );
      
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
            // calculate_future_pos() found 
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
      ++t; // increment t because the estimate ends with a good value
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
  map_tools::bearing_t named_bearing = map_tools::name_bearing( bearing );
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
  
  // These buffer zones have been made wider in the direction of the plane's travel
  // in light of A*'s propensity for taking risky paths.
  switch( named_bearing )
  {
    case map_tools::N:
      (*danger_space)[ time ].safely_add_danger_at( x - 2, y - 3, d );
      // dag left+up
      (*danger_space)[ time ].safely_add_danger_at( x - 1, y - 3, d );
      // straight up
      (*danger_space)[ time ].safely_add_danger_at( x , y - 3, d );
      // dag right+up
      (*danger_space)[ time ].safely_add_danger_at( x + 1, y - 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 2, y - 3, d );
      break;
      
    case map_tools::NE:
      // straight up
      (*danger_space)[ time ].safely_add_danger_at( x , y - 3, d );
      // dag right+up
      (*danger_space)[ time ].safely_add_danger_at( x + 1, y - 3, d );
      
      (*danger_space)[ time ].safely_add_danger_at( x + 2, y - 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y - 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y - 2, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y - 1, d );
      // straight right
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y , d );
      break;
      
    case map_tools::E:
      // dag right+up
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y - 2, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y - 1, d );
      // straight right
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y , d );
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y + 1, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y + 2, d );
      break;
      
    case map_tools::SE:
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y + 1, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y + 2, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 3, y + 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 2, y + 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 1, y + 3, d );
      // straight down
      (*danger_space)[ time ].safely_add_danger_at( x , y + 3, d );
      break;
      
    case map_tools::S:
      (*danger_space)[ time ].safely_add_danger_at( x - 2, y + 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 1, y + 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x , y + 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 1, y + 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x + 2, y + 3, d );
      break;
      
    case map_tools::SW:
      // straight down
      (*danger_space)[ time ].safely_add_danger_at( x , y + 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 1, y + 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 2, y + 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y + 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y + 2, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y + 1, d );
      // straight left
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y, d );
      break;
      
    case map_tools::W:
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y - 2, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y - 1, d );
      // straight right
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y , d );
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y + 1, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y + 2, d );
      break;
      
    case map_tools::NW:
      // straight left
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y - 1, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y - 2, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 3, y - 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 2, y - 3, d );
      (*danger_space)[ time ].safely_add_danger_at( x - 1, y - 3, d );
      // straight up
      (*danger_space)[ time ].safely_add_danger_at( x , y - 3, d );
      break;
      
  } // end switch case
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


vector< estimate > danger_grid::calculate_future_pos( Plane & plane, int &time )
{
  bool turned=false;//did the plane turn?
  
  vector< estimate > theFuture;//prevents memory errors
	theFuture.reserve(1);
	
  Position current, destination;
  
  if(time==0)//meaning that the plane is now moving towards it's next goal be it an avoidance point or a final destination
	{
  	current=plane.getLocation();
  	destination=plane.getDestination();
    }
	
  else//the function will always be called twice. if its moving to its final goal the distance will be 0 and it will break out immediatly
	{
		destination=plane.getFinalDestination();
		current=plane.getDestination(); 
	}

  //distance formula: line to destination
  int x1=current.getX(),x2=destination.getX(),y1=current.getY(),y2=destination.getY();
  double xDistance=( fabs((double)x2-x1) ),yDistance=( fabs((double)y2-y1) );
  double distance = sqrt((double)(xDistance*xDistance)+(yDistance*yDistance));

  if(xDistance==0&&yDistance==0)//your there!!!!!!!(hopefully) or your next destination was your goal
    {time++; return theFuture;}

  //find the angle to the waypoint
  double angle=(180-RADtoDEGREES*(asin((double)xDistance/(double)distance)));
  if(y2<y1)
    angle=(RADtoDEGREES*(asin((double)xDistance/(double)distance)));
  
  if((x2-x1)<0)//positive means that the plane is headed to the left aka west
    angle=(-1)*angle;//the plane goes from -180 to +180(this is diffrent that the bearing domain used in other parts of this project)
  double bearing=0;
  
	//begin looking to see if a turn needs to be made
  if(time==0)
    bearing = plane.getBearing();//the planes bearing
  else
    bearing=bearingAfterAvoid;//an estimated bearing upon the planes arrival to the avoidance point, not 100% correct because it is based off of guessed positions
	
  map_tools::bearing_t bearingNamed = map_tools::name_bearing(bearing);
  
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
  //check to see if a turn needs to be made
  if(fabs(fabs(angle)-fabs(bearing))>22.5)
	{
		turned=true;//yup the plane turned
		turn(bearing,angle,theFuture,x1,y1,x2,y2);
		//recalculate everything from the current loaction after the turn
		xDistance=fabs((double)x2-x1);
		yDistance=fabs((double)y2-y1);
		
		if(x1==x2 && y1==y2)//your there? odd place to want to go
		  {time++; theFuture.pop_back(); return theFuture;}
		  
		distance = sqrt((double)(xDistance*xDistance)+(yDistance*yDistance));

		angle=(180-RADtoDEGREES*(asin((double)xDistance/(double)distance)));
		if(y2<y1)
				angle=(RADtoDEGREES*(asin((double)xDistance/(double)distance)));
		if((x2-x1)<0)//positive means that the plane is headed to the left aka west
			angle=(-1)*angle;//the plane goes from -180 to +180
  }

  //find closest straight line angle
  double neighbors[2];
  neighoboringAngles(angle, neighbors[0], neighbors[1]);
	
	//find which angle is closer
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

  if(theFuture[theFuture.size()-2].danger>.3)
    dangerRecurse(theFuture[theFuture.size()-2], dest, theFuture, time);
	else
  dangerRecurse(theFuture[theFuture.size()-3],dest,theFuture,time);
  
  //calculate the bearing after the avoidance point
  xDistance=fabs((double)theFuture[theFuture.size()-4].x-theFuture[theFuture.size()-2].x);
  yDistance=fabs((double)theFuture[theFuture.size()-4].y-theFuture[theFuture.size()-2].y);
  distance = sqrt((double)(xDistance*xDistance)+(yDistance*yDistance));
  angle=(180-RADtoDEGREES*(asin((double)xDistance/(double)distance)));
  if(y2<y1)
    angle=(RADtoDEGREES*(asin((double)xDistance/(double)distance)));
  if((x2-x1)<0)//positive means that the plane is headed to the left aka west
    angle=(-1)*angle;//the plane goes from -180 to +180
	bearingAfterAvoid=angle;
  
  
	//add prediction to one square ahead of goal
	//find closest straight line from target bearing
	angle=plane.getBearingToDest();
  neighoboringAngles(angle, neighbors[0], neighbors[1]);
  if(fabs(angle-neighbors[0])>=fabs(angle-neighbors[1]))//distance
  {closestAngle=neighbors[1]; otherAngle=neighbors[0];}
  else
  {closestAngle=neighbors[0]; otherAngle=neighbors[1];}
	
	//predict 3 spaces past the goal
	//note this could be changed into a for loop however as i knew we wanted exactly 3 spaces past i didn't bother
  if(destination.getX()==plane.getFinalDestination().getX() && destination.getY()==plane.getFinalDestination().getY())
  {
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
  }
	
  return theFuture;
}

void danger_grid::dangerRecurse(estimate e, int destination[], vector<estimate> &theFuture,int &time)
{
  //basically does most of the work and is a slimmed down version of calculate_furture_pos built to be called recursivly
	//this function is simply an implementation of the algorthim i came up with and speak about in our paper
	//it would be 100% possiable to make it not recursive just check out the pseudocode in the paper and see what you can do with it
  int x1=e.x;
  int y1=e.y;
  int x2=destination[0];
  int y2=destination[1];
  int dest[2]={x2,y2};
  
  double xDistance=( fabs((double)x2-x1) ), yDistance=( fabs((double)y2-y1) );
  if(xDistance==0&&yDistance==0)//your there!!!!!!!(hopefully)
		{return;}
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

void danger_grid::neighoboringAngles(double angle, double &first, double &second)
{
	//this is a part that would need to be changed if the domain changed
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


void danger_grid::placeDanger(double angle, vector<estimate> &e, double closest, double other, int x, int y, double danger)
{
	//another place the domain would change things
	
	//apply a ceiling to the danger to make things eaiser for A* to search
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

void danger_grid::turn(double startingAngle, double endAngle, vector<estimate> &e, int &x, int &y, int x2, int y2)
{
	
	int inside_outside=2;//decided which estimated position to pick 3 is outside 2 is inside
	//inside makes for a wider turn outside makes for a sharper turn 2 tends to be a better choice
	double rightDistance,leftDistance;
	
	//based on the starting angle the turn will look diffrent
	switch((int) startingAngle)
	{
		case 0://NORTH
		
		rightDistance=sqrt( (x2-(x+1))*(x2-(x+1)) + (y2-(y))*(y2-(y)) );
		leftDistance=sqrt( (x2-(x-1))*(x2-(x-1)) + (y2-(y))*(y2-(y)) );
		while(fabs(fabs((double)endAngle)-fabs((double)startingAngle)) >22.5)
		{
			if(rightDistance<=leftDistance)
			{
				placeDanger(22.5 , e , 0 , 45 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)//pretty sure it will always trigger here but i could be wrong
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		
			else
			{
				placeDanger(-22.5 , e , 0 , -45 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		}
		break;

		case 45://NORTHEAST

		rightDistance=sqrt( (x2-(x+1))*(x2-(x+1)) + (y2-(y+1))*(y2-(y+1)) );
		leftDistance=sqrt( (x2-(x-1))*(x2-(x-1)) + (y2-(y-1))*(y2-(y-1)) );

		while(fabs(fabs((double)endAngle)-fabs((double)startingAngle)) >22.5)
		{
			if(rightDistance<=leftDistance)
			{
				placeDanger(67.5 , e , 45 , 90 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		
			else
			{
				placeDanger(22.5 , e , 45 , 0 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		}
		break;

		case 90://EAST

		rightDistance=sqrt( (x2-(x))*(x2-(x)) + (y2-(y+1))*(y2-(y+1)) );
		leftDistance=sqrt( (x2-(x))*(x2-(x)) + (y2-(y-1))*(y2-(y-1)) );

		while(fabs(fabs((double)endAngle)-fabs((double)startingAngle)) >22.5)
		{
			if(rightDistance<=leftDistance)
			{
				placeDanger(112.5 , e , 90 , 135 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		
			else
			{
				placeDanger(67.5 , e , 90 , 45 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		}
		break;

		case 135://SOUTHEAST

		rightDistance=sqrt( (x2-(x-1))*(x2-(x-1)) + (y2-(y+1))*(y2-(y+1)) );
		leftDistance=sqrt( (x2-(x+1))*(x2-(x+1)) + (y2-(y-1))*(y2-(y-1)) );

		while(fabs(fabs((double)endAngle)-fabs((double)startingAngle)) >22.5)
		{
			if(rightDistance<=leftDistance)
			{
				placeDanger(157.5 , e , 135 , 180 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		
			else
			{
				placeDanger(112.5 , e , 135 , 90 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		}
		break;

		case 180://SOUTH
		case -180://SOUTH (awwwww yeaaaaaaa)
		
		rightDistance=sqrt( (x2-(x-1))*(x2-(x-1)) + (y2-(y))*(y2-(y)) );
		leftDistance=sqrt( (x2-(x+1))*(x2-(x+1)) + (y2-(y))*(y2-(y)) );

		while(fabs(fabs((double)endAngle)-fabs((double)startingAngle)) >22.5)
		{
			if(rightDistance<=leftDistance)
			{
				placeDanger(-157.5 , e , -180 , -135 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		
			else
			{
				placeDanger(157.5 , e , 180 , 135 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		}
		break;

		case -135://SOUTHWEST
		
		rightDistance=sqrt( (x2-(x-1))*(x2-(x-1)) + (y2-(y-1))*(y2-(y-1)) );
		leftDistance=sqrt( (x2-(x+1))*(x2-(x+1)) + (y2-(y-1))*(y2-(y-1)) );

		while(fabs(fabs((double)endAngle)-fabs((double)startingAngle)) >22.5)
		{
			if(rightDistance<=leftDistance)
			{
				placeDanger(-112.5 , e , -135 , -90 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		
			else
			{
				placeDanger(-157.5 , e , -135 , -180 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		}
		break;

		case -90://SOUTH
		
		rightDistance=sqrt( (x2-(x))*(x2-(x)) + (y2-(y-1))*(y2-(y-1)) );
		leftDistance=sqrt( (x2-(x))*(x2-(x)) + (y2-(y+1))*(y2-(y+1)) );

		while(fabs(fabs((double)endAngle)-fabs((double)startingAngle)) >22.5)
		{
			if(rightDistance<=leftDistance)
			{
				placeDanger(-67.5 , e , -90 , -45 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		
			else
			{
				placeDanger(-112.5 , e , -90 , -135 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		}
		break;

		case -45://NORTHWEST
		
		rightDistance=sqrt( (x2-(x+1))*(x2-(x+1)) + (y2-(y-1))*(y2-(y-1)) );
		leftDistance=sqrt( (x2-(x-1))*(x2-(x-1)) + (y2-(y+1))*(y2-(y+1)) );

		while(fabs(fabs((double)endAngle)-fabs((double)startingAngle)) >22.5)
		{
			if(rightDistance<=leftDistance)
			{
				placeDanger(-22.5 , e , -45 , 0 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		
			else
			{
				placeDanger(-67.5 , e , -45 , -90 , x,y,.5);
				e.push_back(estimate(0,0,-1));
				x=e[e.size()-inside_outside].x;
				y=e[e.size()-inside_outside].y;
				if(endAngle>startingAngle)
					startingAngle+=22.5;
				else
					startingAngle-=22.5;
			}
		}
		break;

		default:
			cout<<"Hmmm thats rather strange why are you here?\n Looks like your angle wasn't properlly assigned for a turn"<<endl;
	}
	return;
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
      dist_map->set_danger_at(crnt_x, crnt_y, 0.5 * sqrt( (crnt_x - goal_x)*(crnt_x - goal_x) + 
                                                    (crnt_y - goal_y)*(crnt_y - goal_y) ) );
    }
  }
  
  // Haven't had time to fully test the paths planned when using encourage_right,
  // so we aren't using it...
  // If you add this back in, be sure to uncomment the line in the destructor!
  /*
  //encouraged_right = new bc::map(*dist_map);
  
  // Add a bit of cost to the left side of the aircraft to encourage it to
  // take avoidance maneuvers to the right
  //encourage_right( dg->get_width_in_squares(), dg->get_height_in_squares(), dg->get_res() );
   */
  
  distance_costs_initialized = true;
  
  danger_space = new vector< bc::map >( look_ahead + look_behind + 1, (*dist_map) );
  
  double d_at_goal;
  
  for( unsigned int crnt_t = 0; crnt_t <= look_ahead; crnt_t++ )
  {
    //d_at_goal = (*dg).get_danger_at(goal_x, goal_y, crnt_t);
    d_at_goal = 0;
    
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


void danger_grid::encourage_right( natural width_in_sqrs, natural height_in_sqrs, double resolution )
{
  bool reached_edge_of_grid;
  vector< vector< bool> > in_list_of_pts;
  vector< coord > pts;
  natural deviation_min = 12;
  natural deg_per_deviation = 6; // how far left we deviate each time
  
  
  // Intially, no squares are present in the list of points; intialize it to false.
  in_list_of_pts.resize( width_in_sqrs ); 
  for( unsigned int x = 0; x < width_in_sqrs; x++ )
  {
    in_list_of_pts[ x ].resize( height_in_sqrs, false );
  }
  
  Position start_pos = (*owner).getLocation();
  
  // For each deviation from a straight line to the goal that we're adding cost to . . .
  for( natural i = 0; i < 4; i++ )
  {
    reached_edge_of_grid = false;
    
    // Starting adding cost to squares which are four times the
    // "resolution" distance from the plane start; get closer to the plane as the
    // angle widens
    double dist = 4 * resolution;
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
      
      the_x = (int)( (int)(cos( bearing ) * d_from_origin + 0.5) / resolution );
      the_y = -(int)( (int)(sin( bearing ) * d_from_origin - 0.5) / resolution );
      
#ifdef DEBUG_DG
      assert( (int)the_x < start_pos.getWidth() + 5 );
      assert( (int)the_y < start_pos.getHeight() + 5 );
      assert( the_x >= 0 );
      assert( the_y >= 0 );
#endif
      
      // If we've reached the edge of the grid, the thing we just pushed back will
      // serve as the "marker" indicating we've moved to the next deviation width
      if( the_x >= width_in_sqrs || the_y >= height_in_sqrs )
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

      dist += resolution; // increase the distance for the next round
    } // end while we haven't reached the edge
  } // end for each deviation from a straight line
  
  double added_cost = default_plane_danger * 0.028; // begin with the highest cost we will add
  
  pts.pop_back(); // the last thing we added was a marker
  
  while( !pts.empty() )
  {
    coord back = pts.back();
    
    // If this is not a marker . . .
    if( back.x < width_in_sqrs && back.y < height_in_sqrs )
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

