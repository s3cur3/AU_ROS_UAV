//
// map.h
// AU_UAV_ROS
//
// Created by Tyler Young on 5/21/11.
//
// A class that stores a two-dimensional representation of the world;
// Each square in this grid has a danger associated with it, and it may have
// one or more aircraft present in it.

#ifndef PLANE_DANGER
#define PLANE_DANGER 0.98
#endif

#ifndef EPSILON
#define EPSILON 0.000001
#endif

#ifndef MAP
#define MAP

#include <vector>
#include <cassert>
#include <iostream>
#include <math.h> // only for ceil()
#include "map_tools.h"

using namespace std;

// A single square in the map; it simply stores the planes in it (if any) and the
// "danger rating" for passing through this square
struct grid_square
{
  vector< unsigned int > planes;
  double danger;
  
  /**
   * The constructor for a grid square; sets danger to 0 initially
   */
  grid_square( )
  {
    danger = 0.0;
  }
};

// A map is composed of grid squares
// It has a width, a height, and a resolution (the size
// of a single grid square--note that the squares are truly square!)
//
// Note that the width, height, and resolution may be in any units, but they must
// be consistent across all measurements.
// Don't forget: latitude is horizontal, longitude is vertical
class map
{
public:
  /**
   * The constructor for a map object
   *
   * Note that the width, height, and resolution may be in any units, but the units
   * must be consistent across all measurements.
   * @param width_of_field The width of the flyable area, in meters
   * @param height_of_field The height of the flyable area, in meters
   * @param map_resolution The resolution (width and height of a given square),
   * in meters
   */
  map( double width_of_field, double height_of_field, double map_resolution );
  
  // Destructor
  ~map( );
  
  /**
   * Return IDs of all aircraft in a given (x,y) square
   * @param x_pos the x position of the square in question
   * @param y_pos the y position of the square in question
   * @return a vector of unsigned ints corresponding to the unique IDs of any
   * aircraft in this square
   */
  vector< unsigned int > get_planes_at( unsigned int x_pos, unsigned int y_pos ) const;
  
  /**
   * Add an aircraft (i.e., its unique ID) to a given (x,y) square
   * NOTE: Adding a plane automatically sets the danger for this square
   * to near-maximum
   * @param x_pos the x position of the square in question
   * @param y_pos the y position of the square in question
   * @param id the unique integer identifying the aircraft
   */
  void add_plane_at( unsigned int x_pos, unsigned int y_pos, unsigned int id );
  
  /**
   * Return the danger rating of a square
   * @param x_pos the x position of the square in question
   * @param y_pos the y position of the square in question
   * @return a double containing the square's "danger" rating
   */
  double get_danger_at( unsigned int x_pos, unsigned int y_pos ) const;
  
  /**
   * Add to the danger rating of a square
   * @param x_pos the x position of the square to set
   * @param y_pos the y position of the square to set
   * @param danger the danger to be assigned to this square
   */
  void add_danger_at( unsigned int x_pos, unsigned int y_pos, double danger );

  /**
   * Set the danger rating of a square
   * @param x_pos the x position of the square to set
   * @param y_pos the y position of the square to set
   * @param danger the danger to be assigned to this square
   */
  void set_danger_at( unsigned int x_pos, unsigned int y_pos, double danger );

  
  unsigned int get_width_in_squares( ) const;
  double get_width_in_meters( ) const;
  unsigned int get_height_in_squares( ) const;
  double get_height_in_meters( ) const;
  
  unsigned int get_resolution( ) const; // (in whatever unit you're using)
  
  // Prints the contents of the map, once with the aircraft and their locations,
  // and once with the danger values. Used for testing.
  void dump( ) const;

  void dump_big_numbers( ) const;

private:
  vector< vector< grid_square > > the_map; // a 2-D vector array of grid squares
  double width; // the x dimension, in your system of measurement (e.g., meters)
  double height; // the y dimension in your system of measurement
  double resolution; // the width and height of a single square in the map, in your system of measurement
  unsigned int squares_wide; // the x dimension, in squares
  unsigned int squares_high; // the y dimension, in squares
};

map::map( double width_of_field, double height_of_field, double map_resolution )
{
#ifdef DEBUG
  assert( width_of_field > EPSILON );
  assert( height_of_field > EPSILON );
  assert( map_resolution > EPSILON );
#endif
  
  width = width_of_field;
  height = height_of_field;
  resolution = map_resolution;
  
  // Let this function fill in the values for width and height in squares
  squares_wide = map_tools::find_width_in_squares( width_of_field, height_of_field,
                                                  map_resolution );
  squares_high = map_tools::find_height_in_squares( width_of_field, height_of_field,
                                                   map_resolution );
#ifdef DEBUG
  assert( squares_high != 0 && squares_high < UINT_MAX );
  assert( squares_wide != 0 && squares_wide < UINT_MAX);
#endif
  
  // This *should* create a 2-d array accessed in [x][y] order
  the_map.resize( squares_wide );
  
  for( unsigned int i = 0; i < the_map.size(); ++i )
    the_map[ i ].resize( squares_high );
  
  for( vector< vector< grid_square > >::iterator i = the_map.begin();
      i != the_map.end(); ++i )
  {
    for( vector< grid_square >::iterator j = i->begin(); j != i->end(); ++j )
    {
      j->danger = 0;
    }
  }
}

map::~map( ) { }

vector< unsigned int > map::get_planes_at( unsigned int x_pos, unsigned int y_pos ) const
{
  return the_map[ x_pos ][ y_pos ].planes;
}

void map::add_plane_at( unsigned int x_pos, unsigned int y_pos, unsigned int id )
{
  the_map[ x_pos ][ y_pos ].planes.push_back( id );
  the_map[ x_pos ][ y_pos ].danger = PLANE_DANGER;
}

double map::get_danger_at( unsigned int x_pos, unsigned int y_pos ) const
{
#ifdef DEBUG
  assert( x_pos < the_map.size() );
  assert( y_pos < the_map[1].size() );
#endif
  return the_map[ x_pos ][ y_pos ].danger;
}

void map::add_danger_at( unsigned int x_pos, unsigned int y_pos, double new_danger )
{
#ifdef DEBUG
  assert( x_pos < the_map.size() );
  assert( y_pos < the_map[1].size() );
#endif
  if( the_map[ x_pos ][ y_pos ].danger > EPSILON )
  {
    // Add to the danger rating, don't simply change it
    if( new_danger > the_map[ x_pos ][ y_pos ].danger )
    {
      the_map[ x_pos ][ y_pos ].danger = new_danger + 
                                         0.25 * the_map[ x_pos ][ y_pos ].danger;
    }
    else
      the_map[ x_pos ][ y_pos ].danger += 0.25 * new_danger;
  }
  else // simply change the danger rating
    the_map[ x_pos ][ y_pos ].danger = new_danger;
#ifdef SMALL_COSTS
  if( the_map[ x_pos ][ y_pos ].danger > 1 )
    the_map[ x_pos ][ y_pos ].danger = PLANE_DANGER;
#endif
}

void map::set_danger_at( unsigned int x_pos, unsigned int y_pos, double new_danger )
{
#ifdef DEBUG
  assert( x_pos < the_map.size() );
  assert( y_pos < the_map[1].size() );
#endif
  
  the_map[ x_pos ][ y_pos ].danger = new_danger;

#ifdef SMALL_COSTS
  if( the_map[ x_pos ][ y_pos ].danger > 1 )
    the_map[ x_pos ][ y_pos ].danger = PLANE_DANGER;
#endif
}

unsigned int map::get_width_in_squares( ) const
{
  return squares_wide;
}

double map::get_width_in_meters( ) const
{
  return width;
}

unsigned int map::get_height_in_squares( ) const
{
  return squares_high;
}

double map::get_height_in_meters( ) const
{
  return height;
}

unsigned int map::get_resolution( ) const
{
  return (unsigned int)resolution;
}

void map::dump( ) const
{
  cout << endl << " Danger ratings:" << endl;
  cout << "    ";
  for( unsigned int top_guide = 0; top_guide < the_map.size(); ++top_guide )
  {
    if( top_guide < 10 )
      cout << " " << top_guide << " ";
    else
      cout << top_guide << " ";
  }
  cout << endl;
  
  for( unsigned int right_index = 0; right_index < the_map[ 0 ].size(); ++right_index )
  {
    if( right_index < 10 )
      cout << " " << right_index << " ";
    else
      cout << right_index << " ";
    
    for( unsigned int left_index = 0; left_index < the_map.size(); ++left_index )
    {
      if( the_map[ left_index ][ right_index ].danger < EPSILON &&
         the_map[ left_index ][ right_index ].danger > -EPSILON )
      {
        cout << " --";
      }
      else
      {
#ifndef SCALE
#define SCALE 1
#endif
        if( (the_map[ left_index ][ right_index ].danger)*SCALE - SCALE > -EPSILON )
          printf("%3.0f", (the_map[ left_index ][ right_index ].danger)*SCALE );
        else
          printf( "%2.0f ", (the_map[ left_index ][ right_index ].danger)*SCALE );
      }
    }
    cout << endl;
  }
  /*cout << endl << " Aircraft present in squares (where more than one are"
   << " present, only the first is listed): " << endl;
   for( unsigned int right_index = 0; right_index < the_map[ 0 ].size(); ++right_index )
   {
   for( unsigned int left_index = 0; left_index < the_map.size(); ++left_index )
   {
   if( the_map[ left_index ][ right_index ].danger < PLANE_DANGER )
   {
   cout << " - ";
   }
   else
   {
   printf( "%2d ", the_map[ left_index ][ right_index ].planes[0] );
   }
   }
   cout << endl;
   }*/
}

void map::dump_big_numbers( ) const
{
  cout << endl << " Danger ratings:" << endl;
  cout << "    ";
  for( unsigned int top_guide = 0; top_guide < the_map.size(); ++top_guide )
  {
    if( top_guide < 10 )
      cout << " " << top_guide << " ";
    else
      cout << top_guide << " ";
  }
  cout << endl;
  
  for( unsigned int right_index = 0; right_index < the_map[ 0 ].size(); ++right_index )
  {
    if( right_index < 10 )
      cout << " " << right_index << " ";
    else
      cout << right_index << " ";
    
    for( unsigned int left_index = 0; left_index < the_map.size(); ++left_index )
    {
      if( the_map[ left_index ][ right_index ].danger < EPSILON &&
         the_map[ left_index ][ right_index ].danger > -EPSILON )
      {
        cout << " --";
      }
      else
      {
#define BIG_SCALE 1
        if( (the_map[ left_index ][ right_index ].danger) > 1.6e+307 )
          printf( " in" );
        else if( (the_map[ left_index ][ right_index ].danger)/BIG_SCALE - 1/BIG_SCALE > -EPSILON )
          printf( "%3.0f", (the_map[ left_index ][ right_index ].danger)/BIG_SCALE );
        else
          printf( "%2.0f ", (the_map[ left_index ][ right_index ].danger)/BIG_SCALE );
      }
    }
    cout << endl;
  }
}

#endif