// Used in the map class; stores a danger rating and the IDs of any aircraft
// present in this square
#ifndef GRID_SQUARE
#define GRID_SQUARE

#include <vector>

using namespace std;

struct grid_square
{
    vector< unsigned int > planes;
    double danger;
    
    /**
     * The constructor for a grid square; sets danger to 0 initially
     */
    grid_square( )
    {
        danger = 0;
    }
    /**
     * The constructor for a grid square; adds only one plane
     * @param initial_danger a double storing the danger rating for the square
     * @param id the integer ID of the only plane to be added initially
     */
    grid_square( double initial_danger, unsigned int id )
    {
        danger = initial_danger;
        planes.push_back( id );
    }
    
    /**
     * The constructor for a grid square; adds only one plane
     * @param initial_danger a double storing the danger rating for the square
     * @param planes a vector of integer IDs of the planes to be added initially
     */
    grid_square( double initial_danger, vector< unsigned int > initial_planes )
    {
        danger = initial_danger;
        planes = initial_planes;
    }
    
    /**
     * The constructor for a grid square; adds no planes
     * @param initial_danger a double storing the danger rating for the square
     */
    grid_square( double initial_danger )
    {
        danger = initial_danger;
    }
    
    void operator= ( unsigned int value )
    {
        danger = value;
    }
    
    double get_danger( )
    {
        return danger;
    }
    
    void set_danger( double new_danger )
    {
        danger = new_danger;
    }
    
    void add_plane( unsigned int id )
    {
        planes.push_back( id );
    }
    
    vector< unsigned int > get_planes( )
    {
        return planes;
    }
};

#endif