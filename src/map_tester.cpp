// map_tester.cpp
//
// Created by Tyler Young on 5/21/11.
//
// Adds some values to a map, prints them out

#include <iostream>
#include <cstdlib>
#include <string>
#include "map.h"

using namespace std;

int main() 
{
    // create a new map with a width of 500 meters and a height
    // of 200 meters, with a resolution of 10 meters square
    map * our_map = new map( 500.0, 200.0, 10.0 );
    cout << "Width of map:      " << our_map->get_width_in_squares( ) << " squares" << endl;
    cout << "Height of map:     " << our_map->get_height_in_squares( ) << " squares" << endl;
    cout << "Width of map:      " << our_map->get_width_in_meters( ) << " meters" << endl;
    cout << "Height of map:     " << our_map->get_height_in_meters( ) << " meters" << endl;
    cout << "Resolution of map: " << our_map->get_resolution( ) << " meters" << endl << endl;
    
    // Add Plane 0 at the lowest left square
    our_map->add_plane_at( 0, 0, 0 );
    our_map->add_plane_at( 10, 10, 2 );
    our_map->add_plane_at( 49, 19, 1 );
    our_map->add_plane_at( 13, 17, 3 );
    // (50, 20) would give an error! (We start counting at 0)
    // our_map->add_plane_at( 50, 20, 4 );
    our_map->add_plane_at( 49, 19, 4 );
    our_map->add_plane_at( 49, 19, 5 );



    cout << "The danger at (10,10) is " << our_map->get_danger_at( 10, 10 ) << endl;
    cout << "The danger at (13,17) is " << our_map->get_danger_at( 13, 17 ) << endl;
    cout << "The danger at (1,1) is " << our_map->get_danger_at( 1, 1 ) << endl;
    our_map->set_danger_at(1, 1, 0.3);
    cout << "The danger at (1,1) is now " << our_map->get_danger_at( 1, 1 ) << endl << endl;
    
    vector< unsigned int > plane_list = our_map->get_planes_at( 49, 19);
    cout << "The following planes are at (49,19): ";
    for( vector< unsigned int >::const_iterator i = plane_list.begin(); i != plane_list.end(); ++i )
    {
        cout << *i << " ";
    }
    cout << endl;
    
    plane_list = our_map->get_planes_at( 3, 3);
    cout << "The following planes are at (3,3): ";
    for( vector< unsigned int >::const_iterator i = plane_list.begin(); i != plane_list.end(); ++i )
    {
        cout << *i << " ";
    }
    cout << endl;
    
    
    our_map->dump( );
}