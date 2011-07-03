//
//  pushpins_from_courses.cpp
//  AU_UAV_ROS
//
//  Created by Tyler Young on 6/23/11.
//
// Used to create a push-pin KML file from a course file


#include <cstdlib>
#include <cassert>
#include <iostream>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include "map_tools.h"

#ifndef EPSILON
#define EPSILON 0.0000001
#endif

#ifndef TO_STRING
#define TO_STRING
template <class T>
inline std::string to_string( const T& t )
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}
#endif


std::string double_to_text(const double & d)
{
  std::stringstream ss;
  ss << std::setprecision( std::numeric_limits<double>::digits10+2);
  ss << d;
  return ss.str();
}

using namespace std;

#ifndef natural
#define natural unsigned int
#endif

// The following vars are required to define a position
// These are const because they remain constant for our airfield.
const double upper_left_longitude = -85.490363;
const double upper_left_latitude = 32.592425;
const double width_in_degrees_longitude = 0.005002;
const double height_in_degrees_latitude = -0.003808;

int main()
{
  // The directory from which to read the courses
  string course_dir = "/Volumes/DATA/Dropbox/school/Auburn/Code/AU_UAV_stack/AU_UAV_ROS/courses/";
  //string course_dir = "/home/trescenzi/Dropbox/Auburn/Code/AU_UAV_stack/AU_UAV_ROS/courses/";
  string name = "final_16_500m_sofa";
  char defaults;
  
  cout << "Defaults are an input file name and path of:"<< endl;
  cout << "   " << course_dir << name << ".course" << endl << endl;
  cout << "Use defaults (y/n)?  ";
  cin >> defaults;
  if( defaults == 'n' || defaults == 'N')
  {
    cout << "File path (e.g., /home/UserName/Desktop/):  ";
    cin >> course_dir;
    
    cout << "File name (excluding path and excluding .course suffix):  ";
    cin >> name;
  }

  // Build the filename strings
  stringstream ss( stringstream::out );
  ss << course_dir;
  ss << name.c_str() << ".course";
  string input_name_with_path = ss.str();
  
  stringstream out( stringstream::out );
  out << course_dir;
  out << name.c_str() << "_pushpins.kml";
  string output_name_with_path = out.str();
  
  // Open the files
  ofstream pushpin_file;
  ifstream input_file;
  input_file.open( input_name_with_path.c_str() );
  pushpin_file.open( output_name_with_path.c_str() );
  
  assert( input_file.is_open() );
  assert( pushpin_file.is_open() );
  
  
  
  // Write the header
  if( true ) // (Solely to allow us to collapse this block in Xcode)
  {
    pushpin_file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>" << endl;
    pushpin_file << "<kml xmlns=\"http://www.opengis.net/kml/2.2\">" << endl;
    pushpin_file << "  <Document>" << endl;
    pushpin_file << "    <name>" << name << " Goal Waypoints</name>" << endl;
    pushpin_file << "    <open>1</open>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin0\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/red-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin1\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/grn-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin2\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/blue-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin3\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin4\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/pink-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin5\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/ltblu-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin6\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/red-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin7\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/grn-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin8\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/blue-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin9\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/ylw-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin10\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/purple-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin11\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/ltblu-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin12\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/wht-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
    
    pushpin_file << "    <Style id=\"pushpin13\">" << endl;
    pushpin_file << "      <IconStyle id=\"mystyle\">" << endl;
    pushpin_file << "       <Icon>" << endl;
    pushpin_file << "         <href>http://maps.google.com/mapfiles/kml/pushpin/wht-pushpin.png</href>" << endl;
    pushpin_file << "         <scale>1.0</scale>" << endl;
    pushpin_file << "       </Icon>" << endl;
    pushpin_file << "      </IconStyle>" << endl;
    pushpin_file << "    </Style>" << endl;
  }
    
  // Counter for each aircraft
  vector< int > plane_index;
  
  // The XML (KML) "folder" which stores all the pins for an individual aircraft
  vector< string > pins;
  
  // For the purpose of calculating the total distance required of each plane
  vector< double > d_traveled;
  
  vector< double > lats;
  vector< double > lons;
  vector< double > prev_lats;
  vector< double > prev_lons;
    
  // Read all the input
  while( !input_file.eof() /* input not empty */ )
  {
    
    // Try to read a line
    string line;
    getline(input_file, line);
    
    // See if we found a valid line
    if(line.length() == 0 || line[0] == '#')
    {
      line.clear(); // Output empty string into line as sentinal value
    }
    
    if( !line.empty() )
    {
      int plane_num, point_num;
      double lat, lon;
      
      stringstream ss;
      ss << line;
      
      ss >> plane_num;
      ss >> lat;
      ss >> lon;
      
      while( plane_num >= (int)plane_index.size() )
      {
        plane_index.push_back( -1 );
        
        lats.push_back( 0.0 );
        lons.push_back( 0.0 );
        prev_lats.push_back( 0.0 );
        prev_lons.push_back( 0.0 );
        
        d_traveled.push_back( 0.0 );
        
        // Create and set up a pins folder
        pins.push_back( "    <Folder>\n" );
        pins.back() += "      <name>UAV " + to_string( plane_index.size() - 1 ) + "</name>\n";
        pins.back() += "      <visibility>1</visibility>\n";
      }
      
      
      // This block does the distance calculation
      if( (lats[ plane_num ] > EPSILON || lats[ plane_num ] < -EPSILON)  && 
          (lons[ plane_num ] > EPSILON || lons[ plane_num ] < -EPSILON) ) 
      { // this isn't the first loc for this plane
        prev_lats[ plane_num ] = lats[ plane_num ];
        prev_lons[ plane_num ] = lons[ plane_num ];
      }
      
      lats[ plane_num ] = lat;
      lons[ plane_num ] = lon;
      
      if( (prev_lats[ plane_num ] > EPSILON || prev_lats[ plane_num ] < -EPSILON)  && 
         (prev_lons[ plane_num ] > EPSILON || prev_lons[ plane_num ] < -EPSILON) ) // prev loc initialized
      {
        d_traveled[ plane_num ] += 
        map_tools::calculate_distance_between_points( lats[ plane_num ], lons[ plane_num ], 
                                                     prev_lats[ plane_num ], prev_lons[ plane_num ],
                                                     "meters" );
      }
      // End of distance calculation bit
      
      
      ++plane_index[ plane_num ];
      point_num = plane_index[ plane_num ];
      
      // Read each plane
      string ident = "p" + to_string( plane_num ) + "pt" + to_string( point_num );
      pins[ plane_num ] += "      <Placemark id=\"" + ident + "\">\n";
      pins[ plane_num ] += "        <name>" + ident + "</name>\n";
//      pins[ plane_num ] += "           <styleUrl>#pushpin</styleUrl>\n";
      pins[ plane_num ] += "           <styleUrl>#pushpin" + to_string(plane_num) + "</styleUrl>\n";
      pins[ plane_num ] += "           <Point>\n";
      pins[ plane_num ] += "             <coordinates>" + double_to_text(lon); 
      pins[ plane_num ] += ", " + double_to_text(lat) + ", 0</coordinates>\n";
      pins[ plane_num ] += "           </Point>\n";
      pins[ plane_num ] += "      </Placemark>\n";
      
      /*
      pushpin_file << "    <Folder>\n" << endl;
      pushpin_file << "      <name>Paths</name>" << endl;
      pushpin_file << "      <visibility>1</visibility>\n" << endl;
      string ident = "p" + to_string( plane_num ) + "pt" + to_string( point_num );
      pushpin_file << "      <Placemark id=\"" << ident << "\">" << endl;
      pushpin_file << "        <name>" << ident << "</name>" << endl;
      pushpin_file << "           <styleUrl>#pushpin</styleUrl>" << endl;
      pushpin_file << "           <Point>" << endl;
      pushpin_file << "             <coordinates>" << double_to_text(lon) << 
      pushpin_file << ", " << double_to_text(lat) <<", 0</coordinates>" << endl;
      pushpin_file << "           </Point>" << endl;
      pushpin_file << "      </Placemark>"<< endl;
      pushpin_file << "    </Folder>" << endl;
       */

      //pushpin_file << << endl;
    } // end if not empty
  } // end while
  for( vector< string >::iterator crnt_pins = pins.begin(); crnt_pins != pins.end(); ++crnt_pins )
  {
    // Close this pin's folder
    (*crnt_pins) += "    </Folder>\n";
    
    // Output the pin
    pushpin_file << (*crnt_pins);
  }
      
  //closing stuff, never changes
  pushpin_file << "  </Document>" << endl;
  pushpin_file << "</kml>" << endl;  
  
  // Output the minimum distances to travel as a comment at the end of the file
  pushpin_file << "<!--" << endl;
  
  double total_d = 0.0;
  cout << endl;
  for( int i = 0; i < (int)d_traveled.size(); i++ )
  {
    total_d += d_traveled[ i ];
    pushpin_file << "     Plane " << i << " min distance: " << 
      to_string( d_traveled[ i ] ) << " meters" << endl;
    cout << "     Plane " << i << " min distance: " << 
      to_string( d_traveled[ i ] ) << " meters" << endl;
  }
  pushpin_file << endl << "     Total min distance: " << 
    to_string( total_d ) << " meters" << endl;
  cout << endl << "     Total min distance: " << 
    to_string( total_d ) << " meters" << endl;
  
  pushpin_file << "-->" << endl;
  
  pushpin_file << "\n"; // *nix text files need to end with a newline

  pushpin_file.close();
  
  cout << endl << "File created!" << endl;
  cout << "Check the following location for your course file:" << endl;
  cout << output_name_with_path;
}
