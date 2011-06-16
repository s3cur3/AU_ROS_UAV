//
//  write_to_log.h
//  AU_UAV_ROS
//
//  Created by Tyler Young on 6/13/11.
//
// Writes some text to our own log file, for use in Ubuntu's ROS.

#ifndef WRITE_TO_LOG
#define WRITE_TO_LOG

#include <iostream>
#include <fstream>
#include <time.h>
#include <sstream>

template <class T>
inline std::string to_string( const T& t )
{
  std::stringstream ss;
  ss << t;
  return ss.str();
}

void create_log( string text, string log_dir )
{
  ofstream log;
  string log_path = log_dir + "visualization_log.txt";
  log.open( log_path.c_str(), ios::out );
  
  log << text << "\n";
  
  log.close();
}

void add_to_log( string text, string log_dir )
{
  ofstream log;
  string log_path = log_dir + "visualization_log.txt";
  log.open( log_path.c_str(), ios::app );
  
  log << "At " << time(NULL) << ":     " << text << "\n";
  
  log.close();
}

#endif