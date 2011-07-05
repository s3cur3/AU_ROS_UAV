//
//  telemetry_data_out.h
//  AU_UAV_ROS
//
//  Created by Tyler Young on 6/10/11.
//
// Used for outputting a plane's location, destination, speed, and so on (primarily
// for use in debugging or visualization)

#ifndef TELEMETRY_DATA_OUT
#define TELEMETRY_DATA_OUT

#include <sstream>
#include <time.h>
#include <iostream>
#include <fstream>
#include "AU_UAV_ROS/TelemetryUpdate.h"

int output_data( const AU_UAV_ROS::TelemetryUpdate::ConstPtr& msg, int index )
{
  // Build the filename string
  stringstream ss( stringstream::out );
  ss << "/mnt/hgfs/Dropbox/school/Auburn/Code/AU_UAV_stack/AU_UAV_ROS/teledata/";
  ss << msg->planeID << "_" << index << ".txt";
  string filename = ss.str();

  ofstream tele;
  tele.open( filename.c_str() );
  if( tele.is_open() )
  {  
    // Write all the relevant data to the text file
    tele << msg->planeID << "\n" << msg->currentLatitude << "\n";
    tele << msg->currentLongitude << "\n" << msg->currentAltitude << "\n";
    tele << msg->groundSpeed << "\n" << msg->destLatitude << "\n";
    tele << msg->destLongitude << "\n" << msg->targetBearing << "\n";
    tele << (clock() / (CLOCKS_PER_SEC / 1000)) << "\n";
    tele.close();
    return 1;
  }
  else
  {
    return 0; // Failed to open file
  }
}
#endif
