//
//  math_tester.cpp
//  AU_UAV_ROS
//
//  Created by Tyler Young on 6/3/11.
//
// Used to determine the efficiency of multiplying integer compared to floating point
// values.

#include <math.h>
#include <cstdlib>
#include <iostream>
#include <time.h>
#include <climits>

using namespace std;

#define SQRT_2 1.41421356
#define SQRT_MULT_10000 14142

// NOTE: Assume int range is -32,767 to 32,767

int main()
{
  srand ( 20 );
  
  int a_1_to_100 = rand() % 100 + 1;
  int b_1_to_100 = rand() % 100 + 1;
  int c_1_to_100 = rand() % 100 + 1;
  int d_1_to_100 = rand() % 100 + 1;
  
  double danger_d_1 = rand() % 100 + 1;
  double danger_d_2 = rand() % 100 + 1;
  int danger_i_1 = (int) danger_d_1;
  int danger_i_2 = (int) danger_d_2;

  double map_weight_d = 20.0;
  int map_weight_i = 20;

  double dist_d = sqrt( (a_1_to_100 - b_1_to_100)*(a_1_to_100 - b_1_to_100) + 
                       (c_1_to_100 - d_1_to_100)*(c_1_to_100 - d_1_to_100) );
  
  unsigned int dist_i_mult_10000 = (unsigned int)( 10000 * sqrt( (a_1_to_100 - b_1_to_100)*(a_1_to_100 - b_1_to_100) + 
                                                      (c_1_to_100 - d_1_to_100)*(c_1_to_100 - d_1_to_100) ) );
  
  
  double travel_cost_d = 10.0;
  int travel_cost_i = 10;
  
  double cost_d = danger_d_1 + (map_weight_d * danger_d_2) + (travel_cost_d * SQRT_2);
  int cost_i = danger_i_1 + (map_weight_i * danger_i_2) + (travel_cost_i * SQRT_MULT_10000);

  
  unsigned int loop_me = UINT_MAX/2;
  cout << loop_me << endl;
  time_t seconds;
  seconds = time(NULL);
  cout << "Start time: " << seconds<<endl;
  
//  double a;
//  for( unsigned int outer = 0; outer < 10; outer++ )
//  {
//    for( unsigned int i = 0; i <= loop_me; i++ )
//      a = cost_d * dist_d;
//  }

  unsigned int b;
  for( unsigned int outer = 0; outer < 10; outer++ )
  {
    for( unsigned int i = 0; i <= loop_me; i++ )
      b = cost_i * dist_i_mult_10000;
  }
  
  seconds = time(NULL);
  cout << "End time:   " << seconds << endl;
  cout << endl << "Done!" << endl;
}