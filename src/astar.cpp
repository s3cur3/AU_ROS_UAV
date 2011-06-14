////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

// A* code largely untouched, all output, simulation, and whatnot is part of UAV Team 6 (really, team 2)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stlastar.h" // See header for copyright and usage information
#include "Plane.h"
#include "best_cost.h"
#include <iostream>
#include <math.h>
#include <vector>
#include <limits>

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0
#define OUTPUT_NODES 0 // use this to output text version of goal states

using namespace std;

// Global data

// The world map

int HEU = 10;
const int MAP_WIDTH = 46;
const int MAP_HEIGHT = 42;
const int ROUTE_ABANDONED = 0; // if 1, says if search terminated
best_cost *bc_grid;
int time_step = 0;
const int MAX_TIME = 20; // 20 steps into the future...I think
const int nothing = 0;

const int DEBUG_MOVE = 0;


// easy calculation for next move
// 0-7 are actual moves, 8 is goal state, 9 is stall state
const int movex[10] = {1, 1, 0, -1, -1, -1, 0, 1, 0, 0};
const int movey[10] = {0, 1, 1, 1, 0, -1, -1, -1, 0, 0};
int movegoals = 8; // 0 - 7
/**
   End user mods
*/

int map[ MAP_WIDTH * MAP_HEIGHT ];

// map helper functions

int GetMap( int x, int y )
{

  if( x < 0 ||
      x >= MAP_WIDTH ||
      y < 0 ||
      y >= MAP_HEIGHT
      )
    {
      return 9;	 
    }

  return map[(y*MAP_WIDTH)+x];
}



// Definitions

class MapSearchNode
{
public:
  unsigned int x;	 // the (x,y) positions of the node
  unsigned int y;	
	
  MapSearchNode() { x = y = 0; }
  MapSearchNode( unsigned int px, unsigned int py ) { x=px; y=py; }

  float GoalDistanceEstimate( MapSearchNode &nodeGoal );
  bool IsGoal( MapSearchNode &nodeGoal );
  bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
  float GetCost( MapSearchNode &successor );
  bool IsSameState( MapSearchNode &rhs );

  void PrintNodeInfo(); 

  int getX();
  int getY();

};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

  // same state in a maze search is simply when (x,y) are the same
  if( (x == rhs.x) &&
      (y == rhs.y) )
    {
      return true;
    }
  else
    {
      return false;
    }

}

void MapSearchNode::PrintNodeInfo()
{
  cout << "Node position : (" << x << ", " << y << ")" << endl;
}

// Added to get node X pos -- Andrew Kaizer
int MapSearchNode::getX(){
  return x;
}

// Added to get node Y pos -- Andrew Kaizer
int MapSearchNode::getY(){
  return y;
}

// Heuristic is BC (DG+BC) -- since no need to calculate, just choose minimal legal cost
float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
  // Manhattan
  if (HEU == 0){
    float xd = fabs(float(((float)x - (float)nodeGoal.x)));
    float yd = fabs(float(((float)y - (float)nodeGoal.y)));
    return xd+yd;
  } else if (HEU == 1){
    // Euclidean
    int xd=nodeGoal.x-x;
    int yd=nodeGoal.y-y;  
    int d=static_cast<int>(sqrt(xd*xd+yd*yd));
    return d;
  } else if (HEU == 2){
    // Minkowski, a good Polish/Lithuanian Empire name
    int xd = abs((int)x-(int)nodeGoal.x);
    int yd = abs((int)y-(int)nodeGoal.y);
    
    if (xd > yd)
      return xd;
    else
      return yd;
  } else if (HEU == 3){
    return bc_grid->get_pos(x, y, time_step);
  }
}

bool MapSearchNode::IsGoal( MapSearchNode &nodeGoal )
{

  if( (x == nodeGoal.x) &&
      (y == nodeGoal.y) )
    {
      return true;
    }

  return false;
}

// This generates the successors to the given Node. It uses a helper function called
// AddSuccessor to give the successors to the AStar class. The A* specific initialisation
// is done for each node internally, so here you just set the state information that
// is specific to the application
bool MapSearchNode::GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node )
{

  int parent_x = -1; 
  int parent_y = -1; 

  if( parent_node )
    {
      parent_x = parent_node->x;
      parent_y = parent_node->y;
    }
	

  MapSearchNode NewNode;

  // push each possible move except allowing the search to go backwards

  if( (GetMap( x-1, y ) < 9) 
      && !((parent_x == x-1) && (parent_y == y))
      ) 
    {
      NewNode = MapSearchNode( x-1, y );
      astarsearch->AddSuccessor( NewNode );
    }	

  if( (GetMap( x, y-1 ) < 9) 
      && !((parent_x == x) && (parent_y == y-1))
      ) 
    {
      NewNode = MapSearchNode( x, y-1 );
      astarsearch->AddSuccessor( NewNode );
    }	

  if( (GetMap( x+1, y ) < 9)
      && !((parent_x == x+1) && (parent_y == y))
      ) 
    {
      NewNode = MapSearchNode( x+1, y );
      astarsearch->AddSuccessor( NewNode );
    }	

		
  if( (GetMap( x, y+1 ) < 9) 
      && !((parent_x == x) && (parent_y == y+1))
      )
    {
      NewNode = MapSearchNode( x, y+1 );
      astarsearch->AddSuccessor( NewNode );
    }	


  // Andrew Kaizer is adding Diagonals below here...
  if( (GetMap( x+1, y+1 ) < 9) 
      && !((parent_x == x+1) && (parent_y == y+1))
      )
    {
      NewNode = MapSearchNode( x+1, y+1 );
      astarsearch->AddSuccessor( NewNode );
    }	
  
  if( (GetMap( x+1, y-1 ) < 9) 
      && !((parent_x == x+1) && (parent_y == y-1))
      )
    {
      NewNode = MapSearchNode( x+1, y-1 );
      astarsearch->AddSuccessor( NewNode );
    }	
  
  if( (GetMap( x-1, y+1 ) < 9) 
      && !((parent_x == x-1) && (parent_y == y+1))
      )
    {
      NewNode = MapSearchNode( x-1, y+1 );
      astarsearch->AddSuccessor( NewNode );
    }	
  
  if( (GetMap( x-1, y-1 ) < 9) 
      && !((parent_x == x-1) && (parent_y == y-1))
      )
    {
      NewNode = MapSearchNode( x-1, y-1 );
      astarsearch->AddSuccessor( NewNode );
    }	
  

  return true;
}

// given this node, what does it cost to move to successor. In the case
// of our map the answer is the map terrain value at this node since that is 
// conceptually where we're moving

float MapSearchNode::GetCost( MapSearchNode &successor )
{

  return (float) GetMap( x, y );

}

// place code in here to make main for UAV Sim friendly -- Andrew Kaizer
// return the next move for the plane to make
// need planeid so our output map (outmap) can be complete -- also colors :)
int other_main(int startx, int starty, int endx, int endy, int planeid ){
  // cout << "STL A* Search implementation\n(C)2001 Justin Heyes-Jones\n";

  // Our sample problem defines the world as a 2d array representing a terrain
  // Each element contains an integer from 0 to 5 which indicates the cost 
  // of travel across the terrain. Zero means the least possible difficulty 
  // in travelling (think ice rink if you can skate) whilst 5 represents the 
  // most difficult. 9 indicates that we cannot pass.

  // Create an instance of the search class...

  AStarSearch<MapSearchNode> astarsearch;

  unsigned int SearchCount = 0;

  const unsigned int NumSearches = 1;
  
  /**
     0: move right
     1: move bottom right
     2: move down
     3: move bottom left
     4: move left
     5: move top left
     6: move top
     7: move top right
     8: solution found...no more moves needed.....
  */
  int moves = 8;
  while(SearchCount < NumSearches)
    {
      // Create a start state
      MapSearchNode nodeStart;
      nodeStart.x = startx;
      nodeStart.y = starty;

      // Define the goal state
      MapSearchNode nodeEnd;
      nodeEnd.x = endx;					      
      nodeEnd.y = endy; 

      // Set Start and goal states
      astarsearch.SetStartAndGoalStates( nodeStart, nodeEnd );
 
      unsigned int SearchState;
      unsigned int SearchSteps = 0;

      do
	{
	  SearchState = astarsearch.SearchStep();

	  SearchSteps++;

	  //cout << SearchSteps << endl;
	  time_step++;
	  if (time_step >= MAX_TIME){
	    time_step = MAX_TIME - 1;
	  }

#if DEBUG_LISTS

	  cout << "Steps:" << SearchSteps << "\n";

	  int len = 0;

	  cout << "Open:\n";
	  MapSearchNode *p = astarsearch.GetOpenListStart();
	  while( p )
	    {
	      len++;
#if !DEBUG_LIST_LENGTHS_ONLY			
	      ((MapSearchNode *)p)->PrintNodeInfo();
#endif
	      p = astarsearch.GetOpenListNext();

	    }

	  cout << "Open list has " << len << " nodes\n";

	  len = 0;

	  cout << "Closed:\n";
	  p = astarsearch.GetClosedListStart();
	  while( p )
	    {
	      len++;
#if !DEBUG_LIST_LENGTHS_ONLY			
	      p->PrintNodeInfo();
#endif			
	      p = astarsearch.GetClosedListNext();

	    }

	  cout << "Closed list has " << len << " nodes\n";
#endif

	}
      while( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SEARCHING );

      if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_SUCCEEDED )
	{
#if OUTPUT_NODES
	  cout << "Search found goal state\n";
#endif

	  MapSearchNode *node = astarsearch.GetSolutionStart();

#if DISPLAY_SOLUTION
	  cout << "Displaying solution\n";
#endif
	  int steps = 0;

#if OUTPUT_NODES
	  node->PrintNodeInfo();	  
#endif


	  for( ;; )
	    {
	      node = astarsearch.GetSolutionNext();

	      if( !node )
		{
		  break;
		}
#if OUTPUT_NODES
	      node->PrintNodeInfo();
#endif
	      

	      if (steps == 0){
		// calculate the next move
		int newy = node->getY(); // startY is base
		int newx = node->getX(); // startX is base
		// cout << newy - starty << ", " << newx - startx << endl; //TEST
		// cases
		if (newy - starty == 0 && newx - startx == 1){		  
		  moves = 0; // right
		}
		else if (newy - starty == 1 && newx - startx == 1){
		  moves = 1; // down right
		}
		else if (newy - starty == 1 && newx - startx == 0){
		  moves = 2; // down
		}
		else if (newy - starty == 1 && newx - startx == -1){
		  moves = 3; // down left
		}
		else if (newy - starty == 0 && newx - startx == -1){
		  moves = 4; // left
		}
		else if (newy - starty == -1 && newx - startx == -1){
		  moves = 5; // top left
		}
		else if (newy - starty == -1 && newx - startx == 0){
		  moves = 6; // top
		}
		else if (newy - starty == -1 && newx - startx == 1){
		  moves = 7; // top right
		}

		startx = newx;
		starty = newy;	      
	      }

	      steps ++;				
	    };
#if OUTPUT_NODES
	  cout << "Solution steps " << steps << endl;
#endif
	  if (steps < 1){
	    moves = 8; // moves should be a 8, if we have no move
	  }	  

	  // Once you're done with the solution you can free the nodes up
	  astarsearch.FreeSolutionNodes();

	
	}
      else if( SearchState == AStarSearch<MapSearchNode>::SEARCH_STATE_FAILED ) 
	{
	  if (ROUTE_ABANDONED > 0)
	    cout << "Search terminated. Did not find goal state\n";       
	  // moves = 9; // stall!  change to 8 (no move) in main loop
	  // moves = p.getstallpoint();
	  // if moves == 9 (no more stall points), than get direction of next waypoint...

	  // endx, endy vs startx, starty
	  if (starty-endy == 0 && startx-endx < 0){		  
	    moves = 0; // right
	  }
	  else if (starty-endy < 0 && startx-endx < 0){
	    moves = 1; // down right
	  }
	  else if (starty-endy < 0 && startx-endx == 0){
	    moves = 2; // down
	  }
	  else if (starty-endy < 0 && startx-endx > 0){
	    moves = 3; // down left
	  }
	  else if (starty-endy == 0 && startx-endx >0){
	    moves = 4; // left
	  }
	  else if (starty-endy > 0 && startx-endx > 0){
	    moves = 5; // top left
	  }
	  else if (starty-endy > 0 && startx-endx == 0){
	    moves = 6; // top
	  }
	  else if (starty-endy > 0 && startx-endx < 0){
	    moves = 7; // top right
	  }

	  // RANDOM MONSTER IS RANDOM
	  int r[] = {(moves-2)%movegoals, (moves-1)%movegoals, moves%movegoals, (moves+1)%movegoals,(moves+2)%movegoals}; 

	  if(r[0] < 0){
	    r[0] = movegoals + r[0]; 
	  }

	  if (r[1] < 0){
	    r[1] = movegoals + r[1];
	  }

	  int rx[] = {startx+movex[r[0]], startx+movex[r[1]], startx+movex[r[2]], startx+movex[r[3]], startx+movex[r[4]] };
	  int ry[] = {starty+movey[r[0]], starty+movey[r[1]], starty+movey[r[2]], starty+movey[r[3]], starty+movey[r[4]] };
	  int coor[] = {
	    ry[0]*MAP_WIDTH+rx[0],
	    ry[1]*MAP_WIDTH+rx[1],
	    ry[2]*MAP_WIDTH+rx[2],
	    ry[3]*MAP_WIDTH+rx[3],
	    ry[4]*MAP_WIDTH+rx[4]
	  };

	  double minkdist = numeric_limits<double>::infinity();

	  double cost[5]; // set to 0 as a default (0 is impossible, anyway)
	  for (int i = 0 ; i < 5; i++){ // magic number 4...for the num of alternate moves
	    if (rx[i] < MAP_WIDTH && ry[i] < MAP_HEIGHT && rx[i] >= 0 && ry[i] >= 0){	 
	      // Manhattan
	      if (HEU == 0){
		float xd = fabs(float(((float)startx+movex[r[i]] - (float)endx)));
		float yd = fabs(float(((float)starty+movey[r[i]] - (float)endy)));
		cost[i] = xd+yd;
	      } else if (HEU == 1){
		// Euclidean
		int xd=endx-startx+movex[r[i]];
		int yd=endy-starty+movey[r[i]];  
		int d=static_cast<int>(sqrt(xd*xd+yd*yd));
		cost[i] = d;
	      } else if (HEU == 2){
		// Minkowski, a good Polish/Lithuanian Empire name
		int xd = abs((int)startx+movex[r[i]]-(int)endx);
		int yd = abs((int)starty+movey[r[i]]-(int)endy);
    
		if (xd > yd)
		  cost[i] = xd;
		else
		  cost[i] = yd;
	      } else if (HEU == 3){
		cost[i] = bc_grid->get_pos(startx+movex[r[i]], starty+movey[r[i]], 0);
	      }   
	    } else {
	      cost[i] = NULL;
	    }
	  }

	  // finally, calculate legal move and best cost...
	  for (int i = 0; i < 5; i++){
	    if (rx[i] < MAP_WIDTH && ry[i] < MAP_HEIGHT && rx[i] >= 0 && ry[i] >= 0){
	      if (cost[i] < minkdist){ // new best candidate
		moves = r[i];
		minkdist = cost[i];
	      }
	    }		
	  }

	  if (minkdist == numeric_limits<double>::infinity())
	    moves = 9; // WE ARE SO DEAD AT THIS POINT, LOL >:D
	}


      // Display the number of loops the search went through
#if OUTPUT_NODES
      cout << "SearchSteps : " << SearchSteps << "\n";
#endif

      SearchCount ++;

      astarsearch.EnsureMemoryFreed();
    }

  return moves;
}

int astar(best_cost *bc, int startx, int starty, int endx, int endy, int planeid)
{
  HEU = 3;
  int move = 9;

  time_step = 0; // resets our time steps - so 0 is the immediate time, 1 is 1 second into the future, 2 is 2 seconds, etc.
  bc_grid = bc;

  move = other_main(startx, starty, endx, endy, planeid);

  if (DEBUG_MOVE > 0){  
    cout << planeid << ": " << move << " (" << startx << ", " << starty << ") --> (" << endx << ", "<< endy << ")" << endl;

    double minimum = 1000;
    int min_move = 9;
    // Tell us all potential immediate moves
    for (int i = 0; i < 8; i++){
      // Rightermost Bound
      if (startx+movex[i] >= MAP_WIDTH || startx+movex[i] < 0 || starty+movey[i] >= MAP_HEIGHT || starty+movey[i] < 0){
	cout << i << " is an illegal move" << endl; 
      } else {
	cout << i << ": " << bc->get_pos(startx+movex[i], starty+movey[i], 0) << endl;
	if (bc->get_pos(startx+movex[i], starty+movey[i], 0) < minimum){
	  minimum = bc->get_pos(startx+movex[i], starty+movey[i], 0);
	  min_move = i;
	}
      }
    }
    cout << min_move << endl;

    for (int y = 0; y < MAP_HEIGHT; y++){
      for (int x = 0; x < MAP_WIDTH; x++){
	if (x == startx && y == starty)
	  cout << "\033[1;33mSS \033[0m";
	else if (x == endx && y == endy)
	  cout << "\033[1;31mDD \033[0m";
	else if (round(bc->get_pos(x, y, 0)) < 10)
	  cout << "0" << round(bc->get_pos(x, y, 0)) << " ";
	else
	  cout << round(bc->get_pos(x, y, 0)) << " ";
      }
      cout << endl;
    }

    getchar();
  }
  
  return move;
}

int astar_heu(int map_g[MAP_WIDTH][MAP_HEIGHT], int startx, int starty, int endx, int endy, int planeid, int heuristic)
{
  for (int y = 0; y < MAP_HEIGHT; y++){
    for (int x = 0; x < MAP_WIDTH; x++){
      map[x+y*MAP_WIDTH] = map_g[x][y];
    }
  }

  int move = 9;
  HEU = heuristic;

  move = other_main(startx, starty, endx, endy, planeid);

  return move;
}
