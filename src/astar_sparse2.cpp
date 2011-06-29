 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

// A* code largely untouched, all output, simulation, and whatnot is part of UAV Team 6 (really, team 2)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stlastar.h" // See header for copyright and usage information
#include <iostream>
#include <math.h>
#include <vector>
#include <limits>

// QUEUE: used for calculating optimal paths
#include <queue>
// STACK: used for calculating from GOAL to START
#include <stack>
// MAP: used to hold all potential optimal values...
#include <map>

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0
#define OUTPUT_NODES 0 // use this to output text version of goal states

using namespace std;
using namespace map_tools;

// Global data
// The world map

const int MAP_WIDTH = 41;
const int MAP_HEIGHT = 37;
const int ROUTE_ABANDONED = 1; // if 1, says if search terminated
best_cost *bc_grid;
const int MAX_TIME = 20; // 20 steps into the future...I think

// easy calculation for next move
// 0-7 are actual moves, 8 is goal state, 9 is stall state
const int movex[10] = {1, 1, 0, -1, -1, -1, 0, 1, 0, 0};
const int movey[10] = {0, 1, 1, 1, 0, -1, -1, -1, 0, 0};
int movegoals = 8; // 0 - 7
int sparse_expansion = 5;
bearing_t initial_bearing;

// Start and End spots
int s_x = -1;
int s_y = -1;
int e_x = -1;
int e_y = -1;
//double initial_bearing = -1;
/**
   End user mods
*/


// This struct is used to store (and reply) with the next "destination" that the plane is flying to
struct point {
  int x;
  int y;
  int t;
} ;

queue<point> expan;

// used to avoid considering the dangerous point
int goal_orientation = -1;

// The danger point is the point of divergence between our pre-sparse search and A* search
point danger_point;

// a_path is the path A* recommends
queue<point> a_path;


int astar_map[ MAP_WIDTH * MAP_HEIGHT ];
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

  return astar_map[(y*MAP_WIDTH)+x];
}


int getGridBearing( int current_x, int current_y, int dest_x, int dest_y ){
    if (current_y-dest_y == 0 && current_x-dest_x < 0){		  
      return 0; // goal is to your right
    }
    else if (current_y-dest_y < 0 && current_x-dest_x < 0){
      return 1; // down right
    }
    else if (current_y-dest_y < 0 && current_x-dest_x == 0){
      return 2; // down
    }
    else if (current_y-dest_y < 0 && current_x-dest_x > 0){
      return 3; // down left
    }
    else if (current_y-dest_y == 0 && current_x-dest_x >0){
      return 4; // left
    }
    else if (current_y-dest_y > 0 && current_x-dest_x > 0){
      return 5; // top left
    }
    else if (current_y-dest_y > 0 && current_x-dest_x == 0){
      return 6; // top
    }
    else if (current_y-dest_y > 0 && current_x-dest_x < 0){
      return 7; // top right
    }
}


// Definitions

class MapSearchNode
{
public:
  unsigned int x;	 // the (x,y) positions of the node
  unsigned int y;
  unsigned int timestep; // AK: the timestep that the node is being considered under
  bearing_t parent_bearing; // AK: Used for children node to understand where their parent is coming from
	
  MapSearchNode() { x = y = 0; timestep = 0; parent_bearing = N; }
  MapSearchNode( unsigned int px, unsigned int py ) { x=px; y=py; }
  MapSearchNode( unsigned int px, unsigned int py, unsigned int t_step) { x=px; y=py; timestep=t_step; } // AK
  //  MapSearchNode( unsigned int px, unsigned int py, unsigned int t_step) { x=px; y=py; timestep=t_step; } // AK

  float GoalDistanceEstimate( MapSearchNode &nodeGoal );
  bool IsGoal( MapSearchNode &nodeGoal );
  bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
  float GetCost( MapSearchNode &successor );
  bool IsSameState( MapSearchNode &rhs );

  void PrintNodeInfo(); 

  int getX();
  int getY();
  int getT();

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

int MapSearchNode::getT(){
  return timestep;
}

// Heuristic is BC (DG+BC) -- since no need to calculate, just choose minimal legal cost
float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
  //cout << x << ", " << y << " : " << timestep << " " << bc_grid->get_pos(x, y, timestep) << endl;
  return bc_grid->get_pos(x, y, timestep);
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
  cout << endl << "(" << x << ", " << y << ") : " << timestep << endl;
  point temp;
  temp.x = x;
  temp.y = y;
  temp.t = timestep;
  expan.push(temp);

  int parent_x = -1; 
  int parent_y = -1; 
  int time_z = -100;
  int bearing = -1;

  set<int> legal_moves;

  if ( !parent_node ){
    cout << "PARENT" << endl;
    if((initial_bearing == N)) {
      bearing=6;//north
      parent_bearing = N;
    } else if(initial_bearing == NW) {
      bearing=5;//north west
      parent_bearing = NW;
    } else if(initial_bearing == W) {
      bearing=4;//west
      parent_bearing = W;
    } else if(initial_bearing == SW) {
      bearing=3;//southwest
      parent_bearing = SW;
    } else if(initial_bearing == S) {
      bearing=2;//south
      parent_bearing = S;
    } else if(initial_bearing == SE) {
      bearing=1;//southeast
      parent_bearing = SE;
    } else if(initial_bearing == E) {
      bearing=0;//east
      parent_bearing = E;
    } else if(initial_bearing == NE) {
      bearing=7;//northeast
      parent_bearing = NE;
    }

    //parent_bearing = bearing;
  } else {
    // Our parent node -they raised us to be the node we were meant to be (ish)
    parent_x = (int)parent_node->x;
    parent_y = (int)parent_node->y;
    
    if (parent_y-(int)y == 0 && parent_x-(int)x < 0){		  
      bearing = 0; // goal is to your right
      parent_bearing = E;
    }
    else if (parent_y-(int)y < 0 && parent_x-(int)x < 0){
      bearing = 1; // down right
      parent_bearing = SE;
    }
    else if (parent_y-(int)y < 0 && parent_x-(int)x == 0){
      bearing = 2; // down
      parent_bearing = S;
    }
    else if (parent_y-(int)y < 0 && parent_x-(int)x > 0){
      bearing = 3; // down left
      parent_bearing = SW;
    }
    else if (parent_y-(int)y == 0 && parent_x-(int)x >0){
      bearing = 4; // left
      parent_bearing = W;
    }
    else if (parent_y-(int)y > 0 && parent_x-(int)x > 0){
      bearing = 5; // top left
      parent_bearing = NW;
    }
    else if (parent_y-(int)y > 0 && parent_x-(int)x == 0){
      bearing = 6; // top
      parent_bearing = N;
    }
    else if (parent_y-(int)y > 0 && parent_x-(int)x < 0){
      bearing = 7; // top right
      parent_bearing = NE;
    }
    
    cout << "My parent is: " << parent_x << ", " << parent_y << ", " << parent_node->timestep << endl;
  } 

  // update our time step
  time_z = timestep+1>19 ? 19 : 1+timestep;
  //cout << "Parent's Time: " << parent_node->timestep << " and our new time: " << time_z << endl;

  //cout << "   BEARING: " << bearing << " AND bear_t " << parent_bearing << endl;
  //getchar();
  // 5 R
  int r[] = {(bearing-2), (bearing-1), bearing%movegoals, (bearing+1)%movegoals,(bearing+2)%movegoals}; 

  if(r[0] < 0){
    r[0] = movegoals + r[0]; 
  }
      
  if (r[1] < 0){
    r[1] = movegoals + r[1];
  }

  int greater_x = 0;
  int lesser_x = 0;
  int greater_y = 0;
  int lesser_y = 0;
  // additional range that is acceptable to look into


  if (s_x > e_x){
    greater_x = s_x + sparse_expansion;
    lesser_x = e_x - sparse_expansion;
  } else {
    greater_x = e_x + sparse_expansion;
    lesser_x = s_x - sparse_expansion;
  }

  if (s_y > e_y){
    greater_y = s_y + sparse_expansion;
    lesser_y = e_y - sparse_expansion;
  } else {
    greater_y = e_y + sparse_expansion;
    lesser_y = s_y - sparse_expansion;
  }

  for (int i = 1; i < 4; i++){
    //cout << "Greater X: " << greater_x << " and Lesser X: " << lesser_x << " and Greater Y: " << greater_y << " and Lesser Y: " << lesser_y <<endl;
    //cout << "X: " << x << " --> " << x + movex[r[i]] << endl;
    //cout << "Y: " << y << " --> " << y + movey[r[i]] << endl;

    
    // Below is a crazy set of cout's that are done inside a unary ? operator
    /*
    (int)x + movex[r[i]] >= 0 ? cout << "Fail 1 " << endl : cout << "Pass 1" << endl;
    (int)x + movex[r[i]] < MAP_WIDTH ? cout << "Fail 2 " << endl : cout << "Pass 2" << endl;
    (int)y + movey[r[i]] >= 0 ? cout << "Fail 3 " << endl : cout << "Pass 3" << endl;
    (int)y + movey[r[i]] < MAP_HEIGHT ? cout << "Fail 4 " << endl : cout << "Pass 4" << endl;
    (int)x + movex[r[i]] >= lesser_x ? cout << "Fail 5 " << endl : cout << "Pass 5" << endl;
    (int)x + movex[r[i]] <= greater_x ? cout << "Fail 6 " << endl : cout << "Pass 6" << endl;
    (int)y + movey[r[i]] >= lesser_y ? cout << "Fail 7 " << endl : cout << "Pass 7" << endl;
    (int)y + movey[r[i]] <= greater_y ? cout << "Fail 8 " << endl : cout << "Pass 8" << endl;
    */
  
    if ((int)x + movex[r[i]] >= 0 && (int)x + movex[r[i]] < MAP_WIDTH &&
	(int)y + movey[r[i]] >= 0 && (int)y + movey[r[i]] < MAP_HEIGHT &&
	(int)x + movex[r[i]] >= lesser_x && (int)x + movex[r[i]] <= greater_x &&
	(int)y + movey[r[i]] >= lesser_y && (int)y + movey[r[i]] <= greater_y){
      legal_moves.insert(r[i]);
      cout << " GO: X -- " << x+movex[r[i]] << ", " << y+movey[r[i]] << " at time: " << time_z << " ---- "  << bc_grid->get_pos(x+movex[r[i]], y+movey[r[i]], time_z) << endl;
    }
  }

  MapSearchNode NewNode;

  // push each possible move except allowing the search to go backwards

  // left
  if( legal_moves.find(4) != legal_moves.end() )
    {
      //cout << "L " << x-1 << ", " << y << endl;
      NewNode = MapSearchNode( x-1, y, time_z );
      astarsearch->AddSuccessor( NewNode );
    }	

  // top
  if( legal_moves.find(6) != legal_moves.end() )
    {
      //cout << "T " << x << ", " << y-1 << endl;
      NewNode = MapSearchNode( x, y-1, time_z );
      astarsearch->AddSuccessor( NewNode );
    }	

  // right
  if( legal_moves.find(0) != legal_moves.end() )
    {
      //cout << "R " << x+1 << ", " << y << endl;
      NewNode = MapSearchNode( x+1, y, time_z );
      astarsearch->AddSuccessor( NewNode );
    }	

  // down	
  if( legal_moves.find(2) != legal_moves.end() )
    {
      //cout << "D " << x << ", " << y+1 << endl;
      NewNode = MapSearchNode( x, y+1, time_z );
      astarsearch->AddSuccessor( NewNode );
    }	


  // Andrew Kaizer is adding Diagonals below here...
  // down right
  if( legal_moves.find(1) != legal_moves.end() )
    {
      //cout << "DR " << x+1 << ", " << y+1 << endl;
      NewNode = MapSearchNode( x+1, y+1, time_z );
      astarsearch->AddSuccessor( NewNode );
    }	
  
  // top right
   if( legal_moves.find(7) != legal_moves.end() )
    {
      //cout << "TR " << x+1 << ", " << y-1 << endl;
      NewNode = MapSearchNode( x+1, y-1, time_z );
      astarsearch->AddSuccessor( NewNode );
    }	
  
   // down left
   if( legal_moves.find(3) != legal_moves.end() )
    {
      //cout << "DL " << x-1 << ", " << y+1 << endl;
      NewNode = MapSearchNode( x-1, y+1,time_z );
      astarsearch->AddSuccessor( NewNode );
    }	
  
   // top left
  if( legal_moves.find(5) != legal_moves.end() )
    {
      //cout << "TL " << x-1 << ", " << y-1 << endl;
      NewNode = MapSearchNode( x-1, y-1, time_z );
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

	  if (!(node->getY()*MAP_WIDTH+node->getX() == nodeStart.y*MAP_WIDTH+nodeStart.x ||
		node->getY()*MAP_WIDTH+node->getX() == nodeEnd.y*MAP_WIDTH+nodeEnd.x)){
	    // Route that astar has selected
	    point p;
	    p.x = node->getX();
	    p.y = node->getY();
	    a_path.push(p);
	    cout << "FIRST" << node->getT() << endl;
	  }

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
	      
	      if (!(node->getY()*MAP_WIDTH+node->getX() == nodeStart.y*MAP_WIDTH+nodeStart.x ||
		    node->getY()*MAP_WIDTH+node->getX() == nodeEnd.y*MAP_WIDTH+nodeEnd.x)){
		// Route that astar has selected
		point p;
		p.x = node->getX();
		p.y = node->getY();
		p.t = node->getT();
		a_path.push(p);
		
		//cout << node->getT() << ": " << p.x << ", " << p.y << " and " << node->getT() << " --> " << bc_grid->get_pos(p.x, p.y, node->getT()) << endl;
		//getchar();
		//cout << "  " << p.x << ", " << p.y+1 << " --> " << bc_grid->get_pos(p.x, p.y+1, node->getT()) << endl;
		//cout << "  " << p.x << ", " << p.y-1 << " --> " << bc_grid->get_pos(p.x, p.y-1, node->getT()) << endl << endl;
	      }

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

	  //cout << planeid << " at " << startx << ", " << starty << endl;
	  //getchar();
	  
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
	      cost[i] = bc_grid->get_pos(startx+movex[r[i]], starty+movey[r[i]], 0);
	    } else {
	      cost[i] = numeric_limits<double>::infinity();
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

	  if (minkdist == numeric_limits<double>::infinity()){
	    moves = 9; // WE ARE SO DEAD AT THIS POINT, LOL >:D
	  }
	  else{
	    point m;
	    m.x = startx+movex[r[moves]];
	    m.y = starty+movey[r[moves]];
	    m.t = 0;
	    a_path.push(m);
	  }
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

// Returns the point of divergence between provable optimal path and a-star path
point astar_point(best_cost *bc, int startx, int starty, int endx, int endy, int planeid, bearing_t current_bear)
{
  //cout << planeid << " : (" << startx << ", " << starty << ") --> (" << endx << ", " << endy << ")" << endl;
  while (!expan.empty()){
    expan.pop();
  }

  s_x = startx;
  s_y = starty;
  e_x = endx;
  e_y = endy;
  initial_bearing = current_bear;
  // SPARSE A* -- FIRST: Calculate the entire path area to see if their is any danger, if none no need to do A-Star
  // if danger is found, A* will kick in and calculate an optimal GRID path to divert to

  // used to determine if the plane advances on the X or the Y axis through time
  // if the optimal first move from Start to End is X1 and the optimal last move from End to Start is X1, then orientation is X based (0) and we need to check based on Y (1)
  // for example: if the start is (23,11) and the goal is (12,29): the first move from S-E is to (22,11), the last move from E-S is to (23,11)
  //  to determine the rest of the optimal paths through time will require us to look at the X values, not the Y values
  goal_orientation = -1;
  
  // if Sparse find a dangerous space in the area, change to false
  bool sparse = true;
  stack<point> goal_to_start;
  queue<point> start_to_goal;
  
  // calculate the bearing to the goal
  int opt_bearing = getGridBearing(startx, starty, endx, endy);
  bool straight_line  = false;
  if (startx == endx || starty == endy){
    straight_line = true;
    // cout << " staight " << endl;
    // getchar();
  }
  
  //straight_line ? cout << "ST " << endl : cout << "NT " << endl;
  // Number of steps using Chebyshev distance from start to goal
  // This is the optimal number steps from start to the goal, as such it is also the minimum number of steps needed to search for Sparse-A*
  int num_steps = 0;

  int min_x = abs(startx-endx);
  int min_y = abs(starty-endy);
  if (min_x > min_y)
    num_steps = min_x;
  else
    num_steps = min_y;

  int k = planeid;
  if (k == 2){
  for (int a = 0; a < 3; a++){
    cout << k << ": " << a << endl;
    cout << "\033[1;34m    ";
    for (int x = 0; x < MAP_WIDTH; x++){
      if (x < 10)
	cout << "0" << x << " ";
      else
	cout << x << " ";
    }
    cout << "\033[0m" << endl;
    for (int y = 0; y < MAP_HEIGHT; y++){
      if (y < 10)
	cout << "0" << y << ": ";
      else
	cout << y << ": " ;
      for (int x = 0; x < MAP_WIDTH; x++){
	if (x == startx && y == starty)
	  cout << "\033[1;3" << k%8 << "mPP \033[0m";
	else if (x == endx && y == endy)
	  cout << "\033[1;35m0"<< round(bc->get_pos(x,y,a)) <<  " \033[0m";
	else if (round(bc->get_pos(x, y, a)) > 99)
	  cout << "\033[0;31m99\033[0m" << " " ;
	else if (round(bc->get_pos(x, y, a)) < 10)
	  cout << "0" << round(bc->get_pos(x, y, a)) << " ";
	else
	  cout << round(bc->get_pos(x, y, a)) << " ";
      }
      cout << endl;
    }
    cout << endl;
  }
  }

  // Markers for our movement
  point forward;
  point backward;
  forward.x = startx;
  forward.y = starty;
  backward.x = endx;
  backward.y = endy;

  for (int i = 0; i < num_steps - 1; i++){
    int f_move = 0;
    int b_move = 0;

    int opt_f_cost = numeric_limits<int>::max();
    int opt_b_cost= numeric_limits<int>::max();

    // construct our optimal move using modman to determine the cost of all moves
    // if we encounter a cheaper move, choose that one
    for (int j = 0; j < 8; j++){
      int f = abs(forward.x + movex[j] - endx) + abs(forward.y + movey[j] - endy);
      int b = abs(backward.x + movex[j] - startx) + abs(backward.y + movey[j] - starty);
      if (f < opt_f_cost){
	opt_f_cost = f;
	f_move = j;
      }

      if (b < opt_b_cost){
	opt_b_cost = b;
	b_move = j;
      }
    }

    backward.x = backward.x + movex[b_move];
    backward.y = backward.y + movey[b_move];
    goal_to_start.push(backward);

    forward.x = forward.x + movex[f_move];
    forward.y = forward.y + movey[f_move];
    start_to_goal.push(forward);    
  } // end creation of optimal forward and backward grid paths

  queue<point> opt_path;

  // we need time to select the correct best cost grid
  int time_select = 1;
  while (!start_to_goal.empty() && !goal_to_start.empty()){
    // grab points from the queue and stack, which are on the same x coordinate (this only holds for a square or rectangular grid)
    point g_t_s = goal_to_start.top();
    point s_t_g = start_to_goal.front();
    goal_to_start.pop();
    start_to_goal.pop();

    // on the first pop, we must check our orientation to determine which spaces to consider for optimality
    if (goal_orientation == -1){
      if ((opt_bearing == 1 || opt_bearing == 3 || opt_bearing == 5 || opt_bearing == 7) && s_t_g.x == g_t_s.x && s_t_g.y == g_t_s.y){
	goal_orientation = 2; // diagonal
	straight_line = true;

	// need to calculate the diagonals bearing, to determine if we have 1/1, 1/-1, -1/1, -1/-1 optimal expansions
	point temp = start_to_goal.front();
	if (temp.x > s_t_g.x && temp.y > s_t_g.y){ 
	  opt_bearing = 1; // +1/+1
	} else if (temp.x > s_t_g.x && temp.y < s_t_g.y){ 
	  opt_bearing = 7; // +1/-1
	} else if (temp.x < s_t_g.x && temp.y > s_t_g.y){
	  opt_bearing = 3; // -1/+1
	} else if (temp.y < s_t_g.x && temp.y < s_t_g.y){
	  opt_bearing = 5; // -1/-1
	}	
	goal_orientation = 0;
      } else if (g_t_s.x == s_t_g.x){
	goal_orientation = 0; //check Y positions
      } else if (g_t_s.y == s_t_g.y){
	goal_orientation = 1;
      } else {
	cout << "ERROR on ORIENTATION, neither X nor Y are the same values!" << endl;
	exit(1);
      }
    }

    // Need to calculate which y's to look at (the ones from (y_small to y_large)
    double small_y = 0;
    double large_y = 0;
    double small_x = 0;
    double large_x = 0;

    if (g_t_s.y > s_t_g.y){
      large_y = g_t_s.y;
      small_y = s_t_g.y;
    } else {
      large_y = s_t_g.y;
      small_y = g_t_s.y;
    }

    if (g_t_s.x > s_t_g.x){
      large_x = g_t_s.x;
      small_x = s_t_g.x;
    } else {
      large_x = s_t_g.x;
      small_x = g_t_s.x;
    }
    
    // calculate the "middle point" that a plane in continuous space would take (i.e. the plane is taking a straight line, not the jagged nature of a grid)
    int avgy = round((large_y+small_y)/2);
    int avgx = round((large_x+small_x)/2);

    // on the off chance that the path is not clear, we need to keep a record of the optimal path through continuous space (i.e. the euclidean path)
    point opt_p;
    opt_p.x = avgx;
    opt_p.y = avgy;			      
    opt_p.t = time_select;

    //cout << "SPARSE ACTION at time " << time_select << " and (" << avgx << ", " << avgy << ") -->" << bc->get_pos(avgx, avgy, time_select) << endl;
    opt_path.push(opt_p);
    if (straight_line){
      double euclidean_danger = sqrt(pow(abs(endx-avgx),2) + pow(abs(endy-avgy), 2));
      double danger_grid = bc->get_pos(avgx, avgy, time_select);
      if (danger_grid > euclidean_danger && sparse == true){
	sparse = false; // failed to find a clear path

	danger_point.x = avgx;
	danger_point.y = avgy;
	danger_point.t = time_select;
      }	 
    } else if (goal_orientation == 2) {
      /*
	OPT_BEARING:
	1: +1/+1
	3: -1/+1
	5: -1/-1
	7: +1/-1
       */

      if (opt_bearing == 1){
	int x = avgx-1;
	int y = avgy-1;

	if (x >= 0){ // means x is within bounds, no need to check 0 as we are doing x++, no negative movement
	  double euclidean_danger = sqrt(pow(abs(endx-x),2) + pow(abs(endy-avgy), 2));
	  double danger_grid = bc->get_pos(x, avgy, time_select);
	  if (danger_grid > euclidean_danger && sparse == true){
	    sparse = false; // failed to find a clear path

	    danger_point.x = avgx;
	    danger_point.y = avgy;
	    danger_point.t = time_select;
	  }	  
	}

	if (y >= 0){ // means y is within bounds
	  double euclidean_danger = sqrt(pow(abs(endx-avgx),2) + pow(abs(endy-y), 2));
	  double danger_grid = bc->get_pos(avgx, y, time_select);
	  if (danger_grid > euclidean_danger && sparse == true){
	    sparse = false; // failed to find a clear path

	    danger_point.x = avgx;
	    danger_point.y = avgy;
	    danger_point.t = time_select;
	  }	  
	}
	
      } else if (opt_bearing == 3) {
	int x = avgx+1;
	int y = avgy-1;

	if (x < MAP_WIDTH){ // means x is within bounds, no need to check 0 as we are doing x++, no negative movement
	  double euclidean_danger = sqrt(pow(abs(endx-x),2) + pow(abs(endy-avgy), 2));
	  double danger_grid = bc->get_pos(x, avgy, time_select);
	  if (danger_grid > euclidean_danger && sparse == true){
	    sparse = false; // failed to find a clear path

	    danger_point.x = avgx;
	    danger_point.y = avgy;
	    danger_point.t = time_select;
	  }	  
	}

	if (y >= 0){ // means y is within bounds
	  double euclidean_danger = sqrt(pow(abs(endx-avgx),2) + pow(abs(endy-y), 2));
	  double danger_grid = bc->get_pos(avgx, y, time_select);
	  if (danger_grid > euclidean_danger && sparse == true){
	    sparse = false; // failed to find a clear path

	    danger_point.x = avgx;
	    danger_point.y = avgy;
	    danger_point.t = time_select;
	  }	  
	}
	
      } else if (opt_bearing == 5) {
	int x = avgx+1;
	int y = avgy+1;

	if (x < MAP_WIDTH){ // means x is within bounds, no need to check 0 as we are doing x++, no negative movement
	  double euclidean_danger = sqrt(pow(abs(endx-x),2) + pow(abs(endy-avgy), 2));
	  double danger_grid = bc->get_pos(x, avgy, time_select);
	  if (danger_grid > euclidean_danger && sparse == true){
	    sparse = false; // failed to find a clear path

	    danger_point.x = avgx;
	    danger_point.y = avgy;
	    danger_point.t = time_select;
	  }	  
	}

	if (y < MAP_HEIGHT){ // means y is within bounds
	  double euclidean_danger = sqrt(pow(abs(endx-avgx),2) + pow(abs(endy-y), 2));
	  double danger_grid = bc->get_pos(avgx, y, time_select);
	  if (danger_grid > euclidean_danger && sparse == true){
	    sparse = false; // failed to find a clear path

	    danger_point.x = avgx;
	    danger_point.y = avgy;
	    danger_point.t = time_select;
	  }	  
	}
	
      } else if (opt_bearing == 7) {
	int x = avgx-1;
	int y = avgy+1;

	if (x >= 0){ // means x is within bounds, no need to check 0 as we are doing x++, no negative movement
	  double euclidean_danger = sqrt(pow(abs(endx-x),2) + pow(abs(endy-avgy), 2));
	  double danger_grid = bc->get_pos(x, avgy, time_select);
	  if (danger_grid > euclidean_danger && sparse == true){
	    sparse = false; // failed to find a clear path

	    danger_point.x = avgx;
	    danger_point.y = avgy;
	    danger_point.t = time_select;
	  }	  
	}

	if (y < MAP_HEIGHT){ // means y is within bounds
	  double euclidean_danger = sqrt(pow(abs(endx-avgx),2) + pow(abs(endy-y), 2));
	  double danger_grid = bc->get_pos(avgx, y, time_select);
	  if (danger_grid > euclidean_danger && sparse == true){
	    sparse = false; // failed to find a clear path

	    danger_point.x = avgx;
	    danger_point.y = avgy;
	    danger_point.t = time_select;
	  }	  
	}
	
      }

      double euclidean_danger = sqrt(pow(abs(endx-avgx),2) + pow(abs(endy-avgy), 2));
      double danger_grid = bc->get_pos(avgx, avgy, time_select);
      //cout << planeid << ": " << x << ", " << avgy << " : " << euclidean_danger << " vs. " << danger_grid << endl;
      if (danger_grid > euclidean_danger && sparse == true){
	sparse = false; // failed to find a clear path
	  
	// Death, Despair, Disaster...a plane will probably be in this spot in the future that we want to avoid
	cout << "Danger: (" << avgx << ", " << avgy << ", " << time_select << ") has " << danger_grid << " vs tolerable " << euclidean_danger << endl;
	danger_point.x = avgx;
	danger_point.y = avgy;
	danger_point.t = time_select;
	//getchar();
      }     
    } else if (goal_orientation == 1){
      for (int x = avgx -1; x<=avgx+1; x++){
	// bounds checking:: not off map, not outside of optimal range, etc.
	if (x < 0 || x >= MAP_WIDTH)
	  continue;

	double euclidean_danger = sqrt(pow(abs(endx-x),2) + pow(abs(endy-avgy), 2));
	double danger_grid = bc->get_pos(x, avgy, time_select);
	//cout << planeid << ": " << x << ", " << avgy << " : " << euclidean_danger << " vs. " << danger_grid << endl;
	if (danger_grid > euclidean_danger && sparse == true){
	  sparse = false; // failed to find a clear path
	  // NOTE: Should BLOCK OFF these three optimal areas from A* (i.e., A* cannot look at these three or so dangerous spaces, no matter how good they may look)
	  // code here
	  
	  // Death, Despair, Disaster...a plane will probably be in this spot in the future that we want to avoid
	  cout << "Danger: (" << x << ", " << avgy << ", " << time_select << ") has " << danger_grid << " vs tolerable " << euclidean_danger << endl;
	  danger_point.x = avgx;
	  danger_point.y = avgy;
	  danger_point.t = time_select;
	  //getchar();
	}	
      }
    } else if (goal_orientation == 0){
      for (int y = avgy-1; y<=avgy+1; y++){
	// bounds checking
	// bounds checking:: not off map, not outside of optimal range, etc.
	if (y < 0 || y >= MAP_HEIGHT)
	  continue;

	double euclidean_danger = sqrt(pow(abs(endx-avgx),2) + pow(abs(endy-y), 2));
	double danger_grid = bc->get_pos(avgx, y, time_select);
	//cout << planeid << ":: " << avgx << ", " << y << " : " << euclidean_danger << " vs. " << danger_grid << endl;
	if (danger_grid > euclidean_danger && sparse == true){
	  sparse = false; // failed to find a clear path
	  // NOTE: Should BLOCK OFF these three optimal areas from A* (i.e., A* cannot look at these three or so dangerous spaces, no matter how good they may look)
	  // code here
	  
	  // Death, Despair, Disaster...a plane will probably be in this spot in the future that we want to avoid
	  cout << "Danger: (" << avgx << ", " << y << ", " << time_select << ") has " << danger_grid << " vs tolerable " << euclidean_danger << endl;
	  danger_point.x = avgx;
	  danger_point.y = avgy;
	  danger_point.t = time_select;
	  // getchar();
	}
      }
    }

    // increase our time to select the next time step in our best_cost/danger_grid time series
    time_select++;
    time_select = time_select>19 ? 19 : time_select;
  }

  if (sparse){
    cout << "Sparse " << planeid << endl;
    point keep_calm_and_carry_on;
    keep_calm_and_carry_on.x = endx;
    keep_calm_and_carry_on.y = endy;
    return keep_calm_and_carry_on;
  }


  // DANGER IS A FOOT, DANGER IS A FOOT
  // i.e. some part of our path's swath is too dangerous, we must attempt to avoid a collision

  // Empty our a_path queue, if anything is left in it from a previous calculation
  while (!a_path.empty()){
    a_path.pop();
  }

  // This is our new avoidance point to move to on the grid; it attempts to be as close as possible to our optimal grid path while avoiding unneccesary danger
  point move;
  move.x = -1;
  move.y = -1;

  bc_grid = bc;

  // third, run the algorithm and get our actual set of points
  other_main(startx, starty, endx, endy, planeid);

  // cout << planeid << ": " << endx << ", " << endy << endl;
  // fourth, compare our point sets
  while (!a_path.empty() && !opt_path.empty()){
    point opt = opt_path.front();
    point a_st = a_path.front();

    // calculate Chebyshev, if within X of a goal, only do straightline
    int x = abs(a_st.x - endx);
    int y = abs(a_st.y - endy);
    int cheb = -1;
    if (x > y)
      cheb = x;
    else
      cheb = y;

    
    if (a_st.x == endx || a_st.y == endy /*|| cheb < 3 */){
      //cout << "staight line " << endl;
      straight_line = true;    
      //getchar();
    } else {
      straight_line = false;
    }
  

    // move tracks the x,y position of the A-star path
    //cout << "X: " << opt.x << " vs " << a_st.x << " AND Y: " << opt.y << " vs. " << a_st.y << " on T: " << opt.t << " vs. " << a_st.t << endl;
    cout << "OPT: (" << opt.x << ", " << opt.y << ") versus A_ST (" << a_st.x << ", " << a_st.y << ") at " << opt.t << " vs. " << a_st.t << endl;
    //cout << "At time " << a_st.t << " we will move to " << a_st.x << ", " << a_st.y << endl;

    // Paths have diverged...end loop
    // Code changed to compare optimal paths: Andrew Kaizer 6/13/2011
    if (!straight_line){
      if (goal_orientation == 0){ // Y
	if ((opt.x != a_st.x) || (opt.y-1 != a_st.y && opt.y != a_st.y && opt.y+1 != a_st.y)){
	  // If the path A* has choosen is not equal to the Bearing heuristic, then
	  // an optimal selection was not choosen; probably to avoid a collision
	  // NOTE: opt_opt_path is provably optimal, if opt_a_star is not opt_opt_path it is not optimal
	  //cout << "Collision avoidance point: " << a_st.x << ", " << a_st.y << ", " << a_st.t <<  endl;
	  //getchar();
	  move = a_st;
	  break;
	}
      } else if (goal_orientation == 1) { // X
	if ((opt.y != a_st.y) || (opt.x-1 != a_st.x && opt.x != a_st.x && opt.x+1 != a_st.x)){
	  // If the path A* has choosen is not equal to the Bearing heuristic, then
	  // an optimal selection was not choosen; probably to avoid a collision
	  // NOTE: opt_opt_path is provably optimal, if opt_a_star is not opt_opt_path it is not optimal
	  //cout << "Collision avoidance point: " << a_st.x << ", " << a_st.y << ", " << a_st.t <<  endl;
	  //getchar();
	  move = a_st;
	  break;
	}
      } else if (goal_orientation == 2) { // diagonal...
	// opt_bearing 1,3,5,7
	if (opt_bearing == 1){ // -1/-1
	  if ((opt.y != a_st.y) && (opt.y-1 != a_st.y) && (opt.x != a_st.x) && (opt.x-1 != a_st.x)){
	    move = a_st;
	    break;
	  }
	} else if (opt_bearing == 3){ // +1/-1
	  if ((opt.y != a_st.y) && (opt.y-1 != a_st.y) && (opt.x != a_st.x) && (opt.x+1 != a_st.x)){
	    move = a_st;
	    break;
	  }
	} else if (opt_bearing == 5){ // +1/+1
	  if ((opt.y != a_st.y) && (opt.y+1 != a_st.y) && (opt.x != a_st.x) && (opt.x+1 != a_st.x)){
	    move = a_st;
	    break;
	  }
	} else if (opt_bearing == 7){ // -1/+1
	  if ((opt.y != a_st.y) && (opt.y+1 != a_st.y) && (opt.x != a_st.x) && (opt.x-1 != a_st.x)){
	    move = a_st;
	    break;
	  }
	}
      }
    }else {
      if (opt.x != a_st.x || opt.y != a_st.y){
	move = a_st;
	break;
      }
    }

 
    /**
    if (opt.x != a_st.x || opt.y != a_st.y){
      move = a_st;
      break;
    }
    */
    opt_path.pop();
    a_path.pop();
  } // end while

  if (move.x == -1){
    //cout << "At tend of path, and A* says that the way is clear...this is either an error in A* or Sparse parsing...must resolve " << endl;
    //getchar();
    move.x = endx;
    move.y = endy;
  } 

  /**
  cout << "Our final move  for " << planeid << " is : " << move.x << ", " << move.y << endl;
  getchar();
	  int nmap[MAP_WIDTH][MAP_HEIGHT];
	  for (int y = 0; y < MAP_HEIGHT; y++){
	    for (int x = 0; x < MAP_WIDTH; x++){
	      nmap[x][y] = -1;
	    }
	  }
	  while (!expan.empty()){
	    point p = expan.front();
	    expan.pop();
	    nmap[p.x][p.y] = p.t;	    
	  }

	  for (int y = 0; y < MAP_HEIGHT; y++){
	    for (int x = 0; x < MAP_WIDTH; x++){
	      if (x == endx && y == endy)
		cout << "EE ";
	      else if (nmap[x][y] == -1)
		cout << ".  ";
	      else if (nmap[x][y] < 10)
		cout << nmap[x][y] << "  ";
	      else
		cout << nmap[x][y] << " "; 
	    }
	    cout << endl;
	  }

	  getchar();
  */
  return move;
}

