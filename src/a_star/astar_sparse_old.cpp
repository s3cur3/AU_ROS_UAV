////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// STL A* Search implementation
// (C)2001 Justin Heyes-Jones
//
// Finding a path on a simple grid maze
// This shows how to do shortest path finding using A*

// A* code largely untouched, all output, simulation, and whatnot is part of UAV Team 6 (really, team 2)

////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "stlastar.h" // See header for copyright and usage information
#include "Plane_fixed.h"
#include <iostream>
#include <math.h>
#include <limits>

// QUEUE: used for calculating optimal paths, other paths
#include <queue>
#include <stack>
#include <map>

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0
#define OUTPUT_NODES 0 // use this to output text version of goal states

using namespace std;
using namespace map_tools;

// Global data
// The world map

int MAP_WIDTH = 41;
int MAP_HEIGHT = 37;
const int ROUTE_ABANDONED = 1; // if 1, says if search terminated
best_cost *bc_grid;

// easy calculation for next move
// 0-7 are actual moves, 8 is goal state, 9 is stall state
const int movex[10] = {1, 1, 0, -1, -1, -1, 0, 1, 0, 0};
const int movey[10] = {0, 1, 1, 1, 0, -1, -1, -1, 0, 0};
int movegoals = 8; // 0 - 7
int sparse_expansion = 20;
bearing_t initial_bearing;

// Start and End spots
int s_x = -1;
int s_y = -1;
int e_x = -1;
int e_y = -1;

std::map<int, bearing_t> astar_to_map;
std::map<bearing_t, int> map_to_astar;

// The position a plane starts in its starting grid
/**
   |-----|-----|
   |     |     |
   |  1  |  2  |
   |     |     |
   |--5--7--6--|
   |     |     |
   |  3  |  4  |
   |     |     |
   |-----|-----|
 */
int rough_quadrant;
/**
   End user mods
*/


// This struct is used to store (and reply) with the next "destination" that the plane is flying to
struct point {
  int x;
  int y;
  int t;
  bearing_t b;
} ;

// The danger point is the point of divergence between our pre-sparse search and A* search
point danger_point;

// a_path is the path A* recommends
bool first_star = true; // the first time we go through A-Star, we use it to create our optimal path according to our boards given constraints (EUCLIDEAN + 22.5)
queue<point> a_path;
queue<point> a_path_out;


int astar_map[ 1 ];
// map helper functions

int GetMap( int x, int y )
{
  return 0;
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


bool similar_bearing(bearing_t initial_bear, bearing_t target_bear){
  for (int i = -2; i <=2; i++){
    int a_star_bearing = map_to_astar[target_bear];
    a_star_bearing = (a_star_bearing + i) % movegoals;
    if (a_star_bearing < 0){
      a_star_bearing = 7 + i;
    }

    if (astar_to_map[a_star_bearing] == initial_bear)
      return true;
  }

  return false;
}

bearing_t opposite_bearing(bearing_t bear){
  if (bear == N)
    return S;
  else if (bear == NE)
    return SW;
  else if (bear == E)
    return W;
  else if (bear == SE)
    return NW;
  else if (bear == S)
    return N;
  else if (bear == SW)
    return NE;
  else if (bear == W)
    return E;
  else if (bear == NW)
    return SE;
  else
    return N;
}

// Definitions

class MapSearchNode
{
public:
  unsigned int x;	 // the (x,y) positions of the node
  unsigned int y;
  unsigned int timestep; // AK: the timestep that the node is being considered under
  /**
     maneuver is a tricky concept...
     Since we have a turn radius of 22.5*, it takes us 2 grid spaces to make a turn (i.e., we cannot do a 45* turn from East to North East, it has to be East (t=0), EastNorthEast(t=1), North East(t=2)).
     However, if we did include a maneuver bearing, then our planes would be stuck going in a single direction (East to East to East, they could never be biased towards East North East!
     We also need to grab a Planes actual position at time 0 in their grid!  This is important because of these expanions:
     IF 0 == TOP_HALF OF GRID
     - - - 3
     - - 2 3
     0 1 2 3
     - - 2 3
     - - - -
     ELSE IF 0 == LOWER_HALF OF GRID
     - - - -
     - - 2 3
     0 1 2 3
     - - 2 3
     - - - 3
     ELSE 0 == MIDDLE OF GRID (lucky us)
     - - - 3
     - - 2 3
     0 1 2 3
     - - 2 3
     - - - 3
   */

  bearing_t parent_bearing; // AK: Used for children node to understand where their parent is coming from

  // I realize that this variable name is absurdly long, but it gets the point across
  // In A* we now tell the child what nodes it might be able to expand.
  // so at t=0, we expand t=1 and tell it what it can look at in t=2 (but we leave the bounds checking to t=1)
  queue<bearing_t> legal_expansions_for_child;

  /** Time for another episode of Node Expansion with UAV-Team-Two
      IF P=N and M=NE, then P*M -> NNE (North-North-East)
   */
	
  MapSearchNode() { 
    x = y = 0; 
    timestep = 0; 
    parent_bearing = initial_bearing; 
  }
  //MapSearchNode( unsigned int px, unsigned int py ) { x=px; y=py; }
  MapSearchNode( unsigned int px, unsigned int py, unsigned int t_step, bearing_t parent, queue<bearing_t> my_maneuver) 
  { 
    x=px; 
    y=py; 
    parent_bearing = parent;
    legal_expansions_for_child=my_maneuver; 
    if (t_step > 19)
      timestep = 19;
    else
      timestep = t_step;
  } // AK

  ~MapSearchNode(){
    while (!legal_expansions_for_child.empty()){
      legal_expansions_for_child.pop();
    }
  }

  float GoalDistanceEstimate( MapSearchNode &nodeGoal );
  bool IsGoal( MapSearchNode &nodeGoal );
  bool GetSuccessors( AStarSearch<MapSearchNode> *astarsearch, MapSearchNode *parent_node );
  float GetCost( MapSearchNode &successor );
  bool IsSameState( MapSearchNode &rhs );

  void PrintNodeInfo(); 

  int getX();
  int getY();
  int getT();
  bearing_t getB();

};

bool MapSearchNode::IsSameState( MapSearchNode &rhs )
{

  // same state in a maze search is simply when (x,y) are the same
  if( (x == rhs.x) &&
      (y == rhs.y) &&
      (timestep == rhs.timestep) ) 
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

bearing_t MapSearchNode::getB(){
  return parent_bearing;
}

// Heuristic is BC (DG+BC) -- since no need to calculate, just choose minimal legal cost
float MapSearchNode::GoalDistanceEstimate( MapSearchNode &nodeGoal )
{
  //cout << x << ", " << y << " : " << timestep << " " << bc_grid->get_pos(x, y, timestep) << endl;
  
  double cost = bc_grid->get_pos(x, y, timestep);

  return cost;
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
  //cout << endl << "I AM NODE: (" << x << ", " << y << ") : " << timestep << endl;
 
  int time_z = -1;
  //int bearing = -1;

  if ( !parent_node ){
    // if we have a parent, we actually only have one really legal move...that is going in the same direction as we current are
    legal_expansions_for_child.push(initial_bearing);
    parent_bearing = initial_bearing;
    cout << parent_bearing << endl;
  } else {
    // Our parent node -they raised us to be the node we were meant to be (ish)
    
    //cout << "My parent is: " << parent_x << ", " << parent_y << ", " << parent_node->timestep << endl;
  } 
  // update our time step
  time_z = timestep+1>20 ? 20 : 1+timestep;
  //cout << "Parent's Time: " << parent_node->timestep << " and our new time: " << time_z << endl;

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

  // in legal_moves, we are checking for the legal moves allowed 2 moves into the future (the next additional move is only our immediate move)
  // Immedaite next move addition
  // Check to see if expansion to the given node is legal...if not, tough luck
  while (!legal_expansions_for_child.empty()){
    bearing_t next_expansion = legal_expansions_for_child.front();
    legal_expansions_for_child.pop();
    // at most we will have 3 possible moves at t=2, usually only 2 moves, though
    
    int a_st_bearing = -1;
    // convert the parent bearing to the A-Star move setup (0 = EAST in A_ST, 0 = NORTH in MAP)
    if (parent_bearing == N)
      a_st_bearing = 6;
    else if (parent_bearing == NE)
      a_st_bearing = 7;
    else if (parent_bearing == E)
      a_st_bearing = 0;
    else if (parent_bearing == SE)
      a_st_bearing = 1;
    else if (parent_bearing == S)
      a_st_bearing = 2;
    else if (parent_bearing == SW)
      a_st_bearing = 3;
    else if (parent_bearing == W)
      a_st_bearing = 4;
    else if (parent_bearing == NW)
      a_st_bearing = 5;

    int r[] = {(a_st_bearing-1), a_st_bearing%movegoals, (a_st_bearing+1)%movegoals}; 

    if(r[0] < 0){ // wraps around from 0 to 7
      r[0] += movegoals;
    }

    int range_start = -1;
    int range_end = -1;


    range_start = 0;
    range_end = 2;
    

    if (time_z == 1){ // at time 1, we only have 1 valid move...
      range_start = 1;
      range_end = 1;
    }
    
    for (int i = range_start; i <= range_end; i++){
      queue<bearing_t> child_expansions;
      int legal_moves = -1;
      if ((int)x + movex[r[i]] >= 0 && (int)x + movex[r[i]] < MAP_WIDTH &&
	  (int)y + movey[r[i]] >= 0 && (int)y + movey[r[i]] < MAP_HEIGHT &&
	  (int)x + movex[r[i]] >= lesser_x && (int)x + movex[r[i]] <= greater_x &&
	  (int)y + movey[r[i]] >= lesser_y && (int)y + movey[r[i]] <= greater_y){

	child_expansions.push(astar_to_map[r[i]]);
	legal_moves = r[i];
	//cout << " GO: X -- " << x+movex[r[i]] << ", " << y+movey[r[i]] << " at time: " << time_z << endl; //<< " ---- "  << bc_grid->get_pos(x+movex[r[i]], y+movey[r[i]], time_z) << endl;

      }        
      MapSearchNode NewNode;
      // push each possible move except allowing the search to go backwards

      // left
      if( legal_moves == 4 )
	{
	  //cout << "L " << x-1 << ", " << y << endl;
	  NewNode = MapSearchNode( x-1, y, time_z, W, child_expansions);
	  astarsearch->AddSuccessor( NewNode );
	}	

      // top
      if( legal_moves == 6 )
	{
	  //cout << "T " << x << ", " << y-1 << endl;
	  NewNode = MapSearchNode( x, y-1, time_z, N, child_expansions);
	  astarsearch->AddSuccessor( NewNode );
	}	

      // right
      if( legal_moves == 0 )
	{
	  //cout << "R " << x+1 << ", " << y << endl;
	  NewNode = MapSearchNode( x+1, y, time_z, E, child_expansions);
	  astarsearch->AddSuccessor( NewNode );
	}	

      // down	
      if( legal_moves == 2 )
	{
	  //cout << "D " << x << ", " << y+1 << endl;
	  NewNode = MapSearchNode( x, y+1, time_z, S, child_expansions);
	  astarsearch->AddSuccessor( NewNode );
	}	


      // Andrew Kaizer is adding Diagonals below here...
      // down right
      if( legal_moves == 1 )
	{
	  //cout << "DR " << x+1 << ", " << y+1 << endl;
	  NewNode = MapSearchNode( x+1, y+1, time_z, SE,child_expansions);
	  astarsearch->AddSuccessor( NewNode );
	}	
  
      // top right
      if( legal_moves == 7 )
	{
	  //cout << "TR " << x+1 << ", " << y-1 << endl;
	  NewNode = MapSearchNode( x+1, y-1, time_z, NE, child_expansions);
	  astarsearch->AddSuccessor( NewNode );
	}	
  
      // down left
      if( legal_moves == 3 )
	{
	  //cout << "DL " << x-1 << ", " << y+1 << endl;
	  NewNode = MapSearchNode( x-1, y+1,time_z, SW, child_expansions);
	  astarsearch->AddSuccessor( NewNode );
	}	
  
      // top left
      if( legal_moves == 5 )
	{
	  //cout << "TL " << x-1 << ", " << y-1 << endl;
	  NewNode = MapSearchNode( x-1, y-1, time_z, NW, child_expansions);
	  astarsearch->AddSuccessor( NewNode );
	}	
    }
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
int other_main(int startx, int starty, int endx, int endy, int planeid ){
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
		p.b = node->getB();
		a_path.push(p);
		a_path_out.push(p);
		
		//cout << node->getT() << ": " << p.x << ", " << p.y << " and " << node->getT() << " --> " << bc_grid->get_pos(p.x, p.y, node->getT()) << endl;
		//getchar();
		//cout << "  " << p.x << ", " << p.y+1 << " --> " << bc_grid->get_pos(p.x, p.y+1, node->getT()) << endl;
		//cout << "  " << p.x << ", " << p.y-1 << " --> " << bc_grid->get_pos(p.x, p.y-1, node->getT()) << endl << endl;
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
	  moves = -1;
	}
      // Display the number of loops the search went through
#if OUTPUT_NODES
      cout << "SearchSteps : " << SearchSteps << "\n";
#endif

      SearchCount ++;

      //astarsearch.CancelSearch();
      //astarsearch.SearchStep();

      astarsearch.EnsureMemoryFreed();
    }

  return moves;
}


point IREFUSETODIE(point previous, point a_st){
  cout << " I REFUSE TO DIE " << endl;
  bearing_t b = astar_to_map[getGridBearing(previous.x, previous.y, a_st.x, a_st.y)];
  double comp1 = numeric_limits<double>::infinity();
  double comp2 = numeric_limits<double>::infinity();
  double comp3 = numeric_limits<double>::infinity(); //only used in diag
  if (b == N || b == W || b == S || b == E){
    if (b == S || b == N){
      cout << "Divergence on a S or N " << endl;
      if (previous.x + 1 < MAP_WIDTH)
	comp1 = bc_grid->get_pos(previous.x+1, previous.y, previous.t);
      if (previous.x - 1 >= 0)
	comp2 = bc_grid->get_pos(previous.x-1, previous.y, previous.t);

      if (comp1 < comp2){
	previous.x += 1;
      } else if (comp2 < comp1) {
	previous.x -= 1;
      } else if (b == N) { // comp1 == comp2
	previous.x -= 1;
      } else if (b == S) {
	previous.x += 1;
      }

    } else if (b == E || b == W) {
      cout << "Divergence on a E or W" << endl;
      if (previous.y + 1 < MAP_HEIGHT)
	comp1 = bc_grid->get_pos(previous.x, previous.y+1, previous.t);
      if (previous.y - 1 >= 0)
	comp2 = bc_grid->get_pos(previous.x, previous.y-1, previous.t);
	  
      if (comp1 < comp2){
	previous.y += 1;	
      } else if (comp2 < comp1){
	previous.y -= 1;
      } else if (b == E){
	previous.y -= 1;
      } else if (b == W){
	previous.y += 1;
      }
    } 
  } else if (b == SE || b == SW || b == NW || b == NE){	
    if (b == SE){
      cout << "Divergence on a SE " << endl;
      if (previous.x + 1 < MAP_WIDTH && previous.y+1 < MAP_HEIGHT)
	comp1 = bc_grid->get_pos(previous.x+1, previous.y+1, previous.t);
      if (previous.x + 1 < MAP_WIDTH)
	comp2 = bc_grid->get_pos(previous.x+1, previous.y, previous.t);
      if (previous.y + 1 < MAP_HEIGHT)
	comp3 = bc_grid->get_pos(previous.x, previous.y+1, previous.t);
	  
      if (comp1 < comp2 && comp1 < comp3){
	previous.x += 1;
	previous.y += 1;
      } else if (comp2 < comp1 && comp2 < comp3) {
	previous.x += 1;
      } else if (comp3 < comp1 && comp3 < comp1) {
	previous.y += 1;
      } else {
	previous = a_st;
      }	  
    } else if (b == NE) {
      cout << "Divergence on a NE " << endl;
      if (previous.x + 1 < MAP_WIDTH && previous.y-1 >= 0)
	comp1 = bc_grid->get_pos(previous.x+1, previous.y-1, previous.t);
      if (previous.x + 1 < MAP_WIDTH)
	comp2 = bc_grid->get_pos(previous.x+1, previous.y, previous.t);
      if (previous.y - 1 >= 0)
	comp3 = bc_grid->get_pos(previous.x, previous.y-1, previous.t);
	  
      if (comp1 < comp2 && comp1 < comp3){
	previous.x += 1;
	previous.y -= 1;
      } else if (comp2 < comp1 && comp2 < comp3) {
	previous.x += 1;
      } else if (comp3 < comp1 && comp3 < comp1) {
	previous.y -= 1;
      } else {
	previous = a_st;
      } 
    } else if (b == NW) {
      cout << "Divergence on a NW" << endl;
      if (previous.x - 1 >= 0 && previous.y-1 >= 0)
	comp1 = bc_grid->get_pos(previous.x-1, previous.y-1, previous.t);
      if (previous.x - 1 >= 0)
	comp2 = bc_grid->get_pos(previous.x-1, previous.y, previous.t);
      if (previous.y - 1 >= 0)
	comp3 = bc_grid->get_pos(previous.x, previous.y-1, previous.t);
	  
      if (comp1 < comp2 && comp1 < comp3){
	previous.x -= 1;
	previous.y -= 1;
      } else if (comp2 < comp1 && comp2 < comp3) {
	previous.x -= 1;
      } else if (comp3 < comp1 && comp3 < comp1) {
	previous.y -= 1;
      } else {
	previous = a_st;
      } 
    } else if (b == SW) {
      cout << "Divergence on a SW" << endl;
      if (previous.x - 1 >= 0 && previous.y+1 < MAP_HEIGHT)
	comp1 = bc_grid->get_pos(previous.x-1, previous.y+1, previous.t);
      if (previous.x - 1 >= 0)
	comp2 = bc_grid->get_pos(previous.x-1, previous.y, previous.t);
      if (previous.y + 1 < MAP_HEIGHT)
	comp3 = bc_grid->get_pos(previous.x, previous.y-1, previous.t);
	  
      if (comp1 < comp2 && comp1 < comp3){
	previous.x -= 1;
	previous.y += 1;
      } else if (comp2 < comp1 && comp2 < comp3) {
	previous.x -= 1;
      } else if (comp3 < comp1 && comp3 < comp1) {
	previous.y += 1;
      } else {
	previous = a_st;
      } 
    }	
  }

  return previous;
} 


bool is_sparse(best_cost *bc){
  int startx = s_x;
  int starty = s_y;
  int endx = e_x;
  int endy = e_y;
  // used to determine if the plane advances on the X or the Y axis through time
  // if the optimal first move from Start to End is X1 and the optimal last move from End to Start is X1, then orientation is X based (0) and we need to check based on Y (1)
  // for example: if the start is (23,11) and the goal is (12,29): the first move from S-E is to (22,11), the last move from E-S is to (23,11)
  //  to determine the rest of the optimal paths through time will require us to look at the X values, not the Y values
  int goal_orientation = -1;
  
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
    if (!sparse){
      while(!start_to_goal.empty())
	start_to_goal.pop();

      while(!goal_to_start.empty())
	goal_to_start.pop();

      break;
    }
    
    // grab points from the queue and stack, which are on the same x coordinate (this only holds for a square or rectangular grid)
    point g_t_s = goal_to_start.top();
    point s_t_g = start_to_goal.front();
    goal_to_start.pop();
    start_to_goal.pop();
    //straight_line = false; // maybe?

    // on the first pop, we must check our orientation to determine which spaces to consider for optimality
    if (goal_orientation == -1){
      if ((opt_bearing == 1 || opt_bearing == 3 || opt_bearing == 5 || opt_bearing == 7) && s_t_g.x == g_t_s.x && s_t_g.y == g_t_s.y){
	goal_orientation = 2; // diagonal
	//straight_line = true;

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

    cout << "SPARSE ACTION at time " << time_select << " and (" << avgx << ", " << avgy << ") with g_orient " << goal_orientation << " -->" << bc->get_pos(avgx, avgy, time_select) << endl;
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
      cout << "DIAG" << endl;
      double euclidean_danger = sqrt(pow(abs(endx-avgx),2) + pow(abs(endy-avgy), 2));
      double danger_grid = bc->get_pos(avgx, avgy, time_select);
      //cout << planeid << ": " << avgx << ", " << avgy << " : " << euclidean_danger << " vs. " << danger_grid << endl;
      if (danger_grid > euclidean_danger && sparse == true){
	sparse = false; // failed to find a clear path
	  
	// Death, Despair, Disaster...a plane will probably be in this spot in the future that we want to avoid
	//cout << "Danger: (" << x << ", " << avgy << ", " << time_select << ") has " << danger_grid << " vs tolerable " << euclidean_danger << endl;
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
	  //cout << "Danger: (" << x << ", " << avgy << ", " << time_select << ") has " << danger_grid << " vs tolerable " << euclidean_danger << endl;
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
    return true;
  } else {
    return false;
  }
}

point immediate_avoidance_point(best_cost *bc, std::map<int, Plane> &planey_the_plane_map, point a_st, point previous){
  cout << "Starting immediate avoidance " << endl;
  // new_avoidance is our new, safer waypoint to move to that is still within the range of legal moves
  point new_avoidance = a_st;

  queue<point> planes_discovered;
  // the owner of the danger has to be withint a_st.t
  // Since we assume planes move one (1) grid per second, the time at a_st.t means that the plane that is a threat should be, at most, t steps away
  // loop through our grid look at time 0 to find these deadly planes
  for (int y = -a_st.t-1; y <= a_st.t+1; y++){
    for (int x = -a_st.t-1; x <= a_st.t+1; x++){
      // make sure our x and y pos are in the graph...would be silly to checkoutside of the graph :P
      if (x >= 0 && x < MAP_WIDTH && y >= 0 && y < MAP_HEIGHT) {
	int x_pos = a_st.x + x;
	int y_pos = a_st.y + y;

	if (bc->get_pos(x_pos, y_pos, 0) >  sqrt(pow(x_pos-e_x, 2) + pow(y_pos-e_y, 2))){
	  point add;
	  add.x = x_pos;
	  add.y = y_pos;
	  planes_discovered.push(add);
	}
      }
    }
  }

  // Any plane we actually discover with the proper coordinates if added to our threat queue, which we will then use (in conjunction with the map) to plot proper avoidance :))))
  queue<int> threat;
  while (!planes_discovered.empty()){
    point plane_threat = planes_discovered.front();
    for( map< int, Plane >::iterator crnt_plane = planey_the_plane_map.begin(); crnt_plane != planey_the_plane_map.end(); ++crnt_plane )
    {     
      assert( (*crnt_plane).second.getId() != -100 );
      Plane myplane =  (*crnt_plane).second;
      Position plane_p = myplane.getLocation();
      if (plane_p.getX() == plane_threat.x && plane_p.getY() == plane_threat.y)
      {
        if (similar_bearing(myplane.get_named_bearing(), initial_bearing)){
          cout << "Aha! We have discovered that plane " << (*crnt_plane).second.getID() << " is a threat to our well being.  Taking avoidance actions...maybe. " << endl;
          threat.push(i);
        } else {
          cout << "We discovered that plane " << (*crnt_plane).second.getID() << " was in our threat area, however is moving away from us, so no need to add to danger list" << endl;
        }
        break;
      }
    } // end for each plane
    planes_discovered.pop();
  } // end while planes discovered not empty

  // determine closest threat that is at least 2 spots away (otherwise we cannot do anything about it)
  double closest = numeric_limits<double>::infinity();
  while(!threat.empty()){
    Plane myplane = planey_the_plane_map[threat.front()];
    point plane_point;
    plane_point.x = myplane.getLocation().getX();
    plane_point.y = myplane.getLocation().getY();
    double distance = sqrt(pow(plane_point.x-s_x, 2) + pow(plane_point.y-s_y, 2));
    if (distance > 3){
      if (distance < closest){
	closest = distance;
	new_avoidance = plane_point;
      }
    }
    threat.pop();
  }

  cout << "A_ST initially said: " << a_st.x << ", " << a_st.y << " at " << a_st.t << endl;
  cout << "Now we say: " << new_avoidance.x << ", " << new_avoidance.y << endl;
  return new_avoidance;
}

// Returns the point of divergence between provable optimal path and a-star path
point astar_point(best_cost *bc, double sx, double sy, int endx, int endy, int planeid, bearing_t current_bear, std::map<int, Plane> *planey_the_plane_map)
{
  bc_grid = bc;

  // Construct atar_to_map std::map (since map is 0=N and A_ST is 0=E)
  astar_to_map[0] = E;
  astar_to_map[1] = SE;
  astar_to_map[2] = S;
  astar_to_map[3] = SW;
  astar_to_map[4] = W;
  astar_to_map[5] = NW;
  astar_to_map[6] = N;
  astar_to_map[7] = NE;

  map_to_astar[E] = 0;
  map_to_astar[SE] = 1;
  map_to_astar[S] = 2;
  map_to_astar[SW] = 3;
  map_to_astar[W] = 4;
  map_to_astar[NW] = 5;
  map_to_astar[N] = 6;
  map_to_astar[NE] = 7;

  MAP_WIDTH = bc->get_width_in_squares();
  MAP_HEIGHT = bc->get_height_in_squares();
 

  while(!a_path.empty()){
    a_path.pop();
  }

  
  while(!a_path_out.empty()){
    a_path_out.pop();
  }
  

  // DETERMINE ROUGH QUADRANT (see top of file for visual display
  int startx = s_x = floor(sx);
  int starty = s_y = floor(sy);

  // For easy debugging (blank lines before and after a plane is announced)
  cout << endl << endl << planeid << " : (" << startx << ", " << starty << ") --> (" << endx << ", " << endy << ")" << endl << endl;

  e_x = endx;
  e_y = endy;
  initial_bearing = current_bear;
  point move;

  // used to avoid considering the dangerous point
  int goal_orientation = getGridBearing(startx, starty, endx, endy);
  bearing_t goal_bearing = astar_to_map[goal_orientation];
  int astar_bearing = map_to_astar[initial_bearing];
  //initial_bearing  = astar_to_map[goal_orientation];
  //cout << initial_bearing<< endl;
  //getchar();
  cout << initial_bearing << " vs " << goal_bearing << endl;

  if (initial_bearing == goal_bearing && is_sparse(bc)){
    cout << "SPARSE " << planeid << endl;
    move.x = endx;
    move.y = endy;
    cout << endl << endl << "In the end we choose; " << move.x << ", " << move.y << endl;
    return move;
  }
  

  queue<point> opt_path;
  bool at_goal = false;

  int current_x = startx;
  int current_y = starty;
  int time = 0;

  
  if (similar_bearing(initial_bearing, goal_bearing)){
    cout << "SIMILAR " << initial_bearing << " vs goal_bearing " << goal_bearing << endl;
    current_x += movex[astar_bearing];
    current_y += movey[astar_bearing];
    time += 1;
    point pr;
    pr.x = current_x;
    pr.y = current_y;
    pr.t = time;
    opt_path.push(pr);
  }
  
  bool new_goal_at_straight = false;

  while (!at_goal){
    double min_cost = numeric_limits<double>::infinity();
    int best_move = -1;
    for (int i = 0; i < 8; i++){
      int new_x = current_x + movex[i];
      int new_y = current_y + movey[i];

      if (sqrt(pow(new_x-endx, 2) + pow(new_y-endy, 2)) < min_cost){
	min_cost = sqrt(pow(new_x-endx, 2) + pow(new_y-endy, 2));
	best_move = i;
      }
    }

    current_x = current_x + movex[best_move];
    current_y = current_y + movey[best_move];
    point pr;
    pr.x = current_x;
    pr.y = current_y;
    pr.t = time+1 < 10 ? time + 1 : 10;
    opt_path.push(pr);
    time += 1;
    if (current_x == endx && current_y == endy)
      at_goal = true;
  }


  // third, run the algorithm and get our actual set of points
  move.x = endx;
  move.y = endy;
  int result = other_main(startx, starty, endx, endy, planeid);
  
  if (result == -1)
    cout << "Sad, A-star found no solution...I hope you like ROS giving you a bridge to nowhere" << endl;

  // Outside Zero means: our plane is trying to move away from our goal (probably due to orientation facing a different direction)
  // However, if this is true, we must do a new type of bounds checking to bring it back in.
  // Our new type of checking asks: if plane is still out of bounds AND the position is safe, continue ELSE return spot
  bool outside_zero = false;

  // only matters if it is outside of zero on the first move -- anything else should be handled by other code
  bool first;

  // These lessers values are used to determine where our grid exists...the greater_x is the larger x between the start and end positions (same for y)
  // These are used in outside_zero for our bounds checking
  int lesser_x = -1;
  int greater_x = -1;
  int lesser_y = -1;
  int greater_y = -1;

  if (startx > endx){
    greater_x = startx;
    lesser_x = endx;
  } else {
    greater_x = endx;
    lesser_x = startx;
  }

  if (starty > endy){
    greater_y = starty;
    lesser_y = endy;
  } else {
    greater_y = endy;
    lesser_y = starty;
  }
  

  point previous;
  previous.x = startx;
  previous.y = starty;
  previous.t = 0;

  if (!a_path.empty()){
    point a_st = a_path.front();
    // based on orientation?
    // initial bearing and goal bearing
    if (/*(a_st.x > greater_x || a_st.x < lesser_x || a_st.y > greater_y || a_st.y < lesser_y) &&*/ !(similar_bearing(initial_bearing, goal_bearing)))
      outside_zero = true;
  }

  bool straight = true;
  if (startx != endx && starty != endy)
    straight = false;

  if (outside_zero)
    cout << "ASTAR OUTSIDE ZERO" << endl;

  // THESE ARE TWO DIFFERENT CASES
  if (outside_zero){
    point double_previous;
    double_previous.x = -1;
    while (!a_path.empty()){
      point a_st = a_path.front();

      cout << "OUTSIZE ZERO MANEUVER: NO OPT CHECK: " << a_st.x << ", " << a_st.y << ": " << a_st.t << endl;
      // since we are using euclidean grid math, we must AVOID on the PIVOT (we do not have straight diagonals)
      // follow the star    

      double predicted_val = sqrt(pow(a_st.x-endx, 2) + pow(a_st.y-endy, 2));
      // if, for some reason, our turn is dangerous -- break from it
      
      if ((bc->get_pos(a_st.x, a_st.y, a_st.t) > predicted_val) && a_st.t > 1){
	cout << "OUTSIDE ZERO MANEUVER BAILING EARLY DUE TO DANGER: (" << a_st.x << ", " << a_st.y << ": " << a_st.t << ")" << endl;
	move = a_st;

	break;
      }
      
      if (a_st.x > greater_x || a_st.x < lesser_x || a_st.y > greater_y || a_st.y < lesser_y){
	outside_zero = true;
      } else {
	cout << "OUTSIDE ZERO MANEUVER SUCCESS: (" << a_st.x << ", " << a_st.y << ": " << a_st.t << ")" << endl;
	move = a_st;
	break;
      } 

      double_previous = previous;
      previous = a_st;
      opt_path.pop();
      a_path.pop();
    }
  } else {
    bool pivot_flag = false;
    while (!a_path.empty()  && !opt_path.empty()){
      point a_st = a_path.front();
      point opt = opt_path.front();

      cout << "OPT: " << opt.x << ", " << opt.y << ": " << opt.t << " VERSUS " << a_st.x << ", " << a_st.y << ": " << a_st.t << endl;
      // follow the star

      // eh, A-star might recommend this, but we should control it (danger)
      if(bc->get_pos(a_st.x, a_st.y, a_st.t) > sqrt(pow(a_st.x-endx, 2) + pow(a_st.y-endy, 2)) && a_st.t > 1){
	cout << "TOO DANGEROUS: GIVING MOVES IMMEDIATELY" <<endl;

	// Immediate term avoidance (3,4,5, and 6 seconds)
	if (a_st.t <= 7  && a_st.t > 3){
	  // I am assuming control of this vessel

	  // avoidance maneuver must follow the "I wanna live" logic, not A* path which is more like a "Hey, Google, I am feeling lucky; please don't rick roll me"
	  // DETERMINE EMPIRICALLY the best place to go; behind the hazardous plane or in front of it.
	  // At times 4, 5, and 6 this is reasonable; at times 2 and 3 it may not be as good, but you should already be out of the way by then :)

	  move = immediate_avoidance_point(bc, *planey_the_plane_map, a_st, previous);	  
	} else {	  
	  move = a_st;
	}
	break;
      }

      if ((opt.x != a_st.x || opt.y != a_st.y) && a_st.t > 1){
	cout << "BREAK ON:  (" << a_st.x << ", " << a_st.y << ": " << a_st.t << ")" << endl;
	if (a_st.t > 3)
	  move = immediate_avoidance_point(bc, *planey_the_plane_map, a_st, previous);
	else
	  move = a_st;
	break;
      }
      
      // Pivot is an odd issue...
      if ((a_st.x == endx || a_st.y == endy) && !straight && a_st.t > 1){
	cout << "Getting the hell outta dodge, we have hit a pivot  (" << a_st.x << ", " << a_st.y << ": " << a_st.t << ")" << endl;
	move = previous; //used to a_st
	break;
      }
      
      
      previous = a_st;
      opt_path.pop();
      a_path.pop();
    } 
  }



  
  cout << "ASTAR COMPLETE" << endl;
  
  /*
  // A-star path output
  if (planeid == 6){
    int nmap[58][58] = {0};
    while(!a_path_out.empty()){
      point t = a_path_out.front();
      a_path_out.pop();
      if (t.x < 60 && t.y < 60)
	nmap[t.x][t.y] = -1;
    }

    cout << planeid << ": " << endl << "   ";
    for (int x = 0; x < 58; x++){
      if (x < 10)
	cout << "0" << x << " ";
      else
	cout << x << " ";
    }

    for (int y = 0; y < 58; y++){
      if (y < 10)
	cout << y << ": ";
      else
	cout << y << ":";
      for (int x = 0; x < 58; x++){
	if (x == startx && y == starty){
	  cout << "S  ";
	}
	else if (x == endx && y == endy)
	  cout << "E  ";
	else if (nmap[x][y] == -1)
	  cout << "R  ";
	else
	  cout << ".  ";
      }
      cout << endl;
    }
  }
  
  if (planeid == 6){
    for (int a = 0; a < 7; a++){
      cout << planeid << ": " << a << endl << "    ";
      for (int x = 0; x < 58; x++){
	if (x < 10)
	  cout << "0" << x << "  ";
	else
	  cout << x << "  ";
      }
      cout << endl;
      for (int y = 0; y < 58; y++){
	if (y < 10)
	  cout << "0" << y << ": ";
	else
	  cout << y << ": " ;
	for (int x = 0; x < 58; x++){
	  if (x == move.x && y == move.y)
	    cout << "MMM ";
	  else if (x == startx && y == starty)
	    cout << "PPP ";
	  else if (x == endx && y == endy)
	    cout << "DDD ";
	  else if (round(bc->get_pos(x, y, a)) > 999)
	    cout << "999 " << "" ;
	  else if (round(bc->get_pos(x, y, a)) > 99)
	    cout << round(bc->get_pos(x, y, a)) << " ";
	  else if (round(bc->get_pos(x, y, a)) > 9)
	    cout << round(bc->get_pos(x, y, a)) << "  ";
	  else if (round(bc->get_pos(x, y, a)) < 10)
	    cout  << round(bc->get_pos(x, y, a)) << "   ";
	  else
	    cout << round(bc->get_pos(x, y, a)) << " ";
	}
	cout << endl;
      }
      cout << endl;
    }
  }
  
  */

  cout << endl << endl << "In the end we choose; " << move.x << ", " << move.y << endl << endl;
  return move;
}
