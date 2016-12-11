//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"
// He Chen
#include <limits>
// He Chen

namespace SteerLib
{
	// He Chen
	// enumeration of sets
	enum Group { NONE, OPEN, CLOSED, INCONS};

	class AStarPlannerNode;
	// fake BinaryMinHeap, just to keep the interface for now
	class STEERLIB_API BinaryMinHeap {
		public:
			bool empty();
			int size();
			void add(AStarPlannerNode *);
			void remove(AStarPlannerNode *);
			void clear(void);
			AStarPlannerNode *getMin(void);
			void deleteMin(void);
			// extractMin returns the pointer of the min, and delete it from the heap
			// suppose extract min is called after the 'empty' check, and always return a valid value
			AStarPlannerNode *extractMin(void);
			AStarPlannerNode *extractMinADA(void);
			void merge(Group group, std::vector<AStarPlannerNode *> &H);
			bool exist(AStarPlannerNode &node);
			std::vector<AStarPlannerNode *> _bmHeap;
			// real Heap
			void bubbleDown(int index);
			void bubbleUp(int index);
			void decreaseKey(AStarPlannerNode *node);
	};
	// key value structure for ADA*
	class STEERLIB_API KeyADA {
		public:
			double k1;
			double k2;
			KeyADA() {k1 = 0; k2 = 0;}
			bool operator<(KeyADA other) const {
				return this->k1 < other.k1 || (this->k1 == other.k1 && this->k2 < other.k2);
			}
	};
		
	// He Chen

	/*
		@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
		@attributes 
		f : the f value of the node
		g : the cost from the start, for the node
		point : the point in (x,0,z) space that corresponds to the current node
		parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
		@operators 
		The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/
	class STEERLIB_API AStarPlannerNode{
		public:
			double f;
			double g;
			Util::Point point;
			AStarPlannerNode* parent;
			Group group;
			AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent, Group _group = NONE, double _rhs = std::numeric_limits<double>::max(), int _index = -1)
			{
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
				group = _group;
				rhs = _rhs;
				index = _index;
			}
			bool operator<(AStarPlannerNode other) const
		    {
		        return this->f < other.f;
		    }
		    bool operator>(AStarPlannerNode other) const
		    {
		        return this->f > other.f;
		    }
		    bool operator==(AStarPlannerNode other) const
		    {
		        return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
		    }
			// He Chen
			double rhs;
			int index; // mark the position in the BinaryMinHeap
			KeyADA keys;
			bool operator!=(AStarPlannerNode other) const {
				return ((this->point.x != other.point.x) || (this->point.z != other.point.z));
			}
			// He Chen

	};
	// He Chen
	// algorithms structure for Sequential Search
	class STEERLIB_API ssAlg {
		public:
			double epsilon;
			BinaryMinHeap openSet;
			std::vector<AStarPlannerNode *> closedSet;
			AStarPlannerNode *startnode, *goalnode;
			std::map<int, AStarPlannerNode *> visitedNode;

			int generatedNode;
			int expandedNode;

			ssAlg() { epsilon = 0, generatedNode = 0, expandedNode = 0, startnode = NULL, goalnode = NULL; }
			ssAlg(double _e, int _gN = 0, int _eN = 0, AStarPlannerNode *s = NULL, AStarPlannerNode *g = NULL) {
				epsilon = _e, generatedNode = _gN, expandedNode = _eN, startnode = s, goalnode = g;
			}
			double heuristic(AStarPlannerNode *node) {
				// use Euclidean distance * epsilon for now
				return epsilon * (node->point - goalnode->point).length();
			}
			void addOpenSet(AStarPlannerNode *node) {
				node->group = OPEN;
				openSet.add(node);
			}
			void addClosedSet(AStarPlannerNode *node) {
				node->group = CLOSED;
				closedSet.push_back(node);
			}
	};
	// He Chen
	

	class STEERLIB_API AStarPlanner{
		public:
			AStarPlanner();
			~AStarPlanner();
			// NOTE: There are four indices that need to be disambiguated
			// -- Util::Points in 3D space(with Y=0)
			// -- (double X, double Z) Points with the X and Z coordinates of the actual points
			// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
			// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
			// When navigating the space or the Grid, do not mix the above up

			/*
				@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
				The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
				and checks cells in bounding box area
				[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
				[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
				This function also contains the griddatabase call that gets traversal costs.
			*/
			bool canBeTraversed ( int id );
			/*
				@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
			*/
			Util::Point getPointFromGridIndex(int id);

			/*
				@function computePath
				DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
				This function executes an A* query
				@parameters
				agent_path : The solution path that is populated by the A* search
				start : The start point
				goal : The goal point
				_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
				append_to_path : An optional argument to append to agent_path instead of overwriting it.
			*/

			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);

			// He Chen
			// weighted A*
			bool weightedAstar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal);
			void initWeightedA(Util::Point start, Util::Point goal);
			std::vector<int> getNeighbors(int id);
			double heuristic(const AStarPlannerNode &node1, const AStarPlannerNode &node2);
			double arccost(const AStarPlannerNode &node1, const AStarPlannerNode &node2);
			// ARA*
			// procedures
			double fvalueARA(AStarPlannerNode &node);
			bool ARAstar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal);
			void initARA(Util::Point start, Util::Point goal);
			bool improvePathARA(std::vector<Util::Point> &agent_path);
			// other helper functions
			void clearClosedSet(void);
			void addOpenSet(AStarPlannerNode *);
			void removeOpenSet(AStarPlannerNode *);
			void addClosedSet(AStarPlannerNode *);
			void addInconsSet(AStarPlannerNode *);
			double getEpsilonPrime(void);
			void gatherStatistics(std::vector<Util::Point> &);
			// data structures
			// it seems better to have a startnode and goalnode
			AStarPlannerNode *_startnode;
			AStarPlannerNode *_goalnode;
			// data structure to store pointers of visited nodes
			std::map<int, AStarPlannerNode *> visitedNode;
			// need some data structures for ARA*: Open, Closed, Incons sets
			double _epsilon;
			BinaryMinHeap _openSet;
			std::vector<AStarPlannerNode *> _closedSet;
			std::vector<AStarPlannerNode *> _inconsSet; // also don't need to use Heap structure
			// data structures for ADA*, have the same start and goal node?
			// functions for ADA*
			void initADA(Util::Point start, Util::Point goal);
			void getKeyADA(AStarPlannerNode *node);
			bool ADAstar(std::vector<Util::Point>&, Util::Point, Util::Point);
			bool improvedPathADA(std::vector<Util::Point>&);
			void reconstructPathADA(std::vector<Util::Point>&);
			void updateStateADA(int);
			void getMinRhs(AStarPlannerNode *);
			bool mapchangeADA(void);
			void updateMap(void);
			// for statistics
			int _expandedNode;
			bool _oldstate;
			// for sequential search
			std::vector<ssAlg> _methods;
			bool sequentialSearch(std::vector<Util::Point> &, Util:: Point, Util::Point);
			void ssExpandState(AStarPlannerNode *, int);
			void ssUpdatef(AStarPlannerNode *, int);
			void ssReconstructPath(std::vector<Util::Point>&, int);
			void ssInit(Util::Point, Util::Point);
			void ssGatherStatistics(std::vector<Util::Point>&);
			// He Chen
		private:
			SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	};

}


#endif
