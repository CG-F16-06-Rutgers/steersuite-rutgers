//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
// He Chen
#define EPSILON 20
#define DELTAEPSILON -0.5
#define NUMBER_OF_ALG 4
#define SSW1 0
#define SSW2 1
// He Chen

namespace SteerLib
{
	AStarPlanner::AStarPlanner(){
		_startnode = NULL;
		_goalnode = NULL;
		_expandedNode = 0;
		_epsilon = EPSILON;
		_oldstate = false;

		// for ss, need to free the space occupied by ssAlg, TODO
	}

	AStarPlanner::~AStarPlanner(){
		delete _startnode;
		delete _goalnode;

		for (std::map<int, AStarPlannerNode *>::iterator iter = visitedNode.begin(); iter != visitedNode.end(); iter++)
			delete iter->second;

		for (int i=0; i<_methods.size(); i++) {
			delete _methods[i].startnode;
			delete _methods[i].goalnode;

			for (std::map<int, AStarPlannerNode *>::iterator iter = (_methods[i].visitedNode).begin(); iter != (_methods[i].visitedNode).end(); iter++)
				delete iter->second;
		}
	}

	bool AStarPlanner::canBeTraversed ( int id ) 
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x,z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x-OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z-OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z+OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i<=x_range_max; i+=GRID_STEP)
		{
			for (int j = z_range_min; j<=z_range_max; j+=GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords( i, j );
				traversal_cost += gSpatialDatabase->getTraversalCost ( index );
				
			}
		}

		if ( traversal_cost > COLLISION_COST ) 
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}



	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path,  Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		std::cerr<<"\nIn A*, suppose start point and goal point can be trversed" << std::endl;
		
		// weighted A*
		if (weightedAstar(agent_path, start, goal)) {
			gatherStatistics(agent_path);
			return true;
		}
		
		//ARA*
		//_expandedNode = 0;
		//if (ARAstar(agent_path, start, goal)) {
		//	gatherStatistics(agent_path);
		//	return true;
		//}
		
		// ADA*
		//if (ADAstar(agent_path, start, goal)) {
		//	gatherStatistics(agent_path);
		//	return true;
		//}

		// Sequential search
		//if (sequentialSearch(agent_path, start, goal)) {
		//	ssGatherStatistics(agent_path);
		//	return true;
		//}

		return false;
	}

	// He Chen
	// now implement the simplified version of Sequential Search
	bool AStarPlanner::sequentialSearch(std::vector<Util::Point> &agent_path, Util::Point start, Util::Point goal) {
		ssInit(start, goal);

		//while (!_methods[0].openSet.empty()) {
		while (_methods[0].openSet.getMin()->f < std::numeric_limits<double>::max()) {
			if (_methods[0].openSet.empty())
				std::cerr << "ERROR: open set for method 0 is empty!!!" << std::endl;
			AStarPlannerNode *current = _methods[0].openSet.getMin();
			for (int i=1; i<NUMBER_OF_ALG; i++) {
				if (_methods[i].openSet.empty())
					std::cerr << "ERROR: open set for method " << i << " is empty!!!" << std::endl;
				AStarPlannerNode *imin = _methods[i].openSet.getMin();
				if (imin->f <= SSW2 * current->f) {
					if (_methods[i].goalnode->g <= imin->f) {
						if (_methods[i].goalnode->g < std::numeric_limits<double>::max()) {
							ssReconstructPath(agent_path, i);
							return true;
						}
					}
					else {
						_methods[i].expandedNode++; // update expanded node
						_methods[i].openSet.deleteMin();
						ssExpandState(imin, i);
						_methods[i].addClosedSet(imin);
					}
				}
				else {
					if (_methods[0].goalnode->g <= current->f) {
						if (_methods[0].goalnode->g < std::numeric_limits<double>::max()) {
							ssReconstructPath(agent_path, 0);
							return true;
						}
					}
					else {
						_methods[0].expandedNode++; // update expanded node
						_methods[0].openSet.deleteMin();
						ssExpandState(current, 0);
						_methods[0].addClosedSet(current);
					}
				}
			}
		}

		return false;
	}

	void AStarPlanner::ssExpandState(AStarPlannerNode *node, int index) {
		double _larged = std::numeric_limits<double>::max();
		std::vector<int> neighborsindex = getNeighbors(gSpatialDatabase->getCellIndexFromLocation(node->point));
		for (int i=0; i<neighborsindex.size(); i++) {
			int id = neighborsindex[i];
			if (canBeTraversed(id)) {
				AStarPlannerNode *neighbor;
				std::map<int, AStarPlannerNode *>::iterator iter = _methods[index].visitedNode.find(id);
				if (iter == _methods[index].visitedNode.end()) {
					neighbor = new AStarPlannerNode(getPointFromGridIndex(id), _larged, _larged, NULL);
					(_methods[index].visitedNode)[id] = neighbor;
				}
				else
					neighbor = iter->second;

				double tmp_g = node->g + arccost(*neighbor, *node);
				if (tmp_g < neighbor->g) {
					neighbor->parent = node;
					neighbor->g = tmp_g;
					// update f value! next
					ssUpdatef(neighbor, index);
					if (neighbor->group != CLOSED) {
						if (neighbor->group == OPEN)
							(_methods[index].openSet).decreaseKey(neighbor);
						else
							_methods[index].addOpenSet(neighbor);
					}
				}
			}
		}
	}

	void AStarPlanner::ssUpdatef(AStarPlannerNode *node, int index) {
		node->f = node->g + SSW1 * _methods[index].heuristic(node);
	}

	void AStarPlanner::ssReconstructPath(std::vector<Util::Point> &agent_path, int i) {
		std::cerr << "ssReconstructPath is called" << std::endl;
		AStarPlannerNode *getpath = _methods[i].goalnode;

		while (*getpath != *(_methods[i].startnode)) {
			if (getpath == NULL)
				std::cerr << "ERROR: getpath is NULL in ssReconstructPath" << std::endl;
			agent_path.push_back(getpath->point);
			getpath = getpath->parent;
		}
		agent_path.push_back(_methods[i].startnode->point);
		std::reverse(agent_path.begin(), agent_path.end());
		// print out the path
		std::cerr << "Method " << i << "! epsilon is: " << _methods[i].epsilon << std::endl;
		for (int j=0; j<agent_path.size(); j++)
			std::cerr << agent_path[j].x << "," << agent_path[j].z << " ";
		std::cerr << std::endl;
	}

	void AStarPlanner::ssInit(Util::Point start, Util::Point goal) {
		double _larged = std::numeric_limits<double>::max();
		int indexs = gSpatialDatabase->getCellIndexFromLocation(start);
		int indexg = gSpatialDatabase->getCellIndexFromLocation(goal);

		// initialize the methods, with four different epsilons
		double tmpEpsilon[NUMBER_OF_ALG] = {1, 2, 5, 15};
		//double tmpEpsilon[NUMBER_OF_ALG] = {0, 1, 2, 5};
		for (int i=0; i<NUMBER_OF_ALG; i++) {
			ssAlg method;
			method.epsilon = tmpEpsilon[i];
			method.startnode = new AStarPlannerNode(start, 0, _larged, NULL);
			method.goalnode = new AStarPlannerNode(goal, _larged, _larged, NULL);
			method.startnode->f = method.startnode->g + SSW1 * method.heuristic(method.startnode);

			method.visitedNode[indexs] = method.startnode;
			method.visitedNode[indexg] = method.goalnode;

			// add to open set based on f
			method.addOpenSet(method.startnode);

			_methods.push_back(method);
		}
		std::cerr << "methods size is: " << _methods.size() << std::endl;
		std::cerr << "The epsilons are: ";
		for (int i=0; i<_methods.size(); i++)
			std::cerr << _methods[i].epsilon << " ";
		std::cerr << std::endl;
	}

	// print out some statistics in sequential Search
	void AStarPlanner::ssGatherStatistics(std::vector<Util::Point> &agent_path) {
		for (int i=0; i<_methods.size(); i++) {
			std::cerr << "Method " << i << ": epsilon: " << _methods[i].epsilon << std::endl;
			std::cerr << "number of generatedNode is: " << (_methods[i].visitedNode).size() << std::endl;
			std::cerr << "number of expandedNode is: " <<(_methods[i].expandedNode) << std::endl;
		}
		// path cost
		double pathcost = 0;
		for (int i=0; i<agent_path.size()-1; i++) {
			pathcost += (agent_path[i+1] - agent_path[i]).length();
		}
		std::cerr << "path cost is: " << pathcost << std::endl;
		std::cerr << "length or number of the node in the path: " << agent_path.size() << std::endl;
	}

	// print out number of generated nodes, expanded nodes, path cost, length, and time
	void AStarPlanner::gatherStatistics(std::vector<Util::Point> &agent_path) {
		std::cerr << "epsilon is: " << _epsilon << std::endl;
		int generatednode = 0;
		for (std::map<int, AStarPlannerNode *>::iterator iter = visitedNode.begin(); iter != visitedNode.end(); iter++)
			generatednode++;
		std::cerr << "number of generatednode is: " << generatednode << std::endl;
		// expanded nodes
		std::cerr << "number of expanded node is: " << _expandedNode << std::endl;
		// path cost
		double pathcost = 0;
		for (int i=0; i<agent_path.size()-1; i++) {
			pathcost += (agent_path[i+1] - agent_path[i]).length();
		}
		std::cerr << "path cost is: " << pathcost << std::endl;
		// length or number of the node in the path
		std::cerr << "length or number of the node in the path: " << agent_path.size() << std::endl;
	}
	// implementation of Anytime Dynamic A star
	void AStarPlanner::initADA(Util::Point start, Util::Point goal) {
		double _larged = std::numeric_limits<double>::max();
		// start node is changing
		if (_startnode == NULL) // first time call this function
			_startnode = new AStarPlannerNode(start, _larged, _larged, NULL, NONE, _larged);
		else if (_startnode->point != start) { // a new start point
			_startnode = new AStarPlannerNode(start, _larged, _larged, NULL, NONE, _larged);
			// TODO maybe reinitialize Sets
		}
		// assume goalnode does not change, so this is first time call, set the value of epsilon
		if (_goalnode == NULL) {
			_goalnode = new AStarPlannerNode(goal, _larged, _larged, NULL, NONE, 0);
			//_epsilon = 1.0; // TODO is it right to set it here?
			addOpenSet(_goalnode);
		}

		getKeyADA(_startnode);
		getKeyADA(_goalnode);

		// update the visitedNode structure
		int indexstart = gSpatialDatabase->getCellIndexFromLocation(start);
		int indexgoal = gSpatialDatabase->getCellIndexFromLocation(goal);
		visitedNode.insert(std::pair<int, AStarPlannerNode *>(indexstart, _startnode));
		visitedNode.insert(std::pair<int, AStarPlannerNode *>(indexgoal, _goalnode));
	}

	void AStarPlanner::getKeyADA(AStarPlannerNode *node) {
		if (node->g > node->rhs) { // a number times numeric_limits<double> may overflow!!!!
			node->keys.k1 = node->rhs + _epsilon * heuristic(*_startnode, *node);
			node->keys.k2 = node->rhs;
		}
		else {
			node->keys.k1 = node->g + heuristic(*_startnode, *node);
			node->keys.k2 = node->g;
		}
	}

	bool AStarPlanner::mapchangeADA() {
		Util::Point obs(9.5, 0, 12.5);
        	int id = gSpatialDatabase->getCellIndexFromLocation(obs);
        	bool ret = canBeTraversed(id);
        	if (ret != _oldstate) {
                	_oldstate = ret;
                	return true;
        	}
        	return false;
	}

	void AStarPlanner::updateMap() {
		_openSet.clear();
		_closedSet.clear();
		_inconsSet.clear();

		for (std::map<int, AStarPlannerNode *>::iterator iter = visitedNode.begin(); iter != visitedNode.end(); iter++)
			delete (iter->second);
		visitedNode.clear();
		_startnode = NULL;
		_goalnode = NULL;
	}

	bool AStarPlanner::ADAstar(std::vector<Util::Point> &agent_path, Util::Point start, Util::Point goal) {
		if (mapchangeADA()) {
			updateMap();
		}

		if (_epsilon < 1) {
                        std::cerr << "epsilon is smaller than 1: " << _epsilon << " should get the optimal solution" << std::endl;
                        return true;
                }
                initADA(start, goal);
                if (_epsilon != EPSILON) { // not the first time, need to update the keys()
                        for (std::map<int, AStarPlannerNode *>::iterator iter = visitedNode.begin(); iter != visitedNode.end(); iter++) {
                                //std::cerr << "the address is: " << iter->second << std::endl; 
                                if (iter->second->group == OPEN || iter->second->group == INCONS) {
                                       /* double tmp = fvalueARA((*iter->second));
                                        if (tmp < iter->second->f) {
                                               std::cerr << "tmp < iter->second->f" << std::endl;
                                        }
                                        else {
                                                std::cerr << "tmp >= iter->second->f" << std::endl;
                                        }
                                        iter->second->f = fvalueARA(*(iter->second));*/
					getKeyADA(iter->second);
                                }
                        }
                        _openSet.merge(OPEN, _inconsSet);

                        // clear the Closed set and set the 'group' to be NONE
                        clearClosedSet();
                }
                if (_openSet.empty()) {
                        std::cerr << "ERROR: Open set is empty before the improvePathARA call" << std::endl;
                }
                _expandedNode = 0;
                bool ret = improvedPathADA(agent_path);
                _epsilon += DELTAEPSILON;
                return ret;
		//int time_limit = 10;
		//double deltaEpsilon = -0.5;
		/*initADA(start, goal);
		std::cerr << "Done the initialization in ADA" << std::endl;

		return improvedPathADA(agent_path);
		*/
		/*while (time_limit > 0) {
			if (!improvedPathADA(agent_path))
				return false;
			time_limit--;
			if (_epsilon > 1)
				_epsilon +=  deltaEpsilon;
			if (changes in edge costs are detected)

		}*/
	}

	
	bool AStarPlanner::improvedPathADA(std::vector<Util::Point> &agent_path) {
		while (!_openSet.empty()) {
			std::cerr << "size of open set: " << _openSet.size() << std::endl;
			AStarPlannerNode *current = _openSet.extractMinADA();
			std::cerr << "Done the extractMinADA" << std::endl;
			std::cerr << "current->keys: " << current->keys.k1 << " " << current->keys.k2 << std::endl;
			std::cerr << "current->rhs: " << current->rhs << std::endl;
			
			if (!(current->keys < _startnode->keys) && (_startnode->rhs == _startnode->g)) {
				std::cerr << "first if true " << std::endl;
				std::cerr << "_startnode->g: " << _startnode->g << std::endl;
				std::cerr << "startnode->keys: " << _startnode->keys.k1 << " " << _startnode->keys.k2 << std::endl;
				addOpenSet(current);
				reconstructPathADA(agent_path);
				return true;
			}
			// update expanded node?
			_expandedNode++;

			if (current->g > current->rhs) {
				std::cerr << "second if true " << std::endl;
				current->g = current->rhs;
				getKeyADA(current); // every time g or rhs is changed, keys should be updated, potentially need to addjust the heap! TODO
				addClosedSet(current);
				// for all predecessor of s, updateState of s'; are these the predecessor?? TODO
				std::vector<int> neighborsindex = getNeighbors(gSpatialDatabase->getCellIndexFromLocation(current->point));
				for (int i=0; i<neighborsindex.size(); i++)
					updateStateADA(neighborsindex[i]);
			}
			else {
				std::cerr << " all here" << std::endl;
				current->g = std::numeric_limits<double>::max();
				// for all updateState(s') and updateState(s)
				std::vector<int> neighborsindex = getNeighbors(gSpatialDatabase->getCellIndexFromLocation(current->point));
				for (int i=0; i<neighborsindex.size(); i++)
					updateStateADA(neighborsindex[i]);
				updateStateADA(gSpatialDatabase->getCellIndexFromLocation(current->point));
			}
		}
		return false;
	}

	// TODO current implementation doesn't seem to be correct
	void AStarPlanner::reconstructPathADA(std::vector<Util::Point> &agent_path) {
		std::cerr << "reconstructPathADA is called" << std::endl;
		agent_path.clear();
		AStarPlannerNode *getpath = _startnode;

		while (*getpath != *_goalnode) {
			agent_path.push_back(getpath->point);
			getpath = getpath->parent;
		}

		agent_path.push_back(_goalnode->point);
		return;
	}

	void AStarPlanner::updateStateADA(int id) {
		if (!canBeTraversed(id))
			return;

		AStarPlannerNode *neighbor;
		std::map<int, AStarPlannerNode *>::iterator iter = visitedNode.find(id);

		if (iter == visitedNode.end()) {
			double _larged = std::numeric_limits<double>::max();
			neighbor = new AStarPlannerNode(getPointFromGridIndex(id), _larged, _larged, NULL, NONE, _larged);
			visitedNode[id] = neighbor;
		}
		else
			neighbor = iter->second;

		if (*neighbor != *_goalnode) {
			getMinRhs(neighbor); // how to get the successor ?? TODO
		}

		if (neighbor->group == OPEN)
			removeOpenSet(neighbor);

		// update the keys!
		getKeyADA(neighbor);

		if (neighbor->g != neighbor->rhs) {
			if (neighbor->group != CLOSED) {
				if (neighbor->group != OPEN)
					addOpenSet(neighbor);
				//else
				//	continue; // do nothing for now
			}
			else
				addInconsSet(neighbor);
		}
	}

	// this function is a little odd. What if the successor is not visited? Are the neighbors the successors?
	void AStarPlanner::getMinRhs(AStarPlannerNode *node) {
		std::vector<int> neighborsindex = getNeighbors(gSpatialDatabase->getCellIndexFromLocation(node->point));

		double tmprhs = std::numeric_limits<double>::max();
		for (int i=0; i<neighborsindex.size(); i++) {
			int id = neighborsindex[i];
			if (canBeTraversed(id)) {
				std::map<int, AStarPlannerNode *>::iterator iter = visitedNode.find(id);
				if (iter != visitedNode.end()) {
					AStarPlannerNode *neighbor = iter->second;
					double tmp = arccost(*node, *neighbor) + neighbor->g;
					if (tmp < tmprhs) { // if the cost is infinite, you probably don't want it to be your parent
						tmprhs = tmp;
						node->rhs = tmprhs;
						node->parent = neighbor;
					}
				}
			}
		}
	}
				

	// initialization, used for ARA*, supposed for now that start and goal doesn't change
	void AStarPlanner::initARA(Util::Point start, Util::Point goal) {
		double _larged = std::numeric_limits<double>::max();
		if (_goalnode == NULL) {
			_goalnode = new AStarPlannerNode(goal, _larged, _larged, NULL);
			int indexgoal = gSpatialDatabase->getCellIndexFromLocation(goal);
			visitedNode[indexgoal] = _goalnode;
		}
		//TODO if (goal != _goalnode.point)
		if (_startnode == NULL) {
			_startnode = new AStarPlannerNode(start, 0, _larged, NULL);
			_startnode->f = fvalueARA(*_startnode);
			int indexstart = gSpatialDatabase->getCellIndexFromLocation(start);
			visitedNode[indexstart] = _startnode;

			addOpenSet(_startnode);
		}
		//TODO if (start != _startnode.point)

		// update the visitedNode structure
	}

	// implementation of ARA*
	bool AStarPlanner::ARAstar(std::vector<Util::Point> &agent_path, Util::Point start, Util::Point goal)
	{
		if (_epsilon < 1) {
			std::cerr << "epsilon is smaller than 1: " << _epsilon << " should get the optimal solution" << std::endl;
			return true;
		}
		initARA(start, goal);
		if (_epsilon != EPSILON) { // not the first time, need to update the fvalues
			for (std::map<int, AStarPlannerNode *>::iterator iter = visitedNode.begin(); iter != visitedNode.end(); iter++) {
                                //std::cerr << "the address is: " << iter->second << std::endl;
                                if (iter->second->group == OPEN || iter->second->group == INCONS) {
                                        double tmp = fvalueARA((*iter->second));
                                        if (tmp < iter->second->f) {
                                                std::cerr << "tmp < iter->second->f" << std::endl;
                                        }
                                        else {
                                                std::cerr << "tmp >= iter->second->f" << std::endl;
                                        }
                                        iter->second->f = fvalueARA(*(iter->second));
                                }
                        }
                        _openSet.merge(OPEN, _inconsSet);

                        // clear the Closed set and set the 'group' to be NONE
                        clearClosedSet();
		}
		if (_openSet.empty()) {
			std::cerr << "Open set is empty before the improvePathARA call" << std::endl;
		}
		_expandedNode = 0;
		bool ret = improvePathARA(agent_path);
		_epsilon += DELTAEPSILON;
		return ret;
		/*
		int time_limit = 1;
		_epsilon = 20;
		double deltaEpsilon = -0.5;
		double epsilonPrime;
		// do some initializations, 'start and goal nodes', visitedNode structure
		initARA(start, goal);
		// add the start node into openSet;
		addOpenSet(_startnode);

		if (!improvePathARA(agent_path)) {
			std::cerr << "Didn't find a path in improvePathARA at the beginning!" << std::endl;
			return false;
		}
		time_limit -= 1;
		epsilonPrime = getEpsilonPrime();

		while (epsilonPrime > 1 && time_limit > 0) {
			_epsilon += deltaEpsilon;
			// merge OpenSet and InconsSet, also need to update the fvalues
			// TODO need a better implementation
			for (std::map<int, AStarPlannerNode *>::iterator iter = visitedNode.begin(); iter != visitedNode.end(); iter++) {
				//std::cerr << "the address is: " << iter->second << std::endl;
				if (iter->second->group == OPEN || iter->second->group == INCONS) {
					double tmp = fvalueARA((*iter->second));
					if (tmp < iter->second->f) {
						std::cerr << "tmp < iter->second->f" << std::endl;
					}
					else {
						std::cerr << "tmp >= iter->second->f" << std::endl;
					}
					iter->second->f = fvalueARA(*(iter->second));
				}
			}
			_openSet.merge(OPEN, _inconsSet);

			// clear the Closed set and set the 'group' to be NONE
			clearClosedSet();

			if (!improvePathARA(agent_path)) {
				std::cerr << "Didnit find a path in improvePathARA!" << std::endl;
				return false;
			}
			epsilonPrime = getEpsilonPrime();
			time_limit--;
		}

		if (epsilonPrime <= 1) {
			std::cerr << "epsilonPrime is smaller than 1, should find the optimal path" << std::endl;
		}
		if (time_limit <= 0) {
			std::cerr << "time is up in ARA*" << std::endl;
		}

		return true;
		*/
	}

	void AStarPlanner::clearClosedSet() {
		for (int i=0; i<_closedSet.size(); i++) {
			_closedSet[i]->group = NONE;
		}
		_closedSet.clear();
	}

	void AStarPlanner::addClosedSet(AStarPlannerNode *node) {
		node->group = CLOSED;
		_closedSet.push_back(node);
	}

	void AStarPlanner::addOpenSet(AStarPlannerNode *node) {
		node->group = OPEN;
		_openSet.add(node);
	}

	void AStarPlanner::removeOpenSet(AStarPlannerNode *node) {
		node->group = NONE;
		_openSet.remove(node);
	}

	void AStarPlanner::addInconsSet(AStarPlannerNode *node) {
		node->group = INCONS;
		_inconsSet.push_back(node);
	}

	bool AStarPlanner::improvePathARA(std::vector<Util::Point> &agent_path) {
		double largestDouble = std::numeric_limits<double>::max();

		while (!_openSet.empty()) {
			AStarPlannerNode *current = _openSet.extractMin();
			if (fvalueARA(*_goalnode) <= current->f) {
				addOpenSet(current);
				std::cerr << "goalnode fvalue: " << _goalnode->f << std::endl;
				std::cerr << "current fvalue: " << current->f << std::endl;
				std::cerr << "improvedPathARA: Found it! fvalue of goalnode is not larger than min of Open Set, break out" << std::endl;
				break;
			}
			/*if (*current == *_goalnode) {
				std::cerr << "improvedPathARA: the current min is the goalnode" << std::endl;
				break;
			}*/

			addClosedSet(current);

			// going to expand node
			_expandedNode++;

			std::cerr << "after update fvalues" << std::endl;
			std::vector<int> neighborsindex = getNeighbors(gSpatialDatabase->getCellIndexFromLocation(current->point));
			for (int i=0; i<neighborsindex.size(); i++) {
				int index = neighborsindex[i];
				AStarPlannerNode *neighbor;
				std::map<int, AStarPlannerNode *>::iterator iter = visitedNode.find(index);

				if (iter == visitedNode.end()) {
					if (canBeTraversed(index)) {
						neighbor = new AStarPlannerNode(getPointFromGridIndex(index), largestDouble, largestDouble, NULL);
						visitedNode[index] = neighbor;
					}
					else
						continue;
				}
				else {
					neighbor = iter->second;
				}

				std::cerr << "after assign neighbor" << std::endl;
				double tmp_g = current->g + arccost(*neighbor, *current);
				if (tmp_g < neighbor->g) {
					std::cerr << "we are doing update in improvePathARA" << std::endl;
					neighbor->parent = current;
					neighbor->g = tmp_g;
					neighbor->f = fvalueARA(*neighbor);
					if (neighbor->group != CLOSED) {
						if (neighbor->group != OPEN) {
							addOpenSet(neighbor);
						}
						else {
							//continue; // don't need to do anything yet // TODO
							_openSet.decreaseKey(neighbor);
						}
					}
					else
						addInconsSet(neighbor);
				}
			}
		}

		if (_openSet.empty()) 
			return false;

		// path reconstruction
		agent_path.clear();
		std::cerr << "now start path reconstruction in ARA*" << std::endl;
		AStarPlannerNode *getpath = _goalnode;
		while (*getpath != *_startnode) {
			agent_path.push_back(getpath->point);
			getpath = getpath->parent;
		}
		agent_path.push_back(_startnode->point);
		std::reverse(agent_path.begin(), agent_path.end());
		// print out the path
		std::cerr << "epsilon is: " << _epsilon << std::endl;
		for (int i=0; i<agent_path.size(); i++) {
			std::cerr << agent_path[i].x << "," << agent_path[i].z << ",";
		}
		std::cerr << std::endl;

		return true;
	}

	// compute epsilon prime
	double AStarPlanner::getEpsilonPrime() {
		if (_openSet.empty() && _inconsSet.empty()) {
			std::cerr << "Error: openSet and inconsSet can NOT be empty at the same time" << std::endl;
			return 0;
		}

		double minOpenIncons = std::numeric_limits<double>::max();
		for (std::map<int, AStarPlannerNode *>::iterator iter = visitedNode.begin(); iter != visitedNode.end(); iter++) {
			// dumb way to traverse the OPEN and INCONS sets
			if (iter->second->group == OPEN || iter->second->group == INCONS) {
				double tmp = iter->second->g + heuristic(*(iter->second), *_goalnode);
				if (tmp < minOpenIncons)
					minOpenIncons = tmp;
			}
		}

		return MIN(_epsilon, _goalnode->g/minOpenIncons);
	}

	// procedure fvalue for ARA*
	double AStarPlanner::fvalueARA(AStarPlannerNode &node) {
		return node.g + _epsilon * heuristic(node, *_goalnode);
	}

	// initialization of weighted Astar
	void AStarPlanner::initWeightedA(Util::Point start, Util::Point goal) {
		double _larged = std::numeric_limits<double>::max();
		_startnode = new AStarPlannerNode(start, 0, _larged, NULL);
		//TODO if (start != _startnode->point)
		_goalnode = new AStarPlannerNode(goal, _larged, _larged, NULL);
		//TODO if (goal != _goalnode.point)
		_startnode->f = _startnode->g + _epsilon * heuristic(*_startnode, *_goalnode); // here is different from initARA

		/*int id1 = gSpatialDatabase->getCellIndexFromLocation(start);
		int id2 = gSpatialDatabase->getCellIndexFromLocation(goal);
		_startnode->point = getPointFromGridIndex(id1);
		_goalnode->point = getPointFromGridIndex(id2);*/

		// update the visitedNode structure
		int indexstart = gSpatialDatabase->getCellIndexFromLocation(start);
		int indexgoal = gSpatialDatabase->getCellIndexFromLocation(goal);
		visitedNode[indexstart] = _startnode;
		visitedNode[indexgoal] = _goalnode;
	}
	
	// implementation of weightedAstar, need to provide the epsilon
	// if epsilon == 1, then it's Astar
	bool AStarPlanner::weightedAstar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal)
	{
		_epsilon = 10.0; // change it as 1.0, 2.0, and 5.0 -> different weight!
		double largestDouble = std::numeric_limits<double>::max();

		// some initialization, make a node for start and goal, and update the visitedNode structure
		initWeightedA(start, goal);

		// add the start node into openSet;
		addOpenSet(_startnode);

		std::cerr << "Openset size before the while loop: " << _openSet.size() << std::endl;
		while (!_openSet.empty()) {
			AStarPlannerNode *current;
			current = _openSet.extractMin();
			std::cerr << "Openset size after extracMin: " << _openSet.size() << std::endl;
			std::cerr << "The current fvalue: " << current->f << std::endl;
			std::cerr << "The current gvalue: " << current->g << std::endl;
			std::cerr << "The Cell index is : " << gSpatialDatabase->getCellIndexFromLocation(current->point) << std::endl;
			if (*current == *_goalnode)
				break;

			addClosedSet(current);

			// going to expand the node
			_expandedNode++;

			std::vector<int> neighborsindex = getNeighbors(gSpatialDatabase->getCellIndexFromLocation(current->point));
			for (int i=0; i<neighborsindex.size(); i++) {
				int index = neighborsindex[i];
				std::cerr << "neighbor cell index is: " << index << " current gvalue is: " << current->g << std::endl;
				AStarPlannerNode *neighbor;
				std::map<int, AStarPlannerNode *>::iterator iter = visitedNode.find(index);

				if (iter == visitedNode.end()) {
					std::cerr << "node does not exist, need to create a new node" << std::endl;
					if (canBeTraversed(index)) {
						std::cerr << "current gvalue before creation: " << current->g << " current address: " << current << std::endl;
						//AStarPlannerNode tmp(getPointFromGridIndex(index), largestDouble, largestDouble, NULL);
						neighbor = new AStarPlannerNode(getPointFromGridIndex(index), largestDouble, largestDouble, NULL);
						visitedNode[index] = neighbor;
						std::cerr << "current gvalue after creation: " << current->g << " neighbor address: " << neighbor << " current address: " << current << std::endl;
					}
					else {
						std::cerr << "point can not be traversed: " << std::endl;
						Util::Point tmp_p = getPointFromGridIndex(index);
						std::cerr << tmp_p.x << ", " << tmp_p.y << ", " << tmp_p.z << std::endl;
						continue;
					}
				}
				else {
					if (!canBeTraversed(index)) {
						std::cerr << "Bug here! it's supposed to be traversable, but it's not!!!!!" << std::endl;
					}
					std::cerr << "node exists, and it can be traversed" << std::endl;
					neighbor = iter->second;
				}

				if (neighbor->group == CLOSED) {
					std::cerr << "neighbor is in CLOSED" << std::endl;
					continue;
				}

				double tmp_g = current->g + arccost(*current, *neighbor);
				std::cerr << "tmp_g: " << tmp_g << " current_g: " << current->g << " arccost: " << arccost(*current, *neighbor) << std::endl;
				if (tmp_g < neighbor->g) {
					neighbor->parent = current;
					neighbor->g = tmp_g;
					neighbor->f = tmp_g + _epsilon * heuristic(*neighbor, *_goalnode);
					if (neighbor->group != OPEN) {
						addOpenSet(neighbor);
					}
					else {
						//continue; // neibhbor is in OPEN already
						// openSet decrease-key!!! TODO don't need to do anything in current implementation
						_openSet.decreaseKey(neighbor);
					}
				}
			}
		}

		if (_openSet.empty()) {
			std::cerr << "weighted A star did not find a path, openSet is empty" << std::endl;
			return false;
		}

		// path reconstruction
		std::cerr << "now start path reconstruction in weighted A star" << std::endl;
		AStarPlannerNode *getpath = _goalnode;
		//agent_path.push_back(goal);
		while (*getpath != *_startnode) {
			agent_path.push_back(getpath->point);
			getpath = getpath->parent;
		}
		agent_path.push_back(_startnode->point);
		//agent_path.push_back(start);
		std::reverse(agent_path.begin(), agent_path.end());

		return true;
	}

	// return a vector of neighbors given the index of the current point
	std::vector<int> AStarPlanner::getNeighbors(int id) {
		std::vector<int> ret;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(id, x, z);
		int x_min, x_max, z_min, z_max;

		x_min = MAX(x-1, 0);
		x_max = MIN(x+1, gSpatialDatabase->getNumCellsX());
		z_min = MAX(z-1, 0);
		z_max = MIN(z+1, gSpatialDatabase->getNumCellsZ());

		for (int i = x_min; i <= x_max; i += 1) {
			for (int j = z_min; j <= z_max; j += 1) {
				if ((i != x) || (j != z)) {
					int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
					ret.push_back(index);
				}
			}
		}

		return ret;
	}

	// calculate the heuristic given two points? or nodes?
	double AStarPlanner::heuristic(const AStarPlannerNode &node1, const AStarPlannerNode &node2) {
		// Euclidean
		return (node1.point - node2.point).length();

		// Manhattan
		return round(std::abs(node1.point.x - node2.point.x)) + round(std::abs(node1.point.z - node2.point.z));
	}

	// calculate the arc cost
	double AStarPlanner::arccost(const AStarPlannerNode &node1, const AStarPlannerNode &node2) {
		return (node1.point - node2.point).length();
	}

	// functions of BinaryMinHeap
	bool BinaryMinHeap::empty() {
		return _bmHeap.empty();
	}

	int BinaryMinHeap::size() {
		return _bmHeap.size();
	}

	void BinaryMinHeap::add(AStarPlannerNode *node) {
		//_bmHeap.push_back(node);
		int length = _bmHeap.size();
		_bmHeap.push_back(node);

		bubbleUp(length);
	}

	void BinaryMinHeap::remove(AStarPlannerNode *node) {
		std::vector<AStarPlannerNode *>::iterator iter = find(_bmHeap.begin(), _bmHeap.end(), node);
		if (iter != _bmHeap.end())
			_bmHeap.erase(iter);
	}

	void BinaryMinHeap::clear() {
		_bmHeap.clear();
	}

	AStarPlannerNode* BinaryMinHeap::extractMin(void) {
		/*if (empty()) {
			std::cerr << "\nin extractMin, not supposed to be here!" << std::endl;
			return NULL;
		}
		double fvalue = _bmHeap[0]->f;
		int findex = 0;
		for (int i=1; i<_bmHeap.size(); i++) {
			if (_bmHeap[i]->f < fvalue) {
				fvalue = _bmHeap[i]->f;
				findex = i;
			}
		}
		AStarPlannerNode *nodep = _bmHeap[findex];
		// need to delete min
		_bmHeap.erase(_bmHeap.begin() + findex);

		return nodep;*/
		// get min
		if (empty())
			return NULL;
		AStarPlannerNode *ret = _bmHeap[0];

		// delete min
		int length = _bmHeap.size();
		_bmHeap[0] = _bmHeap[length-1];
		_bmHeap.pop_back();
		bubbleDown(0);

		return ret;
	}

	AStarPlannerNode* BinaryMinHeap::getMin(void) {
		return _bmHeap[0];
	}

	void BinaryMinHeap::deleteMin(void) {
		int length = _bmHeap.size();
		if (length == 0)
			return;
		_bmHeap[0] = _bmHeap[length-1];
		_bmHeap.pop_back();
		bubbleDown(0);
	}

	AStarPlannerNode* BinaryMinHeap::extractMinADA(void) {
		if (empty()) {
			std::cerr << "\nin extractMin, not supposed to be here!" << std::endl;
			return NULL;
		}
		KeyADA tmpkeys;
		tmpkeys.k1 = _bmHeap[0]->keys.k1;
		tmpkeys.k2 = _bmHeap[0]->keys.k2;
		int kindex = 0;

		for (int i=1; i<_bmHeap.size(); i++) {
			if (_bmHeap[i]->keys < tmpkeys) {
				tmpkeys.k1 = _bmHeap[i]->keys.k1;
				tmpkeys.k2 = _bmHeap[i]->keys.k2;
				kindex = i;
			}
		}
		AStarPlannerNode *nodep = _bmHeap[kindex];
		_bmHeap.erase(_bmHeap.begin() + kindex);

		return nodep;
	}

	void BinaryMinHeap::merge(Group group, std::vector<AStarPlannerNode *> &H) {
		for (int i=0; i<H.size(); i++) {
			H[i]->group = group;
			add(H[i]);
		}
	}

	bool BinaryMinHeap::exist(AStarPlannerNode &node) {
		for (int i=0; i<_bmHeap.size(); i++)
			if (*(_bmHeap[i]) == node)
				return true;

		return false;
	}

	void BinaryMinHeap::bubbleDown(int index) {
		int length = _bmHeap.size();
		int leftChild = 2*index + 1;
		int rightChild = 2*index + 2;

		if (leftChild >= length) // index is a leaf
			return;
		int minIndex = index;
		if (_bmHeap[index]->f > _bmHeap[leftChild]->f) {
			minIndex = leftChild;
		}
		if ((rightChild < length) && (_bmHeap[minIndex]->f > _bmHeap[rightChild]->f))
			minIndex = rightChild;
		if (minIndex != index) { // need a swap
			AStarPlannerNode *tmp = _bmHeap[index];
			_bmHeap[index] = _bmHeap[minIndex];
			_bmHeap[minIndex] = tmp;
			bubbleDown(minIndex);
		}
	}

	void BinaryMinHeap::bubbleUp(int index) {
		if (index == 0)
			return;
		int parentIndex = (index-1)/2;
		if (_bmHeap[parentIndex]->f > _bmHeap[index]->f) {
			AStarPlannerNode *tmp = _bmHeap[parentIndex];
			_bmHeap[parentIndex] = _bmHeap[index];
			_bmHeap[index] = tmp;
			bubbleUp(parentIndex);
		}
	}

	void BinaryMinHeap::decreaseKey(AStarPlannerNode *node) {
		for (int i=0; i<_bmHeap.size(); i++) {
			if (_bmHeap[i] == node) {
				bubbleUp(i);
				return;
			}
		}
	}
	// He Chen
}
