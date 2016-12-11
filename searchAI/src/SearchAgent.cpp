//
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "SteerLib.h"
#include "SearchAgent.h"
#include "SearchAIModule.h"
#include "util/Color.h"

/// @file SearchAgent.cpp
/// @brief Implements the SearchAgent class.

#define MAX_FORCE_MAGNITUDE 3.0f
#define MAX_SPEED 0.13f
#define AGENT_MASS 1.0f
#define GOAL_REGION 0.1f
#define DURATION 15

// He Chen
#define ACCELERATION 0.5
#define MAXSPEED 10
#define PREFERED_SPEED 10
#define GOAL_THRESHOLD_MULTIPLIER 2.5
#define PERSONAL_SPACE_THRESHOLD 0.3
#define QUERY_RADIUS 3.0f
#define BODY_FORCE 1500.0f
#define SLIDING_FRICTION_FORCE 3000.0f
// He Chen

SearchAgent::SearchAgent()
{
	_oldstate = false;
	_first = true;
	_lastime = 0;
	_enabled = false;
}

SearchAgent::~SearchAgent()
{
	if (_enabled) {
		Util::AxisAlignedBox bounds(__position.x-_radius, __position.x+_radius, 0.0f, 0.0f, __position.z-_radius, __position.z+_radius);
		gSpatialDatabase->removeObject( this, bounds);
	}
}

void SearchAgent::disable()
{
	Util::AxisAlignedBox bounds(__position.x-_radius, __position.x+_radius, 0.0f, 0.0f, __position.z-_radius, __position.z+_radius);
	gSpatialDatabase->removeObject( this, bounds);
	_enabled = false;
}

void SearchAgent::reset(const SteerLib::AgentInitialConditions & initialConditions, SteerLib::EngineInterface * engineInfo)
{
	// compute the "old" bounding box of the agent before it is reset.  its OK that it will be invalid if the agent was previously disabled
	// because the value is not used in that case.
	std::cout<<"Reset is called";
	Util::AxisAlignedBox oldBounds(__position.x-_radius, __position.x+_radius, 0.0f, 0.0f, __position.z-_radius, __position.z+_radius);

	// initialize the agent based on the initial conditions
	__position = initialConditions.position;
	_forward = initialConditions.direction;
	_radius = initialConditions.radius;
	_velocity = initialConditions.speed * Util::normalize(initialConditions.direction);

	// compute the "new" bounding box of the agent
	Util::AxisAlignedBox newBounds(__position.x-_radius, __position.x+_radius, 0.0f, 0.0f, __position.z-_radius, __position.z+_radius);

	if (!_enabled) {
		// if the agent was not enabled, then it does not already exist in the database, so add it.
		gSpatialDatabase->addObject( this, newBounds);
	}
	else {
		// if the agent was enabled, then the agent already existed in the database, so update it instead of adding it.
		gSpatialDatabase->updateObject( this, oldBounds, newBounds);
	}

	_enabled = true;

	if (initialConditions.goals.size() == 0) {
		throw Util::GenericException("No goals were specified!\n");
	}

	// iterate over the sequence of goals specified by the initial conditions.
	for (unsigned int i=0; i<initialConditions.goals.size(); i++) {
		if (initialConditions.goals[i].goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
			_goalQueue.push(initialConditions.goals[i]);
			if (initialConditions.goals[i].targetIsRandom) {
				// if the goal is random, we must randomly generate the goal.
				_goalQueue.back().targetLocation = gSpatialDatabase->randomPositionWithoutCollisions(1.0f, true);
			}
		}
		else {
			throw Util::GenericException("Unsupported goal type; SearchAgent only supports GOAL_TYPE_SEEK_STATIC_TARGET.");
		}
	}

	assert(_forward.length()!=0.0f);
	assert(_goalQueue.size() != 0);
	assert(_radius != 0.0f);

	// He Chen, used for weighted A star
	computePlan();
	// He Chen
}


void SearchAgent::computePlan()
{
	std::cout<<"\nComputing agent plan ";
	// He Chen
	if (_goalQueue.empty())
		std::cerr << "goal is empty!" << std::endl;
	else
		std::cerr << "goal is NOT empty!" << std::endl;
	// He Chen

	if (!_goalQueue.empty())
	{
		// He Chen
		SteerLib::AgentGoalInfo tmp;
		while (!_goalQueue.empty()) {
			tmp = _goalQueue.front();
			_goalQueue.pop();
		}
		_goalQueue.push(tmp);
		// He Chen

		Util::Point global_goal = _goalQueue.front().targetLocation;
		if (astar.computePath(__path, __position, _goalQueue.front().targetLocation, gSpatialDatabase))
		{

			while (!_goalQueue.empty())
				_goalQueue.pop();

			for (int i = 0; i < __path.size(); ++i)
			{
				SteerLib::AgentGoalInfo goal_path_pt;
				goal_path_pt.targetLocation = __path[i];
				_goalQueue.push(goal_path_pt);
			}
			SteerLib::AgentGoalInfo goal_path_pt;
			goal_path_pt.targetLocation = global_goal;
			_goalQueue.push(goal_path_pt);
		}
		// else
		// {
		// 	for(int i = 0;i<20;++i)
		// 		_goalQueue.push(_goalQueue.front());
		// }
	}


}

bool SearchAgent::mapchange() { // need to update the edge cost after change happens
	Util::Point obs(9.5, 0, 12.5);
	int id = gSpatialDatabase->getCellIndexFromLocation(obs);
	bool ret = astar.canBeTraversed(id);
	if (ret != _oldstate) {
		_oldstate = ret;
		return true;
	}
	return false;
}


void SearchAgent::updateAI(float timeStamp, float dt, unsigned int frameNumber)
{
	Util::AutomaticFunctionProfiler profileThisFunction( &SearchAIGlobals::gPhaseProfilers->aiProfiler );

	
	// He Chen -> ARA* and ADA*
/*
	double time_limit = 2;
	//std::cerr << "timeStamp: " << timeStamp << " dt: " << dt << " frameNumber: " << frameNumber << std::endl;
	if (timeStamp < time_limit) {
		computePlan();
		return;
	}

	if (mapchange()) {
		std::cerr << "map changed!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
		astar._epsilon = 20;
		_lastime = timeStamp;
	}

	if (timeStamp - _lastime < time_limit) {
		std::cerr << "compute the plan after map change!!" << std::endl;
		computePlan();
		return;
	}
*/
	// He Chen -> Social Forces
	//Util::AxisAlignedBox oldBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
	if (_first) {
		_first = false;
		_goalQueue.pop();
	}
	SteerLib::AgentGoalInfo goalInfo = _goalQueue.front();
	Util::Vector goalDirection;
	goalDirection = normalize(goalInfo.targetLocation - position());

	//std::cerr << "goalDirection: " << goalDirection.x << ", " << goalDirection.y << ", " << goalDirection.z << std::endl;
	//std::cerr << "intial velocity: " << _velocity.x << ", " << _velocity.y << ", " << _velocity.z << std::endl;
	Util::Vector prefForce = (((goalDirection * PREFERED_SPEED) - velocity()) / (ACCELERATION/dt));
	//std::cerr << "prefForce: " << prefForce.x << ", " << prefForce.y << ", " << prefForce.z << std::endl;
	prefForce = prefForce + velocity();
	Util::Vector repulsionForce = calcRepulsionForce(dt);
	//Util::Vector proximityForce = calcProximityForce(dt);
	//_velocity = (prefForce) + repulsionForce + proximityForce;
	_velocity = prefForce + repulsionForce;

	_velocity = clamp(velocity(), MAXSPEED);
	_velocity.y = 0.0f;
	//std::cerr << "size of goalqueue: " << _goalQueue.size() << std::endl;
	//std::cerr << "velocity is: " << _velocity.x << ", " << _velocity.y << ", " << _velocity.z << std::endl;

	__position = position() + (velocity() * dt);
	//std::cerr << "position is: " << __position.x << ", " << __position.y << ", " << __position.z << std::endl;
	//Util::AxisAlignedBox newBounds(_position.x - _radius, _position.x + _radius, 0.0f, 0.0f, _position.z - _radius, _position.z + _radius);
        //getSimulationEngine()->getSpatialDatabase()->updateObject( this, oldBounds, newBounds);
	//gSpatialDatabase->updateObject(this, oldBounds, newBounds);

	if ((goalInfo.targetLocation - position()).length() < radius()*GOAL_THRESHOLD_MULTIPLIER)
        {
                _goalQueue.pop();
                // std::cout << "Made it to a goal" << std::endl;
                if (_goalQueue.size() != 0)
                {
                        // in this case, there are still more goals, so start steering to the next goal.
                        goalDirection = _goalQueue.front().targetLocation - __position;
                        _prefVelocity = Util::Vector(goalDirection.x, 0.0f, goalDirection.z);
                }
                else
                {
                        // in this case, there are no more goals, so disable the agent and remove it from the spatial database.
                        disable();
                        return;
                }
        }

        // Hear the 2D solution from RVO is converted into the 3D used by SteerSuite
        // _velocity = Vector(velocity().x, 0.0f, velocity().z);
        if ( velocity().lengthSquared() > 0.0 )
        {
                // Only assign forward direction if agent is moving
                // Otherwise keep last forward
                _forward = normalize(_velocity);
        }
	return;
	// He Chen
	double steps = (DURATION/(double)__path.size());
	if(timeStamp*dt > last_waypoint*steps)
	{	
		if(!_goalQueue.empty())
		{
			__position = _goalQueue.front().targetLocation;
			std::cout<<"Waypoint: "<< __position;
			_goalQueue.pop();
			last_waypoint++;
		}
	}
}

Util::Vector SearchAgent::calcRepulsionForce(float dt)
{
#ifdef _DEBUG_
        std::cout << "wall repulsion; " << calcWallRepulsionForce(dt) << " agent repulsion " <<
                        (_SocialForcesParams.sf_agent_repulsion_importance * calcAgentRepulsionForce(dt)) << std::endl;
#endif
        return calcWallRepulsionForce(dt);
}

Util::Vector SearchAgent::calcWallRepulsionForce(float dt)
{

        Util::Vector wall_repulsion_force = Util::Vector(0,0,0);


        std::set<SteerLib::SpatialDatabaseItemPtr> _neighbors;
                gSpatialDatabase->getItemsInRange(_neighbors,
                                _position.x-(this->_radius + QUERY_RADIUS),
                                _position.x+(this->_radius + QUERY_RADIUS),
                                _position.z-(this->_radius + QUERY_RADIUS),
                                _position.z+(this->_radius + QUERY_RADIUS),
                                dynamic_cast<SteerLib::SpatialDatabaseItemPtr>(this));

        SteerLib::ObstacleInterface * tmp_ob;

        for (std::set<SteerLib::SpatialDatabaseItemPtr>::iterator neighbour = _neighbors.begin();  neighbour != _neighbors.end();  neighbour++)
        // for (std::set<SteerLib::ObstacleInterface * >::iterator tmp_o = _neighbors.begin();  tmp_o != _neighbors.end();  tmp_o++)
        {
                if ( !(*neighbour)->isAgent() )
                {
                        tmp_ob = dynamic_cast<SteerLib::ObstacleInterface *>(*neighbour);
                }
                else
                {
                        continue;
                }
                if ( tmp_ob->computePenetration(this->position(), this->radius()) > 0.000001 )
                {
                        SteerLib::CircleObstacle * cir_obs = dynamic_cast<SteerLib::CircleObstacle *>(tmp_ob);
                        if ( cir_obs != NULL)
                        {
                                // std::cout << "Intersected circle obstacle" << std::endl;
                                Util::Vector wall_normal = position() - cir_obs->position();
                                // wall distance
                                float distance = wall_normal.length() - cir_obs->radius();

                                wall_normal = normalize(wall_normal);
                                wall_repulsion_force = wall_repulsion_force +
                                        ((
                                                (
                                                        (
                                                                        wall_normal
                                                        )
                                                        *
                                                        (
                                                                radius() +
                                                                PERSONAL_SPACE_THRESHOLD -
                                                                (
                                                                        distance
                                                                )
                                                        )
                                                )
                                                /
                                                distance
                                        )* BODY_FORCE * dt);

                                // tangential force
                                // std::cout << "wall tangent " << rightSideInXZPlane(wall_normal) <<
                                        //      " dot is " << dot(forward(),  rightSideInXZPlane(wall_normal)) <<
                                                // std::endl;
                                wall_repulsion_force = wall_repulsion_force +
                                (
                                        dot(forward(),  rightSideInXZPlane(wall_normal))
                                        *
                                        rightSideInXZPlane(wall_normal)
                                        *
                                        cir_obs->computePenetration(this->position(), this->radius())
                                )* SLIDING_FRICTION_FORCE * dt;

                        }
                        else
                        {
                                Util::Vector wall_normal = calcWallNormal( tmp_ob );
                                std::pair<Util::Point,Util::Point> line = calcWallPointsFromNormal(tmp_ob, wall_normal);
                                // Util::Point midpoint = Util::Point((line.first.x+line.second.x)/2, ((line.first.y+line.second.y)/2)+1,
                                        //      (line.first.z+line.second.z)/2);
                                std::pair<float, Util::Point> min_stuff = minimum_distance(line.first, line.second, position());
                                // wall distance
                                wall_repulsion_force = wall_repulsion_force +
                                        ((
                                                (
                                                        (
                                                                        wall_normal
                                                        )
                                                        *
                                                        (
                                                                radius() +
                                                                PERSONAL_SPACE_THRESHOLD -
                                                                (
                                                                        min_stuff.first
                                                                )
                                                        )
                                                )
                                                /
                                                min_stuff.first
                                        )* BODY_FORCE * dt);
                                // tangential force
                                // std::cout << "wall tangent " << rightSideInXZPlane(wall_normal) <<
                                        //      " dot is " << dot(forward(),  rightSideInXZPlane(wall_normal)) <<
                                                // std::endl;
                                wall_repulsion_force = wall_repulsion_force +
                                (
                                        dot(forward(),  rightSideInXZPlane(wall_normal))
                                        *
                                        rightSideInXZPlane(wall_normal)
                                        *
                                        tmp_ob->computePenetration(this->position(), this->radius())
                                )* SLIDING_FRICTION_FORCE * dt;
                        }
                }

        }
        return wall_repulsion_force;
}

std::pair<float, Util::Point> SearchAgent::minimum_distance(Util::Point l1, Util::Point l2, Util::Point p)
{
  // Return minimum distance between line segment vw and point p
  float lSq = (l1 - l2).lengthSquared();  // i.e. |l2-l1|^2 -  avoid a sqrt
  if (lSq == 0.0)
          return std::make_pair((p - l2).length(),l1 );   // l1 == l2 case
  // Consider the line extending the segment, parameterized as l1 + t (l2 - l1).
  // We find projection of point p onto the line.
  // It falls where t = [(p-l1) . (l2-l1)] / |l2-l1|^2
  const float t = dot(p - l1, l2 - l1) / lSq;
  if (t < 0.0)
  {
          return std::make_pair((p - l1).length(), l1);       // Beyond the 'l1' end of the segment
  }
  else if (t > 1.0)
  {
          return std::make_pair((p - l2).length(), l2);  // Beyond the 'l2' end of the segment
  }
  const Util::Point projection = l1 + t * (l2 - l1);  // Projection falls on the segment
  return std::make_pair((p - projection).length(), projection) ;
}

std::pair<Util::Point, Util::Point> SearchAgent::calcWallPointsFromNormal(SteerLib::ObstacleInterface* obs, Util::Vector normal)
{
        Util::AxisAlignedBox box = obs->getBounds();
        if ( normal.z == 1)
        {
                return std::make_pair(Util::Point(box.xmin,0,box.zmax), Util::Point(box.xmax,0,box.zmax));
                // Ended here;
        }
        else if ( normal.z == -1 )
        {
                return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmax,0,box.zmin));
        }
        else if ( normal.x == 1)
        {
                return std::make_pair(Util::Point(box.xmax,0,box.zmin), Util::Point(box.xmax,0,box.zmax));
        }
        else // normal.x == -1
        {
                return std::make_pair(Util::Point(box.xmin,0,box.zmin), Util::Point(box.xmin,0,box.zmax));
        }
}

Util::Vector SearchAgent::calcWallNormal(SteerLib::ObstacleInterface* obs)
{
        Util::AxisAlignedBox box = obs->getBounds();
        if ( position().x > box.xmax )
        {
                if ( position().z > box.zmax)
                {
                        if (fabs(position().z - box.zmax) >
                                fabs(position().x - box.xmax))
                        {
                                return Util::Vector(0, 0, 1);
                        }
                        else
                        {
                                return Util::Vector(1, 0, 0);
                        }

                }
                else if ( position().z < box.zmin )
                {
                        if (fabs(position().z - box.zmin) >
                                fabs(position().x - box.xmax))
                        {
                                return Util::Vector(0, 0, -1);
                        }
                        else
                        {
                                return Util::Vector(1, 0, 0);
                        }

                }
                else
                { // in between zmin and zmax
                        return Util::Vector(1, 0, 0);
                }

        }
        else if ( position().x < box.xmin )
        {
                if ( position().z > box.zmax )
                {
                        if (fabs(position().z - box.zmax) >
                                fabs(position().x - box.xmin))
                        {
                                return Util::Vector(0, 0, 1);
                        }
                        else
                        {
                                return Util::Vector(-1, 0, 0);
                        }

                }
                else if ( position().z < box.zmin )
                {
                        if (fabs(position().z - box.zmin) >
                                fabs(position().x - box.xmin))
                        {
                                return Util::Vector(0, 0, -1);
                        }
                        else
                        {
                                return Util::Vector(-1, 0, 0);
                        }

                }
                else
                { // in between zmin and zmax
                        return Util::Vector(-1, 0, 0);
                }
        }
        else // between xmin and xmax
        {
                if ( position().z > box.zmax )
                {
                        return Util::Vector(0, 0, 1);
                }
                else if ( position().z < box.zmin)
                {
                        return Util::Vector(0, 0, -1);
                }
                else
                { // What do we do if the agent is inside the wall?? Lazy Normal
                        return calcWallNormal( obs );
                }
        }

}


void SearchAgent::draw()
{
#ifdef ENABLE_GUI
	// if the agent is selected, do some annotations just for demonstration
	if (gEngine->isAgentSelected(this)) {
		Util::Ray ray;
		ray.initWithUnitInterval(__position, _forward);
		float t = 0.0f;
		SteerLib::SpatialDatabaseItem * objectFound;
		Util::DrawLib::drawLine(ray.pos, ray.eval(1.0f));
		if (gSpatialDatabase->trace(ray, t, objectFound, this, false)) {
			Util::DrawLib::drawAgentDisc(__position, _forward, _radius, Util::gBlue);
		}
		else {
			Util::DrawLib::drawAgentDisc(__position, _forward, _radius);
		}
	}
	else {
		Util::DrawLib::drawAgentDisc(__position, _forward, _radius, Util::gGray40);
	}
	if (_goalQueue.front().goalType == SteerLib::GOAL_TYPE_SEEK_STATIC_TARGET) {
		Util::DrawLib::drawFlag(_goalQueue.front().targetLocation);
	}
	
	if(__path.size()>0)
	{
		for(int i = 1; i<__path.size(); ++i)
			Util::DrawLib::drawLine(__path[i-1], __path[i], Util::Color(1.0f, 0.0f, 0.0f), 2);
		Util::DrawLib::drawCircle(__path[__path.size()-1], Util::Color(0.0f, 1.0f, 0.0f));
	}
#endif
}
