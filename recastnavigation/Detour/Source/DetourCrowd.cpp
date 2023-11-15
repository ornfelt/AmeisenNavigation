//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

/**
 * @file DetourCrowd.cpp
 *
 * @brief Contains the implementation of the dtCrowd class and related functions.
 */

#define _USE_MATH_DEFINES
#include <string.h>
#include <float.h>
#include <stdlib.h>
#include <new>
#include "DetourCrowd.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourObstacleAvoidance.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAssert.h"
#include "DetourAlloc.h"

 /**
  * @brief Allocates memory for a new dtCrowd instance.
  *
  * @return A pointer to the allocated dtCrowd instance.
  * @see dtFreeCrowd()
  */
dtCrowd* dtAllocCrowd()
{
	void* mem = dtAlloc(sizeof(dtCrowd), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtCrowd;
}

/**
 * @brief Frees memory allocated for a dtCrowd instance.
 *
 * @param ptr A pointer to the dtCrowd instance to be freed.
 * @see dtAllocCrowd()
 */
void dtFreeCrowd(dtCrowd* ptr)
{
	if (!ptr) return;
	ptr->~dtCrowd();
	dtFree(ptr);
}

/** Maximum allowed iterations per update. */
static const int MAX_ITERS_PER_UPDATE = 100;

/** Maximum number of nodes in the path queue. */
static const int MAX_PATHQUEUE_NODES = 4096;

/** Maximum number of common nodes. */
static const int MAX_COMMON_NODES = 512;

/**
 * @brief Interpolates a value between two given values.
 *
 * @param t The interpolation parameter, typically in the range [t0, t1].
 * @param t0 The start value.
 * @param t1 The end value.
 * @return The interpolated value.
 */
inline float tween(const float t, const float t0, const float t1)
{
	return dtClamp((t - t0) / (t1 - t0), 0.0f, 1.0f);
}

/**
 * @brief Integrates agent's movement over a small time step.
 *
 * This function integrates the agent's position based on its velocity and acceleration
 * constraints. It ensures that the agent's velocity does not exceed the maximum allowed
 * acceleration.
 *
 * @param ag A pointer to the dtCrowdAgent to be integrated.
 * @param dt The time step for integration.
 */
static void integrate(dtCrowdAgent* ag, const float dt)
{
	// Fake dynamic constraint.
	const float maxDelta = ag->params.maxAcceleration * dt;
	float dv[3];
	dtVsub(dv, ag->nvel, ag->vel);
	float ds = dtVlen(dv);
	if (ds > maxDelta)
		dtVscale(dv, dv, maxDelta / ds);
	dtVadd(ag->vel, ag->vel, dv);

	// Integrate
	if (dtVlen(ag->vel) > 0.0001f)
		dtVmad(ag->npos, ag->npos, ag->vel, dt);
	else
		dtVset(ag->vel, 0, 0, 0);
}

/**
 * @brief Checks if the agent is over an off-mesh connection within a certain radius.
 *
 * @param ag The dtCrowdAgent to check.
 * @param radius The radius within which to consider the agent over an off-mesh connection.
 * @return True if the agent is over an off-mesh connection, false otherwise.
 */
static bool overOffmeshConnection(const dtCrowdAgent* ag, const float radius)
{
	if (!ag->ncorners)
		return false;

	const bool offMeshConnection = (ag->cornerFlags[ag->ncorners - 1] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ? true : false;
	if (offMeshConnection)
	{
		const float distSq = dtVdist2DSqr(ag->npos, &ag->cornerVerts[(ag->ncorners - 1) * 3]);
		if (distSq < radius * radius)
			return true;
	}

	return false;
}

/**
 * @brief Calculates the distance to the agent's goal within a specified range.
 *
 * @param ag The dtCrowdAgent for which to calculate the distance to the goal.
 * @param range The maximum range within which to consider the goal reached.
 * @return The distance to the goal within the specified range.
 */
static float getDistanceToGoal(const dtCrowdAgent* ag, const float range)
{
	if (!ag->ncorners)
		return range;

	const bool endOfPath = (ag->cornerFlags[ag->ncorners - 1] & DT_STRAIGHTPATH_END) ? true : false;
	if (endOfPath)
		return dtMin(dtVdist2D(ag->npos, &ag->cornerVerts[(ag->ncorners - 1) * 3]), range);

	return range;
}

/**
 * @brief Calculates a smooth steering direction for the agent.
 *
 * @param ag The dtCrowdAgent for which to calculate the steering direction.
 * @param dir The calculated smooth steering direction (output).
 */
static void calcSmoothSteerDirection(const dtCrowdAgent* ag, float* dir)
{
	if (!ag->ncorners)
	{
		dtVset(dir, 0, 0, 0);
		return;
	}

	const int ip0 = 0;
	const int ip1 = dtMin(1, ag->ncorners - 1);
	const float* p0 = &ag->cornerVerts[ip0 * 3];
	const float* p1 = &ag->cornerVerts[ip1 * 3];

	float dir0[3], dir1[3];
	dtVsub(dir0, p0, ag->npos);
	dtVsub(dir1, p1, ag->npos);
	dir0[1] = 0;
	dir1[1] = 0;

	float len0 = dtVlen(dir0);
	float len1 = dtVlen(dir1);
	if (len1 > 0.001f)
		dtVscale(dir1, dir1, 1.0f / len1);

	dir[0] = dir0[0] - dir1[0] * len0 * 0.5f;
	dir[1] = 0;
	dir[2] = dir0[2] - dir1[2] * len0 * 0.5f;

	dtVnormalize(dir);
}

/**
 * @brief Calculates a straight steering direction for the agent.
 *
 * @param ag The dtCrowdAgent for which to calculate the steering direction.
 * @param dir The calculated straight steering direction (output).
 */
static void calcStraightSteerDirection(const dtCrowdAgent* ag, float* dir)
{
	if (!ag->ncorners)
	{
		dtVset(dir, 0, 0, 0);
		return;
	}
	dtVsub(dir, &ag->cornerVerts[0], ag->npos);
	dir[1] = 0;
	dtVnormalize(dir);
}

/**
 * @brief Adds a neighbor to the list of neighboring agents within a specified range.
 *
 * This function inserts a neighbor based on the distance. It maintains the list of neighbors in
 * increasing order of distance.
 *
 * @param idx The index of the neighbor agent.
 * @param dist The distance from the current agent to the neighbor.
 * @param neis The array of dtCrowdNeighbour structures representing neighbors.
 * @param nneis The current number of neighbors in the array.
 * @param maxNeis The maximum number of neighbors allowed in the array.
 * @return The updated number of neighbors in the array after adding the new neighbor.
 */
static int addNeighbour(const int idx, const float dist,
	dtCrowdNeighbour* neis, const int nneis, const int maxNeis)
{
	// Insert neighbour based on the distance.
	dtCrowdNeighbour* nei = 0;
	if (!nneis)
	{
		nei = &neis[nneis];
	}
	else if (dist >= neis[nneis - 1].dist)
	{
		if (nneis >= maxNeis)
			return nneis;
		nei = &neis[nneis];
	}
	else
	{
		int i;
		for (i = 0; i < nneis; ++i)
			if (dist <= neis[i].dist)
				break;

		const int tgt = i + 1;
		const int n = dtMin(nneis - i, maxNeis - tgt);

		dtAssert(tgt + n <= maxNeis);

		if (n > 0)
			memmove(&neis[tgt], &neis[i], sizeof(dtCrowdNeighbour) * n);
		nei = &neis[i];
	}

	memset(nei, 0, sizeof(dtCrowdNeighbour));

	nei->idx = idx;
	nei->dist = dist;

	return dtMin(nneis + 1, maxNeis);
}

/**
 * @brief Gets neighboring agents within a specified range and height.
 *
 * This function retrieves neighboring agents within a specified range and height from the
 * proximity grid. It skips a specific agent if provided.
 *
 * @param pos The position of the current agent.
 * @param height The height of the current agent.
 * @param range The maximum range within which to consider neighboring agents.
 * @param skip The dtCrowdAgent to skip when checking for neighbors (can be NULL).
 * @param result The array of dtCrowdNeighbour structures representing neighboring agents (output).
 * @param maxResult The maximum number of neighbors to retrieve.
 * @param agents An array of dtCrowdAgent pointers representing all agents in the crowd.
 * @param grid The dtProximityGrid used for querying neighboring agents.
 * @return The number of neighboring agents found.
 */
static int getNeighbours(const float* pos, const float height, const float range,
	const dtCrowdAgent* skip, dtCrowdNeighbour* result, const int maxResult,
	dtCrowdAgent** agents, const int /*nagents*/, dtProximityGrid* grid)
{
	int n = 0;

	static const int MAX_NEIS = 32;
	unsigned short ids[MAX_NEIS];
	int nids = grid->queryItems(pos[0] - range, pos[2] - range,
		pos[0] + range, pos[2] + range,
		ids, MAX_NEIS);

	for (int i = 0; i < nids; ++i)
	{
		const dtCrowdAgent* ag = agents[ids[i]];

		if (ag == skip) continue;

		// Check for overlap.
		float diff[3];
		dtVsub(diff, pos, ag->npos);
		if (dtMathFabsf(diff[1]) >= (height + ag->params.height) / 2.0f)
			continue;
		diff[1] = 0;
		const float distSqr = dtVlenSqr(diff);
		if (distSqr > dtSqr(range))
			continue;

		n = addNeighbour(ids[i], distSqr, result, n, maxResult);
	}
	return n;
}

/**
 * @brief Adds a crowd agent to the optimization queue based on topology optimization time.
 *
 * This function inserts a crowd agent into the optimization queue based on the greatest
 * topology optimization time. It maintains the queue in decreasing order of topology optimization time.
 *
 * @param newag The crowd agent to be added to the optimization queue.
 * @param agents An array of dtCrowdAgent pointers representing all crowd agents.
 * @param nagents The current number of agents in the optimization queue.
 * @param maxAgents The maximum number of agents allowed in the optimization queue.
 * @return The updated number of agents in the optimization queue after adding the new agent.
 */
static int addToOptQueue(dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents)
{
	// Insert neighbour based on greatest time.
	int slot = 0;
	if (!nagents)
	{
		slot = nagents;
	}
	else if (newag->topologyOptTime <= agents[nagents - 1]->topologyOptTime)
	{
		if (nagents >= maxAgents)
			return nagents;
		slot = nagents;
	}
	else
	{
		int i;
		for (i = 0; i < nagents; ++i)
			if (newag->topologyOptTime >= agents[i]->topologyOptTime)
				break;

		const int tgt = i + 1;
		const int n = dtMin(nagents - i, maxAgents - tgt);

		dtAssert(tgt + n <= maxAgents);

		if (n > 0)
			memmove(&agents[tgt], &agents[i], sizeof(dtCrowdAgent*) * n);
		slot = i;
	}

	agents[slot] = newag;

	return dtMin(nagents + 1, maxAgents);
}

/**
 * @brief Adds a crowd agent to the path replanning queue based on target replanning time.
 *
 * This function inserts a crowd agent into the path replanning queue based on the greatest
 * target replanning time. It maintains the queue in decreasing order of target replanning time.
 *
 * @param newag The crowd agent to be added to the path replanning queue.
 * @param agents An array of dtCrowdAgent pointers representing all crowd agents.
 * @param nagents The current number of agents in the path replanning queue.
 * @param maxAgents The maximum number of agents allowed in the path replanning queue.
 * @return The updated number of agents in the path replanning queue after adding the new agent.
 */
static int addToPathQueue(dtCrowdAgent* newag, dtCrowdAgent** agents, const int nagents, const int maxAgents)
{
	// Insert neighbour based on greatest time.
	int slot = 0;
	if (!nagents)
	{
		slot = nagents;
	}
	else if (newag->targetReplanTime <= agents[nagents - 1]->targetReplanTime)
	{
		if (nagents >= maxAgents)
			return nagents;
		slot = nagents;
	}
	else
	{
		int i;
		for (i = 0; i < nagents; ++i)
			if (newag->targetReplanTime >= agents[i]->targetReplanTime)
				break;

		const int tgt = i + 1;
		const int n = dtMin(nagents - i, maxAgents - tgt);

		dtAssert(tgt + n <= maxAgents);

		if (n > 0)
			memmove(&agents[tgt], &agents[i], sizeof(dtCrowdAgent*) * n);
		slot = i;
	}

	agents[slot] = newag;

	return dtMin(nagents + 1, maxAgents);
}

/**
@class dtCrowd
@par

This is the core class of the @ref crowd module.  See the @ref crowd documentation for a summary
of the crowd features.

A common method for setting up the crowd is as follows:

-# Allocate the crowd using #dtAllocCrowd.
-# Initialize the crowd using #init().
-# Set the avoidance configurations using #setObstacleAvoidanceParams().
-# Add agents using #addAgent() and make an initial movement request using #requestMoveTarget().

A common process for managing the crowd is as follows:

-# Call #update() to allow the crowd to manage its agents.
-# Retrieve agent information using #getActiveAgents().
-# Make movement requests using #requestMoveTarget() when movement goal changes.
-# Repeat every frame.

Some agent configuration settings can be updated using #updateAgentParameters().  But the crowd owns the
agent position.  So it is not possible to update an active agent's position.  If agent position
must be fed back into the crowd, the agent must be removed and re-added.

Notes:

- Path related information is available for newly added agents only after an #update() has been
  performed.
- Agent objects are kept in a pool and re-used.  So it is important when using agent objects to check the value of
  #dtCrowdAgent::active to determine if the agent is actually in use or not.
- This class is meant to provide 'local' movement. There is a limit of 256 polygons in the path corridor.
  So it is not meant to provide automatic pathfinding services over long distances.

@see dtAllocCrowd(), dtFreeCrowd(), init(), dtCrowdAgent
*/
dtCrowd::dtCrowd() :
	m_maxAgents(0),
	m_agents(0),
	m_activeAgents(0),
	m_agentAnims(0),
	m_obstacleQuery(0),
	m_grid(0),
	m_pathResult(0),
	m_maxPathResult(0),
	m_maxAgentRadius(0),
	m_velocitySampleCount(0),
	m_navquery(0)
{
}

/**
 * @brief Destructor for the crowd object.
 *
 * This function purges all allocated resources and deallocates memory used by the crowd object.
 */
dtCrowd::~dtCrowd()
{
	purge();
}

/**
 * @brief Purges all resources and reinitializes the crowd.
 *
 * This function frees all resources associated with the crowd and reinitializes it to its default state.
 */
void dtCrowd::purge()
{
	for (int i = 0; i < m_maxAgents; ++i)
		m_agents[i].~dtCrowdAgent();
	dtFree(m_agents);
	m_agents = 0;
	m_maxAgents = 0;

	dtFree(m_activeAgents);
	m_activeAgents = 0;

	dtFree(m_agentAnims);
	m_agentAnims = 0;

	dtFree(m_pathResult);
	m_pathResult = 0;

	dtFreeProximityGrid(m_grid);
	m_grid = 0;

	dtFreeObstacleAvoidanceQuery(m_obstacleQuery);
	m_obstacleQuery = 0;

	dtFreeNavMeshQuery(m_navquery);
	m_navquery = 0;
}

/**
 * @brief Initializes the crowd with specified parameters.
 *
 * This function initializes the crowd with the specified maximum number of agents, maximum agent radius, and navigation mesh.
 * It also allocates and initializes various internal data structures required for crowd management.
 *
 * @param maxAgents The maximum number of agents that the crowd can handle.
 * @param maxAgentRadius The maximum radius of an agent in the crowd.
 * @param nav A pointer to the navigation mesh that the crowd agents will navigate on.
 * @return True if initialization is successful, false otherwise.
 *
 * @note This function may be called more than once to reinitialize the crowd.
 */
bool dtCrowd::init(const int maxAgents, const float maxAgentRadius, dtNavMesh* nav)
{
	purge();

	m_maxAgents = maxAgents;
	m_maxAgentRadius = maxAgentRadius;

	// Larger than agent radius because it is also used for agent recovery.
	dtVset(m_agentPlacementHalfExtents, m_maxAgentRadius * 2.0f, m_maxAgentRadius * 1.5f, m_maxAgentRadius * 2.0f);

	m_grid = dtAllocProximityGrid();
	if (!m_grid)
		return false;
	if (!m_grid->init(m_maxAgents * 4, maxAgentRadius * 3))
		return false;

	m_obstacleQuery = dtAllocObstacleAvoidanceQuery();
	if (!m_obstacleQuery)
		return false;
	if (!m_obstacleQuery->init(6, 8))
		return false;

	// Init obstacle query params.
	memset(m_obstacleQueryParams, 0, sizeof(m_obstacleQueryParams));
	for (int i = 0; i < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS; ++i)
	{
		dtObstacleAvoidanceParams* params = &m_obstacleQueryParams[i];
		params->velBias = 0.4f;
		params->weightDesVel = 2.0f;
		params->weightCurVel = 0.75f;
		params->weightSide = 0.75f;
		params->weightToi = 2.5f;
		params->horizTime = 2.5f;
		params->gridSize = 33;
		params->adaptiveDivs = 7;
		params->adaptiveRings = 2;
		params->adaptiveDepth = 5;
	}

	// Allocate temp buffer for merging paths.
	m_maxPathResult = 256;
	m_pathResult = (dtPolyRef*)dtAlloc(sizeof(dtPolyRef) * m_maxPathResult, DT_ALLOC_PERM);
	if (!m_pathResult)
		return false;

	if (!m_pathq.init(m_maxPathResult, MAX_PATHQUEUE_NODES, nav))
		return false;

	m_agents = (dtCrowdAgent*)dtAlloc(sizeof(dtCrowdAgent) * m_maxAgents, DT_ALLOC_PERM);
	if (!m_agents)
		return false;

	m_activeAgents = (dtCrowdAgent**)dtAlloc(sizeof(dtCrowdAgent*) * m_maxAgents, DT_ALLOC_PERM);
	if (!m_activeAgents)
		return false;

	m_agentAnims = (dtCrowdAgentAnimation*)dtAlloc(sizeof(dtCrowdAgentAnimation) * m_maxAgents, DT_ALLOC_PERM);
	if (!m_agentAnims)
		return false;

	for (int i = 0; i < m_maxAgents; ++i)
	{
		new(&m_agents[i]) dtCrowdAgent();
		m_agents[i].active = false;
		if (!m_agents[i].corridor.init(m_maxPathResult))
			return false;
	}

	for (int i = 0; i < m_maxAgents; ++i)
	{
		m_agentAnims[i].active = false;
	}

	// The navquery is mostly used for local searches, no need for large node pool.
	m_navquery = dtAllocNavMeshQuery();
	if (!m_navquery)
		return false;
	if (dtStatusFailed(m_navquery->init(nav, MAX_COMMON_NODES)))
		return false;

	return true;
}

/**
 * @brief Sets obstacle avoidance parameters for a specific index.
 *
 * This function sets the obstacle avoidance parameters for a specific index.
 *
 * @param idx The index of the obstacle avoidance parameters to set.
 * @param params A pointer to the obstacle avoidance parameters to set.
 */
void dtCrowd::setObstacleAvoidanceParams(const int idx, const dtObstacleAvoidanceParams* params)
{
	if (idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
		memcpy(&m_obstacleQueryParams[idx], params, sizeof(dtObstacleAvoidanceParams));
}

/**
 * @brief Gets the obstacle avoidance parameters for a specific index.
 *
 * This function retrieves the obstacle avoidance parameters for a specific index.
 *
 * @param idx The index of the obstacle avoidance parameters to retrieve.
 * @return A pointer to the obstacle avoidance parameters if the index is valid, or nullptr otherwise.
 */
const dtObstacleAvoidanceParams* dtCrowd::getObstacleAvoidanceParams(const int idx) const
{
	if (idx >= 0 && idx < DT_CROWD_MAX_OBSTAVOIDANCE_PARAMS)
		return &m_obstacleQueryParams[idx];
	return 0;
}

/**
 * @brief Gets the total number of agents in the crowd.
 *
 * This function returns the maximum number of agents that the crowd can handle.
 *
 * @return The maximum number of agents in the crowd.
 */
int dtCrowd::getAgentCount() const
{
	return m_maxAgents;
}

/**
 * @brief Gets a const pointer to a crowd agent by index.
 *
 * This function retrieves a const pointer to a crowd agent by its index.
 *
 * @param idx The index of the agent to retrieve.
 * @return A const pointer to the crowd agent if the index is valid, or nullptr otherwise.
 *
 * @note Agents in the pool may not be in use. Check dtCrowdAgent::active before using the returned object.
 */
const dtCrowdAgent* dtCrowd::getAgent(const int idx)
{
	if (idx < 0 || idx >= m_maxAgents)
		return 0;
	return &m_agents[idx];
}

/**
 * @brief Gets a mutable pointer to a crowd agent by index.
 *
 * This function retrieves a mutable pointer to a crowd agent by its index, allowing you to modify agent parameters.
 *
 * @param idx The index of the agent to retrieve.
 * @return A mutable pointer to the crowd agent if the index is valid, or nullptr otherwise.
 *
 * @note Agents in the pool may not be in use. Check dtCrowdAgent::active before using the returned object.
 */
dtCrowdAgent* dtCrowd::getEditableAgent(const int idx)
{
	if (idx < 0 || idx >= m_maxAgents)
		return 0;
	return &m_agents[idx];
}

/**
 * @brief Updates the parameters of a crowd agent.
 *
 * This function updates the parameters of a crowd agent specified by its index.
 *
 * @param idx The index of the agent to update.
 * @param params A pointer to the crowd agent parameters to set.
 */
void dtCrowd::updateAgentParameters(const int idx, const dtCrowdAgentParams* params)
{
	if (idx < 0 || idx >= m_maxAgents)
		return;
	memcpy(&m_agents[idx].params, params, sizeof(dtCrowdAgentParams));
}

/**
 * @brief Adds a new agent to the crowd at the specified position.
 *
 * This function adds a new agent to the crowd at the specified position. The agent's position will be constrained to the surface of the navigation mesh.
 *
 * @param pos The initial position of the agent.
 * @param params A pointer to the crowd agent parameters for the new agent.
 * @return The index of the added agent if successful, or -1 if no empty slot is available.
 *
 * @note Agents in the pool may not be in use. Check dtCrowdAgent::active before using the returned object.
 */
int dtCrowd::addAgent(const float* pos, const dtCrowdAgentParams* params)
{
	// Find empty slot.
	int idx = -1;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active)
		{
			idx = i;
			break;
		}
	}
	if (idx == -1)
		return -1;

	dtCrowdAgent* ag = &m_agents[idx];

	updateAgentParameters(idx, params);

	// Find nearest position on navmesh and place the agent there.
	float nearest[3];
	dtPolyRef ref = 0;
	dtVcopy(nearest, pos);
	dtStatus status = m_navquery->findNearestPoly(pos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ref, nearest);
	if (dtStatusFailed(status))
	{
		dtVcopy(nearest, pos);
		ref = 0;
	}

	ag->corridor.reset(ref, nearest);
	ag->boundary.reset();
	ag->partial = false;

	ag->topologyOptTime = 0;
	ag->targetReplanTime = 0;
	ag->nneis = 0;

	dtVset(ag->dvel, 0, 0, 0);
	dtVset(ag->nvel, 0, 0, 0);
	dtVset(ag->vel, 0, 0, 0);
	dtVcopy(ag->npos, nearest);

	ag->desiredSpeed = 0;

	if (ref)
		ag->state = DT_CROWDAGENT_STATE_WALKING;
	else
		ag->state = DT_CROWDAGENT_STATE_INVALID;

	ag->targetState = DT_CROWDAGENT_TARGET_NONE;

	ag->active = true;

	return idx;
}

/**
 * @brief Deactivates an agent, removing it from processing.
 *
 * This function deactivates an agent specified by its index. The agent is marked as inactive and will no longer be processed. The agent's dtCrowdAgent object is not removed from the pool and can be reactivated for reuse.
 *
 * @param idx The index of the agent to deactivate.
 */
void dtCrowd::removeAgent(const int idx)
{
	if (idx >= 0 && idx < m_maxAgents)
	{
		m_agents[idx].active = false;
	}
}

/**
 * @brief Requests replanning of an agent's move target.
 *
 * This function requests replanning of an agent's move target. It is used when an agent needs to replan its path to a new target location.
 *
 * @param idx The index of the agent.
 * @param ref The new target's polygon reference ID.
 * @param pos The new target position in world coordinates.
 * @return True if the request was successfully processed, false otherwise.
 *
 * @note The position will be constrained to the surface of the navigation mesh.
 * @note The request will be processed during the next update.
 */
bool dtCrowd::requestMoveTargetReplan(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	// Initialize request.
	ag->targetRef = ref;
	dtVcopy(ag->targetPos, pos);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = true;
	if (ag->targetRef)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;

	return true;
}

/**
 * @brief Requests a move target for an agent.
 *
 * This function requests a move target for an agent, setting the target's polygon reference ID and position in world coordinates.
 *
 * @param idx The index of the agent.
 * @param ref The target's polygon reference ID.
 * @param pos The target position in world coordinates.
 * @return True if the request was successfully processed, false otherwise.
 *
 * @note This method is used when a new target is set.
 * @note The position will be constrained to the surface of the navigation mesh.
 * @note The request will be processed during the next update.
 */
bool dtCrowd::requestMoveTarget(const int idx, dtPolyRef ref, const float* pos)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;
	if (!ref)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	// Initialize request.
	ag->targetRef = ref;
	dtVcopy(ag->targetPos, pos);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	if (ag->targetRef)
		ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
	else
		ag->targetState = DT_CROWDAGENT_TARGET_FAILED;

	return true;
}

/**
 * @brief Requests a move target based on velocity for an agent.
 *
 * This function requests a move target for an agent based on velocity. It sets the target velocity for the agent to follow.
 *
 * @param idx The index of the agent.
 * @param vel The target velocity as a 3D vector.
 * @return True if the request was successfully processed, false otherwise.
 */
bool dtCrowd::requestMoveVelocity(const int idx, const float* vel)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	// Initialize request.
	ag->targetRef = 0;
	dtVcopy(ag->targetPos, vel);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	ag->targetState = DT_CROWDAGENT_TARGET_VELOCITY;

	return true;
}

/**
 * @brief Resets the move target for an agent.
 *
 * This function resets the move target for an agent, setting it to none and clearing any previous target-related information.
 *
 * @param idx The index of the agent.
 * @return True if the reset was successfully processed, false otherwise.
 */
bool dtCrowd::resetMoveTarget(const int idx)
{
	if (idx < 0 || idx >= m_maxAgents)
		return false;

	dtCrowdAgent* ag = &m_agents[idx];

	// Initialize request.
	ag->targetRef = 0;
	dtVset(ag->targetPos, 0, 0, 0);
	dtVset(ag->dvel, 0, 0, 0);
	ag->targetPathqRef = DT_PATHQ_INVALID;
	ag->targetReplan = false;
	ag->targetState = DT_CROWDAGENT_TARGET_NONE;

	return true;
}

/**
 * @brief Retrieves the active agents in the crowd.
 *
 * This function retrieves an array of pointers to the active agents in the crowd. Active agents are those currently being processed.
 *
 * @param agents An array to store pointers to the active agents.
 * @param maxAgents The maximum number of agents that can be stored in the 'agents' array.
 * @return The number of active agents retrieved.
 */
int dtCrowd::getActiveAgents(dtCrowdAgent** agents, const int maxAgents)
{
	int n = 0;
	for (int i = 0; i < m_maxAgents; ++i)
	{
		if (!m_agents[i].active) continue;
		if (n < maxAgents)
			agents[n++] = &m_agents[i];
	}
	return n;
}

/**
 * @brief Updates move requests for the agents in the crowd.
 *
 * This function updates move requests for the agents in the crowd, including requesting new paths and handling path results. It also handles target replanning.
 *
 * @param dt The time step for the update.
 */
void dtCrowd::updateMoveRequest(const float /*dt*/)
{
	const int PATH_MAX_AGENTS = 8;
	dtCrowdAgent* queue[PATH_MAX_AGENTS];
	int nqueue = 0;

	// Fire off new requests.
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgent* ag = &m_agents[i];
		if (!ag->active)
			continue;
		if (ag->state == DT_CROWDAGENT_STATE_INVALID)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		if (ag->targetState == DT_CROWDAGENT_TARGET_REQUESTING)
		{
			const dtPolyRef* path = ag->corridor.getPath();
			const int npath = ag->corridor.getPathCount();
			dtAssert(npath);

			static const int MAX_RES = 32;
			float reqPos[3];
			dtPolyRef reqPath[MAX_RES];	// The path to the request location
			int reqPathCount = 0;

			// Quick search towards the goal.
			static const int MAX_ITER = 20;
			m_navquery->initSlicedFindPath(path[0], ag->targetRef, ag->npos, ag->targetPos, &m_filters[ag->params.queryFilterType]);
			m_navquery->updateSlicedFindPath(MAX_ITER, 0);
			dtStatus status = 0;
			if (ag->targetReplan) // && npath > 10)
			{
				// Try to use existing steady path during replan if possible.
				status = m_navquery->finalizeSlicedFindPathPartial(path, npath, reqPath, &reqPathCount, MAX_RES);
			}
			else
			{
				// Try to move towards target when goal changes.
				status = m_navquery->finalizeSlicedFindPath(reqPath, &reqPathCount, MAX_RES);
			}

			if (!dtStatusFailed(status) && reqPathCount > 0)
			{
				// In progress or succeed.
				if (reqPath[reqPathCount - 1] != ag->targetRef)
				{
					// Partial path, constrain target position inside the last polygon.
					status = m_navquery->closestPointOnPoly(reqPath[reqPathCount - 1], ag->targetPos, reqPos, 0);
					if (dtStatusFailed(status))
						reqPathCount = 0;
				}
				else
				{
					dtVcopy(reqPos, ag->targetPos);
				}
			}
			else
			{
				reqPathCount = 0;
			}

			if (!reqPathCount)
			{
				// Could not find path, start the request from current location.
				dtVcopy(reqPos, ag->npos);
				reqPath[0] = path[0];
				reqPathCount = 1;
			}

			ag->corridor.setCorridor(reqPos, reqPath, reqPathCount);
			ag->boundary.reset();
			ag->partial = false;

			if (reqPath[reqPathCount - 1] == ag->targetRef)
			{
				ag->targetState = DT_CROWDAGENT_TARGET_VALID;
				ag->targetReplanTime = 0.0;
			}
			else
			{
				// The path is longer or potentially unreachable, full plan.
				ag->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE;
			}
		}

		if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_QUEUE)
		{
			nqueue = addToPathQueue(ag, queue, nqueue, PATH_MAX_AGENTS);
		}
	}

	for (int i = 0; i < nqueue; ++i)
	{
		dtCrowdAgent* ag = queue[i];
		ag->targetPathqRef = m_pathq.request(ag->corridor.getLastPoly(), ag->targetRef,
			ag->corridor.getTarget(), ag->targetPos, &m_filters[ag->params.queryFilterType]);
		if (ag->targetPathqRef != DT_PATHQ_INVALID)
			ag->targetState = DT_CROWDAGENT_TARGET_WAITING_FOR_PATH;
	}

	// Update requests.
	m_pathq.update(MAX_ITERS_PER_UPDATE);

	dtStatus status;

	// Process path results.
	for (int i = 0; i < m_maxAgents; ++i)
	{
		dtCrowdAgent* ag = &m_agents[i];
		if (!ag->active)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		if (ag->targetState == DT_CROWDAGENT_TARGET_WAITING_FOR_PATH)
		{
			// Poll path queue.
			status = m_pathq.getRequestStatus(ag->targetPathqRef);
			if (dtStatusFailed(status))
			{
				// Path find failed, retry if the target location is still valid.
				ag->targetPathqRef = DT_PATHQ_INVALID;
				if (ag->targetRef)
					ag->targetState = DT_CROWDAGENT_TARGET_REQUESTING;
				else
					ag->targetState = DT_CROWDAGENT_TARGET_FAILED;
				ag->targetReplanTime = 0.0;
			}
			else if (dtStatusSucceed(status))
			{
				const dtPolyRef* path = ag->corridor.getPath();
				const int npath = ag->corridor.getPathCount();
				dtAssert(npath);

				// Apply results.
				float targetPos[3];
				dtVcopy(targetPos, ag->targetPos);

				dtPolyRef* res = m_pathResult;
				bool valid = true;
				int nres = 0;
				status = m_pathq.getPathResult(ag->targetPathqRef, res, &nres, m_maxPathResult);
				if (dtStatusFailed(status) || !nres)
					valid = false;

				if (dtStatusDetail(status, DT_PARTIAL_RESULT))
					ag->partial = true;
				else
					ag->partial = false;

				// Merge result and existing path.
				// The agent might have moved whilst the request is
				// being processed, so the path may have changed.
				// We assume that the end of the path is at the same location
				// where the request was issued.

				// The last ref in the old path should be the same as
				// the location where the request was issued..
				if (valid && path[npath - 1] != res[0])
					valid = false;

				if (valid)
				{
					// Put the old path infront of the old path.
					if (npath > 1)
					{
						// Make space for the old path.
						if ((npath - 1) + nres > m_maxPathResult)
							nres = m_maxPathResult - (npath - 1);

						memmove(res + npath - 1, res, sizeof(dtPolyRef) * nres);
						// Copy old path in the beginning.
						memcpy(res, path, sizeof(dtPolyRef) * (npath - 1));
						nres += npath - 1;

						// Remove trackbacks
						for (int j = 0; j < nres; ++j)
						{
							if (j - 1 >= 0 && j + 1 < nres)
							{
								if (res[j - 1] == res[j + 1])
								{
									memmove(res + (j - 1), res + (j + 1), sizeof(dtPolyRef) * (nres - (j + 1)));
									nres -= 2;
									j -= 2;
								}
							}
						}
					}

					// Check for partial path.
					if (res[nres - 1] != ag->targetRef)
					{
						// Partial path, constrain target position inside the last polygon.
						float nearest[3];
						status = m_navquery->closestPointOnPoly(res[nres - 1], targetPos, nearest, 0);
						if (dtStatusSucceed(status))
							dtVcopy(targetPos, nearest);
						else
							valid = false;
					}
				}

				if (valid)
				{
					// Set current corridor.
					ag->corridor.setCorridor(targetPos, res, nres);
					// Force to update boundary.
					ag->boundary.reset();
					ag->targetState = DT_CROWDAGENT_TARGET_VALID;
				}
				else
				{
					// Something went wrong.
					ag->targetState = DT_CROWDAGENT_TARGET_FAILED;
				}

				ag->targetReplanTime = 0.0;
			}
		}
	}
}

/**
 * @brief Updates topology optimization for a set of agents in the crowd.
 *
 * This function updates topology optimization for a specified set of agents in the crowd. It manages the optimization process based on certain conditions and timing.
 *
 * @param agents An array of agent pointers for which topology optimization should be performed.
 * @param nagents The number of agents in the 'agents' array.
 * @param dt The time step for the update.
 */
void dtCrowd::updateTopologyOptimization(dtCrowdAgent** agents, const int nagents, const float dt)
{
	if (!nagents)
		return;

	const float OPT_TIME_THR = 0.5f; // seconds
	const int OPT_MAX_AGENTS = 1;
	dtCrowdAgent* queue[OPT_MAX_AGENTS];
	int nqueue = 0;

	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;
		if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_TOPO) == 0)
			continue;
		ag->topologyOptTime += dt;
		if (ag->topologyOptTime >= OPT_TIME_THR)
			nqueue = addToOptQueue(ag, queue, nqueue, OPT_MAX_AGENTS);
	}

	for (int i = 0; i < nqueue; ++i)
	{
		dtCrowdAgent* ag = queue[i];
		ag->corridor.optimizePathTopology(m_navquery, &m_filters[ag->params.queryFilterType]);
		ag->topologyOptTime = 0;
	}
}

/**
 * @brief Checks the validity of the path for agents in the crowd.
 *
 * This function checks the validity of the path for agents in the crowd, repositions agents if needed, and triggers replanning when necessary.
 *
 * @param[in] agents An array of agents to check path validity for.
 * @param[in] nagents The number of agents in the 'agents' array.
 * @param[in] dt The time step for the update.
 */
void dtCrowd::checkPathValidity(dtCrowdAgent** agents, const int nagents, const float dt)
{
	static const int CHECK_LOOKAHEAD = 10;
	static const float TARGET_REPLAN_DELAY = 1.0; // seconds

	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		ag->targetReplanTime += dt;

		bool replan = false;

		// First check that the current location is valid.
		const int idx = getAgentIndex(ag);
		float agentPos[3];
		dtPolyRef agentRef = ag->corridor.getFirstPoly();
		dtVcopy(agentPos, ag->npos);
		if (!m_navquery->isValidPolyRef(agentRef, &m_filters[ag->params.queryFilterType]))
		{
			// Current location is not valid, try to reposition.
			// TODO: this can snap agents, how to handle that?
			float nearest[3];
			dtVcopy(nearest, agentPos);
			agentRef = 0;
			m_navquery->findNearestPoly(ag->npos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &agentRef, nearest);
			dtVcopy(agentPos, nearest);

			if (!agentRef)
			{
				// Could not find location in navmesh, set state to invalid.
				ag->corridor.reset(0, agentPos);
				ag->partial = false;
				ag->boundary.reset();
				ag->state = DT_CROWDAGENT_STATE_INVALID;
				continue;
			}

			// Make sure the first polygon is valid, but leave other valid
			// polygons in the path so that replanner can adjust the path better.
			ag->corridor.fixPathStart(agentRef, agentPos);
			//			ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
			ag->boundary.reset();
			dtVcopy(ag->npos, agentPos);

			replan = true;
		}

		// If the agent does not have move target or is controlled by velocity, no need to recover the target nor replan.
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		// Try to recover move request position.
		if (ag->targetState != DT_CROWDAGENT_TARGET_NONE && ag->targetState != DT_CROWDAGENT_TARGET_FAILED)
		{
			if (!m_navquery->isValidPolyRef(ag->targetRef, &m_filters[ag->params.queryFilterType]))
			{
				// Current target is not valid, try to reposition.
				float nearest[3];
				dtVcopy(nearest, ag->targetPos);
				ag->targetRef = 0;
				m_navquery->findNearestPoly(ag->targetPos, m_agentPlacementHalfExtents, &m_filters[ag->params.queryFilterType], &ag->targetRef, nearest);
				dtVcopy(ag->targetPos, nearest);
				replan = true;
			}
			if (!ag->targetRef)
			{
				// Failed to reposition target, fail moverequest.
				ag->corridor.reset(agentRef, agentPos);
				ag->partial = false;
				ag->targetState = DT_CROWDAGENT_TARGET_NONE;
			}
		}

		// If nearby corridor is not valid, replan.
		if (!ag->corridor.isValid(CHECK_LOOKAHEAD, m_navquery, &m_filters[ag->params.queryFilterType]))
		{
			// Fix current path.
//			ag->corridor.trimInvalidPath(agentRef, agentPos, m_navquery, &m_filter);
//			ag->boundary.reset();
			replan = true;
		}

		// If the end of the path is near and it is not the requested location, replan.
		if (ag->targetState == DT_CROWDAGENT_TARGET_VALID)
		{
			if (ag->targetReplanTime > TARGET_REPLAN_DELAY &&
				ag->corridor.getPathCount() < CHECK_LOOKAHEAD &&
				ag->corridor.getLastPoly() != ag->targetRef)
				replan = true;
		}

		// Try to replan path to goal.
		if (replan)
		{
			if (ag->targetState != DT_CROWDAGENT_TARGET_NONE)
			{
				requestMoveTargetReplan(idx, ag->targetRef, ag->targetPos);
			}
		}
	}
}

/**
 * @brief Updates the crowd simulation for the specified time step.
 *
 * This function updates the crowd simulation for the given time step, including pathfinding, collision avoidance, and agent movement.
 *
 * @param[in] dt The time step for the update.
 * @param[out] debug Optional debug information for a specific agent (can be nullptr).
 */
void dtCrowd::update(const float dt, dtCrowdAgentDebugInfo* debug)
{
	m_velocitySampleCount = 0;

	const int debugIdx = debug ? debug->idx : -1;

	dtCrowdAgent** agents = m_activeAgents;
	int nagents = getActiveAgents(agents, m_maxAgents);

	// Check that all agents still have valid paths.
	checkPathValidity(agents, nagents, dt);

	// Update async move request and path finder.
	updateMoveRequest(dt);

	// Optimize path topology.
	updateTopologyOptimization(agents, nagents, dt);

	// Register agents to proximity grid.
	m_grid->clear();
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		const float* p = ag->npos;
		const float r = ag->params.radius;
		m_grid->addItem((unsigned short)i, p[0] - r, p[2] - r, p[0] + r, p[2] + r);
	}

	// Get nearby navmesh segments and agents to collide with.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		// Update the collision boundary after certain distance has been passed or
		// if it has become invalid.
		const float updateThr = ag->params.collisionQueryRange * 0.25f;
		if (dtVdist2DSqr(ag->npos, ag->boundary.getCenter()) > dtSqr(updateThr) ||
			!ag->boundary.isValid(m_navquery, &m_filters[ag->params.queryFilterType]))
		{
			ag->boundary.update(ag->corridor.getFirstPoly(), ag->npos, ag->params.collisionQueryRange,
				m_navquery, &m_filters[ag->params.queryFilterType]);
		}
		// Query neighbour agents
		ag->nneis = getNeighbours(ag->npos, ag->params.height, ag->params.collisionQueryRange,
			ag, ag->neis, DT_CROWDAGENT_MAX_NEIGHBOURS,
			agents, nagents, m_grid);
		for (int j = 0; j < ag->nneis; j++)
			ag->neis[j].idx = getAgentIndex(agents[ag->neis[j].idx]);
	}

	// Find next corner to steer to.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		// Find corners for steering
		ag->ncorners = ag->corridor.findCorners(ag->cornerVerts, ag->cornerFlags, ag->cornerPolys,
			DT_CROWDAGENT_MAX_CORNERS, m_navquery, &m_filters[ag->params.queryFilterType]);

		// Check to see if the corner after the next corner is directly visible,
		// and short cut to there.
		if ((ag->params.updateFlags & DT_CROWD_OPTIMIZE_VIS) && ag->ncorners > 0)
		{
			const float* target = &ag->cornerVerts[dtMin(1, ag->ncorners - 1) * 3];
			ag->corridor.optimizePathVisibility(target, ag->params.pathOptimizationRange, m_navquery, &m_filters[ag->params.queryFilterType]);

			// Copy data for debug purposes.
			if (debugIdx == i)
			{
				dtVcopy(debug->optStart, ag->corridor.getPos());
				dtVcopy(debug->optEnd, target);
			}
		}
		else
		{
			// Copy data for debug purposes.
			if (debugIdx == i)
			{
				dtVset(debug->optStart, 0, 0, 0);
				dtVset(debug->optEnd, 0, 0, 0);
			}
		}
	}

	// Trigger off-mesh connections (depends on corners).
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
			continue;

		// Check
		const float triggerRadius = ag->params.radius * 2.25f;
		if (overOffmeshConnection(ag, triggerRadius))
		{
			// Prepare to off-mesh connection.
			const int idx = (int)(ag - m_agents);
			dtCrowdAgentAnimation* anim = &m_agentAnims[idx];

			// Adjust the path over the off-mesh connection.
			dtPolyRef refs[2];
			if (ag->corridor.moveOverOffmeshConnection(ag->cornerPolys[ag->ncorners - 1], refs,
				anim->startPos, anim->endPos, m_navquery))
			{
				dtVcopy(anim->initPos, ag->npos);
				anim->polyRef = refs[1];
				anim->active = true;
				anim->t = 0.0f;
				anim->tmax = (dtVdist2D(anim->startPos, anim->endPos) / ag->params.maxSpeed) * 0.5f;

				ag->state = DT_CROWDAGENT_STATE_OFFMESH;
				ag->ncorners = 0;
				ag->nneis = 0;
				continue;
			}
			else
			{
				// Path validity check will ensure that bad/blocked connections will be replanned.
			}
		}
	}

	// Calculate steering.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE)
			continue;

		float dvel[3] = { 0,0,0 };

		if (ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			dtVcopy(dvel, ag->targetPos);
			ag->desiredSpeed = dtVlen(ag->targetPos);
		}
		else
		{
			// Calculate steering direction.
			if (ag->params.updateFlags & DT_CROWD_ANTICIPATE_TURNS)
				calcSmoothSteerDirection(ag, dvel);
			else
				calcStraightSteerDirection(ag, dvel);

			// Calculate speed scale, which tells the agent to slowdown at the end of the path.
			const float slowDownRadius = ag->params.radius * 2;	// TODO: make less hacky.
			const float speedScale = getDistanceToGoal(ag, slowDownRadius) / slowDownRadius;

			ag->desiredSpeed = ag->params.maxSpeed;
			dtVscale(dvel, dvel, ag->desiredSpeed * speedScale);
		}

		// Separation
		if (ag->params.updateFlags & DT_CROWD_SEPARATION)
		{
			const float separationDist = ag->params.collisionQueryRange;
			const float invSeparationDist = 1.0f / separationDist;
			const float separationWeight = ag->params.separationWeight;

			float w = 0;
			float disp[3] = { 0,0,0 };

			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];

				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				diff[1] = 0;

				const float distSqr = dtVlenSqr(diff);
				if (distSqr < 0.00001f)
					continue;
				if (distSqr > dtSqr(separationDist))
					continue;
				const float dist = dtMathSqrtf(distSqr);
				const float weight = separationWeight * (1.0f - dtSqr(dist * invSeparationDist));

				dtVmad(disp, disp, diff, weight / dist);
				w += 1.0f;
			}

			if (w > 0.0001f)
			{
				// Adjust desired velocity.
				dtVmad(dvel, dvel, disp, 1.0f / w);
				// Clamp desired velocity to desired speed.
				const float speedSqr = dtVlenSqr(dvel);
				const float desiredSqr = dtSqr(ag->desiredSpeed);
				if (speedSqr > desiredSqr)
					dtVscale(dvel, dvel, desiredSqr / speedSqr);
			}
		}

		// Set the desired velocity.
		dtVcopy(ag->dvel, dvel);
	}

	// Velocity planning.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];

		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		if (ag->params.updateFlags & DT_CROWD_OBSTACLE_AVOIDANCE)
		{
			m_obstacleQuery->reset();

			// Add neighbours as obstacles.
			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];
				m_obstacleQuery->addCircle(nei->npos, nei->params.radius, nei->vel, nei->dvel);
			}

			// Append neighbour segments as obstacles.
			for (int j = 0; j < ag->boundary.getSegmentCount(); ++j)
			{
				const float* s = ag->boundary.getSegment(j);
				if (dtTriArea2D(ag->npos, s, s + 3) < 0.0f)
					continue;
				m_obstacleQuery->addSegment(s, s + 3);
			}

			dtObstacleAvoidanceDebugData* vod = 0;
			if (debugIdx == i)
				vod = debug->vod;

			// Sample new safe velocity.
			bool adaptive = true;
			int ns = 0;

			const dtObstacleAvoidanceParams* params = &m_obstacleQueryParams[ag->params.obstacleAvoidanceType];

			if (adaptive)
			{
				ns = m_obstacleQuery->sampleVelocityAdaptive(ag->npos, ag->params.radius, ag->desiredSpeed,
					ag->vel, ag->dvel, ag->nvel, params, vod);
			}
			else
			{
				ns = m_obstacleQuery->sampleVelocityGrid(ag->npos, ag->params.radius, ag->desiredSpeed,
					ag->vel, ag->dvel, ag->nvel, params, vod);
			}
			m_velocitySampleCount += ns;
		}
		else
		{
			// If not using velocity planning, new velocity is directly the desired velocity.
			dtVcopy(ag->nvel, ag->dvel);
		}
	}

	// Integrate.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;
		integrate(ag, dt);
	}

	// Handle collisions.
	static const float COLLISION_RESOLVE_FACTOR = 0.7f;

	for (int iter = 0; iter < 4; ++iter)
	{
		for (int i = 0; i < nagents; ++i)
		{
			dtCrowdAgent* ag = agents[i];
			const int idx0 = getAgentIndex(ag);

			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			dtVset(ag->disp, 0, 0, 0);

			float w = 0;

			for (int j = 0; j < ag->nneis; ++j)
			{
				const dtCrowdAgent* nei = &m_agents[ag->neis[j].idx];
				const int idx1 = getAgentIndex(nei);

				float diff[3];
				dtVsub(diff, ag->npos, nei->npos);
				diff[1] = 0;

				float dist = dtVlenSqr(diff);
				if (dist > dtSqr(ag->params.radius + nei->params.radius))
					continue;
				dist = dtMathSqrtf(dist);
				float pen = (ag->params.radius + nei->params.radius) - dist;
				if (dist < 0.0001f)
				{
					// Agents on top of each other, try to choose diverging separation directions.
					if (idx0 > idx1)
						dtVset(diff, -ag->dvel[2], 0, ag->dvel[0]);
					else
						dtVset(diff, ag->dvel[2], 0, -ag->dvel[0]);
					pen = 0.01f;
				}
				else
				{
					pen = (1.0f / dist) * (pen * 0.5f) * COLLISION_RESOLVE_FACTOR;
				}

				dtVmad(ag->disp, ag->disp, diff, pen);

				w += 1.0f;
			}

			if (w > 0.0001f)
			{
				const float iw = 1.0f / w;
				dtVscale(ag->disp, ag->disp, iw);
			}
		}

		for (int i = 0; i < nagents; ++i)
		{
			dtCrowdAgent* ag = agents[i];
			if (ag->state != DT_CROWDAGENT_STATE_WALKING)
				continue;

			dtVadd(ag->npos, ag->npos, ag->disp);
		}
	}

	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		if (ag->state != DT_CROWDAGENT_STATE_WALKING)
			continue;

		// Move along navmesh.
		ag->corridor.movePosition(ag->npos, m_navquery, &m_filters[ag->params.queryFilterType]);
		// Get valid constrained position back.
		dtVcopy(ag->npos, ag->corridor.getPos());

		// If not using path, truncate the corridor to just one poly.
		if (ag->targetState == DT_CROWDAGENT_TARGET_NONE || ag->targetState == DT_CROWDAGENT_TARGET_VELOCITY)
		{
			ag->corridor.reset(ag->corridor.getFirstPoly(), ag->npos);
			ag->partial = false;
		}
	}

	// Update agents using off-mesh connection.
	for (int i = 0; i < nagents; ++i)
	{
		dtCrowdAgent* ag = agents[i];
		const int idx = (int)(ag - m_agents);
		dtCrowdAgentAnimation* anim = &m_agentAnims[idx];
		if (!anim->active)
			continue;

		anim->t += dt;
		if (anim->t > anim->tmax)
		{
			// Reset animation
			anim->active = false;
			// Prepare agent for walking.
			ag->state = DT_CROWDAGENT_STATE_WALKING;
			continue;
		}

		// Update position
		const float ta = anim->tmax * 0.15f;
		const float tb = anim->tmax;
		if (anim->t < ta)
		{
			const float u = tween(anim->t, 0.0, ta);
			dtVlerp(ag->npos, anim->initPos, anim->startPos, u);
		}
		else
		{
			const float u = tween(anim->t, ta, tb);
			dtVlerp(ag->npos, anim->startPos, anim->endPos, u);
		}

		// Update velocity.
		dtVset(ag->vel, 0, 0, 0);
		dtVset(ag->dvel, 0, 0, 0);
	}
}