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
 * @file DetourPathQueue.h
 * @brief Contains the declaration of the dtPathQueue class, which is used for asynchronous pathfinding.
 */

#ifndef DETOURPATHQUEUE_H
#define DETOURPATHQUEUE_H

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

 /// Constant representing an invalid path queue reference.
static const unsigned int DT_PATHQ_INVALID = 0;

/// Type representing a path queue reference.
typedef unsigned int dtPathQueueRef;

/**
 * @class dtPathQueue
 * @brief Manages asynchronous pathfinding requests.
 *
 * The dtPathQueue class is used to manage asynchronous pathfinding requests. It allows multiple pathfinding
 * requests to be processed simultaneously without blocking the main application thread.
 */
class dtPathQueue
{
	struct PathQuery
	{
		dtPathQueueRef ref; ///< The reference to the pathfinding request.
		float startPos[3]; ///< The starting position for the pathfinding query.
		float endPos[3]; ///< The ending position for the pathfinding query.
		dtPolyRef startRef; ///< The starting polygon reference.
		dtPolyRef endRef; ///< The ending polygon reference.
		dtPolyRef* path; ///< Pointer to the result path.
		int npath; ///< The number of polygons in the result path.
		dtStatus status; ///< The status of the pathfinding query.
		int keepAlive; ///< Keeps the query alive for a specified number of updates.
		const dtQueryFilter* filter; ///< The query filter to apply. (Note: This can be dangerous to store!)
	};

	static const int MAX_QUEUE = 8; ///< Maximum number of pathfinding queries in the queue.
	PathQuery m_queue[MAX_QUEUE]; ///< The array of pathfinding queries.
	dtPathQueueRef m_nextHandle; ///< The next available path queue reference.
	int m_maxPathSize; ///< The maximum path size.
	int m_queueHead; ///< The queue head index.
	dtNavMeshQuery* m_navquery; ///< Pointer to the navigation mesh query object.

	void purge();

public:
	/**
	 * @brief Constructs a new dtPathQueue instance.
	 */
	dtPathQueue();
	/**
	 * @brief Destroys the dtPathQueue instance.
	 */
	~dtPathQueue();

	/**
	 * @brief Initializes the path queue.
	 *
	 * @param[in] maxPathSize The maximum path size.
	 * @param[in] maxSearchNodeCount The maximum number of search nodes.
	 * @param[in] nav The navigation mesh.
	 * @return True if the initialization was successful, false otherwise.
	 */
	bool init(const int maxPathSize, const int maxSearchNodeCount, dtNavMesh* nav);

	/**
	 * @brief Updates the path queue.
	 *
	 * This method should be called regularly to update the path queue and process pathfinding requests.
	 *
	 * @param[in] maxIters The maximum number of iterations to update the path queue.
	 */
	void update(const int maxIters);

	/**
	 * @brief Requests a pathfinding query.
	 *
	 * @param[in] startRef The starting polygon reference.
	 * @param[in] endRef The ending polygon reference.
	 * @param[in] startPos The starting position for the query. [(x, y, z)]
	 * @param[in] endPos The ending position for the query. [(x, y, z)]
	 * @param[in] filter The query filter to apply.
	 * @return The path queue reference for the request.
	 */
	dtPathQueueRef request(dtPolyRef startRef, dtPolyRef endRef,
		const float* startPos, const float* endPos,
		const dtQueryFilter* filter);

	/**
	 * @brief Gets the status of a pathfinding request.
	 *
	 * @param[in] ref The path queue reference.
	 * @return The status of the pathfinding request.
	 */
	dtStatus getRequestStatus(dtPathQueueRef ref) const;

	/**
	 * @brief Gets the result of a pathfinding request.
	 *
	 * @param[in] ref The path queue reference.
	 * @param[out] path The pointer to the result path.
	 * @param[out] pathSize The number of polygons in the result path.
	 * @param[in] maxPath The maximum number of polygons the result path can hold.
	 * @return The status of the pathfinding request.
	 */
	dtStatus getPathResult(dtPathQueueRef ref, dtPolyRef* path, int* pathSize, const int maxPath);

	/**
	 * @brief Gets the navigation mesh query object associated with the path queue.
	 *
	 * @return The navigation mesh query object.
	 */
	inline const dtNavMeshQuery* getNavQuery() const { return m_navquery; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtPathQueue(const dtPathQueue&);
	dtPathQueue& operator=(const dtPathQueue&);
};

#endif // DETOURPATHQUEUE_H
