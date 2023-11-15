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

#ifndef DETOURLOCALBOUNDARY_H
#define DETOURLOCALBOUNDARY_H

#include "DetourNavMeshQuery.h"

/**
 * @brief Class representing a local boundary for navigation queries.
 *
 * The local boundary is used to define a region around a navigation agent for
 * which local queries are performed to adjust the agent's path based on the
 * nearby obstacles and navigation polygons.
 */
class dtLocalBoundary
{
	static const int MAX_LOCAL_SEGS = 8;   ///< Maximum number of local segments.
	static const int MAX_LOCAL_POLYS = 16; ///< Maximum number of local polygons.

	/**
	 * @brief Structure representing a segment for the local boundary.
	 */
	struct Segment
	{
		float s[6];	///< Segment start/end
		float d;	///< Distance for pruning.
	};

	float m_center[3];                  ///< Center position of the local boundary.
	Segment m_segs[MAX_LOCAL_SEGS];     ///< Array of local segments.
	int m_nsegs;                        ///< Number of local segments.

	dtPolyRef m_polys[MAX_LOCAL_POLYS]; ///< Array of local polygons.
	int m_npolys;                       ///< Number of local polygons.

	void addSegment(const float dist, const float* s);

public:
	/**
	 * @brief Constructor for dtLocalBoundary.
	 */
	dtLocalBoundary();
	/**
	 * @brief Destructor for dtLocalBoundary.
	 */
	~dtLocalBoundary();

	/**
	 * @brief Resets the local boundary.
	 *
	 * Clears all segments and polygons from the local boundary.
	 */
	void reset();

	/**
	 * @brief Updates the local boundary based on a navigation query.
	 *
	 * This function updates the local boundary using information from a
	 * navigation query. It adjusts the boundary based on the nearby
	 * obstacles and navigation polygons.
	 *
	 * @param[in] ref The reference of the navigation polygon.
	 * @param[in] pos The position of the navigation agent.
	 * @param[in] collisionQueryRange The range for collision queries.
	 * @param[in] navquery The navigation query instance.
	 * @param[in] filter The query filter used for the query.
	 */
	void update(dtPolyRef ref, const float* pos, const float collisionQueryRange,
		dtNavMeshQuery* navquery, const dtQueryFilter* filter);

	/**
	 * @brief Checks if the local boundary is valid.
	 *
	 * This function checks if the local boundary is valid, which means it
	 * contains at least one valid segment.
	 *
	 * @param[in] navquery The navigation query instance.
	 * @param[in] filter The query filter used for the query.
	 * @return True if the local boundary is valid, false otherwise.
	 */
	bool isValid(dtNavMeshQuery* navquery, const dtQueryFilter* filter);

	/**
	 * @brief Gets the center position of the local boundary.
	 * @return A pointer to the center position array.
	 */
	inline const float* getCenter() const { return m_center; }
	/**
	 * @brief Gets the number of segments in the local boundary.
	 * @return The number of segments.
	 */
	inline int getSegmentCount() const { return m_nsegs; }
	/**
	 * @brief Gets a specific segment from the local boundary.
	 * @param[in] i The index of the segment to retrieve.
	 * @return A pointer to the segment start/end coordinates array.
	 */
	inline const float* getSegment(int i) const { return m_segs[i].s; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtLocalBoundary(const dtLocalBoundary&);
	dtLocalBoundary& operator=(const dtLocalBoundary&);
};

#endif // DETOURLOCALBOUNDARY_H
