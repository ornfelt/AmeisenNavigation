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

#include <float.h>
#include <string.h>
#include "DetourLocalBoundary.h"
#include "DetourNavMeshQuery.h"
#include "DetourCommon.h"
#include "DetourAssert.h"

/**
 * @brief Constructor for the dtLocalBoundary class.
 *
 * Initializes a local boundary instance.
 */
dtLocalBoundary::dtLocalBoundary() :
	m_nsegs(0),
	m_npolys(0)
{
	dtVset(m_center, FLT_MAX, FLT_MAX, FLT_MAX);
}

/**
 * @brief Destructor for the dtLocalBoundary class.
 *
 * Cleans up the local boundary instance.
 */
dtLocalBoundary::~dtLocalBoundary()
{
}

/**
 * @brief Reset the local boundary to its initial state.
 *
 * Clears all data stored in the local boundary instance.
 */
void dtLocalBoundary::reset()
{
	dtVset(m_center, FLT_MAX, FLT_MAX, FLT_MAX);
	m_npolys = 0;
	m_nsegs = 0;
}

/**
 * @brief Add a segment to the local boundary.
 *
 * Adds a segment to the local boundary, sorted by distance.
 *
 * @param dist The distance of the segment.
 * @param s The segment data (start and end points).
 */
void dtLocalBoundary::addSegment(const float dist, const float* s)
{
	// Insert neighbour based on the distance.
	Segment* seg = 0;
	if (!m_nsegs)
	{
		// First, trivial accept.
		seg = &m_segs[0];
	}
	else if (dist >= m_segs[m_nsegs - 1].d)
	{
		// Further than the last segment, skip.
		if (m_nsegs >= MAX_LOCAL_SEGS)
			return;
		// Last, trivial accept.
		seg = &m_segs[m_nsegs];
	}
	else
	{
		// Insert inbetween.
		int i;
		for (i = 0; i < m_nsegs; ++i)
			if (dist <= m_segs[i].d)
				break;
		const int tgt = i + 1;
		const int n = dtMin(m_nsegs - i, MAX_LOCAL_SEGS - tgt);
		dtAssert(tgt + n <= MAX_LOCAL_SEGS);
		if (n > 0)
			memmove(&m_segs[tgt], &m_segs[i], sizeof(Segment) * n);
		seg = &m_segs[i];
	}

	seg->d = dist;
	memcpy(seg->s, s, sizeof(float) * 6);

	if (m_nsegs < MAX_LOCAL_SEGS)
		m_nsegs++;
}

/**
 * @brief Update the local boundary based on a position and polygon reference.
 *
 * Updates the local boundary based on the current position and polygon reference.
 *
 * @param ref The polygon reference of the current position.
 * @param pos The current position.
 * @param collisionQueryRange The collision query range.
 * @param navquery The navigation mesh query object.
 * @param filter The query filter used for filtering polygons.
 */
void dtLocalBoundary::update(dtPolyRef ref, const float* pos, const float collisionQueryRange,
	dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	static const int MAX_SEGS_PER_POLY = DT_VERTS_PER_POLYGON * 3;

	if (!ref)
	{
		dtVset(m_center, FLT_MAX, FLT_MAX, FLT_MAX);
		m_nsegs = 0;
		m_npolys = 0;
		return;
	}

	dtVcopy(m_center, pos);

	// First query non-overlapping polygons.
	navquery->findLocalNeighbourhood(ref, pos, collisionQueryRange,
		filter, m_polys, 0, &m_npolys, MAX_LOCAL_POLYS);

	// Secondly, store all polygon edges.
	m_nsegs = 0;
	float segs[MAX_SEGS_PER_POLY * 6];
	int nsegs = 0;
	for (int j = 0; j < m_npolys; ++j)
	{
		navquery->getPolyWallSegments(m_polys[j], filter, segs, 0, &nsegs, MAX_SEGS_PER_POLY);
		for (int k = 0; k < nsegs; ++k)
		{
			const float* s = &segs[k * 6];
			// Skip too distant segments.
			float tseg;
			const float distSqr = dtDistancePtSegSqr2D(pos, s, s + 3, tseg);
			if (distSqr > dtSqr(collisionQueryRange))
				continue;
			addSegment(distSqr, s);
		}
	}
}

/**
 * @brief Check if the local boundary is still valid.
 *
 * Checks if the local boundary's polygons are still valid according to a query filter.
 *
 * @param navquery The navigation mesh query object.
 * @param filter The query filter used for filtering polygons.
 * @return True if the local boundary is valid, false otherwise.
 */
bool dtLocalBoundary::isValid(dtNavMeshQuery* navquery, const dtQueryFilter* filter)
{
	if (!m_npolys)
		return false;

	// Check that all polygons still pass query filter.
	for (int i = 0; i < m_npolys; ++i)
	{
		if (!navquery->isValidPolyRef(m_polys[i], filter))
			return false;
	}

	return true;
}