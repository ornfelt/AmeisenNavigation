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
 * @file DetourProximityGrid.cpp
 * @brief Implementation of a proximity grid used for spatial queries in navigation mesh generation.
 */

#include <string.h>
#include <new>
#include "DetourProximityGrid.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"

 /**
  * @brief Allocate memory for a proximity grid.
  *
  * @return A pointer to the allocated proximity grid.
  */
dtProximityGrid* dtAllocProximityGrid()
{
	void* mem = dtAlloc(sizeof(dtProximityGrid), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtProximityGrid;
}

/**
 * @brief Free memory for a proximity grid.
 *
 * @param[in] ptr A pointer to the proximity grid to be freed.
 */
void dtFreeProximityGrid(dtProximityGrid* ptr)
{
	if (!ptr) return;
	ptr->~dtProximityGrid();
	dtFree(ptr);
}

/**
 * @brief Hashes 2D grid positions.
 *
 * @param[in] x The x-coordinate of the grid position.
 * @param[in] y The y-coordinate of the grid position.
 * @param[in] n The size of the grid.
 * @return The hash value.
 */
inline int hashPos2(int x, int y, int n)
{
	return ((x * 73856093) ^ (y * 19349663)) & (n - 1);
}

/**
 * @brief Construct a dtProximityGrid object.
 */
dtProximityGrid::dtProximityGrid() :
	m_cellSize(0),
	m_invCellSize(0),
	m_pool(0),
	m_poolHead(0),
	m_poolSize(0),
	m_buckets(0),
	m_bucketsSize(0)
{
}

/**
 * @brief Destruct the dtProximityGrid object and free allocated memory.
 */
dtProximityGrid::~dtProximityGrid()
{
	dtFree(m_buckets);
	dtFree(m_pool);
}

/**
 * @brief Initialize the proximity grid.
 *
 * @param[in] poolSize The size of the object pool.
 * @param[in] cellSize The size of grid cells.
 * @return True if initialization succeeds, false otherwise.
 */
bool dtProximityGrid::init(const int poolSize, const float cellSize)
{
	dtAssert(poolSize > 0);
	dtAssert(cellSize > 0.0f);

	m_cellSize = cellSize;
	m_invCellSize = 1.0f / m_cellSize;

	// Allocate hashs buckets
	m_bucketsSize = dtNextPow2(poolSize);
	m_buckets = (unsigned short*)dtAlloc(sizeof(unsigned short) * m_bucketsSize, DT_ALLOC_PERM);
	if (!m_buckets)
		return false;

	// Allocate pool of items.
	m_poolSize = poolSize;
	m_poolHead = 0;
	m_pool = (Item*)dtAlloc(sizeof(Item) * m_poolSize, DT_ALLOC_PERM);
	if (!m_pool)
		return false;

	clear();

	return true;
}

/**
 * @brief Clear the proximity grid.
 */
void dtProximityGrid::clear()
{
	memset(m_buckets, 0xff, sizeof(unsigned short) * m_bucketsSize);
	m_poolHead = 0;
	m_bounds[0] = 0xffff;
	m_bounds[1] = 0xffff;
	m_bounds[2] = -0xffff;
	m_bounds[3] = -0xffff;
}

/**
 * @brief Add an item to the proximity grid.
 *
 * @param[in] id The ID of the item.
 * @param[in] minx The minimum x-coordinate of the item's bounding box.
 * @param[in] miny The minimum y-coordinate of the item's bounding box.
 * @param[in] maxx The maximum x-coordinate of the item's bounding box.
 * @param[in] maxy The maximum y-coordinate of the item's bounding box.
 */
void dtProximityGrid::addItem(const unsigned short id,
	const float minx, const float miny,
	const float maxx, const float maxy)
{
	const int iminx = (int)dtMathFloorf(minx * m_invCellSize);
	const int iminy = (int)dtMathFloorf(miny * m_invCellSize);
	const int imaxx = (int)dtMathFloorf(maxx * m_invCellSize);
	const int imaxy = (int)dtMathFloorf(maxy * m_invCellSize);

	m_bounds[0] = dtMin(m_bounds[0], iminx);
	m_bounds[1] = dtMin(m_bounds[1], iminy);
	m_bounds[2] = dtMax(m_bounds[2], imaxx);
	m_bounds[3] = dtMax(m_bounds[3], imaxy);

	for (int y = iminy; y <= imaxy; ++y)
	{
		for (int x = iminx; x <= imaxx; ++x)
		{
			if (m_poolHead < m_poolSize)
			{
				const int h = hashPos2(x, y, m_bucketsSize);
				const unsigned short idx = (unsigned short)m_poolHead;
				m_poolHead++;
				Item& item = m_pool[idx];
				item.x = (short)x;
				item.y = (short)y;
				item.id = id;
				item.next = m_buckets[h];
				m_buckets[h] = idx;
			}
		}
	}
}

/**
 * @brief Query items within a specified bounding box in the proximity grid.
 *
 * @param[in] minx The minimum x-coordinate of the bounding box.
 * @param[in] miny The minimum y-coordinate of the bounding box.
 * @param[in] maxx The maximum x-coordinate of the bounding box.
 * @param[in] maxy The maximum y-coordinate of the bounding box.
 * @param[out] ids An array to store the IDs of the queried items.
 * @param[in] maxIds The maximum number of IDs that can be stored in the array.
 * @return The number of items found and stored in the 'ids' array.
 */
int dtProximityGrid::queryItems(const float minx, const float miny,
	const float maxx, const float maxy,
	unsigned short* ids, const int maxIds) const
{
	const int iminx = (int)dtMathFloorf(minx * m_invCellSize);
	const int iminy = (int)dtMathFloorf(miny * m_invCellSize);
	const int imaxx = (int)dtMathFloorf(maxx * m_invCellSize);
	const int imaxy = (int)dtMathFloorf(maxy * m_invCellSize);

	int n = 0;

	for (int y = iminy; y <= imaxy; ++y)
	{
		for (int x = iminx; x <= imaxx; ++x)
		{
			const int h = hashPos2(x, y, m_bucketsSize);
			unsigned short idx = m_buckets[h];
			while (idx != 0xffff)
			{
				Item& item = m_pool[idx];
				if ((int)item.x == x && (int)item.y == y)
				{
					// Check if the id exists already.
					const unsigned short* end = ids + n;
					unsigned short* i = ids;
					while (i != end && *i != item.id)
						++i;
					// Item not found, add it.
					if (i == end)
					{
						if (n >= maxIds)
							return n;
						ids[n++] = item.id;
					}
				}
				idx = item.next;
			}
		}
	}

	return n;
}

/**
 * @brief Get the number of items at a specific grid cell.
 *
 * @param[in] x The x-coordinate of the grid cell.
 * @param[in] y The y-coordinate of the grid cell.
 * @return The number of items at the specified grid cell.
 */
int dtProximityGrid::getItemCountAt(const int x, const int y) const
{
	int n = 0;

	const int h = hashPos2(x, y, m_bucketsSize);
	unsigned short idx = m_buckets[h];
	while (idx != 0xffff)
	{
		Item& item = m_pool[idx];
		if ((int)item.x == x && (int)item.y == y)
			n++;
		idx = item.next;
	}

	return n;
}