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

#ifndef DETOURPROXIMITYGRID_H
#define DETOURPROXIMITYGRID_H

/**
 * @class dtProximityGrid
 * @brief Represents a proximity grid used for spatial queries.
 */
class dtProximityGrid
{
	/**
	 * @brief The size of each grid cell.
	 */
	float m_cellSize;
	/**
	 * @brief The inverse of the cell size, used for calculations.
	 */
	float m_invCellSize;

	/**
	 * @brief Represents an item stored in the proximity grid.
	 * An item consists of an ID, grid cell coordinates (x, y), and an index
	 * pointing to the next item in the same grid cell.
	 */
	struct Item
	{
		unsigned short id;
		short x, y;
		unsigned short next;
	};

	/**
	 * @brief The pool of items used for grid storage.
	 */
	Item* m_pool;

	/**
	 * @brief The head index of the item pool.
	 */
	int m_poolHead;

	/**
	 * @brief The size of the item pool.
	 */
	int m_poolSize;

	/**
	 * @brief The array storing buckets for grid cells.
	 */
	unsigned short* m_buckets;

	/**
	 * @brief The size of the buckets array.
	 */
	int m_bucketsSize;

	/**
	 * @brief The bounds of the proximity grid.
	 * The bounds are represented as [minx, miny, maxx, maxy].
	 */
	int m_bounds[4];

public:
	/**
	 * @brief Constructs a new dtProximityGrid instance.
	 */
	dtProximityGrid();
	/**
	 * @brief Destroys the dtProximityGrid instance.
	 */
	~dtProximityGrid();

	/**
	 * @brief Initializes the proximity grid with the specified parameters.
	 * @param[in] poolSize The size of the grid's item pool.
	 * @param[in] cellSize The size of grid cells.
	 * @return True if initialization succeeded, false otherwise.
	 */
	bool init(const int poolSize, const float cellSize);

	/**
	 * @brief Clears the proximity grid and resets it to an empty state.
	 */
	void clear();

	/**
	 * @brief Adds an item with a specified ID and bounding box to the proximity grid.
	 * @param[in] id The ID of the item.
	 * @param[in] minx The minimum x-coordinate of the item's bounding box.
	 * @param[in] miny The minimum y-coordinate of the item's bounding box.
	 * @param[in] maxx The maximum x-coordinate of the item's bounding box.
	 * @param[in] maxy The maximum y-coordinate of the item's bounding box.
	 */
	void addItem(const unsigned short id,
		const float minx, const float miny,
		const float maxx, const float maxy);

	/**
	 * @brief Queries for items within a specified bounding box.
	 * @param[in] minx The minimum x-coordinate of the query bounding box.
	 * @param[in] miny The minimum y-coordinate of the query bounding box.
	 * @param[in] maxx The maximum x-coordinate of the query bounding box.
	 * @param[in] maxy The maximum y-coordinate of the query bounding box.
	 * @param[out] ids The array to store the IDs of the queried items.
	 * @param[in] maxIds The maximum number of item IDs that can be stored in the array.
	 * @return The number of items found and stored in the 'ids' array.
	 */
	int queryItems(const float minx, const float miny,
		const float maxx, const float maxy,
		unsigned short* ids, const int maxIds) const;

	/**
	 * @brief Gets the number of items located at a specific cell within the grid.
	 * @param[in] x The x-coordinate of the cell.
	 * @param[in] y The y-coordinate of the cell.
	 * @return The number of items located at the specified cell.
	 */
	int getItemCountAt(const int x, const int y) const;

	/**
	 * @brief Gets the bounds of the proximity grid.
	 * @return A pointer to an array of four integers representing the bounds: [minx, miny, maxx, maxy].
	 */
	inline const int* getBounds() const { return m_bounds; }
	/**
	 * @brief Gets the size of grid cells.
	 * @return The size of grid cells.
	 */
	inline float getCellSize() const { return m_cellSize; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtProximityGrid(const dtProximityGrid&);
	dtProximityGrid& operator=(const dtProximityGrid&);
};

/**
 * @brief Allocates memory for a new dtProximityGrid instance.
 * @return A pointer to the allocated dtProximityGrid instance.
 */
dtProximityGrid* dtAllocProximityGrid();
/**
 * @brief Frees the memory occupied by a dtProximityGrid instance.
 * @param[in] ptr A pointer to the dtProximityGrid instance to be deallocated.
 */
void dtFreeProximityGrid(dtProximityGrid* ptr);

#endif // DETOURPROXIMITYGRID_H
