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
#include <stdio.h>
#include "DetourNavMesh.h"
#include "DetourNode.h"
#include "DetourCommon.h"
#include "DetourMath.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include <new>

/**
 * @brief Checks if two slabs overlap in a 2D space.
 *
 * This function checks if two slabs defined by their minimum and maximum coordinates
 * in the X and Y dimensions overlap, given a certain padding along the X-axis.
 *
 * @param[in] amin The minimum coordinates of the first slab.
 * @param[in] amax The maximum coordinates of the first slab.
 * @param[in] bmin The minimum coordinates of the second slab.
 * @param[in] bmax The maximum coordinates of the second slab.
 * @param[in] px Padding along the X-axis.
 * @param[in] py Padding along the Y-axis (not used for overlap check).
 * @return True if the slabs overlap, false otherwise.
 */
inline bool overlapSlabs(const float* amin, const float* amax,
	const float* bmin, const float* bmax,
	const float px, const float py)
{
	// Check for horizontal overlap.
	// The segment is shrunken a little so that slabs which touch
	// at end points are not connected.
	const float minx = dtMax(amin[0] + px, bmin[0] + px);
	const float maxx = dtMin(amax[0] - px, bmax[0] - px);
	if (minx > maxx)
		return false;

	// Check vertical overlap.
	const float ad = (amax[1] - amin[1]) / (amax[0] - amin[0]);
	const float ak = amin[1] - ad * amin[0];
	const float bd = (bmax[1] - bmin[1]) / (bmax[0] - bmin[0]);
	const float bk = bmin[1] - bd * bmin[0];
	const float aminy = ad * minx + ak;
	const float amaxy = ad * maxx + ak;
	const float bminy = bd * minx + bk;
	const float bmaxy = bd * maxx + bk;
	const float dmin = bminy - aminy;
	const float dmax = bmaxy - amaxy;

	// Crossing segments always overlap.
	if (dmin * dmax < 0)
		return true;

	// Check for overlap at endpoints.
	const float thr = dtSqr(py * 2);
	if (dmin * dmin <= thr || dmax * dmax <= thr)
		return true;

	return false;
}

/**
 * @brief Gets the coordinate of a slab along a specified side.
 *
 * This function calculates the coordinate of a slab along a specified side (0, 2, 4, or 6).
 *
 * @param[in] va The vertex coordinates of the slab.
 * @param[in] side The side of the slab for which to calculate the coordinate.
 * @return The coordinate of the slab along the specified side.
 */
static float getSlabCoord(const float* va, const int side)
{
	if (side == 0 || side == 4)
		return va[0];
	else if (side == 2 || side == 6)
		return va[2];
	return 0;
}

/**
 * @brief Calculates the minimum and maximum endpoints of a slab along a specified side.
 *
 * This function calculates the minimum and maximum endpoints of a slab along a specified side (0, 2, 4, or 6).
 *
 * @param[in] va The vertex coordinates of one endpoint of the slab.
 * @param[in] vb The vertex coordinates of the other endpoint of the slab.
 * @param[out] bmin The minimum coordinates of the slab along the specified side.
 * @param[out] bmax The maximum coordinates of the slab along the specified side.
 * @param[in] side The side of the slab for which to calculate the endpoints.
 */
static void calcSlabEndPoints(const float* va, const float* vb, float* bmin, float* bmax, const int side)
{
	if (side == 0 || side == 4)
	{
		if (va[2] < vb[2])
		{
			bmin[0] = va[2];
			bmin[1] = va[1];
			bmax[0] = vb[2];
			bmax[1] = vb[1];
		}
		else
		{
			bmin[0] = vb[2];
			bmin[1] = vb[1];
			bmax[0] = va[2];
			bmax[1] = va[1];
		}
	}
	else if (side == 2 || side == 6)
	{
		if (va[0] < vb[0])
		{
			bmin[0] = va[0];
			bmin[1] = va[1];
			bmax[0] = vb[0];
			bmax[1] = vb[1];
		}
		else
		{
			bmin[0] = vb[0];
			bmin[1] = vb[1];
			bmax[0] = va[0];
			bmax[1] = va[1];
		}
	}
}

/**
 * @brief Computes a tile hash value for a given (x, y) coordinate pair.
 *
 * This function computes a hash value for a tile based on its (x, y) coordinates
 * and a provided mask. The hash value can be used for indexing or hashing tiles.
 *
 * @param[in] x The x-coordinate of the tile.
 * @param[in] y The y-coordinate of the tile.
 * @param[in] mask The mask to apply to the hash value.
 * @return The computed tile hash value.
 */
inline int computeTileHash(int x, int y, const int mask)
{
	const unsigned int h1 = 0x8da6b343; // Large multiplicative constants;
	const unsigned int h2 = 0xd8163841; // here arbitrarily chosen primes
	unsigned int n = h1 * x + h2 * y;
	return (int)(n & mask);
}

/**
 * @brief Allocates a link from the free list of links in a tile.
 *
 * This function allocates a link from the free list of links in a tile. Links are
 * used for connecting polygons in the navigation mesh.
 *
 * @param[in] tile A pointer to the tile from which to allocate a link.
 * @return The index of the allocated link or DT_NULL_LINK if the free list is empty.
 */
inline unsigned int allocLink(dtMeshTile* tile)
{
	if (tile->linksFreeList == DT_NULL_LINK)
		return DT_NULL_LINK;
	unsigned int link = tile->linksFreeList;
	tile->linksFreeList = tile->links[link].next;
	return link;
}

/**
 * @brief Frees a link in a mesh tile.
 *
 * @param tile The tile containing the link to free.
 * @param link The index of the link to free.
 */
inline void freeLink(dtMeshTile* tile, unsigned int link)
{
	tile->links[link].next = tile->linksFreeList;
	tile->linksFreeList = link;
}

/**
 * @brief Allocates memory for a navigation mesh.
 *
 * @return A pointer to the allocated navigation mesh or nullptr if allocation fails.
 */
dtNavMesh* dtAllocNavMesh()
{
	void* mem = dtAlloc(sizeof(dtNavMesh), DT_ALLOC_PERM);
	if (!mem) return 0;
	return new(mem) dtNavMesh;
}

/**
 * @brief Frees the memory used by a navigation mesh.
 *
 * This function will only free the memory for tiles with the #DT_TILE_FREE_DATA
 * flag set.
 *
 * @param navmesh A pointer to the navigation mesh to free.
 */
void dtFreeNavMesh(dtNavMesh* navmesh)
{
	if (!navmesh) return;
	navmesh->~dtNavMesh();
	dtFree(navmesh);
}

//////////////////////////////////////////////////////////////////////////////////////////

/**
@class dtNavMesh

The navigation mesh consists of one or more tiles defining three primary types of structural data:

A polygon mesh which defines most of the navigation graph. (See rcPolyMesh for its structure.)
A detail mesh used for determining surface height on the polygon mesh. (See rcPolyMeshDetail for its structure.)
Off-mesh connections, which define custom point-to-point edges within the navigation graph.

The general build process is as follows:

-# Create rcPolyMesh and rcPolyMeshDetail data using the Recast build pipeline.
-# Optionally, create off-mesh connection data.
-# Combine the source data into a dtNavMeshCreateParams structure.
-# Create a tile data array using dtCreateNavMeshData().
-# Allocate at dtNavMesh object and initialize it. (For single tile navigation meshes,
   the tile data is loaded during this step.)
-# For multi-tile navigation meshes, load the tile data using dtNavMesh::addTile().

Notes:

- This class is usually used in conjunction with the dtNavMeshQuery class for pathfinding.
- Technically, all navigation meshes are tiled. A 'solo' mesh is simply a navigation mesh initialized
  to have only a single tile.
- This class does not implement any asynchronous methods. So the ::dtStatus result of all methods will
  always contain either a success or failure flag.

@see dtNavMeshQuery, dtCreateNavMeshData, dtNavMeshCreateParams, #dtAllocNavMesh, #dtFreeNavMesh
*/
dtNavMesh::dtNavMesh() :
	m_tileWidth(0),
	m_tileHeight(0),
	m_maxTiles(0),
	m_tileLutSize(0),
	m_tileLutMask(0),
	m_posLookup(0),
	m_nextFree(0),
	m_tiles(0)
{
#ifndef DT_POLYREF64
	m_saltBits = 0;
	m_tileBits = 0;
	m_polyBits = 0;
#endif
	memset(&m_params, 0, sizeof(dtNavMeshParams));
	m_orig[0] = 0;
	m_orig[1] = 0;
	m_orig[2] = 0;
}

/**
 * @brief Destructor for dtNavMesh.
 * Frees the memory used by tiles with the DT_TILE_FREE_DATA flag set.
 */
dtNavMesh::~dtNavMesh()
{
	for (int i = 0; i < m_maxTiles; ++i)
	{
		if (m_tiles[i].flags & DT_TILE_FREE_DATA)
		{
			dtFree(m_tiles[i].data);
			m_tiles[i].data = 0;
			m_tiles[i].dataSize = 0;
		}
	}
	dtFree(m_posLookup);
	dtFree(m_tiles);
}

/**
 * @brief Initializes the navigation mesh with the given parameters.
 *
 * @param params A pointer to the navigation mesh parameters.
 * @return The status of the initialization operation.
 */
dtStatus dtNavMesh::init(const dtNavMeshParams* params)
{
	memcpy(&m_params, params, sizeof(dtNavMeshParams));
	dtVcopy(m_orig, params->orig);
	m_tileWidth = params->tileWidth;
	m_tileHeight = params->tileHeight;

	// Init tiles
	m_maxTiles = params->maxTiles;
	m_tileLutSize = dtNextPow2(params->maxTiles / 4);
	if (!m_tileLutSize) m_tileLutSize = 1;
	m_tileLutMask = m_tileLutSize - 1;

	m_tiles = (dtMeshTile*)dtAlloc(sizeof(dtMeshTile) * m_maxTiles, DT_ALLOC_PERM);
	if (!m_tiles)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	m_posLookup = (dtMeshTile**)dtAlloc(sizeof(dtMeshTile*) * m_tileLutSize, DT_ALLOC_PERM);
	if (!m_posLookup)
		return DT_FAILURE | DT_OUT_OF_MEMORY;
	memset(m_tiles, 0, sizeof(dtMeshTile) * m_maxTiles);
	memset(m_posLookup, 0, sizeof(dtMeshTile*) * m_tileLutSize);
	m_nextFree = 0;
	for (int i = m_maxTiles - 1; i >= 0; --i)
	{
		m_tiles[i].salt = 1;
		m_tiles[i].next = m_nextFree;
		m_nextFree = &m_tiles[i];
	}

	// Init ID generator values.
#ifndef DT_POLYREF64
	m_tileBits = dtIlog2(dtNextPow2((unsigned int)params->maxTiles));
	m_polyBits = dtIlog2(dtNextPow2((unsigned int)params->maxPolys));
	// Only allow 31 salt bits, since the salt mask is calculated using 32bit uint and it will overflow.
	m_saltBits = dtMin((unsigned int)31, 32 - m_tileBits - m_polyBits);

	if (m_saltBits < 10)
		return DT_FAILURE | DT_INVALID_PARAM;
#endif

	return DT_SUCCESS;
}

/**
 * @brief Initializes the navigation mesh from serialized data.
 *
 * @param data A pointer to the serialized navigation mesh data.
 * @param dataSize The size of the serialized data.
 * @param flags Flags specifying the initialization options.
 * @return The status of the initialization operation.
 */
dtStatus dtNavMesh::init(unsigned char* data, const int dataSize, const int flags)
{
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->version != DT_NAVMESH_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;

	dtNavMeshParams params;
	dtVcopy(params.orig, header->bmin);
	params.tileWidth = header->bmax[0] - header->bmin[0];
	params.tileHeight = header->bmax[2] - header->bmin[2];
	params.maxTiles = 1;
	params.maxPolys = header->polyCount;

	dtStatus status = init(&params);
	if (dtStatusFailed(status))
		return status;

	return addTile(data, dataSize, flags, 0, 0);
}

/**
 * @brief Gets the navigation mesh parameters.
 *
 * This function returns a pointer to the navigation mesh parameters.
 * The parameters are created automatically when the single tile initialization is performed.
 *
 * @return A pointer to the navigation mesh parameters.
 */
const dtNavMeshParams* dtNavMesh::getParams() const
{
	return &m_params;
}

//////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Finds connecting polygons between two points on a tile.
 *
 * This function identifies and returns connecting polygons between two points
 * specified by `va` and `vb` on the given tile's specified side.
 *
 * @param va The starting point of the line segment.
 * @param vb The ending point of the line segment.
 * @param tile The tile on which the search is performed.
 * @param side The side of the tile to search for connecting polygons.
 * @param con An array to store the found connecting polygon references.
 * @param conarea An array to store the corresponding connecting polygon areas.
 * @param maxcon The maximum number of connecting polygons to find.
 * @return The number of connecting polygons found.
 */
int dtNavMesh::findConnectingPolys(const float* va, const float* vb,
	const dtMeshTile* tile, int side,
	dtPolyRef* con, float* conarea, int maxcon) const
{
	if (!tile) return 0;

	float amin[2], amax[2];
	calcSlabEndPoints(va, vb, amin, amax, side);
	const float apos = getSlabCoord(va, side);

	// Remove links pointing to 'side' and compact the links array.
	float bmin[2], bmax[2];
	unsigned short m = DT_EXT_LINK | (unsigned short)side;
	int n = 0;

	dtPolyRef base = getPolyRefBase(tile);

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];
		const int nv = poly->vertCount;
		for (int j = 0; j < nv; ++j)
		{
			// Skip edges which do not point to the right side.
			if (poly->neis[j] != m) continue;

			const float* vc = &tile->verts[poly->verts[j] * 3];
			const float* vd = &tile->verts[poly->verts[(j + 1) % nv] * 3];
			const float bpos = getSlabCoord(vc, side);

			// Segments are not close enough.
			if (dtAbs(apos - bpos) > 0.01f)
				continue;

			// Check if the segments touch.
			calcSlabEndPoints(vc, vd, bmin, bmax, side);

			if (!overlapSlabs(amin, amax, bmin, bmax, 0.01f, tile->header->walkableClimb)) continue;

			// Add return value.
			if (n < maxcon)
			{
				conarea[n * 2 + 0] = dtMax(amin[0], bmin[0]);
				conarea[n * 2 + 1] = dtMin(amax[0], bmax[0]);
				con[n] = base | (dtPolyRef)i;
				n++;
			}
			break;
		}
	}
	return n;
}

/**
 * @brief Unconnects links between two tiles.
 *
 * This function removes the links between the specified `tile` and `target` tiles.
 *
 * @param tile The source tile.
 * @param target The target tile from which links should be removed.
 */
void dtNavMesh::unconnectLinks(dtMeshTile* tile, dtMeshTile* target)
{
	if (!tile || !target) return;

	const unsigned int targetNum = decodePolyIdTile(getTileRef(target));

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];
		unsigned int j = poly->firstLink;
		unsigned int pj = DT_NULL_LINK;
		while (j != DT_NULL_LINK)
		{
			if (decodePolyIdTile(tile->links[j].ref) == targetNum)
			{
				// Remove link.
				unsigned int nj = tile->links[j].next;
				if (pj == DT_NULL_LINK)
					poly->firstLink = nj;
				else
					tile->links[pj].next = nj;
				freeLink(tile, j);
				j = nj;
			}
			else
			{
				// Advance
				pj = j;
				j = tile->links[j].next;
			}
		}
	}
}

/**
 * @brief Connects external links between two tiles.
 *
 * This function connects external links between the specified `tile` and `target` tiles
 * on the specified `side`.
 *
 * @param tile The source tile.
 * @param target The target tile to connect links with.
 * @param side The side of the tile to connect external links.
 */
void dtNavMesh::connectExtLinks(dtMeshTile* tile, dtMeshTile* target, int side)
{
	if (!tile) return;

	// Connect border links.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];

		// Create new links.
//		unsigned short m = DT_EXT_LINK | (unsigned short)side;

		const int nv = poly->vertCount;
		for (int j = 0; j < nv; ++j)
		{
			// Skip non-portal edges.
			if ((poly->neis[j] & DT_EXT_LINK) == 0)
				continue;

			const int dir = (int)(poly->neis[j] & 0xff);
			if (side != -1 && dir != side)
				continue;

			// Create new links
			const float* va = &tile->verts[poly->verts[j] * 3];
			const float* vb = &tile->verts[poly->verts[(j + 1) % nv] * 3];
			dtPolyRef nei[4];
			float neia[4 * 2];
			int nnei = findConnectingPolys(va, vb, target, dtOppositeTile(dir), nei, neia, 4);
			for (int k = 0; k < nnei; ++k)
			{
				unsigned int idx = allocLink(tile);
				if (idx != DT_NULL_LINK)
				{
					dtLink* link = &tile->links[idx];
					link->ref = nei[k];
					link->edge = (unsigned char)j;
					link->side = (unsigned char)dir;

					link->next = poly->firstLink;
					poly->firstLink = idx;

					// Compress portal limits to a byte value.
					if (dir == 0 || dir == 4)
					{
						float tmin = (neia[k * 2 + 0] - va[2]) / (vb[2] - va[2]);
						float tmax = (neia[k * 2 + 1] - va[2]) / (vb[2] - va[2]);
						if (tmin > tmax)
							dtSwap(tmin, tmax);
						link->bmin = (unsigned char)(dtClamp(tmin, 0.0f, 1.0f) * 255.0f);
						link->bmax = (unsigned char)(dtClamp(tmax, 0.0f, 1.0f) * 255.0f);
					}
					else if (dir == 2 || dir == 6)
					{
						float tmin = (neia[k * 2 + 0] - va[0]) / (vb[0] - va[0]);
						float tmax = (neia[k * 2 + 1] - va[0]) / (vb[0] - va[0]);
						if (tmin > tmax)
							dtSwap(tmin, tmax);
						link->bmin = (unsigned char)(dtClamp(tmin, 0.0f, 1.0f) * 255.0f);
						link->bmax = (unsigned char)(dtClamp(tmax, 0.0f, 1.0f) * 255.0f);
					}
				}
			}
		}
	}
}

/**
 * @brief Connects external off-mesh links between two tiles.
 *
 * This function connects external off-mesh links between the specified `tile` and `target` tiles
 * on the specified `side`.
 *
 * @param tile The source tile.
 * @param target The target tile to connect off-mesh links with.
 * @param side The side of the tile to connect external off-mesh links.
 */
void dtNavMesh::connectExtOffMeshLinks(dtMeshTile* tile, dtMeshTile* target, int side)
{
	if (!tile) return;

	// Connect off-mesh links.
	// We are interested on links which land from target tile to this tile.
	const unsigned char oppositeSide = (side == -1) ? 0xff : (unsigned char)dtOppositeTile(side);

	for (int i = 0; i < target->header->offMeshConCount; ++i)
	{
		dtOffMeshConnection* targetCon = &target->offMeshCons[i];
		if (targetCon->side != oppositeSide)
			continue;

		dtPoly* targetPoly = &target->polys[targetCon->poly];
		// Skip off-mesh connections which start location could not be connected at all.
		if (targetPoly->firstLink == DT_NULL_LINK)
			continue;

		const float halfExtents[3] = { targetCon->rad, target->header->walkableClimb, targetCon->rad };

		// Find polygon to connect to.
		const float* p = &targetCon->pos[3];
		float nearestPt[3];
		dtPolyRef ref = findNearestPolyInTile(tile, p, halfExtents, nearestPt);
		if (!ref)
			continue;
		// findNearestPoly may return too optimistic results, further check to make sure.
		if (dtSqr(nearestPt[0] - p[0]) + dtSqr(nearestPt[2] - p[2]) > dtSqr(targetCon->rad))
			continue;
		// Make sure the location is on current mesh.
		float* v = &target->verts[targetPoly->verts[1] * 3];
		dtVcopy(v, nearestPt);

		// Link off-mesh connection to target poly.
		unsigned int idx = allocLink(target);
		if (idx != DT_NULL_LINK)
		{
			dtLink* link = &target->links[idx];
			link->ref = ref;
			link->edge = (unsigned char)1;
			link->side = oppositeSide;
			link->bmin = link->bmax = 0;
			// Add to linked list.
			link->next = targetPoly->firstLink;
			targetPoly->firstLink = idx;
		}

		// Link target poly to off-mesh connection.
		if (targetCon->flags & DT_OFFMESH_CON_BIDIR)
		{
			unsigned int tidx = allocLink(tile);
			if (tidx != DT_NULL_LINK)
			{
				const unsigned short landPolyIdx = (unsigned short)decodePolyIdPoly(ref);
				dtPoly* landPoly = &tile->polys[landPolyIdx];
				dtLink* link = &tile->links[tidx];
				link->ref = getPolyRefBase(target) | (dtPolyRef)(targetCon->poly);
				link->edge = 0xff;
				link->side = (unsigned char)(side == -1 ? 0xff : side);
				link->bmin = link->bmax = 0;
				// Add to linked list.
				link->next = landPoly->firstLink;
				landPoly->firstLink = tidx;
			}
		}
	}
}

/**
 * @brief Connects internal links within a tile.
 *
 * This function connects internal links within the specified `tile`. It builds edge
 * links for each polygon in the tile.
 *
 * @param tile The tile for which internal links are to be connected.
 */
void dtNavMesh::connectIntLinks(dtMeshTile* tile)
{
	if (!tile) return;

	dtPolyRef base = getPolyRefBase(tile);

	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* poly = &tile->polys[i];
		poly->firstLink = DT_NULL_LINK;

		if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
			continue;

		// Build edge links backwards so that the links will be
		// in the linked list from lowest index to highest.
		for (int j = poly->vertCount - 1; j >= 0; --j)
		{
			// Skip hard and non-internal edges.
			if (poly->neis[j] == 0 || (poly->neis[j] & DT_EXT_LINK)) continue;

			unsigned int idx = allocLink(tile);
			if (idx != DT_NULL_LINK)
			{
				dtLink* link = &tile->links[idx];
				link->ref = base | (dtPolyRef)(poly->neis[j] - 1);
				link->edge = (unsigned char)j;
				link->side = 0xff;
				link->bmin = link->bmax = 0;
				// Add to linked list.
				link->next = poly->firstLink;
				poly->firstLink = idx;
			}
		}
	}
}

/**
 * @brief Connects base off-mesh links within a tile.
 *
 * This function connects base off-mesh links within the specified `tile`. It creates links
 * for off-mesh connections starting points.
 *
 * @param tile The tile for which base off-mesh links are to be connected.
 */
void dtNavMesh::baseOffMeshLinks(dtMeshTile* tile)
{
	if (!tile) return;

	dtPolyRef base = getPolyRefBase(tile);

	// Base off-mesh connection start points.
	for (int i = 0; i < tile->header->offMeshConCount; ++i)
	{
		dtOffMeshConnection* con = &tile->offMeshCons[i];
		dtPoly* poly = &tile->polys[con->poly];

		const float halfExtents[3] = { con->rad, tile->header->walkableClimb, con->rad };

		// Find polygon to connect to.
		const float* p = &con->pos[0]; // First vertex
		float nearestPt[3];
		dtPolyRef ref = findNearestPolyInTile(tile, p, halfExtents, nearestPt);
		if (!ref) continue;
		// findNearestPoly may return too optimistic results, further check to make sure.
		if (dtSqr(nearestPt[0] - p[0]) + dtSqr(nearestPt[2] - p[2]) > dtSqr(con->rad))
			continue;
		// Make sure the location is on current mesh.
		float* v = &tile->verts[poly->verts[0] * 3];
		dtVcopy(v, nearestPt);

		// Link off-mesh connection to target poly.
		unsigned int idx = allocLink(tile);
		if (idx != DT_NULL_LINK)
		{
			dtLink* link = &tile->links[idx];
			link->ref = ref;
			link->edge = (unsigned char)0;
			link->side = 0xff;
			link->bmin = link->bmax = 0;
			// Add to linked list.
			link->next = poly->firstLink;
			poly->firstLink = idx;
		}

		// Start end-point is always connect back to off-mesh connection.
		unsigned int tidx = allocLink(tile);
		if (tidx != DT_NULL_LINK)
		{
			const unsigned short landPolyIdx = (unsigned short)decodePolyIdPoly(ref);
			dtPoly* landPoly = &tile->polys[landPolyIdx];
			dtLink* link = &tile->links[tidx];
			link->ref = base | (dtPolyRef)(con->poly);
			link->edge = 0xff;
			link->side = 0xff;
			link->bmin = link->bmax = 0;
			// Add to linked list.
			link->next = landPoly->firstLink;
			landPoly->firstLink = tidx;
		}
	}
}

namespace
{
	template<bool onlyBoundary>
	/**
	 * @brief Finds the closest point on detail edges within a polygon.
	 *
	 * This function calculates the closest point on detail edges within a polygon
	 * of the specified `tile` based on the given `pos` and stores the result in the
	 * `closest` parameter.
	 *
	 * @tparam onlyBoundary If true, considers only boundary edges.
	 * @param tile The tile containing the polygon.
	 * @param poly The polygon for which the closest point is to be found.
	 * @param pos The input position.
	 * @param closest The resulting closest point on detail edges.
	 */
	void closestPointOnDetailEdges(const dtMeshTile* tile, const dtPoly* poly, const float* pos, float* closest)
	{
		const unsigned int ip = (unsigned int)(poly - tile->polys);
		const dtPolyDetail* pd = &tile->detailMeshes[ip];

		float dmin = FLT_MAX;
		float tmin = 0;
		const float* pmin = 0;
		const float* pmax = 0;

		for (int i = 0; i < pd->triCount; i++)
		{
			const unsigned char* tris = &tile->detailTris[(pd->triBase + i) * 4];
			const int ANY_BOUNDARY_EDGE =
				(DT_DETAIL_EDGE_BOUNDARY << 0) |
				(DT_DETAIL_EDGE_BOUNDARY << 2) |
				(DT_DETAIL_EDGE_BOUNDARY << 4);
			if (onlyBoundary && (tris[3] & ANY_BOUNDARY_EDGE) == 0)
				continue;

			const float* v[3];
			for (int j = 0; j < 3; ++j)
			{
				if (tris[j] < poly->vertCount)
					v[j] = &tile->verts[poly->verts[tris[j]] * 3];
				else
					v[j] = &tile->detailVerts[(pd->vertBase + (tris[j] - poly->vertCount)) * 3];
			}

			for (int k = 0, j = 2; k < 3; j = k++)
			{
				if ((dtGetDetailTriEdgeFlags(tris[3], j) & DT_DETAIL_EDGE_BOUNDARY) == 0 &&
					(onlyBoundary || tris[j] < tris[k]))
				{
					// Only looking at boundary edges and this is internal, or
					// this is an inner edge that we will see again or have already seen.
					continue;
				}

				float t;
				float d = dtDistancePtSegSqr2D(pos, v[j], v[k], t);
				if (d < dmin)
				{
					dmin = d;
					tmin = t;
					pmin = v[j];
					pmax = v[k];
				}
			}
		}

		dtVlerp(closest, pmin, pmax, tmin);
	}
}

/**
 * @brief Retrieves the height at a given position within a polygon.
 *
 * This function calculates the height at the specified `pos` within the given `poly`
 * in the provided `tile`. The resulting height is stored in the `height` parameter.
 *
 * @param tile The tile containing the polygon.
 * @param poly The polygon for which the height is to be retrieved.
 * @param pos The input position.
 * @param height The resulting height at the location.
 * @return True if the height was successfully retrieved, false otherwise.
 */
bool dtNavMesh::getPolyHeight(const dtMeshTile* tile, const dtPoly* poly, const float* pos, float* height) const
{
	// Off-mesh connections do not have detail polys and getting height
	// over them does not make sense.
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
		return false;

	const unsigned int ip = (unsigned int)(poly - tile->polys);
	const dtPolyDetail* pd = &tile->detailMeshes[ip];

	float verts[DT_VERTS_PER_POLYGON * 3];
	const int nv = poly->vertCount;
	for (int i = 0; i < nv; ++i)
		dtVcopy(&verts[i * 3], &tile->verts[poly->verts[i] * 3]);

	if (!dtPointInPolygon(pos, verts, nv))
		return false;

	if (!height)
		return true;

	// Find height at the location.
	for (int j = 0; j < pd->triCount; ++j)
	{
		const unsigned char* t = &tile->detailTris[(pd->triBase + j) * 4];
		const float* v[3];
		for (int k = 0; k < 3; ++k)
		{
			if (t[k] < poly->vertCount)
				v[k] = &tile->verts[poly->verts[t[k]] * 3];
			else
				v[k] = &tile->detailVerts[(pd->vertBase + (t[k] - poly->vertCount)) * 3];
		}
		float h;
		if (dtClosestHeightPointTriangle(pos, v[0], v[1], v[2], h))
		{
			*height = h;
			return true;
		}
	}

	// If all triangle checks failed above (can happen with degenerate triangles
	// or larger floating point values) the point is on an edge, so just select
	// closest. This should almost never happen so the extra iteration here is
	// ok.
	float closest[3];
	closestPointOnDetailEdges<false>(tile, poly, pos, closest);
	*height = closest[1];
	return true;
}

/**
 * @brief Finds the closest point on a polygon given a reference and an input position.
 *
 * This function calculates the closest point on the polygon specified by `ref`
 * to the provided `pos` and stores the result in the `closest` parameter. If `posOverPoly`
 * is not nullptr, it is set to true if the position is over the polygon, false otherwise.
 *
 * @param ref The reference to the polygon.
 * @param pos The input position.
 * @param closest The resulting closest point on the polygon.
 * @param posOverPoly If not nullptr, set to true if the position is over the polygon, false otherwise.
 */
void dtNavMesh::closestPointOnPoly(dtPolyRef ref, const float* pos, float* closest, bool* posOverPoly) const
{
	const dtMeshTile* tile = 0;
	const dtPoly* poly = 0;
	getTileAndPolyByRefUnsafe(ref, &tile, &poly);

	dtVcopy(closest, pos);
	if (getPolyHeight(tile, poly, pos, &closest[1]))
	{
		if (posOverPoly)
			*posOverPoly = true;
		return;
	}

	if (posOverPoly)
		*posOverPoly = false;

	// Off-mesh connections don't have detail polygons.
	if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
	{
		const float* v0 = &tile->verts[poly->verts[0] * 3];
		const float* v1 = &tile->verts[poly->verts[1] * 3];
		float t;
		dtDistancePtSegSqr2D(pos, v0, v1, t);
		dtVlerp(closest, v0, v1, t);
		return;
	}

	// Outside poly that is not an offmesh connection.
	closestPointOnDetailEdges<true>(tile, poly, pos, closest);
}

/**
 * @brief Finds the nearest polygon within a tile given a center and half extents.
 *
 * This function searches for the nearest polygon within the specified `tile` based on
 * the provided `center` and `halfExtents` of a bounding box. The resulting nearest
 * polygon reference is returned, and its closest point is stored in `nearestPt`.
 *
 * @param tile The tile in which to search for the nearest polygon.
 * @param center The center of the bounding box used for the search.
 * @param halfExtents The half extents of the bounding box used for the search.
 * @param nearestPt The resulting closest point on the nearest polygon.
 * @return The reference to the nearest polygon.
 */
dtPolyRef dtNavMesh::findNearestPolyInTile(const dtMeshTile* tile,
	const float* center, const float* halfExtents,
	float* nearestPt) const
{
	float bmin[3], bmax[3];
	dtVsub(bmin, center, halfExtents);
	dtVadd(bmax, center, halfExtents);

	// Get nearby polygons from proximity grid.
	dtPolyRef polys[128];
	int polyCount = queryPolygonsInTile(tile, bmin, bmax, polys, 128);

	// Find nearest polygon amongst the nearby polygons.
	dtPolyRef nearest = 0;
	float nearestDistanceSqr = FLT_MAX;
	for (int i = 0; i < polyCount; ++i)
	{
		dtPolyRef ref = polys[i];
		float closestPtPoly[3];
		float diff[3];
		bool posOverPoly = false;
		float d;
		closestPointOnPoly(ref, center, closestPtPoly, &posOverPoly);

		// If a point is directly over a polygon and closer than
		// climb height, favor that instead of straight line nearest point.
		dtVsub(diff, center, closestPtPoly);
		if (posOverPoly)
		{
			d = dtAbs(diff[1]) - tile->header->walkableClimb;
			d = d > 0 ? d * d : 0;
		}
		else
		{
			d = dtVlenSqr(diff);
		}

		if (d < nearestDistanceSqr)
		{
			dtVcopy(nearestPt, closestPtPoly);
			nearestDistanceSqr = d;
			nearest = ref;
		}
	}

	return nearest;
}

/**
 * @brief Queries for polygons within a tile that intersect with a specified bounding box.
 *
 * This function queries for polygons within the provided `tile` that intersect with
 * the bounding box defined by `qmin` and `qmax`. The polygon references found are stored
 * in the `polys` array, up to a maximum of `maxPolys`. The function returns the number
 * of polygons found.
 *
 * @param tile The tile to query for polygons.
 * @param qmin The minimum bounds of the bounding box.
 * @param qmax The maximum bounds of the bounding box.
 * @param polys An array to store the polygon references.
 * @param maxPolys The maximum number of polygons to store in the `polys` array.
 * @return The number of polygons found and stored in the `polys` array.
 */
int dtNavMesh::queryPolygonsInTile(const dtMeshTile* tile, const float* qmin, const float* qmax,
	dtPolyRef* polys, const int maxPolys) const
{
	if (tile->bvTree)
	{
		const dtBVNode* node = &tile->bvTree[0];
		const dtBVNode* end = &tile->bvTree[tile->header->bvNodeCount];
		const float* tbmin = tile->header->bmin;
		const float* tbmax = tile->header->bmax;
		const float qfac = tile->header->bvQuantFactor;

		// Calculate quantized box
		unsigned short bmin[3], bmax[3];
		// dtClamp query box to world box.
		float minx = dtClamp(qmin[0], tbmin[0], tbmax[0]) - tbmin[0];
		float miny = dtClamp(qmin[1], tbmin[1], tbmax[1]) - tbmin[1];
		float minz = dtClamp(qmin[2], tbmin[2], tbmax[2]) - tbmin[2];
		float maxx = dtClamp(qmax[0], tbmin[0], tbmax[0]) - tbmin[0];
		float maxy = dtClamp(qmax[1], tbmin[1], tbmax[1]) - tbmin[1];
		float maxz = dtClamp(qmax[2], tbmin[2], tbmax[2]) - tbmin[2];
		// Quantize
		bmin[0] = (unsigned short)(qfac * minx) & 0xfffe;
		bmin[1] = (unsigned short)(qfac * miny) & 0xfffe;
		bmin[2] = (unsigned short)(qfac * minz) & 0xfffe;
		bmax[0] = (unsigned short)(qfac * maxx + 1) | 1;
		bmax[1] = (unsigned short)(qfac * maxy + 1) | 1;
		bmax[2] = (unsigned short)(qfac * maxz + 1) | 1;

		// Traverse tree
		dtPolyRef base = getPolyRefBase(tile);
		int n = 0;
		while (node < end)
		{
			const bool overlap = dtOverlapQuantBounds(bmin, bmax, node->bmin, node->bmax);
			const bool isLeafNode = node->i >= 0;

			if (isLeafNode && overlap)
			{
				if (n < maxPolys)
					polys[n++] = base | (dtPolyRef)node->i;
			}

			if (overlap || isLeafNode)
				node++;
			else
			{
				const int escapeIndex = -node->i;
				node += escapeIndex;
			}
		}

		return n;
	}
	else
	{
		float bmin[3], bmax[3];
		int n = 0;
		dtPolyRef base = getPolyRefBase(tile);
		for (int i = 0; i < tile->header->polyCount; ++i)
		{
			dtPoly* p = &tile->polys[i];
			// Do not return off-mesh connection polygons.
			if (p->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			// Calc polygon bounds.
			const float* v = &tile->verts[p->verts[0] * 3];
			dtVcopy(bmin, v);
			dtVcopy(bmax, v);
			for (int j = 1; j < p->vertCount; ++j)
			{
				v = &tile->verts[p->verts[j] * 3];
				dtVmin(bmin, v);
				dtVmax(bmax, v);
			}
			if (dtOverlapBounds(qmin, qmax, bmin, bmax))
			{
				if (n < maxPolys)
					polys[n++] = base | (dtPolyRef)i;
			}
		}
		return n;
	}
}

/**
 * @brief Adds a tile to the navigation mesh.
 *
 * This function adds a new tile to the navigation mesh using the provided `data`,
 * `dataSize`, `flags`, and `lastRef`. The `lastRef` parameter is used to restore
 * a tile with the same tile reference it had previously used. In this case, the
 * `dtPolyRef` values for the tile will be restored to their previous values.
 *
 * @note The add operation will fail if the data is in the wrong format, the allocated
 * tile space is full, or there is already a tile at the specified reference.
 *
 * @note The nav mesh assumes exclusive access to the data passed and will make changes
 * to the dynamic portion of the data. Therefore, the data should not be reused in other
 * nav meshes until the tile has been successfully removed from this nav mesh.
 *
 * @param data A pointer to the tile data to be added.
 * @param dataSize The size of the tile data in bytes.
 * @param flags Flags associated with the tile.
 * @param lastRef The last tile reference used for this tile, or 0 if none.
 * @param[out] result The resulting tile reference if the operation is successful.
 * @return The status of the operation (success or failure).
 *
 * @see dtCreateNavMeshData, #removeTile
 */
dtStatus dtNavMesh::addTile(unsigned char* data, int dataSize, int flags,
	dtTileRef lastRef, dtTileRef* result)
{
	// Make sure the data is in right format.
	dtMeshHeader* header = (dtMeshHeader*)data;
	if (header->magic != DT_NAVMESH_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (header->version != DT_NAVMESH_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;

	// Make sure the location is free.
	if (getTileAt(header->x, header->y, header->layer))
		return DT_FAILURE | DT_ALREADY_OCCUPIED;

	// Allocate a tile.
	dtMeshTile* tile = 0;
	if (!lastRef)
	{
		if (m_nextFree)
		{
			tile = m_nextFree;
			m_nextFree = tile->next;
			tile->next = 0;
		}
	}
	else
	{
		// Try to relocate the tile to specific index with same salt.
		int tileIndex = (int)decodePolyIdTile((dtPolyRef)lastRef);
		if (tileIndex >= m_maxTiles)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
		// Try to find the specific tile id from the free list.
		dtMeshTile* target = &m_tiles[tileIndex];
		dtMeshTile* prev = 0;
		tile = m_nextFree;
		while (tile && tile != target)
		{
			prev = tile;
			tile = tile->next;
		}
		// Could not find the correct location.
		if (tile != target)
			return DT_FAILURE | DT_OUT_OF_MEMORY;
		// Remove from freelist
		if (!prev)
			m_nextFree = tile->next;
		else
			prev->next = tile->next;

		// Restore salt.
		tile->salt = decodePolyIdSalt((dtPolyRef)lastRef);
	}

	// Make sure we could allocate a tile.
	if (!tile)
		return DT_FAILURE | DT_OUT_OF_MEMORY;

	// Insert tile into the position lut.
	int h = computeTileHash(header->x, header->y, m_tileLutMask);
	tile->next = m_posLookup[h];
	m_posLookup[h] = tile;

	// Patch header pointers.
	const int headerSize = dtAlign4(sizeof(dtMeshHeader));
	const int vertsSize = dtAlign4(sizeof(float) * 3 * header->vertCount);
	const int polysSize = dtAlign4(sizeof(dtPoly) * header->polyCount);
	const int linksSize = dtAlign4(sizeof(dtLink) * (header->maxLinkCount));
	const int detailMeshesSize = dtAlign4(sizeof(dtPolyDetail) * header->detailMeshCount);
	const int detailVertsSize = dtAlign4(sizeof(float) * 3 * header->detailVertCount);
	const int detailTrisSize = dtAlign4(sizeof(unsigned char) * 4 * header->detailTriCount);
	const int bvtreeSize = dtAlign4(sizeof(dtBVNode) * header->bvNodeCount);
	const int offMeshLinksSize = dtAlign4(sizeof(dtOffMeshConnection) * header->offMeshConCount);

	unsigned char* d = data + headerSize;
	tile->verts = dtGetThenAdvanceBufferPointer<float>(d, vertsSize);
	tile->polys = dtGetThenAdvanceBufferPointer<dtPoly>(d, polysSize);
	tile->links = dtGetThenAdvanceBufferPointer<dtLink>(d, linksSize);
	tile->detailMeshes = dtGetThenAdvanceBufferPointer<dtPolyDetail>(d, detailMeshesSize);
	tile->detailVerts = dtGetThenAdvanceBufferPointer<float>(d, detailVertsSize);
	tile->detailTris = dtGetThenAdvanceBufferPointer<unsigned char>(d, detailTrisSize);
	tile->bvTree = dtGetThenAdvanceBufferPointer<dtBVNode>(d, bvtreeSize);
	tile->offMeshCons = dtGetThenAdvanceBufferPointer<dtOffMeshConnection>(d, offMeshLinksSize);

	// If there are no items in the bvtree, reset the tree pointer.
	if (!bvtreeSize)
		tile->bvTree = 0;

	// Build links freelist
	tile->linksFreeList = 0;
	tile->links[header->maxLinkCount - 1].next = DT_NULL_LINK;
	for (int i = 0; i < header->maxLinkCount - 1; ++i)
		tile->links[i].next = i + 1;

	// Init tile.
	tile->header = header;
	tile->data = data;
	tile->dataSize = dataSize;
	tile->flags = flags;

	connectIntLinks(tile);

	// Base off-mesh connections to their starting polygons and connect connections inside the tile.
	baseOffMeshLinks(tile);
	connectExtOffMeshLinks(tile, tile, -1);

	// Create connections with neighbour tiles.
	static const int MAX_NEIS = 32;
	dtMeshTile* neis[MAX_NEIS];
	int nneis;

	// Connect with layers in current tile.
	nneis = getTilesAt(header->x, header->y, neis, MAX_NEIS);
	for (int j = 0; j < nneis; ++j)
	{
		if (neis[j] == tile)
			continue;

		connectExtLinks(tile, neis[j], -1);
		connectExtLinks(neis[j], tile, -1);
		connectExtOffMeshLinks(tile, neis[j], -1);
		connectExtOffMeshLinks(neis[j], tile, -1);
	}

	// Connect with neighbour tiles.
	for (int i = 0; i < 8; ++i)
	{
		nneis = getNeighbourTilesAt(header->x, header->y, i, neis, MAX_NEIS);
		for (int j = 0; j < nneis; ++j)
		{
			connectExtLinks(tile, neis[j], i);
			connectExtLinks(neis[j], tile, dtOppositeTile(i));
			connectExtOffMeshLinks(tile, neis[j], i);
			connectExtOffMeshLinks(neis[j], tile, dtOppositeTile(i));
		}
	}

	if (result)
		*result = getTileRef(tile);

	return DT_SUCCESS;
}

/**
 * @brief Retrieves the tile at the specified position and layer.
 *
 * This function retrieves the tile at the specified `(x, y, layer)` position within
 * the navigation mesh.
 *
 * @param x The x-coordinate of the tile's position.
 * @param y The y-coordinate of the tile's position.
 * @param layer The layer of the tile.
 * @return A pointer to the tile at the specified position and layer, or nullptr if not found.
 */
const dtMeshTile* dtNavMesh::getTileAt(const int x, const int y, const int layer) const
{
	// Find tile based on hash.
	int h = computeTileHash(x, y, m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y &&
			tile->header->layer == layer)
		{
			return tile;
		}
		tile = tile->next;
	}
	return 0;
}

/**
 * @brief Retrieves neighboring tiles at the specified position.
 *
 * This function retrieves neighboring tiles at the `(x, y)` position, considering
 * the specified `side` to determine the direction. The neighboring tiles are stored
 * in the provided `tiles` array, up to the maximum number specified by `maxTiles`.
 *
 * @param x The x-coordinate of the position.
 * @param y The y-coordinate of the position.
 * @param side The side to consider for neighboring tiles (0-7).
 * @param[out] tiles An array to store neighboring tiles.
 * @param maxTiles The maximum number of neighboring tiles to retrieve.
 * @return The number of neighboring tiles retrieved.
 */
int dtNavMesh::getNeighbourTilesAt(const int x, const int y, const int side, dtMeshTile** tiles, const int maxTiles) const
{
	int nx = x, ny = y;
	switch (side)
	{
	case 0: nx++; break;
	case 1: nx++; ny++; break;
	case 2: ny++; break;
	case 3: nx--; ny++; break;
	case 4: nx--; break;
	case 5: nx--; ny--; break;
	case 6: ny--; break;
	case 7: nx++; ny--; break;
	};

	return getTilesAt(nx, ny, tiles, maxTiles);
}

/**
 * @brief Retrieves tiles at the specified position.
 *
 * This function retrieves tiles at the `(x, y)` position and stores them in the provided
 * `tiles` array, up to the maximum number specified by `maxTiles`.
 *
 * @param x The x-coordinate of the position.
 * @param y The y-coordinate of the position.
 * @param[out] tiles An array to store the tiles at the specified position.
 * @param maxTiles The maximum number of tiles to retrieve.
 * @return The number of tiles retrieved.
 */
int dtNavMesh::getTilesAt(const int x, const int y, dtMeshTile** tiles, const int maxTiles) const
{
	int n = 0;

	// Find tile based on hash.
	int h = computeTileHash(x, y, m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y)
		{
			if (n < maxTiles)
				tiles[n++] = tile;
		}
		tile = tile->next;
	}

	return n;
}

/**
 * @brief Retrieves tiles at the specified position.
 *
 * This function retrieves tiles at the `(x, y)` position and stores them in the provided
 * `tiles` array, up to the maximum number specified by `maxTiles`. This version of the
 * function returns a constant pointer to the tiles.
 *
 * @param x The x-coordinate of the position.
 * @param y The y-coordinate of the position.
 * @param[out] tiles An array to store the tiles at the specified position.
 * @param maxTiles The maximum number of tiles to retrieve.
 * @return The number of tiles retrieved.
 */
int dtNavMesh::getTilesAt(const int x, const int y, dtMeshTile const** tiles, const int maxTiles) const
{
	int n = 0;

	// Find tile based on hash.
	int h = computeTileHash(x, y, m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y)
		{
			if (n < maxTiles)
				tiles[n++] = tile;
		}
		tile = tile->next;
	}

	return n;
}

/**
 * @brief Retrieves the tile reference at the specified position and layer.
 *
 * This function retrieves the tile reference at the specified `(x, y, layer)` position within
 * the navigation mesh.
 *
 * @param x The x-coordinate of the tile's position.
 * @param y The y-coordinate of the tile's position.
 * @param layer The layer of the tile.
 * @return The tile reference at the specified position and layer, or 0 if not found.
 */
dtTileRef dtNavMesh::getTileRefAt(const int x, const int y, const int layer) const
{
	// Find tile based on hash.
	int h = computeTileHash(x, y, m_tileLutMask);
	dtMeshTile* tile = m_posLookup[h];
	while (tile)
	{
		if (tile->header &&
			tile->header->x == x &&
			tile->header->y == y &&
			tile->header->layer == layer)
		{
			return getTileRef(tile);
		}
		tile = tile->next;
	}
	return 0;
}

/**
 * @brief Retrieves the tile based on its reference.
 *
 * This function retrieves a tile based on its reference and returns a pointer to it.
 *
 * @param ref The reference of the tile to retrieve.
 * @return A pointer to the tile with the specified reference, or nullptr if not found.
 */
const dtMeshTile* dtNavMesh::getTileByRef(dtTileRef ref) const
{
	if (!ref)
		return 0;
	unsigned int tileIndex = decodePolyIdTile((dtPolyRef)ref);
	unsigned int tileSalt = decodePolyIdSalt((dtPolyRef)ref);
	if ((int)tileIndex >= m_maxTiles)
		return 0;
	const dtMeshTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return 0;
	return tile;
}

/**
 * @brief Retrieves the maximum number of tiles in the navigation mesh.
 *
 * This function returns the maximum number of tiles that can be stored in the navigation mesh.
 *
 * @return The maximum number of tiles in the navigation mesh.
 */
int dtNavMesh::getMaxTiles() const
{
	return m_maxTiles;
}

/**
 * @brief Retrieves a pointer to a tile by index.
 *
 * This function retrieves a pointer to a tile by its index.
 *
 * @param i The index of the tile to retrieve.
 * @return A pointer to the tile at the specified index.
 */
dtMeshTile* dtNavMesh::getTile(int i)
{
	return &m_tiles[i];
}

/**
 * @brief Retrieves a constant pointer to a tile by index.
 *
 * This function retrieves a constant pointer to a tile by its index.
 *
 * @param i The index of the tile to retrieve.
 * @return A constant pointer to the tile at the specified index.
 */
const dtMeshTile* dtNavMesh::getTile(int i) const
{
	return &m_tiles[i];
}

/**
 * @brief Calculates the tile location for a given position.
 *
 * This function calculates the tile location `(tx, ty)` for a given position `(pos)`.
 *
 * @param pos The position for which to calculate the tile location.
 * @param[out] tx The x-coordinate of the calculated tile location.
 * @param[out] ty The y-coordinate of the calculated tile location.
 */
void dtNavMesh::calcTileLoc(const float* pos, int* tx, int* ty) const
{
	*tx = (int)floorf((pos[0] - m_orig[0]) / m_tileWidth);
	*ty = (int)floorf((pos[2] - m_orig[2]) / m_tileHeight);
}

/**
 * @brief Retrieves the tile and polygon based on a given polygon reference.
 *
 * This function retrieves the tile and polygon based on the specified polygon reference (ref).
 *
 * @param ref The polygon reference for which to retrieve the tile and polygon.
 * @param[out] tile A pointer to the tile containing the specified polygon.
 * @param[out] poly A pointer to the specified polygon.
 * @return The status of the operation, DT_SUCCESS on success, or an error code on failure.
 */
dtStatus dtNavMesh::getTileAndPolyByRef(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	*tile = &m_tiles[it];
	*poly = &m_tiles[it].polys[ip];
	return DT_SUCCESS;
}

/**
 * @brief Retrieves the tile and polygon based on a given polygon reference (unsafe).
 *
 * This function retrieves the tile and polygon based on the specified polygon reference (ref) without validating it.
 * Use this function only if it is known that the provided polygon reference is valid to improve performance.
 *
 * @warning Only use this function if it is known that the provided polygon
 * reference is valid. This function is faster than #getTileAndPolyByRef, but
 * it does not validate the reference.
 *
 * @param ref The polygon reference for which to retrieve the tile and polygon.
 * @param[out] tile A pointer to the tile containing the specified polygon.
 * @param[out] poly A pointer to the specified polygon.
 */
void dtNavMesh::getTileAndPolyByRefUnsafe(const dtPolyRef ref, const dtMeshTile** tile, const dtPoly** poly) const
{
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	*tile = &m_tiles[it];
	*poly = &m_tiles[it].polys[ip];
}

/**
 * @brief Checks if a polygon reference is valid.
 *
 * This function checks if a given polygon reference is valid within the navigation mesh.
 *
 * @param ref The polygon reference to check for validity.
 * @return True if the polygon reference is valid, false otherwise.
 */
bool dtNavMesh::isValidPolyRef(dtPolyRef ref) const
{
	if (!ref) return false;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return false;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return false;
	if (ip >= (unsigned int)m_tiles[it].header->polyCount) return false;
	return true;
}

/**
 * @brief Removes a tile from the navigation mesh and retrieves its data.
 *
 * This function removes a tile from the navigation mesh based on the specified tile reference (ref) and retrieves its data.
 * The removed tile can be re-added to the navigation mesh later using the data.
 *
 * @param ref The tile reference for the tile to be removed.
 * @param[out] data A pointer to the data of the removed tile.
 * @param[out] dataSize The size of the removed tile's data.
 * @return The status of the operation, DT_SUCCESS on success, or an error code on failure.
 *
 * @see #addTile
 */
dtStatus dtNavMesh::removeTile(dtTileRef ref, unsigned char** data, int* dataSize)
{
	if (!ref)
		return DT_FAILURE | DT_INVALID_PARAM;
	unsigned int tileIndex = decodePolyIdTile((dtPolyRef)ref);
	unsigned int tileSalt = decodePolyIdSalt((dtPolyRef)ref);
	if ((int)tileIndex >= m_maxTiles)
		return DT_FAILURE | DT_INVALID_PARAM;
	dtMeshTile* tile = &m_tiles[tileIndex];
	if (tile->salt != tileSalt)
		return DT_FAILURE | DT_INVALID_PARAM;

	// Remove tile from hash lookup.
	int h = computeTileHash(tile->header->x, tile->header->y, m_tileLutMask);
	dtMeshTile* prev = 0;
	dtMeshTile* cur = m_posLookup[h];
	while (cur)
	{
		if (cur == tile)
		{
			if (prev)
				prev->next = cur->next;
			else
				m_posLookup[h] = cur->next;
			break;
		}
		prev = cur;
		cur = cur->next;
	}

	// Remove connections to neighbour tiles.
	static const int MAX_NEIS = 32;
	dtMeshTile* neis[MAX_NEIS];
	int nneis;

	// Disconnect from other layers in current tile.
	nneis = getTilesAt(tile->header->x, tile->header->y, neis, MAX_NEIS);
	for (int j = 0; j < nneis; ++j)
	{
		if (neis[j] == tile) continue;
		unconnectLinks(neis[j], tile);
	}

	// Disconnect from neighbour tiles.
	for (int i = 0; i < 8; ++i)
	{
		nneis = getNeighbourTilesAt(tile->header->x, tile->header->y, i, neis, MAX_NEIS);
		for (int j = 0; j < nneis; ++j)
			unconnectLinks(neis[j], tile);
	}

	// Reset tile.
	if (tile->flags & DT_TILE_FREE_DATA)
	{
		// Owns data
		dtFree(tile->data);
		tile->data = 0;
		tile->dataSize = 0;
		if (data) *data = 0;
		if (dataSize) *dataSize = 0;
	}
	else
	{
		if (data) *data = tile->data;
		if (dataSize) *dataSize = tile->dataSize;
	}

	tile->header = 0;
	tile->flags = 0;
	tile->linksFreeList = 0;
	tile->polys = 0;
	tile->verts = 0;
	tile->links = 0;
	tile->detailMeshes = 0;
	tile->detailVerts = 0;
	tile->detailTris = 0;
	tile->bvTree = 0;
	tile->offMeshCons = 0;

	// Update salt, salt should never be zero.
#ifdef DT_POLYREF64
	tile->salt = (tile->salt + 1) & ((1 << DT_SALT_BITS) - 1);
#else
	tile->salt = (tile->salt + 1) & ((1 << m_saltBits) - 1);
#endif
	if (tile->salt == 0)
		tile->salt++;

	// Add to free list.
	tile->next = m_nextFree;
	m_nextFree = tile;

	return DT_SUCCESS;
}

/**
 * @brief Retrieves the tile reference for a given tile.
 *
 * This function retrieves the tile reference for a specified tile within the navigation mesh.
 *
 * @param tile A pointer to the tile for which to retrieve the reference.
 * @return The tile reference for the specified tile.
 */
dtTileRef dtNavMesh::getTileRef(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = (unsigned int)(tile - m_tiles);
	return (dtTileRef)encodePolyId(tile->salt, it, 0);
}

/**
 * @brief Gets the base polygon reference for a given tile.
 *
 * This function retrieves the base polygon reference for a specified tile within the navigation mesh.
 * The base reference can be combined with polygon indices to access individual polygons in the tile.
 *
 * @param tile A pointer to the tile for which to retrieve the base polygon reference.
 * @return The base polygon reference for the specified tile.
 *
 * @par
 * Example use case:
 * @code
 * const dtPolyRef base = navmesh->getPolyRefBase(tile);
 * for (int i = 0; i < tile->header->polyCount; ++i)
 * {
 *     const dtPoly* p = &tile->polys[i];
 *     const dtPolyRef ref = base | (dtPolyRef)i;
 *
 *     // Use the reference to access the polygon data.
 * }
 * @endcode
 */
dtPolyRef dtNavMesh::getPolyRefBase(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const unsigned int it = (unsigned int)(tile - m_tiles);
	return encodePolyId(tile->salt, it, 0);
}

/**
 * @brief Represents the state of a tile within the navigation mesh.
 *
 * The `dtTileState` structure stores information about a tile, including its magic number,
 * data version number, and tile reference at the time of storing the data.
 */
struct dtTileState
{
	int magic;       //!< Magic number, used to identify the data.
	int version;     //!< Data version number.
	dtTileRef ref;   //!< Tile reference at the time of storing the data.
};

/**
 * @brief Represents the state of a polygon within a tile in the navigation mesh.
 *
 * The `dtPolyState` structure stores information about a polygon, including its flags
 * and area ID.
 */
struct dtPolyState
{
	unsigned short flags;   //!< Flags (see dtPolyFlags).
	unsigned char area;     //!< Area ID of the polygon.
};

/**
 * @brief Gets the size of the tile state data.
 *
 * This function calculates the size of the tile state data, which includes the tile state header
 * and the state information for individual polygons in the tile.
 *
 * @param tile A pointer to the tile for which to calculate the state data size.
 * @return The size of the tile state data in bytes.
 *
 * @see #storeTileState
 */
int dtNavMesh::getTileStateSize(const dtMeshTile* tile) const
{
	if (!tile) return 0;
	const int headerSize = dtAlign4(sizeof(dtTileState));
	const int polyStateSize = dtAlign4(sizeof(dtPolyState) * tile->header->polyCount);
	return headerSize + polyStateSize;
}

/**
 * @brief Stores the state of a tile within the navigation mesh, including polygon flags and area IDs.
 *
 * This function stores the non-structural data of a tile, such as polygon flags and area IDs.
 * The state data is only valid until the tile reference changes.
 *
 * @param[in] tile The tile for which to store the state.
 * @param[out] data The buffer to store the tile state data.
 * @param[in] maxDataSize The maximum size of the data buffer.
 * @return The status of the operation. #DT_SUCCESS if successful, or an error code otherwise.
 * @see #getTileStateSize, #restoreTileState
 */
dtStatus dtNavMesh::storeTileState(const dtMeshTile* tile, unsigned char* data, const int maxDataSize) const
{
	// Make sure there is enough space to store the state.
	const int sizeReq = getTileStateSize(tile);
	if (maxDataSize < sizeReq)
		return DT_FAILURE | DT_BUFFER_TOO_SMALL;

	dtTileState* tileState = dtGetThenAdvanceBufferPointer<dtTileState>(data, dtAlign4(sizeof(dtTileState)));
	dtPolyState* polyStates = dtGetThenAdvanceBufferPointer<dtPolyState>(data, dtAlign4(sizeof(dtPolyState) * tile->header->polyCount));

	// Store tile state.
	tileState->magic = DT_NAVMESH_STATE_MAGIC;
	tileState->version = DT_NAVMESH_STATE_VERSION;
	tileState->ref = getTileRef(tile);

	// Store per poly state.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		const dtPoly* p = &tile->polys[i];
		dtPolyState* s = &polyStates[i];
		s->flags = p->flags;
		s->area = p->getArea();
	}

	return DT_SUCCESS;
}

/**
 * @brief Restores the state of a tile within the navigation mesh, including polygon flags and area IDs.
 *
 * This function restores the non-structural data of a tile, such as polygon flags and area IDs.
 * It does not impact the tile's #dtTileRef and #dtPolyRef values.
 *
 * @param[in,out] tile The tile for which to restore the state.
 * @param[in] data The buffer containing the tile state data.
 * @param[in] maxDataSize The maximum size of the data buffer.
 * @return The status of the operation. #DT_SUCCESS if successful, or an error code otherwise.
 * @see #storeTileState
 */
dtStatus dtNavMesh::restoreTileState(dtMeshTile* tile, const unsigned char* data, const int maxDataSize)
{
	// Make sure there is enough space to store the state.
	const int sizeReq = getTileStateSize(tile);
	if (maxDataSize < sizeReq)
		return DT_FAILURE | DT_INVALID_PARAM;

	const dtTileState* tileState = dtGetThenAdvanceBufferPointer<const dtTileState>(data, dtAlign4(sizeof(dtTileState)));
	const dtPolyState* polyStates = dtGetThenAdvanceBufferPointer<const dtPolyState>(data, dtAlign4(sizeof(dtPolyState) * tile->header->polyCount));

	// Check that the restore is possible.
	if (tileState->magic != DT_NAVMESH_STATE_MAGIC)
		return DT_FAILURE | DT_WRONG_MAGIC;
	if (tileState->version != DT_NAVMESH_STATE_VERSION)
		return DT_FAILURE | DT_WRONG_VERSION;
	if (tileState->ref != getTileRef(tile))
		return DT_FAILURE | DT_INVALID_PARAM;

	// Restore per poly state.
	for (int i = 0; i < tile->header->polyCount; ++i)
	{
		dtPoly* p = &tile->polys[i];
		const dtPolyState* s = &polyStates[i];
		p->flags = s->flags;
		p->setArea(s->area);
	}

	return DT_SUCCESS;
}

/**
 * @brief Retrieves the endpoints of an off-mesh connection polygon.
 *
 * Off-mesh connections are stored in the navigation mesh as special 2-vertex
 * polygons with a single edge. At least one of the vertices is expected to be
 * inside a normal polygon. So an off-mesh connection is "entered" from a
 * normal polygon at one of its endpoints. This is the polygon identified by
 * the prevRef parameter.
 *
 * @param[in] prevRef The polygon reference that leads to the off-mesh connection.
 * @param[in] polyRef The polygon reference of the off-mesh connection.
 * @param[out] startPos The 3D position of the starting point of the connection.
 * @param[out] endPos The 3D position of the ending point of the connection.
 * @return The status of the operation. #DT_SUCCESS if successful, or an error code otherwise.
 */
dtStatus dtNavMesh::getOffMeshConnectionPolyEndPoints(dtPolyRef prevRef, dtPolyRef polyRef, float* startPos, float* endPos) const
{
	unsigned int salt, it, ip;

	if (!polyRef)
		return DT_FAILURE;

	// Get current polygon
	decodePolyId(polyRef, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	const dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	const dtPoly* poly = &tile->polys[ip];

	// Make sure that the current poly is indeed off-mesh link.
	if (poly->getType() != DT_POLYTYPE_OFFMESH_CONNECTION)
		return DT_FAILURE;

	// Figure out which way to hand out the vertices.
	int idx0 = 0, idx1 = 1;

	// Find link that points to first vertex.
	for (unsigned int i = poly->firstLink; i != DT_NULL_LINK; i = tile->links[i].next)
	{
		if (tile->links[i].edge == 0)
		{
			if (tile->links[i].ref != prevRef)
			{
				idx0 = 1;
				idx1 = 0;
			}
			break;
		}
	}

	dtVcopy(startPos, &tile->verts[poly->verts[idx0] * 3]);
	dtVcopy(endPos, &tile->verts[poly->verts[idx1] * 3]);

	return DT_SUCCESS;
}

/**
 * @brief Retrieves the off-mesh connection data by its polygon reference.
 *
 * @param[in] ref The polygon reference of the off-mesh connection.
 * @return A pointer to the off-mesh connection data, or nullptr if not found.
 */
const dtOffMeshConnection* dtNavMesh::getOffMeshConnectionByRef(dtPolyRef ref) const
{
	unsigned int salt, it, ip;

	if (!ref)
		return 0;

	// Get current polygon
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return 0;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return 0;
	const dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return 0;
	const dtPoly* poly = &tile->polys[ip];

	// Make sure that the current poly is indeed off-mesh link.
	if (poly->getType() != DT_POLYTYPE_OFFMESH_CONNECTION)
		return 0;

	const unsigned int idx = ip - tile->header->offMeshBase;
	dtAssert(idx < (unsigned int)tile->header->offMeshConCount);
	return &tile->offMeshCons[idx];
}

/**
 * @brief Sets the flags of a polygon by its polygon reference.
 *
 * @param[in] ref The polygon reference of the polygon to modify.
 * @param[in] flags The new flags to assign to the polygon.
 * @return The status of the operation. #DT_SUCCESS if successful, or an error code otherwise.
 */
dtStatus dtNavMesh::setPolyFlags(dtPolyRef ref, unsigned short flags)
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	dtPoly* poly = &tile->polys[ip];

	// Change flags.
	poly->flags = flags;

	return DT_SUCCESS;
}

/**
 * @brief Gets the flags of a polygon by its polygon reference.
 *
 * @param[in] ref The polygon reference of the polygon to query.
 * @param[out] resultFlags The resulting flags of the polygon.
 * @return The status of the operation. #DT_SUCCESS if successful, or an error code otherwise.
 */
dtStatus dtNavMesh::getPolyFlags(dtPolyRef ref, unsigned short* resultFlags) const
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	const dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	const dtPoly* poly = &tile->polys[ip];

	*resultFlags = poly->flags;

	return DT_SUCCESS;
}

/**
 * @brief Sets the area of a polygon by its polygon reference.
 *
 * @param[in] ref The polygon reference of the polygon to modify.
 * @param[in] area The new area value to assign to the polygon.
 * @return The status of the operation. #DT_SUCCESS if successful, or an error code otherwise.
 */
dtStatus dtNavMesh::setPolyArea(dtPolyRef ref, unsigned char area)
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	dtPoly* poly = &tile->polys[ip];

	poly->setArea(area);

	return DT_SUCCESS;
}

/**
 * @brief Gets the area of a polygon by its polygon reference.
 *
 * @param[in] ref The polygon reference of the polygon to query.
 * @param[out] resultArea The resulting area value of the polygon.
 * @return The status of the operation. #DT_SUCCESS if successful, or an error code otherwise.
 */
dtStatus dtNavMesh::getPolyArea(dtPolyRef ref, unsigned char* resultArea) const
{
	if (!ref) return DT_FAILURE;
	unsigned int salt, it, ip;
	decodePolyId(ref, salt, it, ip);
	if (it >= (unsigned int)m_maxTiles) return DT_FAILURE | DT_INVALID_PARAM;
	if (m_tiles[it].salt != salt || m_tiles[it].header == 0) return DT_FAILURE | DT_INVALID_PARAM;
	const dtMeshTile* tile = &m_tiles[it];
	if (ip >= (unsigned int)tile->header->polyCount) return DT_FAILURE | DT_INVALID_PARAM;
	const dtPoly* poly = &tile->polys[ip];

	*resultArea = poly->getArea();

	return DT_SUCCESS;
}