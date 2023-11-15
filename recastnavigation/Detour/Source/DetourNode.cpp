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

#include "DetourNode.h"
#include "DetourAlloc.h"
#include "DetourAssert.h"
#include "DetourCommon.h"
#include <string.h>

#ifdef DT_POLYREF64
/**
 * @brief Hash function for 64-bit polyrefs.
 *
 * This function hashes a 64-bit polygon reference using a specific algorithm.
 * It is adapted from Thomas Wang's hash function.
 * From Thomas Wang, https://gist.github.com/badboy/6267743
 *
 * @param a The input polygon reference to be hashed.
 * @return The hashed value as an unsigned integer.
 */
inline unsigned int dtHashRef(dtPolyRef a)
{
	a = (~a) + (a << 18); // a = (a << 18) - a - 1;
	a = a ^ (a >> 31);
	a = a * 21; // a = (a + (a << 2)) + (a << 4);
	a = a ^ (a >> 11);
	a = a + (a << 6);
	a = a ^ (a >> 22);
	return (unsigned int)a;
}
#else
/**
 * @brief Hash function for 32-bit polyrefs.
 *
 * This function hashes a 32-bit polygon reference using a specific algorithm.
 *
 * @param a The input polygon reference to be hashed.
 * @return The hashed value as an unsigned integer.
 */
inline unsigned int dtHashRef(dtPolyRef a)
{
	a += ~(a << 15);
	a ^= (a >> 10);
	a += (a << 3);
	a ^= (a >> 6);
	a += ~(a << 11);
	a ^= (a >> 16);
	return (unsigned int)a;
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Constructor for the dtNodePool class.
 *
 * This constructor initializes a node pool with a maximum number of nodes and a hash size.
 *
 * @param maxNodes The maximum number of nodes in the pool.
 * @param hashSize The size of the hash table used for quick node lookup.
 */
dtNodePool::dtNodePool(int maxNodes, int hashSize) :
	m_nodes(0),
	m_first(0),
	m_next(0),
	m_maxNodes(maxNodes),
	m_hashSize(hashSize),
	m_nodeCount(0)
{
	dtAssert(dtNextPow2(m_hashSize) == (unsigned int)m_hashSize);
	// pidx is special as 0 means "none" and 1 is the first node. For that reason
	// we have 1 fewer nodes available than the number of values it can contain.
	dtAssert(m_maxNodes > 0 && m_maxNodes <= DT_NULL_IDX && m_maxNodes <= (1 << DT_NODE_PARENT_BITS) - 1);

	m_nodes = (dtNode*)dtAlloc(sizeof(dtNode) * m_maxNodes, DT_ALLOC_PERM);
	m_next = (dtNodeIndex*)dtAlloc(sizeof(dtNodeIndex) * m_maxNodes, DT_ALLOC_PERM);
	m_first = (dtNodeIndex*)dtAlloc(sizeof(dtNodeIndex) * hashSize, DT_ALLOC_PERM);

	dtAssert(m_nodes);
	dtAssert(m_next);
	dtAssert(m_first);

	memset(m_first, 0xff, sizeof(dtNodeIndex) * m_hashSize);
	memset(m_next, 0xff, sizeof(dtNodeIndex) * m_maxNodes);
}

/**
 * @brief Destructor for the dtNodePool class.
 *
 * This destructor frees the memory allocated for the node pool.
 */
dtNodePool::~dtNodePool()
{
	dtFree(m_nodes);
	dtFree(m_next);
	dtFree(m_first);
}

/**
 * @brief Clear the node pool.
 *
 * This function clears the contents of the node pool, resetting it to an empty state.
 */
void dtNodePool::clear()
{
	memset(m_first, 0xff, sizeof(dtNodeIndex) * m_hashSize);
	m_nodeCount = 0;
}

/**
 * @brief Find nodes with a given ID.
 *
 * This function searches for nodes with a specified polygon reference ID and stores
 * them in an array, up to a maximum number of nodes.
 *
 * @param id The polygon reference ID to search for.
 * @param nodes An array to store found nodes.
 * @param maxNodes The maximum number of nodes to find.
 * @return The number of nodes found.
 */
unsigned int dtNodePool::findNodes(dtPolyRef id, dtNode** nodes, const int maxNodes)
{
	int n = 0;
	unsigned int bucket = dtHashRef(id) & (m_hashSize - 1);
	dtNodeIndex i = m_first[bucket];
	while (i != DT_NULL_IDX)
	{
		if (m_nodes[i].id == id)
		{
			if (n >= maxNodes)
				return n;
			nodes[n++] = &m_nodes[i];
		}
		i = m_next[i];
	}

	return n;
}

/**
 * @brief Find a node with a given ID and state.
 *
 * This function searches for a node with a specified polygon reference ID and state.
 *
 * @param id The polygon reference ID to search for.
 * @param state The state to match for the found node.
 * @return A pointer to the found node or nullptr if not found.
 */
dtNode* dtNodePool::findNode(dtPolyRef id, unsigned char state)
{
	unsigned int bucket = dtHashRef(id) & (m_hashSize - 1);
	dtNodeIndex i = m_first[bucket];
	while (i != DT_NULL_IDX)
	{
		if (m_nodes[i].id == id && m_nodes[i].state == state)
			return &m_nodes[i];
		i = m_next[i];
	}
	return 0;
}

/**
 * @brief Get or create a node with a given ID and state.
 *
 * This function either returns an existing node with the specified polygon reference ID
 * and state or creates a new node if it doesn't exist.
 *
 * @param id The polygon reference ID.
 * @param state The state of the node.
 * @return A pointer to the node with the specified ID and state.
 */
dtNode* dtNodePool::getNode(dtPolyRef id, unsigned char state)
{
	unsigned int bucket = dtHashRef(id) & (m_hashSize - 1);
	dtNodeIndex i = m_first[bucket];
	dtNode* node = 0;
	while (i != DT_NULL_IDX)
	{
		if (m_nodes[i].id == id && m_nodes[i].state == state)
			return &m_nodes[i];
		i = m_next[i];
	}

	if (m_nodeCount >= m_maxNodes)
		return 0;

	i = (dtNodeIndex)m_nodeCount;
	m_nodeCount++;

	// Init node
	node = &m_nodes[i];
	node->pidx = 0;
	node->cost = 0;
	node->total = 0;
	node->id = id;
	node->state = state;
	node->flags = 0;

	m_next[i] = m_first[bucket];
	m_first[bucket] = i;

	return node;
}

//////////////////////////////////////////////////////////////////////////////////////////
/**
 * @brief Constructor for the dtNodeQueue class.
 *
 * This constructor initializes a node queue with a maximum capacity.
 *
 * @param n The maximum capacity of the node queue.
 */
dtNodeQueue::dtNodeQueue(int n) :
	m_heap(0),
	m_capacity(n),
	m_size(0)
{
	dtAssert(m_capacity > 0);

	m_heap = (dtNode**)dtAlloc(sizeof(dtNode*) * (m_capacity + 1), DT_ALLOC_PERM);
	dtAssert(m_heap);
}


/**
 * @brief Destructor for the dtNodeQueue class.
 *
 * This destructor frees the memory allocated for the node queue.
 */
dtNodeQueue::~dtNodeQueue()
{
	dtFree(m_heap);
}

/**
 * @brief Bubble up a node in the heap to maintain the heap property.
 *
 * This function moves a node up in the heap to maintain the min-heap property.
 *
 * @param i The index of the node to be moved up.
 * @param node The node to be moved.
 */
void dtNodeQueue::bubbleUp(int i, dtNode* node)
{
	int parent = (i - 1) / 2;
	// note: (index > 0) means there is a parent
	while ((i > 0) && (m_heap[parent]->total > node->total))
	{
		m_heap[i] = m_heap[parent];
		i = parent;
		parent = (i - 1) / 2;
	}
	m_heap[i] = node;
}

/**
 * @brief Trickle down a node in the heap to maintain the heap property.
 *
 * This function moves a node down in the heap to maintain the min-heap property.
 *
 * @param i The index of the node to be moved down.
 * @param node The node to be moved.
 */
void dtNodeQueue::trickleDown(int i, dtNode* node)
{
	int child = (i * 2) + 1;
	while (child < m_size)
	{
		if (((child + 1) < m_size) &&
			(m_heap[child]->total > m_heap[child + 1]->total))
		{
			child++;
		}
		m_heap[i] = m_heap[child];
		i = child;
		child = (i * 2) + 1;
	}
	bubbleUp(i, node);
}