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

#ifndef DETOURNODE_H
#define DETOURNODE_H

#include "DetourNavMesh.h"

/**
 * @brief Flags used to define the state of a navigation node.
 */
enum dtNodeFlags
{
    DT_NODE_OPEN = 0x01,                   ///< Node is open and can be visited.
    DT_NODE_CLOSED = 0x02,                 ///< Node is closed and cannot be visited.
    DT_NODE_PARENT_DETACHED = 0x04         ///< Parent of the node is not adjacent, found using raycast.
};

/**
 * @brief Type definition for the index of a navigation node.
 */
typedef unsigned short dtNodeIndex;

/// Special value for an invalid node index.
static const dtNodeIndex DT_NULL_IDX = (dtNodeIndex)~0;
/// Number of bits used to represent parent node index.
static const int DT_NODE_PARENT_BITS = 24;
/// Number of bits used to represent node state.
static const int DT_NODE_STATE_BITS = 2;

/**
 * @struct dtNode
 * @brief Represents a navigation node in the navigation mesh.
 */
struct dtNode
{
    float pos[3];								///< Position of the node.
    float cost;									///< Cost from previous node to current node.
    float total;								///< Cost up to the node.
    unsigned int pidx : DT_NODE_PARENT_BITS;	///< Index to parent node.
    unsigned int state : DT_NODE_STATE_BITS;	///< extra state information. A polyRef can have multiple nodes with different extra info. see DT_MAX_STATES_PER_NODE
    unsigned int flags : 3;						///< Node flags. A combination of dtNodeFlags.
    dtPolyRef id;								///< Polygon ref the node corresponds to.
};

/// Maximum number of extra states per node. See dtNode::state.
static const int DT_MAX_STATES_PER_NODE = 1 << DT_NODE_STATE_BITS;

/**
 * @class dtNodePool
 * @brief Represents a pool of navigation nodes.
 */
class dtNodePool
{
public:
	/**
     * @brief Constructor for the dtNodePool class.
     * @param[in] maxNodes The maximum number of nodes in the pool.
     * @param[in] hashSize The size of the hash table used for node lookup.
     */
    dtNodePool(int maxNodes, int hashSize);
	/**
     * @brief Destructor for the dtNodePool class.
     */
    ~dtNodePool();
	/**
     * @brief Clears the node pool, releasing all nodes.
     */
    void clear();

	/**
     * @brief Gets a dtNode by reference and extra state information. Allocates a new node if not found.
     *
     * There can be more than one node for the same polyRef but with different extra state information.
     *
     * @param[in] id The polygon reference for which to get the node.
     * @param[in] state Extra state information for the node (default is 0).
     * @return A pointer to the retrieved or allocated node.
     */
    dtNode* getNode(dtPolyRef id, unsigned char state = 0);
	/**
     * @brief Finds a dtNode by polyRef and extra state information.
     * @param[in] id The polygon reference for which to find the node.
     * @param[in] state Extra state information for the node.
     * @return A pointer to the found node, or nullptr if not found.
     */
    dtNode* findNode(dtPolyRef id, unsigned char state);
	/**
     * @brief Finds multiple nodes with the same polyRef but different extra state information.
     * @param[in] id The polygon reference for which to find nodes.
     * @param[out] nodes An array to store the found nodes.
     * @param[in] maxNodes The maximum number of nodes to find and store.
     * @return The number of nodes found and stored in the 'nodes' array.
     */
    unsigned int findNodes(dtPolyRef id, dtNode** nodes, const int maxNodes);

	/**
     * @brief Gets the index of a node within the pool.
     * @param[in] node A pointer to the node.
     * @return The index of the node within the pool.
     */
    inline unsigned int getNodeIdx(const dtNode* node) const
    {
        if (!node) return 0;
        return (unsigned int)(node - m_nodes) + 1;
    }

	/**
     * @brief Gets a node at a specific index within the pool.
     * @param[in] idx The index of the node to retrieve.
     * @return A pointer to the node at the specified index.
     */
    inline dtNode* getNodeAtIdx(unsigned int idx)
    {
        if (!idx) return 0;
        return &m_nodes[idx - 1];
    }

	/**
     * @brief Gets a constant node at a specific index within the pool.
     * @param[in] idx The index of the node to retrieve.
     * @return A constant pointer to the node at the specified index.
     */
    inline const dtNode* getNodeAtIdx(unsigned int idx) const
    {
        if (!idx) return 0;
        return &m_nodes[idx - 1];
    }

	/**
     * @brief Gets the memory used by the node pool.
     * @return The memory used by the node pool in bytes.
     */
    inline int getMemUsed() const
    {
        return sizeof(*this) +
            sizeof(dtNode) * m_maxNodes +
            sizeof(dtNodeIndex) * m_maxNodes +
            sizeof(dtNodeIndex) * m_hashSize;
    }

	/**
     * @brief Gets the maximum number of nodes in the pool.
     * @return The maximum number of nodes.
     */
    inline int getMaxNodes() const { return m_maxNodes; }

	/**
     * @brief Gets the size of the hash table used for node lookup.
     * @return The size of the hash table.
     */
    inline int getHashSize() const { return m_hashSize; }

    /**
     * @brief Gets the index of the first node in a specific bucket of the hash table.
     * @param[in] bucket The index of the bucket.
     * @return The index of the first node in the specified bucket.
     */
    inline dtNodeIndex getFirst(int bucket) const { return m_first[bucket]; }

    /**
     * @brief Gets the index of the next node in the pool.
     * @param[in] i The index of the current node.
     * @return The index of the next node in the pool.
     */
    inline dtNodeIndex getNext(int i) const { return m_next[i]; }

    /**
     * @brief Gets the number of nodes currently in the pool.
     * @return The number of nodes in the pool.
     */
    inline int getNodeCount() const { return m_nodeCount; }

private:
    // Explicitly disabled copy constructor and copy assignment operator.
    dtNodePool(const dtNodePool&);
    dtNodePool& operator=(const dtNodePool&);

    dtNode* m_nodes;
    dtNodeIndex* m_first;
    dtNodeIndex* m_next;
    const int m_maxNodes;
    const int m_hashSize;
    int m_nodeCount;
};

/**
 * @class dtNodeQueue
 * @brief Represents a priority queue of navigation nodes.
 */
class dtNodeQueue
{
public:
	/**
     * @brief Constructor for the dtNodeQueue class.
     * @param[in] n The capacity of the queue.
     */
    dtNodeQueue(int n);

    /**
     * @brief Destructor for the dtNodeQueue class.
     */
    ~dtNodeQueue();

	/**
     * @brief Clears the queue, resetting its size to zero.
     */
    inline void clear() { m_size = 0; }

	/**
     * @brief Gets the node at the top of the queue.
     * @return A pointer to the node at the top of the queue.
     */
    inline dtNode* top() { return m_heap[0]; }

	/**
     * @brief Pops the top node from the queue.
     * @return A pointer to the popped node.
     */
    inline dtNode* pop()
    {
        dtNode* result = m_heap[0];
        m_size--;
        trickleDown(0, m_heap[m_size]);
        return result;
    }

	/**
     * @brief Pushes a node onto the queue.
     * @param[in] node A pointer to the node to push onto the queue.
     */
    inline void push(dtNode* node)
    {
        m_size++;
        bubbleUp(m_size - 1, node);
    }

	/**
     * @brief Modifies the position of a node in the queue.
     * @param[in] node A pointer to the node to modify.
     */
    inline void modify(dtNode* node)
    {
        for (int i = 0; i < m_size; ++i)
        {
            if (m_heap[i] == node)
            {
                bubbleUp(i, node);
                return;
            }
        }
    }

	/**
     * @brief Checks if the queue is empty.
     * @return True if the queue is empty, false otherwise.
     */
    inline bool empty() const { return m_size == 0; }

	/**
     * @brief Gets the memory used by the node queue.
     * @return The memory used by the node queue in bytes.
     */
    inline int getMemUsed() const
    {
        return sizeof(*this) +
            sizeof(dtNode*) * (m_capacity + 1);
    }

	/**
     * @brief Gets the capacity of the queue.
     * @return The capacity of the queue.
     */
    inline int getCapacity() const { return m_capacity; }

private:
    // Explicitly disabled copy constructor and copy assignment operator.
    dtNodeQueue(const dtNodeQueue&);
    dtNodeQueue& operator=(const dtNodeQueue&);

	/**
     * @brief Bubbles up a node in the queue.
     * @param[in] i The index of the node to bubble up.
     * @param[in] node A pointer to the node to bubble up.
     */
    void bubbleUp(int i, dtNode* node);
	/**
     * @brief Trickles down a node in the queue.
     * @param[in] i The index of the node to trickle down.
     * @param[in] node A pointer to the node to trickle down.
     */
    void trickleDown(int i, dtNode* node);

	dtNode** m_heap; ///< The array representing the binary heap.
    const int m_capacity; ///< The capacity of the queue.
    int m_size; ///< The current size of the queue.
};

#endif // DETOURNODE_H
