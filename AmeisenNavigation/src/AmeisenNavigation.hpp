#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include "../../recastnavigation/Detour/Include/DetourCommon.h"
#include "../../recastnavigation/Detour/Include/DetourNavMesh.h"
#include "../../recastnavigation/Detour/Include/DetourNavMeshQuery.h"

#include "Clients/AmeisenNavClient.hpp"

#ifdef _DEBUG
#define ANAV_DEBUG_ONLY(x) x
#define ANAV_ERROR_MSG(x) x
#else
#define ANAV_DEBUG_ONLY(x)
#define ANAV_ERROR_MSG(x)
#endif

/// Magic number for MMAP format.
constexpr auto MMAP_MAGIC = 0x4D4D4150;
/// Version number for MMAP format.
constexpr auto MMAP_VERSION = 15;

constexpr auto MAX_RAND_F = static_cast<float>(RAND_MAX);
/**
 * @brief Generates a random floating-point number between 0 and 1.
 *
 * @return A random floating-point number between 0 and 1.
 */
inline float GetRandomFloat() { return static_cast<float>(rand()) / MAX_RAND_F; };

/**
 * @brief Macro to print all values of a vector3 in the format "[x, y, z]".
 *
 * @param vec The vector3 to be printed.
 */
#define PRINT_VEC3(vec) "[" << vec[0] << ", " << vec[1] << ", " << vec[2] << "]"

 /**
  * @brief Enumerates the possible MMAP format types.
  */
enum class MMAP_FORMAT
{
	UNKNOWN,
	TC335A,
	SF548
};

/**
 * @brief Represents the header structure for an MMAP tile.
 */
struct MmapTileHeader
{
	unsigned int mmapMagic;
	unsigned int dtVersion;
	unsigned int mmapVersion;
	unsigned int size;
	char usesLiquids;
	char padding[3];
};

/**
 * @class AmeisenNavigation
 * @brief Class for handling pathfinding and navigation.
 */
class AmeisenNavigation
{
private:
	std::string MmapFolder; ///< Folder where the MMAPs are stored.
	int MaxPathNodes; ///< Maximum number of nodes a path can have.
	int MaxSearchNodes; ///< Maximum number of nodes detour searches for.

	std::unordered_map<int, std::pair<std::mutex, dtNavMesh*>> NavMeshMap; ///< Map of NavMeshes for different maps.
	std::unordered_map<int, AmeisenNavClient*> Clients; ///< Map of registered clients.

public:
	/**
	 * @brief Constructor for AmeisenNavigation class.
	 *
	 * @param mmapFolder Folder where the MMAPs are stored.
	 * @param maxPathNodes Maximum number of nodes a path can have.
	 * @param maxSearchNodes Maximum number of nodes detour searches for.
	 */
	AmeisenNavigation(const std::string& mmapFolder, int maxPathNodes, int maxSearchNodes)
		: MmapFolder(mmapFolder),
		MaxPathNodes(maxPathNodes),
		MaxSearchNodes(maxSearchNodes),
		NavMeshMap(),
		Clients()
	{
		// seed the random generator
		srand(static_cast<unsigned int>(time(0)));
	}

	/**
	 * @brief Destructor for AmeisenNavigation.
	 * Cleans up memory and resources.
	 */
	~AmeisenNavigation()
	{
		for (const auto& client : Clients)
		{
			if (client.second)
			{
				delete client.second;
			}
		}

		for (auto& client : NavMeshMap)
		{
			if (client.second.second)
			{
				const std::lock_guard<std::mutex> lock(client.second.first);
				dtFreeNavMesh(client.second.second);
			}
		}
	}

	/**
	 * @brief Copy constructor is deleted to prevent copying.
	 */
	AmeisenNavigation(const AmeisenNavigation&) = delete;
	/**
	 * @brief Assignment operator is deleted to prevent assignment.
	 */
	AmeisenNavigation& operator=(const AmeisenNavigation&) = delete;

	/**
	 * @brief Call this to register a new client.
	 *
	 * @param id Unique id for the client.
	 * @param version Version of the client.
	 */
	void NewClient(int id, CLIENT_VERSION version) noexcept;

	/**
	 * @brief Call this to free a client.
	 *
	 * @param id Unique id of the client.
	 */
	void FreeClient(int id) noexcept;

	/**
	 * @brief Try to find a path from start to end.
	 *
	 * @param clientId Id of the client to run this on.
	 * @param mapId The map id to search a path on.
	 * @param startPosition Start position vector.
	 * @param endPosition End position vector.
	 * @param path Path buffer.
	 * @param pathSize The path's size.
	 * @return True when a path was found, false if not.
	 */
	bool GetPath(int clientId, int mapId, const float* startPosition, const float* endPosition, float* path, int* pathSize) noexcept;

	/**
	 * @brief Try to find a path from start to end but randomize the final positions by x meters.
	 *
	 * @param clientId Id of the client to run this on.
	 * @param mapId The map id to search a path on.
	 * @param startPosition Start position vector.
	 * @param endPosition End position vector.
	 * @param path Path buffer.
	 * @param pathSize The path's size.
	 * @param maxRandomDistance Max distance to the original position.
	 * @return True when a path was found, false if not.
	 */
	bool GetRandomPath(int clientId, int mapId, const float* startPosition, const float* endPosition, float* path, int* pathSize, float maxRandomDistance) noexcept;

	/**
	 * @brief Try to move towards a specific location.
	 *
	 * @param clientId Id of the client to run this on.
	 * @param mapId The map id to search a path on.
	 * @param startPosition Start position vector.
	 * @param endPosition End position vector.
	 * @param positionToGoTo Target position.
	 * @return True when a path was found, false if not.
	 */
	bool MoveAlongSurface(int clientId, int mapId, const float* startPosition, const float* endPosition, float* positionToGoTo) noexcept;

	/**
	 * @brief Get a random point anywhere on the map.
	 *
	 * @param clientId Id of the client to run this on.
	 * @param mapId The map id to search a path on.
	 * @param position Random position.
	 * @return True when a point has been found, false if not.
	 */
	bool GetRandomPoint(int clientId, int mapId, float* position) noexcept;

	/**
	 * @brief Get a random point within x meters of the start position.
	 *
	 * @param clientId Id of the client to run this on.
	 * @param mapId The map id to search a path on.
	 * @param startPosition Start position vector.
	 * @param radius Max distance to search for a random position.
	 * @param position Random position.
	 * @return True when a point has been found, false if not.
	 */
	bool GetRandomPointAround(int clientId, int mapId, const float* startPosition, float radius, float* position) noexcept;

	/**
	 * @brief Cast a movement ray along the mesh and see whether it collides with a wall or not.
	 *
	 * @param clientId Id of the client to run this on.
	 * @param mapId The map id to search a path on.
	 * @param startPosition Start position vector.
	 * @param endPosition End position vector.
	 * @param raycastHit Detour raycast result.
	 * @return True when the ray hit no wall, false if it did.
	 */
	bool CastMovementRay(int clientId, int mapId, const float* startPosition, const float* endPosition, dtRaycastHit* raycastHit) noexcept;

	/**
	 * @brief Smooth a path using the chaikin-curve algorithm.
	 *
	 * @param input Input path.
	 * @param inputSize Input path size.
	 * @param output Output path.
	 * @param outputSize Output path's size.
	 */
	void SmoothPathChaikinCurve(const float* input, int inputSize, float* output, int* outputSize) const noexcept;

	/**
	 * @brief Smooth a path using the catmull-rom-spline algorithm.
	 *
	 * @param input Input path.
	 * @param inputSize Input path size.
	 * @param output Output path.
	 * @param outputSize Output path's size.
	 * @param points How many points to generate, the more the smoother the path will be.
	 * @param alpha Alpha to use for the catmull-rom-spline algorithm.
	 */
	void SmoothPathCatmullRom(const float* input, int inputSize, float* output, int* outputSize, int points, float alpha) const noexcept;

	/**
	 * @brief Load the MMAPs for a map.
	 *
	 * @param mapId Id of the mmaps to load.
	 * @return True if loaded, false if something went wrong.
	 */
	bool LoadMmaps(int mapId) noexcept;

private:
	/**
	 * @brief Try to find the nearest polygon for a given position.
	 *
	 * @param clientId Id of the client to run this on.
	 * @param mapId The map id to search a path on.
	 * @param position Current position.
	 * @param closestPointOnPoly Closest point on the found polygon.
	 * @return Reference to the found polygon if found, else 0.
	 */
	inline dtPolyRef GetNearestPoly(int clientId, int mapId, float* position, float* closestPointOnPoly) const noexcept
	{
		dtPolyRef polyRef;
		float extents[3] = { 6.0f, 6.0f, 6.0f };
		const auto& client = Clients.at(clientId);
		bool result = dtStatusSucceed(client->GetNavmeshQuery(mapId)->findNearestPoly(position, extents, &client->QueryFilter(), &polyRef, closestPointOnPoly));
		return result ? polyRef : 0;
	}

	/**
	 * @brief Helper function to insert a vector3 into a float buffer.
	 *
	 * @param target Target float buffer.
	 * @param index Index where the vector3 should be inserted.
	 * @param vec Source vector3.
	 * @param offset Offset of the source vector3.
	 */
	inline void InsertVector3(float* target, int& index, const float* vec, int offset) const noexcept
	{
		target[index] = vec[offset + 0];
		target[index + 1] = vec[offset + 1];
		target[index + 2] = vec[offset + 2];
		index += 3;
	}

	/**
	 * @brief Helper function to scale two vectors and add them.
	 * Used by the smoothing algorithms.
	 *
	 * @param vec0 First vector to scale.
	 * @param fac0 Scaling factor for the first vector.
	 * @param vec1 Second vector to scale.
	 * @param fac1 Scaling factor for the second vector.
	 * @param s0 Output for the scaled first vector.
	 * @param s1 Output for the scaled second vector.
	 * @param output Output vector for the sum of the scaled vectors.
	 */
	inline void ScaleAndAddVector3(const float* vec0, float fac0, const float* vec1, float fac1, float* s0, float* s1, float* output) const noexcept
	{
		dtVscale(s0, vec0, fac0);
		dtVscale(s1, vec1, fac1);
		dtVadd(output, s0, s1);
	}

	/**
	 * @brief Convert the Recast and Detour coordinates to WoW coordinates.
	 *
	 * @param pos Position to convert.
	 */
	inline void RDToWowCoords(float* pos) const noexcept
	{
		float oz = pos[2];
		pos[2] = pos[1];
		pos[1] = pos[0];
		pos[0] = oz;
	}

	/**
	 * @brief Convert the WoW coordinates to Recast and Detour coordinates.
	 *
	 * @param pos Position to convert.
	 */
	inline void WowToRDCoords(float* pos) const noexcept
	{
		float ox = pos[0];
		pos[0] = pos[1];
		pos[1] = pos[2];
		pos[2] = ox;
	}

	/**
	 * @brief Check if the MMAPs are loaded for the given mapId.
	 *
	 * @param mapId The map id to check.
	 * @return True if MMAPs are loaded, false otherwise.
	 */
	inline bool IsMmapLoaded(int mapId) noexcept { return NavMeshMap[mapId].second != nullptr; }

	/**
	 * @brief Check if the given client id is valid.
	 *
	 * @param clientId The client id to check.
	 * @return True if the client id is valid, false otherwise.
	 */
	inline bool IsValidClient(int clientId) noexcept { return Clients[clientId] != nullptr; }

	/**
	 * @brief Initialize a NavMeshQuery for the given client and load MMAPs.
	 *
	 * @param clientId The client id for which to initialize the query.
	 * @param mapId The map id for which to load MMAPs.
	 * @return True if initialization and loading were successful, false otherwise.
	 */
	bool InitQueryAndLoadMmaps(int clientId, int mapId) noexcept;

	/**
	 * @brief Used by the GetPath and GetRandomPath methods to generate a path.
	 *
	 * @param clientId The client id for which to generate the path.
	 * @param mapId The map id on which to generate the path.
	 * @param startPosition Start position vector.
	 * @param endPosition End position vector.
	 * @param path Path buffer to store the generated path.
	 * @param pathSize Size of the generated path.
	 * @param visited Output for visited polygon references (optional).
	 * @return True when a path was generated, false if not.
	 */
	bool CalculateNormalPath(int clientId, int mapId, const float* startPosition, const float* endPosition, float* path, int* pathSize, dtPolyRef* visited = nullptr) noexcept;

	/**
	 * @brief Used to detect the MMAP file format.
	 *
	 * @return Detected MMAP format or MMAP_FORMAT::UNKNOWN.
	 */
	MMAP_FORMAT TryDetectMmapFormat() noexcept;
};
