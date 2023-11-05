#pragma once

#include "../../recastnavigation/Detour/Include/DetourCommon.h"
#include "../../recastnavigation/Detour/Include/DetourNavMeshQuery.h"

#include "335a/NavArea335a.hpp"
#include "548/NavArea548.hpp"

#include "ClientVersion.hpp"

/**
 * @brief A class representing a client for Ameisen navigation.
 */
class AmeisenNavClient
{
	int Id; /**< The ID of the client. */
	CLIENT_VERSION CVersion; /**< The client version. */
	dtQueryFilter Filter; /**< The navigation query filter. */
	std::unordered_map<int, dtNavMeshQuery*> NavMeshQuery; /**< A map of map IDs to navigation mesh queries. */
	size_t BufferSize; /**< The size of the path buffer. */
	dtPolyRef* PolyPathBuffer; /**< The buffer for storing polygon paths. */
	dtPolyRef* MiscPathBuffer; /**< The buffer for storing miscellaneous paths. */

public:
	/**
	 * @brief Constructor for AmeisenNavClient.
	 *
	 * @param id The ID of the client.
	 * @param version The client version.
	 * @param polypathBufferSize The size of the polygon path buffer.
	 */
	AmeisenNavClient(int id, CLIENT_VERSION version, size_t polypathBufferSize)
		: Id(id),
		CVersion(version),
		BufferSize(polypathBufferSize),
		NavMeshQuery(),
		PolyPathBuffer(new dtPolyRef[polypathBufferSize]),
		MiscPathBuffer(nullptr),
		Filter()
	{
		switch (version)
		{
		case CLIENT_VERSION::V335A:
			Filter.setIncludeFlags(static_cast<char>(NavArea335a::GROUND) | static_cast<char>(NavArea335a::WATER));
			Filter.setExcludeFlags(static_cast<char>(NavArea335a::EMPTY) | static_cast<char>(NavArea335a::GROUND_STEEP) | static_cast<char>(NavArea335a::MAGMA_SLIME));
			break;

		case CLIENT_VERSION::V548:
			Filter.setIncludeFlags(static_cast<char>(NavArea548::GROUND) | static_cast<char>(NavArea548::WATER));
			Filter.setExcludeFlags(static_cast<char>(NavArea548::EMPTY) | static_cast<char>(NavArea548::MAGMA) | static_cast<char>(NavArea548::SLIME));
			break;

		default:
			break;
		}
	}

	/**
	 * @brief Destructor for AmeisenNavClient.
	 */
	~AmeisenNavClient()
	{
		for (const auto& query : NavMeshQuery)
		{
			dtFreeNavMeshQuery(query.second);
		}

		delete[] PolyPathBuffer;

		if (MiscPathBuffer)
		{
			delete[] MiscPathBuffer;
		}
	}

	AmeisenNavClient(const AmeisenNavClient&) = delete;
	AmeisenNavClient& operator=(const AmeisenNavClient&) = delete;

	/**
	 * @brief Get the ID of the client.
	 *
	 * @return The client's ID.
	 */
	inline int GetId() noexcept { return Id; }

	/**
	 * @brief Get the navigation query filter.
	 *
	 * @return The navigation query filter.
	 */
	inline dtQueryFilter& QueryFilter() noexcept { return Filter; }

	/**
	 * @brief Get the navigation mesh query for a specific map.
	 *
	 * @param mapId The ID of the map.
	 * @return The navigation mesh query for the specified map.
	 */
	inline dtNavMeshQuery* GetNavmeshQuery(int mapId) noexcept { return NavMeshQuery[mapId]; }

	/**
	 * @brief Get the polygon path buffer.
	 *
	 * @return The polygon path buffer.
	 */
	inline dtPolyRef* GetPolyPathBuffer() noexcept { return PolyPathBuffer; }

	/**
	 * @brief Get the miscellaneous path buffer.
	 *
	 * @return The miscellaneous path buffer.
	 */
	inline dtPolyRef* GetMiscPathBuffer() noexcept { return MiscPathBuffer ? MiscPathBuffer : MiscPathBuffer = new dtPolyRef[BufferSize]; }

	/**
	 * @brief Set the navigation mesh query for a specific map.
	 *
	 * @param mapId The ID of the map.
	 * @param query The navigation mesh query for the specified map.
	 */
	inline void SetNavmeshQuery(int mapId, dtNavMeshQuery* query) noexcept { NavMeshQuery[mapId] = query; }
};