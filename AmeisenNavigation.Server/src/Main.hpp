#pragma once

#include "AnTcpServer.hpp"

#include "../../AmeisenNavigation/src/AmeisenNavigation.hpp"

#include "Config/Config.hpp"
#include "Logging/AmeisenLogger.hpp"

#include <filesystem>
#include <iostream>
#include <mutex>

constexpr auto AMEISENNAV_VERSION = "1.8.0.0";

constexpr auto VEC3_SIZE = sizeof(float) * 3;

/**
 * @brief Enumeration representing different message types.
 */
enum class MessageType
{
	PATH, /**< @brief Message type for PATH. */
	MOVE_ALONG_SURFACE, /**< @brief Message type for MOVE_ALONG_SURFACE. */
	RANDOM_POINT, /**< @brief Message type for RANDOM_POINT. */
	RANDOM_POINT_AROUND, /**< @brief Message type for RANDOM_POINT_AROUND. */
	CAST_RAY, /**< @brief Message type for CAST_RAY. */
	RANDOM_PATH, /**< @brief Message type for RANDOM_PATH. */
};

/**
 * @brief Enumeration representing different path types.
 */
enum class PathType
{
	STRAIGHT, /**< @brief Straight path. */
	RANDOM, /**< @brief Random path. */
};

/**
 * @brief Flags for path requests.
 */
enum class PathRequestFlags : int
{
	NONE = 0, /**< @brief No flags. */
	CHAIKIN = 1, /**< @brief Flag for Chaikin smoothing. */
	CATMULLROM = 2, /**< @brief Flag for Catmull-Rom smoothing. */
};

/**
 * @brief Data structure for path requests.
 */
struct PathRequestData
{
	int mapId; /**< @brief Map ID. */
	int flags; /**< @brief Flags for path request. */
	float start[3]; /**< @brief Start position for the path. */
	float end[3]; /**< @brief End position for the path. */
};


/**
 * @brief Data structure for move requests.
 */
struct MoveRequestData
{
	int mapId; /**< @brief Map ID. */
	float start[3]; /**< @brief Start position for the move. */
	float end[3]; /**< @brief End position for the move. */
};

/**
 * @brief Data structure for ray casting requests.
 */
struct CastRayData
{
	int mapId; /**< @brief Map ID. */
	float start[3]; /**< @brief Start position for the ray cast. */
	float end[3]; /**< @brief End position for the ray cast. */
};

/**
 * @brief Data structure for random point generation requests around a point.
 */
struct RandomPointAroundData
{
	int mapId; /**< @brief Map ID. */
	float start[3]; /**< @brief Start position for random point generation. */
	float radius; /**< @brief Radius for generating random points around the start position. */
};

/**
 * @brief Pointer to the AnTcpServer instance.
 */
inline AnTcpServer* Server = nullptr;

/**
 * @brief Pointer to the AmeisenNavigation instance.
 */
inline AmeisenNavigation* Nav = nullptr;

/**
 * @brief Pointer to the AmeisenNavConfig instance.
 */
inline AmeisenNavConfig* Config = nullptr;

/**
 * @brief Map that associates client IDs with path buffers.
 */
inline std::unordered_map<int, std::pair<float*, float*>> ClientPathBuffers;

/**
 * @brief Signal handler for SIGINT.
 *
 * @param signal The signal code.
 * @return Returns 0.
 */
int __stdcall SigIntHandler(unsigned long signal);

/**
 * @brief Callback function for client connection.
 *
 * @param handler The client handler.
 */
void OnClientConnect(ClientHandler* handler) noexcept;
/**
 * @brief Callback function for client disconnection.
 *
 * @param handler The client handler.
 */
void OnClientDisconnect(ClientHandler* handler) noexcept;

/**
 * @brief Callback function for handling PATH messages.
 *
 * @param handler The client handler.
 * @param type The message type.
 * @param data The message data.
 * @param size The message size.
 */
void PathCallback(ClientHandler* handler, char type, const void* data, int size) noexcept;
/**
 * @brief Callback function for handling RANDOM_PATH messages.
 *
 * @param handler The client handler.
 * @param type The message type.
 * @param data The message data.
 * @param size The message size.
 */
void RandomPathCallback(ClientHandler* handler, char type, const void* data, int size) noexcept;
/**
 * @brief Callback function for handling RANDOM_POINT messages.
 *
 * @param handler The client handler.
 * @param type The message type.
 * @param data The message data.
 * @param size The message size.
 */
void RandomPointCallback(ClientHandler* handler, char type, const void* data, int size) noexcept;
/**
 * @brief Callback function for handling RANDOM_POINT_AROUND messages.
 *
 * @param handler The client handler.
 * @param type The message type.
 * @param data The message data.
 * @param size The message size.
 */
void RandomPointAroundCallback(ClientHandler* handler, char type, const void* data, int size) noexcept;
/**
 * @brief Callback function for handling MOVE_ALONG_SURFACE messages.
 *
 * @param handler The client handler.
 * @param type The message type.
 * @param data The message data.
 * @param size The message size.
 */
void MoveAlongSurfaceCallback(ClientHandler* handler, char type, const void* data, int size) noexcept;
/**
 * @brief Callback function for handling CAST_RAY messages.
 *
 * @param handler The client handler.
 * @param type The message type.
 * @param data The message data.
 * @param size The message size.
 */
void CastRayCallback(ClientHandler* handler, char type, const void* data, int size) noexcept;

/**
 * @brief Generic callback function for handling path-related messages.
 *
 * @param handler The client handler.
 * @param type The message type.
 * @param data The message data.
 * @param size The message size.
 * @param pathType The type of path being handled.
 */
void GenericPathCallback(ClientHandler* handler, char type, const void* data, int size, PathType pathType) noexcept;
