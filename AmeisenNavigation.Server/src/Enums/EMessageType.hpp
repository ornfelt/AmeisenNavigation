#pragma once

/**
 * @brief Enumeration representing different message types.
 */
enum class EMessageType : char
{
	PATH, /**< @brief Message type for PATH. */
	MOVE_ALONG_SURFACE, /**< @brief Message type for MOVE_ALONG_SURFACE. */
	RANDOM_POINT, /**< @brief Message type for RANDOM_POINT. */
	RANDOM_POINT_AROUND, /**< @brief Message type for RANDOM_POINT_AROUND. */
};