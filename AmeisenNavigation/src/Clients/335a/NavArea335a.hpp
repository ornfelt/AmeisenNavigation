#pragma once

/**
 * @enum NavTerrainFlag335a
 * Represents terrain flags for navigation in a 3.3.5a client.
 */
enum class NavTerrainFlag335a : char
{
	EMPTY = 0,         /**< Empty terrain. */
	GROUND = 11,       /**< Ground terrain. */
	GROUND_STEEP = 10, /**< Steep ground terrain. */
	WATER = 9,         /**< Water terrain. */
	MAGMA_SLIME = 8,   /**< Magma slime terrain. */
};

/**
 * @enum NavArea335a
 * Represents navigation areas based on terrain flags for a 3.3.5a client.
 */
enum class NavArea335a : char
{
	EMPTY = 0x00, /**< Empty navigation area. */
	GROUND = 1 << (static_cast<char>(NavTerrainFlag335a::GROUND) - static_cast<char>(NavTerrainFlag335a::GROUND)), /**< Ground navigation area. */
	GROUND_STEEP = 1 << (static_cast<char>(NavTerrainFlag335a::GROUND) - static_cast<char>(NavTerrainFlag335a::GROUND_STEEP)), /**< Steep ground navigation area. */
	WATER = 1 << (static_cast<char>(NavTerrainFlag335a::GROUND) - static_cast<char>(NavTerrainFlag335a::WATER)), /**< Water navigation area. */
	MAGMA_SLIME = 1 << (static_cast<char>(NavTerrainFlag335a::GROUND) - static_cast<char>(NavTerrainFlag335a::MAGMA_SLIME)) /**< Magma slime navigation area. */
};
