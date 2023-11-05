#pragma once

/**
 * @enum NavArea548
 * Represents navigation areas for a 5.4.8 client.
 */
enum class NavArea548 : char
{
	EMPTY = 0,   /**< Empty navigation area. */
	GROUND = 1,  /**< Ground navigation area. */
	MAGMA = 2,   /**< Magma navigation area. */
	SLIME = 4,   /**< Slime navigation area. */
	WATER = 8    /**< Water navigation area. */
};
