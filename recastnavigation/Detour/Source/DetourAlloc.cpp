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

/**
 * @file DetourAlloc.cpp
 * @brief Memory allocation functions for Detour.
 */

#include <stdlib.h>
#include "DetourAlloc.h"

 /**
  * @brief Default memory allocation function.
  * @param size The size of memory to allocate.
  * @param hint The allocation hint.
  * @return A pointer to the allocated memory block.
  */
static void* dtAllocDefault(size_t size, dtAllocHint)
{
	return malloc(size);
}

/**
 * @brief Default memory deallocation function.
 * @param ptr A pointer to the memory block to deallocate.
 */
static void dtFreeDefault(void* ptr)
{
	free(ptr);
}

static dtAllocFunc* sAllocFunc = dtAllocDefault; ///< Pointer to the custom allocation function.
static dtFreeFunc* sFreeFunc = dtFreeDefault;   ///< Pointer to the custom deallocation function.

/**
 * @brief Set custom memory allocation and deallocation functions.
 * @param allocFunc A pointer to the custom allocation function. Pass nullptr to use the default.
 * @param freeFunc A pointer to the custom deallocation function. Pass nullptr to use the default.
 */
void dtAllocSetCustom(dtAllocFunc* allocFunc, dtFreeFunc* freeFunc)
{
	sAllocFunc = allocFunc ? allocFunc : dtAllocDefault;
	sFreeFunc = freeFunc ? freeFunc : dtFreeDefault;
}

/**
 * @brief Allocate memory with a specified size and hint.
 * @param size The size of memory to allocate.
 * @param hint The allocation hint.
 * @return A pointer to the allocated memory block.
 */
void* dtAlloc(size_t size, dtAllocHint hint)
{
	return sAllocFunc(size, hint);
}

/**
 * @brief Deallocate memory previously allocated using dtAlloc.
 * @param ptr A pointer to the memory block to deallocate.
 */
void dtFree(void* ptr)
{
	if (ptr)
		sFreeFunc(ptr);
}