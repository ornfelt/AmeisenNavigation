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
 * @file DetourAssert.h
 * @brief Custom assertion handling for Detour.
 */

#ifndef DETOURASSERT_H
#define DETOURASSERT_H

// Note: This header file's only purpose is to include define assert.
// Feel free to change the file and include your own implementation instead.

#ifdef NDEBUG

// From http://cnicholson.net/2009/02/stupid-c-tricks-adventures-in-assert/
/**
 * @def dtAssert(x)
 *
 * A macro for assertion in release (non-debug) builds.
 * In release builds, assertions are disabled, and this macro does nothing.
 *
 * @param x The expression to evaluate.
 */
#	define dtAssert(x) do { (void)sizeof(x); } while((void)(__LINE__==-1),false)

#else

/**
 * @brief An assertion failure function.
 *
 * This function is invoked when an assertion fails in debug builds.
 *
 * @param expression The asserted expression.
 * @param file The filename where the assertion failed.
 * @param line The line number where the assertion failed.
 * @see dtAssertFailSetCustom
 */
typedef void (dtAssertFailFunc)(const char* expression, const char* file, int line);

/**
 * @brief Sets the base custom assertion failure function to be used by Detour.
 *
 * This function sets the custom assertion failure function for Detour.
 *
 * @param assertFailFunc The function to be invoked in case of assertion failure in #dtAssert.
 */
void dtAssertFailSetCustom(dtAssertFailFunc* assertFailFunc);

/**
 * @brief Gets the base custom assertion failure function to be used by Detour.
 *
 * This function returns a pointer to the current custom assertion failure function.
 */
dtAssertFailFunc* dtAssertFailGetCustom();

#	include <assert.h>
/**
 * @def dtAssert(expression)
 *
 * A macro for assertion in debug builds.
 * In debug builds, assertions are enabled, and this macro checks the specified expression.
 * If the expression evaluates to false, the custom assertion failure function is invoked.
 *
 * @param expression The expression to evaluate.
 */
#	define dtAssert(expression) \
		{ \
			dtAssertFailFunc* failFunc = dtAssertFailGetCustom(); \
			if(failFunc == NULL) { assert(expression); } \
			else if(!(expression)) { (*failFunc)(#expression, __FILE__, __LINE__); } \
		}

#endif

#endif // DETOURASSERT_H
