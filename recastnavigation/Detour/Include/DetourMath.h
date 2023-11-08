/**
@defgroup detour Detour

Members in this module are wrappers around the standard math library
*/

#ifndef DETOURMATH_H
#define DETOURMATH_H

#include <math.h>
// This include is required because libstdc++ has problems with isfinite
// if cmath is included before math.h.
#include <cmath>


/**
 * @brief Computes the absolute value of a floating-point number.
 * @param[in] x The input floating-point number.
 * @return The absolute value of the input number.
 */
inline float dtMathFabsf(float x) { return fabsf(x); }
/**
 * @brief Computes the square root of a floating-point number.
 * @param[in] x The input floating-point number.
 * @return The square root of the input number.
 */
inline float dtMathSqrtf(float x) { return sqrtf(x); }
/**
 * @brief Computes the largest integer less than or equal to a floating-point number.
 * @param[in] x The input floating-point number.
 * @return The largest integer less than or equal to the input number.
 */
inline float dtMathFloorf(float x) { return floorf(x); }
/**
 * @brief Computes the smallest integer greater than or equal to a floating-point number.
 * @param[in] x The input floating-point number.
 * @return The smallest integer greater than or equal to the input number.
 */
inline float dtMathCeilf(float x) { return ceilf(x); }
/**
 * @brief Computes the cosine of an angle.
 * @param[in] x The input angle in radians.
 * @return The cosine of the input angle.
 */
inline float dtMathCosf(float x) { return cosf(x); }
/**
 * @brief Computes the sine of an angle.
 * @param[in] x The input angle in radians.
 * @return The sine of the input angle.
 */
inline float dtMathSinf(float x) { return sinf(x); }
/**
 * @brief Computes the arctangent of the ratio of its arguments.
 * @param[in] y The numerator of the ratio.
 * @param[in] x The denominator of the ratio.
 * @return The arctangent of the ratio of y to x, in radians.
 */
inline float dtMathAtan2f(float y, float x) { return atan2f(y, x); }
/**
 * @brief Checks if a floating-point number is finite.
 * @param[in] x The input floating-point number.
 * @return True if the input number is finite, false otherwise.
 */
inline bool dtMathIsfinite(float x) { return std::isfinite(x); }

#endif
