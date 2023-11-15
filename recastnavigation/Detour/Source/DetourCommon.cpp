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
 * @file DetourCommon.cpp
 * @brief Common utility functions for Detour.
 */

#include "DetourCommon.h"
#include "DetourMath.h"

 //////////////////////////////////////////////////////////////////////////////////////////

 /**
  * @brief Find the closest point on a triangle to a given point.
  * @param[out] closest The closest point on the triangle.
  * @param[in] p The point to which the closest point is calculated.
  * @param[in] a The first vertex of the triangle.
  * @param[in] b The second vertex of the triangle.
  * @param[in] c The third vertex of the triangle.
  */
void dtClosestPtPointTriangle(float* closest, const float* p,
	const float* a, const float* b, const float* c)
{
	// Check if P in vertex region outside A
	float ab[3], ac[3], ap[3];
	dtVsub(ab, b, a);
	dtVsub(ac, c, a);
	dtVsub(ap, p, a);
	float d1 = dtVdot(ab, ap);
	float d2 = dtVdot(ac, ap);
	if (d1 <= 0.0f && d2 <= 0.0f)
	{
		// barycentric coordinates (1,0,0)
		dtVcopy(closest, a);
		return;
	}

	// Check if P in vertex region outside B
	float bp[3];
	dtVsub(bp, p, b);
	float d3 = dtVdot(ab, bp);
	float d4 = dtVdot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3)
	{
		// barycentric coordinates (0,1,0)
		dtVcopy(closest, b);
		return;
	}

	// Check if P in edge region of AB, if so return projection of P onto AB
	float vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
	{
		// barycentric coordinates (1-v,v,0)
		float v = d1 / (d1 - d3);
		closest[0] = a[0] + v * ab[0];
		closest[1] = a[1] + v * ab[1];
		closest[2] = a[2] + v * ab[2];
		return;
	}

	// Check if P in vertex region outside C
	float cp[3];
	dtVsub(cp, p, c);
	float d5 = dtVdot(ab, cp);
	float d6 = dtVdot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6)
	{
		// barycentric coordinates (0,0,1)
		dtVcopy(closest, c);
		return;
	}

	// Check if P in edge region of AC, if so return projection of P onto AC
	float vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
	{
		// barycentric coordinates (1-w,0,w)
		float w = d2 / (d2 - d6);
		closest[0] = a[0] + w * ac[0];
		closest[1] = a[1] + w * ac[1];
		closest[2] = a[2] + w * ac[2];
		return;
	}

	// Check if P in edge region of BC, if so return projection of P onto BC
	float va = d3 * d6 - d5 * d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
	{
		// barycentric coordinates (0,1-w,w)
		float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		closest[0] = b[0] + w * (c[0] - b[0]);
		closest[1] = b[1] + w * (c[1] - b[1]);
		closest[2] = b[2] + w * (c[2] - b[2]);
		return;
	}

	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	float denom = 1.0f / (va + vb + vc);
	float v = vb * denom;
	float w = vc * denom;
	closest[0] = a[0] + ab[0] * v + ac[0] * w;
	closest[1] = a[1] + ab[1] * v + ac[1] * w;
	closest[2] = a[2] + ab[2] * v + ac[2] * w;
}

/**
 * @brief Check if a line segment intersects a 2D polygon.
 * @param[in] p0 The start point of the line segment.
 * @param[in] p1 The end point of the line segment.
 * @param[in] verts The vertices of the polygon.
 * @param[in] nverts The number of vertices in the polygon.
 * @param[out] tmin The parameter tmin for the intersection point.
 * @param[out] tmax The parameter tmax for the intersection point.
 * @param[out] segMin The index of the segment where the intersection begins.
 * @param[out] segMax The index of the segment where the intersection ends.
 * @return True if the line segment intersects the polygon, false otherwise.
 */
bool dtIntersectSegmentPoly2D(const float* p0, const float* p1,
	const float* verts, int nverts,
	float& tmin, float& tmax,
	int& segMin, int& segMax)
{
	static const float EPS = 0.00000001f;

	tmin = 0;
	tmax = 1;
	segMin = -1;
	segMax = -1;

	float dir[3];
	dtVsub(dir, p1, p0);

	for (int i = 0, j = nverts - 1; i < nverts; j = i++)
	{
		float edge[3], diff[3];
		dtVsub(edge, &verts[i * 3], &verts[j * 3]);
		dtVsub(diff, p0, &verts[j * 3]);
		const float n = dtVperp2D(edge, diff);
		const float d = dtVperp2D(dir, edge);
		if (fabsf(d) < EPS)
		{
			// S is nearly parallel to this edge
			if (n < 0)
				return false;
			else
				continue;
		}
		const float t = n / d;
		if (d < 0)
		{
			// segment S is entering across this edge
			if (t > tmin)
			{
				tmin = t;
				segMin = j;
				// S enters after leaving polygon
				if (tmin > tmax)
					return false;
			}
		}
		else
		{
			// segment S is leaving across this edge
			if (t < tmax)
			{
				tmax = t;
				segMax = j;
				// S leaves before entering polygon
				if (tmax < tmin)
					return false;
			}
		}
	}

	return true;
}

/**
 * @brief Compute the squared distance from a point to a line segment in 2D.
 * @param[in] pt The point for which to find the squared distance.
 * @param[in] p The start point of the line segment.
 * @param[in] q The end point of the line segment.
 * @param[out] t The parameter 't' for the closest point on the segment.
 * @return The squared distance from the point to the line segment.
 */
float dtDistancePtSegSqr2D(const float* pt, const float* p, const float* q, float& t)
{
	float pqx = q[0] - p[0];
	float pqz = q[2] - p[2];
	float dx = pt[0] - p[0];
	float dz = pt[2] - p[2];
	float d = pqx * pqx + pqz * pqz;
	t = pqx * dx + pqz * dz;
	if (d > 0) t /= d;
	if (t < 0) t = 0;
	else if (t > 1) t = 1;
	dx = p[0] + t * pqx - pt[0];
	dz = p[2] + t * pqz - pt[2];
	return dx * dx + dz * dz;
}

/**
 * @brief Compute the center of a polygon given its vertices.
 * @param[out] tc The center of the polygon.
 * @param[in] idx The indices of the vertices that make up the polygon.
 * @param[in] nidx The number of vertices in the polygon.
 * @param[in] verts The vertices array.
 */
void dtCalcPolyCenter(float* tc, const unsigned short* idx, int nidx, const float* verts)
{
	tc[0] = 0.0f;
	tc[1] = 0.0f;
	tc[2] = 0.0f;
	for (int j = 0; j < nidx; ++j)
	{
		const float* v = &verts[idx[j] * 3];
		tc[0] += v[0];
		tc[1] += v[1];
		tc[2] += v[2];
	}
	const float s = 1.0f / nidx;
	tc[0] *= s;
	tc[1] *= s;
	tc[2] *= s;
}

/**
 * @brief Calculate the closest height of a point to a triangle.
 * @param[in] p The point for which to find the closest height.
 * @param[in] a The first vertex of the triangle.
 * @param[in] b The second vertex of the triangle.
 * @param[in] c The third vertex of the triangle.
 * @param[out] h The closest height of the point to the triangle.
 * @return True if the point is inside or on the triangle, false otherwise.
 */
bool dtClosestHeightPointTriangle(const float* p, const float* a, const float* b, const float* c, float& h)
{
	const float EPS = 1e-6f;
	float v0[3], v1[3], v2[3];

	dtVsub(v0, c, a);
	dtVsub(v1, b, a);
	dtVsub(v2, p, a);

	// Compute scaled barycentric coordinates
	float denom = v0[0] * v1[2] - v0[2] * v1[0];
	if (fabsf(denom) < EPS)
		return false;

	float u = v1[2] * v2[0] - v1[0] * v2[2];
	float v = v0[0] * v2[2] - v0[2] * v2[0];

	if (denom < 0) {
		denom = -denom;
		u = -u;
		v = -v;
	}

	// If point lies inside the triangle, return interpolated ycoord.
	if (u >= 0.0f && v >= 0.0f && (u + v) <= denom) {
		h = a[1] + (v0[1] * u + v1[1] * v) / denom;
		return true;
	}
	return false;
}

/**
 * @brief Check if a point is inside a 2D polygon.
 *
 * @par
 * All points are projected onto the xz-plane, so the y-values are ignored.
 *
 * @param[in] pt The point to check.
 * @param[in] verts The vertices of the polygon.
 * @param[in] nverts The number of vertices in the polygon.
 * @return True if the point is inside the polygon, false otherwise.
 */
bool dtPointInPolygon(const float* pt, const float* verts, const int nverts)
{
	// TODO: Replace pnpoly with triArea2D tests?
	int i, j;
	bool c = false;
	for (i = 0, j = nverts - 1; i < nverts; j = i++)
	{
		const float* vi = &verts[i * 3];
		const float* vj = &verts[j * 3];
		if (((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0] - vi[0]) * (pt[2] - vi[2]) / (vj[2] - vi[2]) + vi[0]))
			c = !c;
	}
	return c;
}

/**
 * @brief Compute the squared distance from a point to the edges of a polygon in 2D.
 *
 * @par
 * All points and edges are projected onto the xz-plane, so the y-values are ignored.
 *
 * @param[in] pt The point for which to compute distances.
 * @param[in] verts The vertices of the polygon.
 * @param[in] nverts The number of vertices in the polygon.
 * @param[out] ed An array to store the squared distances from the point to each edge.
 * @param[out] et An array to store the parameter 't' for the closest point on each edge.
 * @return True if the point is inside the polygon, false otherwise.
 */
bool dtDistancePtPolyEdgesSqr(const float* pt, const float* verts, const int nverts,
	float* ed, float* et)
{
	// TODO: Replace pnpoly with triArea2D tests?
	int i, j;
	bool c = false;
	for (i = 0, j = nverts - 1; i < nverts; j = i++)
	{
		const float* vi = &verts[i * 3];
		const float* vj = &verts[j * 3];
		if (((vi[2] > pt[2]) != (vj[2] > pt[2])) &&
			(pt[0] < (vj[0] - vi[0]) * (pt[2] - vi[2]) / (vj[2] - vi[2]) + vi[0]))
			c = !c;
		ed[j] = dtDistancePtSegSqr2D(pt, vj, vi, et[j]);
	}
	return c;
}

/**
 * @brief Project a polygon onto an axis and compute its range.
 *
 * @param[in] axis The axis to project onto.
 * @param[in] poly The vertices of the polygon.
 * @param[in] npoly The number of vertices in the polygon.
 * @param[out] rmin The minimum value of the projection.
 * @param[out] rmax The maximum value of the projection.
 */
static void projectPoly(const float* axis, const float* poly, const int npoly,
	float& rmin, float& rmax)
{
	rmin = rmax = dtVdot2D(axis, &poly[0]);
	for (int i = 1; i < npoly; ++i)
	{
		const float d = dtVdot2D(axis, &poly[i * 3]);
		rmin = dtMin(rmin, d);
		rmax = dtMax(rmax, d);
	}
}

/**
 * @brief Check if two ranges overlap within a given epsilon.
 *
 * @param[in] amin The minimum value of the first range.
 * @param[in] amax The maximum value of the first range.
 * @param[in] bmin The minimum value of the second range.
 * @param[in] bmax The maximum value of the second range.
 * @param[in] eps The epsilon value for comparison.
 * @return True if the ranges overlap, false otherwise.
 */
inline bool overlapRange(const float amin, const float amax,
	const float bmin, const float bmax,
	const float eps)
{
	return ((amin + eps) > bmax || (amax - eps) < bmin) ? false : true;
}

/**
 * @brief Check if two 2D convex polygons overlap.
 *
 * @par
 * All vertices are projected onto the xz-plane, so the y-values are ignored.
 *
 * @param[in] polya The vertices of the first polygon.
 * @param[in] npolya The number of vertices in the first polygon.
 * @param[in] polyb The vertices of the second polygon.
 * @param[in] npolyb The number of vertices in the second polygon.
 * @return True if the two polygons overlap, false otherwise.
 */
bool dtOverlapPolyPoly2D(const float* polya, const int npolya,
	const float* polyb, const int npolyb)
{
	const float eps = 1e-4f;

	for (int i = 0, j = npolya - 1; i < npolya; j = i++)
	{
		const float* va = &polya[j * 3];
		const float* vb = &polya[i * 3];
		const float n[3] = { vb[2] - va[2], 0, -(vb[0] - va[0]) };
		float amin, amax, bmin, bmax;
		projectPoly(n, polya, npolya, amin, amax);
		projectPoly(n, polyb, npolyb, bmin, bmax);
		if (!overlapRange(amin, amax, bmin, bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	for (int i = 0, j = npolyb - 1; i < npolyb; j = i++)
	{
		const float* va = &polyb[j * 3];
		const float* vb = &polyb[i * 3];
		const float n[3] = { vb[2] - va[2], 0, -(vb[0] - va[0]) };
		float amin, amax, bmin, bmax;
		projectPoly(n, polya, npolya, amin, amax);
		projectPoly(n, polyb, npolyb, bmin, bmax);
		if (!overlapRange(amin, amax, bmin, bmax, eps))
		{
			// Found separating axis
			return false;
		}
	}
	return true;
}

/**
 * @brief Returns a random point in a convex polygon.
 *
 * @par
 * Adapted from Graphics Gems article.
 *
 * @param[in] pts The vertices of the polygon.
 * @param[in] npts The number of vertices in the polygon.
 * @param[in,out] areas An array to store triangle areas.
 * @param[in] s The first random parameter (0 <= s <= 1).
 * @param[in] t The second random parameter (0 <= t <= 1).
 * @param[out] out The random point inside the polygon.
 */
void dtRandomPointInConvexPoly(const float* pts, const int npts, float* areas,
	const float s, const float t, float* out)
{
	// Calc triangle araes
	float areasum = 0.0f;
	for (int i = 2; i < npts; i++) {
		areas[i] = dtTriArea2D(&pts[0], &pts[(i - 1) * 3], &pts[i * 3]);
		areasum += dtMax(0.001f, areas[i]);
	}
	// Find sub triangle weighted by area.
	const float thr = s * areasum;
	float acc = 0.0f;
	float u = 1.0f;
	int tri = npts - 1;
	for (int i = 2; i < npts; i++) {
		const float dacc = areas[i];
		if (thr >= acc && thr < (acc + dacc))
		{
			u = (thr - acc) / dacc;
			tri = i;
			break;
		}
		acc += dacc;
	}

	float v = dtMathSqrtf(t);

	const float a = 1 - v;
	const float b = (1 - u) * v;
	const float c = u * v;
	const float* pa = &pts[0];
	const float* pb = &pts[(tri - 1) * 3];
	const float* pc = &pts[tri * 3];

	out[0] = a * pa[0] + b * pb[0] + c * pc[0];
	out[1] = a * pa[1] + b * pb[1] + c * pc[1];
	out[2] = a * pa[2] + b * pb[2] + c * pc[2];
}

/**
 * @brief Calculate the cross product in the xz-plane between two vectors.
 *
 * @param[in] a The first vector.
 * @param[in] b The second vector.
 * @return The result of the cross product in the xz-plane.
 */
inline float vperpXZ(const float* a, const float* b) { return a[0] * b[2] - a[2] * b[0]; }

/**
 * @brief Check if two 2D line segments intersect.
 *
 * @par
 * All vertices are projected onto the xz-plane, so the y-values are ignored.
 *
 * @param[in] ap The start point of the first line segment.
 * @param[in] aq The end point of the first line segment.
 * @param[in] bp The start point of the second line segment.
 * @param[in] bq The end point of the second line segment.
 * @param[out] s The parameter value of the intersection point on the first line segment.
 * @param[out] t The parameter value of the intersection point on the second line segment.
 * @return True if the two line segments intersect, false otherwise.
 */
bool dtIntersectSegSeg2D(const float* ap, const float* aq,
	const float* bp, const float* bq,
	float& s, float& t)
{
	float u[3], v[3], w[3];
	dtVsub(u, aq, ap);
	dtVsub(v, bq, bp);
	dtVsub(w, ap, bp);
	float d = vperpXZ(u, v);
	if (fabsf(d) < 1e-6f) return false;
	s = vperpXZ(v, w) / d;
	t = vperpXZ(u, w) / d;
	return true;
}