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

#ifndef DETOUROBSTACLEAVOIDANCE_H
#define DETOUROBSTACLEAVOIDANCE_H

/**
 * @struct dtObstacleCircle
 * @brief Represents a circular obstacle for obstacle avoidance.
 */
struct dtObstacleCircle
{
	float p[3];				///< Position of the obstacle
	float vel[3];			///< Velocity of the obstacle
	float dvel[3];			///< Velocity of the obstacle
	float rad;				///< Radius of the obstacle
	float dp[3], np[3];		///< Use for side selection during sampling.
};

/**
 * @struct dtObstacleSegment
 * @brief Represents a line segment obstacle for obstacle avoidance.
 */
struct dtObstacleSegment
{
	float p[3], q[3];		///< End points of the obstacle segment
	bool touch;
};

/**
 * @class dtObstacleAvoidanceDebugData
 * @brief Debug data for obstacle avoidance queries.
 */
class dtObstacleAvoidanceDebugData
{
public:
	/**
	 * @brief Constructor for dtObstacleAvoidanceDebugData.
	 */
	dtObstacleAvoidanceDebugData();
	/**
	 * @brief Destructor for dtObstacleAvoidanceDebugData.
	 */
	~dtObstacleAvoidanceDebugData();

	/**
	 * @brief Initializes the debug data with a maximum number of samples.
	 * @param maxSamples The maximum number of samples to allocate space for.
	 * @return True if initialization was successful, false otherwise.
	 */
	bool init(const int maxSamples);
	/**
	 * @brief Resets the debug data to its initial state.
	 */
	void reset();
	/**
	 * @brief Adds a sample to the debug data.
	 * @param vel The velocity sample.
	 * @param ssize The sample size.
	 * @param pen The penalty value.
	 * @param vpen The desired velocity penalty.
	 * @param vcpen The current velocity penalty.
	 * @param spen The preferred side penalty.
	 * @param tpen The collision time penalty.
	 */
	void addSample(const float* vel, const float ssize, const float pen,
		const float vpen, const float vcpen, const float spen, const float tpen);

	/**
	 * @brief Normalizes the samples in the debug data.
	 */
	void normalizeSamples();

	/**
	 * @brief Gets the total number of samples in the debug data.
	 * @return The number of samples.
	 */
	inline int getSampleCount() const { return m_nsamples; }
	/**
	 * @brief Gets the velocity sample at the specified index.
	 * @param i The index of the sample.
	 * @return A pointer to the velocity sample.
	 */
	inline const float* getSampleVelocity(const int i) const { return &m_vel[i * 3]; }
	/**
	 * @brief Gets the size of the sample at the specified index.
	 * @param i The index of the sample.
	 * @return The size of the sample.
	 */
	inline float getSampleSize(const int i) const { return m_ssize[i]; }
	/**
	 * @brief Gets the penalty value of the sample at the specified index.
	 * @param i The index of the sample.
	 * @return The penalty value.
	 */
	inline float getSamplePenalty(const int i) const { return m_pen[i]; }
	/**
	 * @brief Gets the desired velocity penalty of the sample at the specified index.
	 * @param i The index of the sample.
	 * @return The desired velocity penalty.
	 */
	inline float getSampleDesiredVelocityPenalty(const int i) const { return m_vpen[i]; }
	/**
	 * @brief Gets the current velocity penalty of the sample at the specified index.
	 * @param i The index of the sample.
	 * @return The current velocity penalty.
	 */
	inline float getSampleCurrentVelocityPenalty(const int i) const { return m_vcpen[i]; }
	/**
	 * @brief Gets the preferred side penalty of the sample at the specified index.
	 * @param i The index of the sample.
	 * @return The preferred side penalty.
	 */
	inline float getSamplePreferredSidePenalty(const int i) const { return m_spen[i]; }
	/**
	 * @brief Gets the collision time penalty of the sample at the specified index.
	 * @param i The index of the sample.
	 * @return The collision time penalty.
	 */
	inline float getSampleCollisionTimePenalty(const int i) const { return m_tpen[i]; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtObstacleAvoidanceDebugData(const dtObstacleAvoidanceDebugData&);
	dtObstacleAvoidanceDebugData& operator=(const dtObstacleAvoidanceDebugData&);

	int m_nsamples;
	int m_maxSamples;
	float* m_vel;
	float* m_ssize;
	float* m_pen;
	float* m_vpen;
	float* m_vcpen;
	float* m_spen;
	float* m_tpen;
};

/**
 * @brief Allocates memory for dtObstacleAvoidanceDebugData.
 * @return A pointer to the allocated debug data.
 */
dtObstacleAvoidanceDebugData* dtAllocObstacleAvoidanceDebugData();
/**
 * @brief Frees memory allocated for dtObstacleAvoidanceDebugData.
 * @param ptr A pointer to the debug data to be freed.
 */
void dtFreeObstacleAvoidanceDebugData(dtObstacleAvoidanceDebugData* ptr);

/**
 * @brief Represents obstacle avoidance parameters for queries.
 */
static const int DT_MAX_PATTERN_DIVS = 32;	///< Max numver of adaptive divs.
static const int DT_MAX_PATTERN_RINGS = 4;	///< Max number of adaptive rings.

/**
 * @struct dtObstacleAvoidanceParams
 * @brief Represents parameters for obstacle avoidance queries.
 */
struct dtObstacleAvoidanceParams
{
	float velBias;
	float weightDesVel;
	float weightCurVel;
	float weightSide;
	float weightToi;
	float horizTime;
	unsigned char gridSize;			///< grid
	unsigned char adaptiveDivs;		///< adaptive
	unsigned char adaptiveRings;	///< adaptive
	unsigned char adaptiveDepth;	///< adaptive
};

/**
 * @class dtObstacleAvoidanceQuery
 * @brief Represents a query for obstacle avoidance.
 */
class dtObstacleAvoidanceQuery
{
public:
	/**
	 * @brief Constructor for dtObstacleAvoidanceQuery.
	 */
	dtObstacleAvoidanceQuery();
	/**
	 * @brief Destructor for dtObstacleAvoidanceQuery.
	 */
	~dtObstacleAvoidanceQuery();

	/**
	 * @brief Initializes the obstacle avoidance query with maximum counts for circles and segments.
	 * @param maxCircles The maximum number of obstacle circles.
	 * @param maxSegments The maximum number of obstacle segments.
	 * @return True if initialization was successful, false otherwise.
	 */
	bool init(const int maxCircles, const int maxSegments);

	/**
	 * @brief Resets the obstacle avoidance query to its initial state.
	 */
	void reset();

	/**
	 * @brief Adds an obstacle circle to the query.
	 * @param pos The position of the obstacle circle.
	 * @param rad The radius of the obstacle circle.
	 * @param vel The velocity of the obstacle circle.
	 * @param dvel The desired velocity of the obstacle circle.
	 */
	void addCircle(const float* pos, const float rad,
		const float* vel, const float* dvel);

	/**
	 * @brief Adds an obstacle segment to the query.
	 * @param p The start point of the obstacle segment.
	 * @param q The end point of the obstacle segment.
	 */
	void addSegment(const float* p, const float* q);

	/**
	 * @brief Samples velocity using a grid-based approach.
	 * @param pos The position of the character.
	 * @param rad The radius of the character.
	 * @param vmax The maximum velocity of the character.
	 * @param vel The current velocity of the character.
	 * @param dvel The desired velocity of the character.
	 * @param nvel The resulting sampled velocity.
	 * @param params The obstacle avoidance parameters.
	 * @param debug Debug data (optional).
	 * @return The number of sampled velocities.
	 */
	int sampleVelocityGrid(const float* pos, const float rad, const float vmax,
		const float* vel, const float* dvel, float* nvel,
		const dtObstacleAvoidanceParams* params,
		dtObstacleAvoidanceDebugData* debug = 0);

	/**
	 * @brief Samples velocity using an adaptive approach.
	 * @param pos The position of the character.
	 * @param rad The radius of the character.
	 * @param vmax The maximum velocity of the character.
	 * @param vel The current velocity of the character.
	 * @param dvel The desired velocity of the character.
	 * @param nvel The resulting sampled velocity.
	 * @param params The obstacle avoidance parameters.
	 * @param debug Debug data (optional).
	 * @return The number of sampled velocities.
	 */
	int sampleVelocityAdaptive(const float* pos, const float rad, const float vmax,
		const float* vel, const float* dvel, float* nvel,
		const dtObstacleAvoidanceParams* params,
		dtObstacleAvoidanceDebugData* debug = 0);

	/**
	 * @brief Gets the number of obstacle circles in the query.
	 * @return The number of obstacle circles.
	 */
	inline int getObstacleCircleCount() const { return m_ncircles; }
	/**
	 * @brief Gets an obstacle circle at the specified index.
	 * @param i The index of the obstacle circle.
	 * @return A pointer to the obstacle circle.
	 */
	const dtObstacleCircle* getObstacleCircle(const int i) { return &m_circles[i]; }

	/**
	 * @brief Gets the number of obstacle segments in the query.
	 * @return The number of obstacle segments.
	 */
	inline int getObstacleSegmentCount() const { return m_nsegments; }
	/**
	 * @brief Gets an obstacle segment at the specified index.
	 * @param i The index of the obstacle segment.
	 * @return A pointer to the obstacle segment.
	 */
	const dtObstacleSegment* getObstacleSegment(const int i) { return &m_segments[i]; }

private:
	// Explicitly disabled copy constructor and copy assignment operator.
	dtObstacleAvoidanceQuery(const dtObstacleAvoidanceQuery&);
	dtObstacleAvoidanceQuery& operator=(const dtObstacleAvoidanceQuery&);

	/**
	 * @brief Prepares the obstacle avoidance query for sampling.
	 * @param pos The position of the character.
	 * @param dvel The desired velocity of the character.
	 */
	void prepare(const float* pos, const float* dvel);

	/**
	 * @brief Processes a velocity sample and computes penalties.
	 * @param vcand The candidate velocity sample.
	 * @param cs The character size.
	 * @param pos The position of the character.
	 * @param rad The radius of the character.
	 * @param vel The current velocity of the character.
	 * @param dvel The desired velocity of the character.
	 * @param minPenalty The minimum penalty value.
	 * @param debug Debug data (optional).
	 * @return The penalty value for the sample.
	 */
	float processSample(const float* vcand, const float cs,
		const float* pos, const float rad,
		const float* vel, const float* dvel,
		const float minPenalty,
		dtObstacleAvoidanceDebugData* debug);

	dtObstacleAvoidanceParams m_params;
	float m_invHorizTime;
	float m_vmax;
	float m_invVmax;

	int m_maxCircles;
	dtObstacleCircle* m_circles;
	int m_ncircles;

	int m_maxSegments;
	dtObstacleSegment* m_segments;
	int m_nsegments;
};

/**
 * @brief Allocates memory for dtObstacleAvoidanceQuery.
 * @return A pointer to the allocated query.
 */
dtObstacleAvoidanceQuery* dtAllocObstacleAvoidanceQuery();
/**
 * @brief Frees memory allocated for dtObstacleAvoidanceQuery.
 * @param ptr A pointer to the query to be freed.
 */
void dtFreeObstacleAvoidanceQuery(dtObstacleAvoidanceQuery* ptr);

#endif // DETOUROBSTACLEAVOIDANCE_H
