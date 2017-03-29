/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam>
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#include <vector>
#include <unordered_map>
#include <mutex>
#include <deque>

#include <boost/thread/mutex.hpp>
#include <boost/thread/shared_mutex.hpp>

#include "util/EigenCoreInclude.h"
#include <g2o/core/sparse_optimizer.h>

#include "util/SophusUtil.h"

#include "DataStructures/Frame.h"

namespace lsd_slam
{

class SlamSystem;


class KeyFrameGraph;
class VertexSim3;
class EdgeSim3;
class FramePoseStruct;

struct KFConstraintStruct
{
	inline KFConstraintStruct()
	{
		firstFrame = secondFrame = 0;
		information.setZero();
		robustKernel = 0;
		edge = 0;

		usage = meanResidual = meanResidualD = meanResidualP = 0;
		reciprocalConsistency = 0;


		idxInAllEdges = -1;
	}

	~KFConstraintStruct();


	Frame::SharedPtr firstFrame;
	Frame::SharedPtr secondFrame;
	Sophus::Sim3d secondToFirst;
	Eigen::Matrix<double, 7, 7> information;
	g2o::RobustKernel* robustKernel;
	EdgeSim3* edge;

	float usage;
	float meanResidualD;
	float meanResidualP;
	float meanResidual;

	float reciprocalConsistency;

	int idxInAllEdges;
};


/**
 * Graph consisting of KeyFrames and constraints, performing optimization.
 */
class KeyFrameGraph
{
friend class IntegrationTest;
public:
	/** Constructs an empty pose graph. */
	KeyFrameGraph( SlamSystem &system );

	/** Deletes the g2o graph. */
	~KeyFrameGraph();

	/** Adds a new KeyFrame to the graph. */
	void addKeyFrame( const Frame::SharedPtr &frame);

	/** Adds a new Frame to the graph. Doesnt actually keep the frame, but only it's pose-struct. */
	//void addFrame(const Frame::SharedPtr &frame);

	void dumpMap(std::string folder);

	/**
	 * Adds a new constraint to the graph.
	 *
	 * The transformation must map world points such that they move as if
	 * attached to a frame which moves from firstFrame to secondFrame:
	 * second->camToWorld * first->worldToCam * point
	 *
	 * If isOdometryConstraint is set, scaleInformation is ignored.
	 */
	void insertConstraint(KFConstraintStruct* constraint);

	//int size() const { return keyframesAll.size(); }

	/** Optimizes the graph. Does not update the keyframe poses,
	 *  only the vertex poses. You must call updateKeyFramePoses() afterwards. */
	int optimize(int num_iterations);
	bool addElementsFromBuffer();


	/**
	 * Creates a hash map of keyframe -> distance to given frame.
	 */
	void calculateGraphDistancesToFrame(const Frame::SharedPtr &frame, std::unordered_map<Frame::SharedPtr, int> &distanceMap);


	int totalPoints;
	int totalEdges;
	int totalVertices;


	//=========================== Keyframe & Posen Lists & Maps ====================================
	// Always lock the list with the corresponding mutex!
	// central point to administer keyframes, iterate over keyframes, do lookups etc.

	// contains ALL edges, as soon as they are created
	boost::shared_mutex edgesListsMutex;
	std::vector< KFConstraintStruct* > edgesAll;



	// contains all keyframes in graph, in some arbitrary (random) order. if a frame is re-tracked,
	// it is put to the end of this list; frames for re-tracking are always chosen from the first third of
	// this list.
	std::mutex keyframesForRetrackMutex;
	std::deque< Frame::SharedPtr > keyframesForRetrack;

	// Used to live with the optimizer, but not needed as signal anymore,
	// now just used as a simple data mutex.
	//std::mutex newKeyFrameMutex;


private:

	SlamSystem &_system;

	/** Pose graph representation in g2o */
	g2o::SparseOptimizer graph;

	std::vector< Frame::SharedPtr > newKeyframesBuffer;
	std::vector< KFConstraintStruct* > newEdgeBuffer;


	int nextEdgeId;
};

}
