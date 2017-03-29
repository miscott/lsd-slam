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
#include <unordered_set>
#include "util/SophusUtil.h"
#include "util/Configuration.h"

#ifdef HAVE_FABMAP
	#include "GlobalMapping/FabMap.h"
#endif

#include "util/MovingAverage.h"
#include "util/settings.h"

#include "DataStructures/Frame.h"


namespace lsd_slam
{


class KeyFrameGraph;
class SE3Tracker;

class SlamSystem;


struct TrackableKFStruct
{
	Frame::SharedPtr ref;
	SE3 refToFrame;
	float dist;
	float angle;
};

/**
 * Given a KeyFrame, tries to find other KeyFrames from a KeyFrameGraph which
 * can be tracked from this frame (in order to insert new constraints into
 * the graph).
 */
class TrackableKeyFrameSearch
{
	friend class SlamSystem;
	
public:
	/** Constructor. */
	TrackableKeyFrameSearch( SlamSystem &system, const std::shared_ptr<KeyFrameGraph> &graph, const Configuration &conf );
	~TrackableKeyFrameSearch();

	/**
	 * Finds candidates for trackable frames.
	 * Returns the most likely candidates first.
	 */
	std::unordered_set<Frame::SharedPtr> findCandidates(const Frame::SharedPtr &keyframe, Frame::SharedPtr &fabMapResult_out, bool includeFABMAP=true, bool closenessTH=1.0);
	Frame::SharedPtr findRePositionCandidate( const Frame::SharedPtr &frame, float maxScore=1);


	inline float getRefFrameScore(float distanceSquared, float usage)
	{
		return distanceSquared*KFDistWeight*KFDistWeight
				+ (1-usage)*(1-usage) * KFUsageWeight * KFUsageWeight;
	}

	MsRateAverage trackPermaRef;


private:

	 SlamSystem &_system;
	/**
	 * Returns a possible loop closure for the keyframe or nullptr if none is found.
	 * Uses FabMap internally.
	 */
	Frame::SharedPtr findAppearanceBasedCandidate(const Frame::SharedPtr &keyframe);
	std::vector<TrackableKFStruct> findEuclideanOverlapFrames(const Frame::SharedPtr &frame, float distanceTH, float angleTH, bool checkBothScales = false);

#ifdef HAVE_FABMAP
	std::unordered_map<int, Frame::SharedPtr> fabmapIDToKeyframe;
	FabMap fabMap;
#endif
	std::shared_ptr<KeyFrameGraph> graph;
	std::unique_ptr<SE3Tracker> tracker;

	float fowX, fowY;

};

}
