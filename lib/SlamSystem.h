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
#include <mutex>
#include <thread>
#include <condition_variable>
#include <boost/thread/shared_mutex.hpp>
#include <memory>
#include <chrono>

#include "util/settings.h"
#include "IOWrapper/Timestamp.h"
#include "opencv2/core/core.hpp"

#include "IOWrapper/Output3DWrapper.h"

#include "DataStructures/Frame.h"

#include "util/SophusUtil.h"
#include "util/MovingAverage.h"
#include "util/Configuration.h"
#include "util/Timer.h"
#include "util/ThreadMutexObject.h"

#include "Tracking/Relocalizer.h"

namespace lsd_slam
{

	// class TrackingReference;
	class KeyFrameGraph;
	// class SE3Tracker;
	// class Sim3Tracker;
	// class DepthMap;
	// class Frame;
	// class DataSet;
	// class LiveSLAMWrapper;
	class Output3DWrapper;
	class FramePoseStruct;
	class TrackableKeyFrameSearch;
	// struct KFConstraintStruct;

	class TrackingThread;
	class OptimizationThread;
	class MappingThread;
	class ConstraintSearchThread;

	using std::unique_ptr;
	using std::shared_ptr;

class SlamSystem {

friend class IntegrationTest;
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SlamSystem( const Configuration &conf );

	SlamSystem( const SlamSystem&) = delete;
	SlamSystem& operator=(const SlamSystem&) = delete;

	~SlamSystem();

	// Creates a new SlamSystem, and passes over relevant configuration info
	SlamSystem *fullReset();

	// tracks a frame.
	// first frame will return Identity = camToWord.
	// returns camToWord transformation of the tracked frame.
	// frameID needs to be monotonically increasing.
	void trackFrame(const Frame::SharedPtr &newFrame, bool blockUntilMapped );
	void trackFrame( Frame *newFrame, bool blockUntilMapped );


	// finalizes the system, i.e. blocks and does all remaining loop-closures etc.
	void finalize();
	ThreadSynchronizer &finalized() { return _finalized; }

	/** Returns the current pose estimate. */
	SE3 getCurrentPoseEstimate();

	Sophus::Sim3f getCurrentPoseEstimateScale();

	//==== KeyFrame maintenance functions ====
	MutexObject< Frame::SharedPtr >  &currentKeyFrame() { return _currentKeyFrame; };

	void changeKeyframe( const Frame::SharedPtr &frame, bool noCreate, bool force, float maxScore);
	void loadNewCurrentKeyframe( const Frame::SharedPtr &keyframeToLoad );
	void createNewCurrentKeyframe( const Frame::SharedPtr &newKeyframeCandidate );

	void storePose( const Frame::SharedPtr &frame );

	// void requestDepthMapScreenshot(const std::string& filename);

	// int findConstraintsForNewKeyFrames(Frame* newKeyFrame, bool forceParent=true, bool useFABMAP=true, float closeCandidatesTH=1.0);

	std::vector<FramePoseStruct::SharedPtr> getAllPoses();

	struct PerformanceData {
		PerformanceData( void ) {;}

		MsRateAverage findConstraint, findReferences;
	} perf;

	Timer timeLastUpdate;

	const Configuration &conf( void ) const     { return _conf; }

	//=== Debugging output functions =====
	shared_ptr<Output3DWrapper> outputWrapper( void )      { return _outputWrapper; }
	void set3DOutputWrapper( Output3DWrapper* outputWrapper ) {	_outputWrapper.reset(outputWrapper); }
	void set3DOutputWrapper( const shared_ptr<Output3DWrapper> &outputWrapper) {	_outputWrapper = outputWrapper; }

	void publishPose(const Sophus::Sim3f &pose ) 	                 { if( _outputWrapper ) _outputWrapper->publishPose(pose);}
	void publishTrackedFrame( const Frame::SharedPtr &frame )      { if( _outputWrapper ) _outputWrapper->publishTrackedFrame( frame ); }
	void publishKeyframeGraph( void )                              { if( _outputWrapper ) _outputWrapper->publishKeyframeGraph( keyFrameGraph() ); }
	void publishKeyframe(  const Frame::SharedPtr &frame )         { if( _outputWrapper ) _outputWrapper->publishKeyframe( frame ); }
	void publishDepthImage( unsigned char* data  )                 { if( _outputWrapper ) _outputWrapper->updateDepthImage( data ); }


	void updateDisplayDepthMap();

	unique_ptr<TrackingThread> trackingThread;
	unique_ptr<OptimizationThread> optThread;
	unique_ptr<MappingThread> mapThread;
	unique_ptr<ConstraintSearchThread> constraintThread;

	// mutex to lock frame pose consistency. within a shared lock of this, *->getCamToWorld() is
	// GUARANTEED to give the same result each call, and to be compatible to each other.
	// locked exclusively during the pose-update by Mapping.
	boost::shared_mutex poseConsistencyMutex;


	shared_ptr<KeyFrameGraph> &keyFrameGraph() { return _keyFrameGraph; };	  // has own locks
	shared_ptr<TrackableKeyFrameSearch> &trackableKeyFrameSearch() { return _trackableKeyFrameSearch; }

	// TODO.   Move back to private
	// contains ALL keyframes, as soon as they are "finished".
	// does NOT yet contain the keyframe that is currently being created.
	//boost::shared_mutex keyframesAllMutex;
	typedef MutexObject< std::vector< Frame::SharedPtr > > KeyframesAll;
	MutexObject< std::vector< Frame::SharedPtr > > keyframesAll;

	// contains ALL frame poses, chronologically, as soon as they are tracked.
	// the corresponding frame may have been removed / deleted in the meantime.
	// these are the ones that are also referenced by the corresponding Frame / Keyframe object
	//boost::shared_mutex allFramePosesMutex;
	typedef MutexObject< std::vector< FramePoseStruct::SharedPtr> > AllFramePoses;
	AllFramePoses allFramePoses;

	/** Maps frame ids to keyframes. Contains ALL Keyframes allocated, including the one that currently being created. */
	/* this is where the shared pointers of Keyframe Frames are kept, so they are not deleted ever */
	//boost::shared_mutex idToKeyFrameMutex;
	typedef MutexObject< std::unordered_map< int, Frame::SharedPtr > > IdToKeyFrame;
	MutexObject< std::unordered_map< int, Frame::SharedPtr > > idToKeyFrame;

private:

	//===== Shared state =====

	const Configuration &_conf;

	std::shared_ptr<KeyFrameGraph> _keyFrameGraph;	  // has own locks


	MutexObject< Frame::SharedPtr >  _currentKeyFrame;

	TrackingReference* mappingTrackingReference;

	std::shared_ptr<TrackableKeyFrameSearch> _trackableKeyFrameSearch;

	shared_ptr<Output3DWrapper> _outputWrapper;	// no lock required

	ThreadSynchronizer _finalized;


	bool _initialized;
	bool initialized( void ) const { return _initialized; }
	bool setInitialized( bool i ) { _initialized = i; return _initialized; }
	void initialize( const Frame::SharedPtr &frame );

	// ======= Functions =====

	void addTimingSamples();



};

}
