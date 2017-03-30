

#pragma once

#include <mutex>
#include <memory>

#include <boost/thread/shared_mutex.hpp>

#include "active_object/active.h"

#include "util/MovingAverage.h"
#include "util/ThreadMutexObject.h"

#include "DepthEstimation/DepthMap.h"
#include "Tracking/TrackingReference.h"

#include "Tracking/Relocalizer.h"


namespace lsd_slam {

	class KeyFrameGraph;
	class SlamSystem;

class MappingThread {
public:

	MappingThread( SlamSystem &system );
	~MappingThread();

	//=== Callbacks into the thread ===
	void mapTrackedFrame( const Frame::SharedPtr &frame, bool doBlock = true )
	{
		// {
		// 	std::lock_guard<std::mutex> lock(unmappedTrackedFramesMutex );
		// 	unmappedTrackedFrames.push_back( frame );
		// }

		if( _thread ) {
			_thread->send( std::bind( &MappingThread::callbackMapTrackedFrame, this, frame ));

			if( doBlock ) trackedFramesMappedSync.wait();
			
		} else {
			callbackMapTrackedFrame( Frame::SharedPtr(frame) );
		}
	}

	// void doIteration( void )
	// { if( _thread ) _thread->send( std::bind( &MappingThread::callbackIdle, this )); }

	void mergeOptimizationUpdate( void )
	{ optimizationUpdateMerged.reset();
		if( _thread ) _thread->send( std::bind( &MappingThread::callbackMergeOptimizationOffset, this )); }

		void newKeyframe( const Frame::SharedPtr &frame, bool doBlock );

		void reloadExistingKF( const Frame::SharedPtr &frame );


		//void finishCurrentKeyFrame( bool block = false );
		// void setNewKeyFrame( const Frame::SharedPtr &frame,  bool block = false );

	// void createNewKeyFrame( const Frame::SharedPtr &frame )
	// {
	// 	if( _newKeyFrame.get() != nullptr ) LOG(WARNING) << "Asked to make " << frame->id() << " a keyframe when " << _newKeyFrame()->id() << " is already pending";
	// 	_newKeyFrame = frame;
	// 	//if( _thread ) {
	// 	//		_thread->send( std::bind( &MappingThread::callbackCreateNewKeyFrame, this, frame ));
	// 	//		LOG(INFO) << "Mq now " << _thread->size();
	// 	//}
	// }

	// bool newKeyFramePending( void )
	// {
	// 		return _newKeyFrame.get() != nullptr;
	// }

	void gtDepthInit( const Frame::SharedPtr &frame );
	void randomInit( const Frame::SharedPtr &frame );


	// SET & READ EVERYWHERE
	// std::mutex currentKeyFrameMutex;

	// std::deque< Frame::SharedPtr > unmappedTrackedFrames;
	// std::mutex unmappedTrackedFramesMutex;
	ThreadSynchronizer trackedFramesMappedSync;

	DepthMap* map;

	ThreadSynchronizer optimizationUpdateMerged;


	// during re-localization used
	Relocalizer relocalizer;

private:

	SlamSystem &_system;

	// MutexObject< Frame::SharedPtr > _newKeyFrame;

	// == Thread callbacks ==
	//void callbackIdle( void );
	void callbackMapTrackedFrame( Frame::SharedPtr frame);
	void callbackNewKeyframe( Frame::SharedPtr frame );
	ThreadSynchronizer newKeyframeSync;

	void callbackReloadExistingKF( Frame::SharedPtr frame );

	//void callbackCreateNewKeyFrame( std::shared_ptr<Frame> frame );

	// == Local functions ==
	void callbackMergeOptimizationOffset();

	bool doMappingIteration();

	bool updateKeyframe( const Frame::SharedPtr &frame );

	void addTimingSamples();

	void finishCurrentKeyframe();
	//ThreadSynchronizer finishCurrentKeyframeSync;

	// void callbackSetNewCurrentKeyframe();

	void discardCurrentKeyframe();


	void debugDisplayDepthMap();


	// std::vector<Frame*> KFForReloc;
	// //int nextRelocIdx;
	// std::shared_ptr<Frame> latestFrameTriedForReloc;

	std::unique_ptr<active_object::Active> _thread;

};


}
