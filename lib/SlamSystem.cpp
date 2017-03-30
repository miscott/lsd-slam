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

#include "SlamSystem.h"

#include <boost/thread/shared_lock_guard.hpp>

#include <g3log/g3log.hpp>

#ifdef ANDROID
#include <android/log.h>
#endif

#include <opencv2/opencv.hpp>

#include "DataStructures/Frame.h"
// #include "Tracking/SE3Tracker.h"
// #include "Tracking/Sim3Tracker.h"
// #include "DepthEstimation/DepthMap.h"
// #include "Tracking/TrackingReference.h"
#include "util/globalFuncs.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "GlobalMapping/TrackableKeyFrameSearch.h"
// #include "GlobalMapping/g2oTypeSim3Sophus.h"
// #include "IOWrapper/ImageDisplay.h"
// #include "IOWrapper/Output3DWrapper.h"
// #include <g2o/core/robust_kernel_impl.h>

#include "DataStructures/FrameMemory.h"
// #include "deque"

// for mkdir
// #include <sys/types.h>
// #include <sys/stat.h>

#include "SlamSystem/MappingThread.h"
#include "SlamSystem/ConstraintSearchThread.h"
#include "SlamSystem/OptimizationThread.h"
#include "SlamSystem/TrackingThread.h"

using namespace lsd_slam;



SlamSystem::SlamSystem( const Configuration &conf )
: _finalized(),
	perf(),
	_conf( conf ),
	_outputWrapper( nullptr ),
	mappingTrackingReference( new TrackingReference() ),
	_keyFrameGraph( new KeyFrameGraph( *this ) ),
	keyframesAll(),
	idToKeyFrame(),
	_currentKeyFrame( nullptr ),
	allFramePoses(),
	_trackableKeyFrameSearch( new TrackableKeyFrameSearch( *this, _keyFrameGraph, conf ) ),
	_initialized( false )
{

	// Because some of these rely on conf(), explicitly call after
 	// static initialization.  Is this true?
	optThread.reset( new OptimizationThread( *this, conf.SLAMEnabled ) );
	mapThread.reset( new MappingThread( *this ) );
	constraintThread.reset( new ConstraintSearchThread( *this, conf.SLAMEnabled ) );
	trackingThread.reset( new TrackingThread( *this ) );

	timeLastUpdate.start();
}


SlamSystem::~SlamSystem()
{
	// keepRunning = false;

	// make sure no-one is waiting for something.
	LOG(INFO) << "... waiting for all threads to exit";
	// newFrameMapped.notify();
	// unmappedTrackedFrames.notifyAll();
	// // unmappedTrackedFramesSignal.notify_all();
	//
	// newKeyFrames.notifyAll();

	// newConstraintCreatedSignal.notify_all();

	mapThread.reset();
	constraintThread.reset();
	optThread.reset();
	trackingThread.reset();
	LOG(INFO) << "DONE waiting for all threads to exit";

	FrameMemory::getInstance().releaseBuffes();

	// Util::closeAllWindows();
}

SlamSystem *SlamSystem::fullReset( void )
{
	SlamSystem *newSystem = new SlamSystem( conf() );
	newSystem->set3DOutputWrapper( outputWrapper() );
	return newSystem;
}



void SlamSystem::finalize()
{
	LOG(INFO) << "Finalizing Graph... adding final constraints!!";

	// This happens in the foreground
	constraintThread->doFullReConstraintTrack();
	constraintThread->fullReConstraintTrackComplete.wait();

	LOG(INFO) << "Finalizing Graph... optimizing!!";
	// This happens in the foreground
	// This will kick off a final map publication with the newly optimized offsets (also in foreground)
	optThread->doFinalOptimization();

	optThread->finalOptimizationComplete.wait();
	mapThread->optimizationUpdateMerged.wait();


	// doFinalOptimization = true;
	// newConstraintMutex.lock();
	// newConstraintAdded = true;
	// newConstraintCreatedSignal.notify_all();
	// newConstraintMutex.unlock();

	// while(doFinalOptimization)
	// {
	// 	usleep(200000);
	// }

	//printf("Finalizing Graph... publishing!!\n");
	//unmappedTrackedFrames.notifyAll();
	// unmappedTrackedFramesMutex.lock();
	// unmappedTrackedFramesSignal.notify_one();
	// unmappedTrackedFramesMutex.unlock();

	// while(doFinalOptimization)
	// {
	// 	usleep(200000);
	// }

	// newFrameMapped.wait();
	// newFrameMapped.wait();

	// usleep(200000);
	LOG(INFO) << "Done Finalizing Graph.!!";
	_finalized.notify();

}


// void SlamSystem::requestDepthMapScreenshot(const std::string& filename)
// {
// 	depthMapScreenshotFilename = filename;
// 	depthMapScreenshotFlag = true;
// }

void SlamSystem::initialize( const Frame::SharedPtr &frame )
{
	LOG_IF(FATAL, !conf().doMapping ) << "WARNING: mapping is disabled, but we just initialized... THIS WILL NOT WORK! Set doMapping to true.";

	if( frame->hasIDepthBeenSet() ) {
		LOG(INFO) << "Using initial Depth estimate in first frame.";
		mapThread->gtDepthInit( frame );
	} else {
		LOG(INFO) << "Doing Random initialization!";
		mapThread->randomInit( frame );
	}

	storePose( frame );

	if( conf().SLAMEnabled ) {
		IdToKeyFrame::LockGuard lock( idToKeyFrame.mutex() );
		idToKeyFrame.ref().insert(std::make_pair( frame->id(), frame ));
	}

	currentKeyFrame().set( frame );

	if( _conf.continuousPCOutput) publishKeyframe( frame );

	setInitialized( true );
}

void SlamSystem::trackFrame( Frame *newFrame, bool blockUntilMapped )
{
	trackFrame( Frame::SharedPtr(newFrame), blockUntilMapped );
}

void SlamSystem::trackFrame(const Frame::SharedPtr &newFrame, bool blockUntilMapped )
{
	if( !initialized() ) initialize( newFrame );

	LOG(INFO) << "Tracking frame; " << ( blockUntilMapped ? "WILL" : "won't") << " block";

	trackingThread->trackFrame( newFrame, blockUntilMapped );

	//TODO: At present only happens at frame rate.  Push to a thread?
	addTimingSamples();
}

void SlamSystem::storePose( const Frame::SharedPtr &frame )
{
	frame->pose->isRegisteredToGraph = true;

	{
	 		AllFramePoses::LockGuard lock( allFramePoses.mutex() );
	 		allFramePoses.ref().push_back( frame->pose );
	}
}

//=== Keyframe maintenance functions ====

void SlamSystem::changeKeyframe( const Frame::SharedPtr &candidate, bool noCreate, bool force, float maxScore)
{
	Frame::SharedPtr newReferenceKF(nullptr);

	if( conf().doKFReActivation && conf().SLAMEnabled )
	{
		Timer timer;
		newReferenceKF = trackableKeyFrameSearch()->findRePositionCandidate( candidate, maxScore );
		perf.findReferences.update( timer );
	}

	if(newReferenceKF != 0) {
		LOG(INFO) << "Reloading existing key frame " << newReferenceKF->id();
		loadNewCurrentKeyframe(newReferenceKF);
	} else {
		if(force)
		{
			if(noCreate)
			{
				LOG(INFO) << "mapping is disabled & moved outside of known map. Starting Relocalizer!";
				trackingThread->setTrackingIsBad();
				//nextRelocIdx = -1; /// What does this do?
			}
			else
			{
				createNewCurrentKeyframe( candidate );
			}
		}
	}
	// createNewKeyFrame = false;
}

void SlamSystem::loadNewCurrentKeyframe( const Frame::SharedPtr &keyframeToLoad)
{
	//std::lock_guard< std::mutex > lock( currentKeyFrame.mutex() );

	// LOG_IF(DEBUG, enablePrintDebugInfo && printThreadingInfo ) << "RE-ACTIVATE KF " << keyframeToLoad->id();

	mapThread->map->setFromExistingKF(keyframeToLoad);

	LOG_IF(DEBUG, enablePrintDebugInfo && printRegularizeStatistics ) << "re-activate frame " << keyframeToLoad->id() << "!";

	currentKeyFrame().set( idToKeyFrame.const_ref().find(keyframeToLoad->id())->second );
	currentKeyFrame()()->depthHasBeenUpdatedFlag = false;
}


void SlamSystem::createNewCurrentKeyframe( const Frame::SharedPtr &newKeyframe )
{
	//mapThread->finalizeKeyFrame( true );

	LOG_IF(INFO, printThreadingInfo) << "CREATE NEW KF " << newKeyframe->id() << ", replacing " << currentKeyFrame()()->id();

	if( conf().SLAMEnabled ) {
		MutexObject< std::unordered_map< int, Frame::SharedPtr > >::LockGuard lock( idToKeyFrame.mutex() );
		idToKeyFrame.ref().insert(std::make_pair( newKeyframe->id(), newKeyframe ));
	}
	// propagate & make new.
	mapThread->map->createKeyFrame( newKeyframe );

	currentKeyFrame().set( newKeyframe );

	// mapThread->setNewKeyFrame( currentKeyFrame(), true );

	if(conf().SLAMEnabled)
	{
		mappingTrackingReference->importFrame( newKeyframe );
		newKeyframe->setPermaRef(mappingTrackingReference);
		mappingTrackingReference->invalidate();

		if(newKeyframe->idxInKeyframes < 0)
		{
			KeyframesAll::LockGuard guard( keyframesAll.mutex() );
			//keyFrameGraph()->keyframesAllMutex.lock();
			newKeyframe->idxInKeyframes = keyframesAll.const_ref().size();
			keyframesAll.ref().push_back(newKeyframe );
			keyFrameGraph()->totalPoints += newKeyframe->numPoints;
			keyFrameGraph()->totalVertices ++;
			//keyFrameGraph()->keyframesAllMutex.unlock();

			constraintThread->newKeyFrame( newKeyframe );
		}
	}

	publishKeyframe( newKeyframe );


	// if(outputWrapper && printPropagationStatistics)
	// {
	//
	// 	Eigen::Matrix<float, 20, 1> data;
	// 	data.setZero();
	// 	data[0] = runningStats.num_prop_attempts / ((float)_conf.slamImage.area());
	// 	data[1] = (runningStats.num_prop_created + runningStats.num_prop_merged) / (float)runningStats.num_prop_attempts;
	// 	data[2] = runningStats.num_prop_removed_colorDiff / (float)runningStats.num_prop_attempts;
	//
	// 	outputWrapper->publishDebugInfo(data);
	// }

	// currentKeyFrameMutex.lock();
	// currentKeyFrameMutex.unlock();
}




//===== Debugging output functions =====


void SlamSystem::addTimingSamples()
{
	mapThread->map->addTimingSample();

	float sPassed = timeLastUpdate.reset();
	if(sPassed > 1.0f)
	{

		LOGF_IF(INFO, enablePrintDebugInfo && printOverallTiming, "MapIt: %3.1fms (%.1fHz); Track: %3.1fms (%.1fHz); Create: %3.1fms (%.1fHz); FindRef: %3.1fms (%.1fHz); PermaTrk: %3.1fms (%.1fHz); Opt: %3.1fms (%.1fHz); FindConst: %3.1fms (%.1fHz);\n",
					mapThread->map->_perf.update.ms(), mapThread->map->_perf.update.rate(),
					trackingThread->perf.ms(), trackingThread->perf.rate(),
					mapThread->map->_perf.create.ms()+mapThread->map->_perf.finalize.ms(), mapThread->map->_perf.create.rate(),
					perf.findReferences.ms(), perf.findReferences.rate(),
					0.0, 0.0,
					//trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->trackPermaRef.ms() : 0, trackableKeyFrameSearch != 0 ? trackableKeyFrameSearch->trackPermaRef.rate() : 0,
					optThread->perf.ms(), optThread->perf.rate(),
					perf.findConstraint.ms(), perf.findConstraint.rate() );
	}

}

void SlamSystem::updateDisplayDepthMap()
{
	if( !conf().displayDepthMap ) return;  //&& !depthMapScreenshotFlag)

	mapThread->map->debugPlotDepthMap();
	double scale = 1;
	if( (bool)currentKeyFrame()() )
		scale = currentKeyFrame()()->getCamToWorld().scale();

	// debug plot depthmap
	char buf1[200];
	char buf2[200];

	if( currentKeyFrame().ref() ) {
	snprintf(buf1,200,"Map: Upd %3.0fms (%2.0fHz); Trk %3.0fms (%2.0fHz); %d / %d",
			mapThread->map->_perf.update.ms(), mapThread->map->_perf.update.rate(),
			trackingThread->perf.ms(), trackingThread->perf.rate(),
			currentKeyFrame()()->numFramesTrackedOnThis, currentKeyFrame()()->numMappedOnThis ); //, (int)unmappedTrackedFrames().size());
		} else {
			snprintf(buf1,200,"Map: Upd %3.0fms (%2.0fHz); Trk %3.0fms (%2.0fHz); xx / xx",
					mapThread->map->_perf.update.ms(), mapThread->map->_perf.update.rate(),
					trackingThread->perf.ms(), trackingThread->perf.rate() );
		}
	// snprintf(buf2,200,"dens %2.0f%%; good %2.0f%%; scale %2.2f; res %2.1f/; usg %2.0f%%; Map: %d F, %d KF, %d E, %.1fm Pts",
	// 		100*currentKeyFrame->numPoints/(float)(conf().slamImage.area()),
	// 		100*tracking_lastGoodPerBad,
	// 		scale,
	// 		tracking_lastResidual,
	// 		100*tracking_lastUsage,
	// 		(int)keyFrameGraph()->allFramePoses.size(),
	// 		keyFrameGraph()->totalVertices,
	// 		(int)keyFrameGraph()->edgesAll.size(),
	// 		1e-6 * (float)keyFrameGraph()->totalPoints);


	if( conf().onSceenInfoDisplay )
		printMessageOnCVImage(mapThread->map->debugImageDepth, buf1, buf2);

	CHECK( mapThread->map->debugImageDepth.data != NULL );
	publishDepthImage( mapThread->map->debugImageDepth.data );
}



SE3 SlamSystem::getCurrentPoseEstimate()
{
	AllFramePoses::LockGuard lock( allFramePoses.mutex() );
	if( allFramePoses.const_ref().size() > 0)
		return se3FromSim3(allFramePoses.const_ref().back()->getCamToWorld());

	return Sophus::SE3();
}

Sophus::Sim3f SlamSystem::getCurrentPoseEstimateScale()
{
	AllFramePoses::LockGuard lock( allFramePoses.mutex() );
	if(allFramePoses.const_ref().size() > 0)
		return allFramePoses.const_ref().back()->getCamToWorld().cast<float>();

	return Sophus::Sim3f();
}

std::vector<FramePoseStruct::SharedPtr> SlamSystem::getAllPoses()
{
	return allFramePoses.get();
}
