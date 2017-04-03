
#include "OptimizationThread.h"

#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"

#include <g3log/g3log.hpp>

#include "SlamSystem.h"
#include "MappingThread.h"

namespace lsd_slam {

	using active_object::ActiveIdle;

OptimizationThread::OptimizationThread( SlamSystem &system, bool enabled )
	: //_haveUnmergedOptimizationOffset( false ),
		_system( system ),
		_thread( enabled ? ActiveIdle::createActiveIdle( std::bind( &OptimizationThread::callbackIdle, this ), std::chrono::milliseconds(2000)) : NULL )
{
	LOG(INFO) << "Started optimization thread";
}

OptimizationThread::~OptimizationThread()
{
	if( _thread ) delete _thread.release();
	LOG(INFO) << "Exited optimization thread";
}





//===== Callbacks for ActiveObject ======

void OptimizationThread::callbackIdle( void )
{
	LOG(DEBUG) << "Running short optimization";
	while(optimizationIteration(5, 0.02)) { _system.mapThread->mergeOptimizationUpdate(); }
}

void OptimizationThread::callbackNewConstraint( void )
{
	LOG(DEBUG) << "Running short optimization";

	_system.keyFrameGraph()->addElementsFromBuffer();

	while(optimizationIteration(5, 0.02)) { _system.mapThread->mergeOptimizationUpdate(); }
}

void OptimizationThread::callbackFinalOptimization( void )
{
	LOG(INFO) << "Running final optimization!";
	optimizationIteration(50, 0.001);
	_system.mapThread->mergeOptimizationUpdate();
	finalOptimizationComplete.notify();
}


//=== Actual meat ====





bool OptimizationThread::optimizationIteration(int itsPerTry, float minChange)
{
	Timer timer;

	// std::lock_guard< std::mutex > lock(g2oGraphAccessMutex);

	// Do the optimization. This can take quite some time!
	int its = _system.keyFrameGraph()->optimize(itsPerTry);

	float maxChange = 0;
	float sumChange = 0;
	float sum = 0;

	{
	// save the optimization result.
	_system.poseConsistencyMutex.lock_shared();
	KeyframeLibrary::LockGuard lock(_system.keyframes().mutex());


	//for(size_t i=0;i<_system.keyframesAll.const_ref().size(); i++)
	for( auto keyframe : _system.keyframes() )
	{
		// set edge error sum to zero
		keyframe->edgeErrorSum = 0;
		keyframe->edgesNum = 0;

		if(!keyframe->pose->isInGraph) continue;

		// get change from last optimization
		Sim3 a = keyframe->pose->graphVertex->estimate();
		Sim3 b = keyframe->getCamToWorld();
		Sophus::Vector7f diff = (a*b.inverse()).log().cast<float>();


		for(int j=0;j<7;j++)
		{
			float d = fabsf((float)(diff[j]));
			if(d > maxChange) maxChange = d;
			sumChange += d;
		}
		sum +=7;

		// set change
		keyframe->pose->setPoseGraphOptResult(
		keyframe->pose->graphVertex->estimate());

		// add error
		for(auto edge : keyframe->pose->graphVertex->edges())
		{
			keyframe->edgeErrorSum += ((EdgeSim3*)(edge))->chi2();
			keyframe->edgesNum++;
		}
	}

	_system.poseConsistencyMutex.unlock_shared();
}

	LOGF_IF(DEBUG, enablePrintDebugInfo && printOptimizationInfo,
					"did %d optimization iterations. Max Pose Parameter Change: %f; avgChange: %f. %s\n",
					its, maxChange, sumChange / sum,
					maxChange > minChange && its == itsPerTry ? "continue optimizing":"Waiting for addition to graph.");

	perf.update( timer );

	return maxChange > minChange && its == itsPerTry;
}

// void OptimizationThread::optimizeGraph( void )
// {
// 	std::unique_lock<std::mutex> g2oLock(g2oGraphAccessMutex);
// 	keyFrameGraph()->optimize(1000);
// 	g2oLock.unlock();
// 	mergeOptimizationOffset();
// }



}
