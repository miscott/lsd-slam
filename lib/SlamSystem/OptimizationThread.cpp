
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
	SlamSystem::KeyframesAll::LockGuard lock(_system.keyframesAll.mutex());


	for(size_t i=0;i<_system.keyframesAll.const_ref().size(); i++)
	{
		// set edge error sum to zero
		_system.keyframesAll.ref()[i]->edgeErrorSum = 0;
		_system.keyframesAll.ref()[i]->edgesNum = 0;

		if(!_system.keyframesAll.const_ref()[i]->pose->isInGraph) continue;

		// get change from last optimization
		Sim3 a = _system.keyframesAll.ref()[i]->pose->graphVertex->estimate();
		Sim3 b = _system.keyframesAll.ref()[i]->getCamToWorld();
		Sophus::Vector7f diff = (a*b.inverse()).log().cast<float>();


		for(int j=0;j<7;j++)
		{
			float d = fabsf((float)(diff[j]));
			if(d > maxChange) maxChange = d;
			sumChange += d;
		}
		sum +=7;

		// set change
		_system.keyframesAll.ref()[i]->pose->setPoseGraphOptResult(
		_system.keyframesAll.ref()[i]->pose->graphVertex->estimate());

		// add error
		for(auto edge : _system.keyframesAll.ref()[i]->pose->graphVertex->edges())
		{
			_system.keyframesAll.ref()[i]->edgeErrorSum += ((EdgeSim3*)(edge))->chi2();
			_system.keyframesAll.ref()[i]->edgesNum++;
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
