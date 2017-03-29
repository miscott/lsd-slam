
#include "App/App.h"

#include "Pangolin_IOWrapper/PangolinOutput3DWrapper.h"

using namespace lsd_slam;

MutexObject<bool> guiDone;
ThreadSynchronizer guiReady;

void runGuiThread(const std::shared_ptr<GUI> &gui )
{
	guiReady.notify();
	startAll.wait();

	while(!pangolin::ShouldQuit())
	{
		if(guiDone()) break;

		LOG(INFO) << "runGuiThread";

		gui->preCall();
		gui->drawKeyframes();

		gui->drawFrustum();

		gui->drawImages();

		gui->postCall();
	}

	guiDone = true;

}
