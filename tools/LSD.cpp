/**
*  This version "LSD" of the main binary has no GUI dependencies.
*  Pangolin, etc are not required to build this version.
*  (of course, it's not that exciting to watch!)
*
*  See LSD_GUI which is functionally identical but does have the
*  Pangolin-based GUI
*
* Based on original LSD-SLAM code from:
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

#include <boost/thread.hpp>

#include "ros/ros.h"

#include "libg3logger/g3logger.h"

#include "SlamSystem.h"

#include "util/settings.h"

#include "util/globalFuncs.h"
#include "util/ThreadMutexObject.h"
#include "util/Configuration.h"

#include "App/App.h"
#include "App/InputThread.h"
#include "ParseArgs.h"


using namespace lsd_slam;


int main( int argc, char** argv )
{
  libg3logger::G3Logger logWorker( argv[0] );
  logWorker.logBanner();

  Configuration conf;
  //ParseArgs args( argc, argv );
  /*
  // Load the configuration object
  conf.inputImage = args.undistorter->inputImageSize();
  conf.slamImage  = args.undistorter->outputImageSize();
  conf.camera     = args.undistorter->getCamera();

  LOG(INFO) << "Slam image: " << conf.slamImage.width << " x " << conf.slamImage.height;

  CHECK( (conf.camera.fx) > 0 && (conf.camera.fy > 0) ) << "Camera focal length is zero";

	std::shared_ptr<SlamSystem> system( new SlamSystem(conf) );

  LOG(INFO) << "Starting input thread.";
  InputThread input( system, args.dataSource, args.undistorter );
  boost::thread inputThread( boost::ref(input) );
  input.inputReady.wait();

  // Wait for all threads to be ready.
  LOG(INFO) << "Starting all threads.";
  startAll.notify();

  // This is idle while(1) loop
  while( !input.inputDone.getValue() )
  { sleep(1); }

  LOG(INFO) << "Finalizing system.";
  system->finalize();
  */
  ros::Rate loop_rate(10);
  while (ros::ok()){

    LOG(INFO) << "I'm working!";
    loop_rate.sleep();
  }
  return 0;
}
