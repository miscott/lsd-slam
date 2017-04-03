
set(GUI_SRCS
  ${CMAKE_CURRENT_SOURCE_DIR}/IOWrapper/OpenCV/ImageDisplay_OpenCV.cpp
)

# Set source files
set(lsdslam_SOURCE_FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/App/App.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/App/g3logger.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/App/InputThread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures/Frame.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures/FramePoseStruct.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DataStructures/FrameMemory.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DepthEstimation/DepthMap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/DepthEstimation/DepthMapPixelHypothesis.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/Configuration.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/FileUtils.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/globalFuncs.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/SophusUtil.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/settings.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/Undistorter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/Undistorter/UndistorterOpenCV.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/util/Undistorter/UndistorterPTAM.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/Sim3Tracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/Relocalizer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/SE3Tracker.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/Tracking/TrackingReference.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/IOWrapper/Timestamp.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GlobalMapping/FabMap.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GlobalMapping/KeyFrameGraph.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GlobalMapping/g2oTypeSim3Sophus.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/GlobalMapping/TrackableKeyFrameSearch.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem/OptimizationThread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem/MappingThread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem/TrackingThread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem/ConstraintSearchThread.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem/IO.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/SlamSystem/KeyframeLibrary.cpp
  ${GUI_SRCS}
)

# build shared library.
add_library(lsdslam STATIC ${lsdslam_SOURCE_FILES})
target_link_libraries(lsdslam
  ${FABMAP_LIB}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBS}
  ${G2O_LIBRARIES}
  ${G3LOG_LIB}
  active_object )
