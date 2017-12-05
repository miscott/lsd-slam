cmake_minimum_required(VERSION 2.8.7)
project(lsd_slam)
include(ExternalProject)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

option(BUILD_GUI "Build Pangolin-based GUI" ON)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})
set(RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR})

include( ${PROJECT_SOURCE_DIR}/cmake/BuildG3Log.cmake )
set( EXTERNAL_PROJECTS g3log )

include( ${PROJECT_SOURCE_DIR}/cmake/BuildLibActiveObject.cmake )
list( APPEND EXTERNAL_PROJECTS libactive_object )

option( USE_OPENMP "Enable OpenMP (currently only in local copy of G2O)" OFF )

##==== Option to build local copy of G2O ============================
option( BUILD_LOCAL_G2O "Build a local copy of G2O" ON )
if( BUILD_LOCAL_G2O )
	set( G2O_USE_OPENMP ${USE_OPENMP} )
	list( APPEND EXTERNAL_PROJECTS G2O )

	include( ${PROJECT_SOURCE_DIR}/cmake/BuildG2O.cmake )
elseif()
	find_package(G2O REQUIRED)

	# These are the G2O libraries I use
	set( G2O_LIBRARIES
		${G2O_CORE_LIBRARY}
		${G2O_STUFF_LIBRARY}
		${G2O_SOLVER_CSPARSE_EXTENSION}
		${G2O_SOLVER_CSPARSE}
		${G2O_TYPES_SBA}
		${G2O_TYPES_SIM3} )
endif()

if(BUILD_GUI)
  ##====  Build local copy of Pangolin? ======================
  option( BUILD_LOCAL_PANGOLIN "Build a local copy of Pangolin" ON )
  if( BUILD_LOCAL_PANGOLIN )
  	list( APPEND EXTERNAL_PROJECTS Pangolin )
  	include( ${PROJECT_SOURCE_DIR}/cmake/BuildPangolin.cmake )
  elseif()
  	find_package(Pangolin 0.1 REQUIRED)
  endif()
endif()


## I haven't packaged suitesparse yet...
find_package(SuiteParse REQUIRED)
list( APPEND G2O_LIBRARIES
  ${CSPARSE_LIBRARY} )
list( APPEND G2O_INCLUDE_DIR
  ${CSPARSE_INCLUDE_DIR}
  ${CHOLMOD_INCLUDE_DIR} )

find_package(Boost COMPONENTS thread filesystem system REQUIRED)
find_package(Eigen3 REQUIRED)

## Command used to build the external projects
set( EXTERNAL_PROJECT_PARALLELISM 4 CACHE STRING "Level of parallelism when making ExternalProjects" )
set( EXTERNAL_PROJECT_MAKE_COMMAND "make" )
if( EXTERNAL_PROJECT_PARALLELISM GREATER 0 )
	message( "Building ExternalProjects in Parallel, to disable set EXTERNAL_PROJECT_PARALLELISM to 0")
	set( EXTERNAL_PROJECT_MAKE_COMMAND make -j${EXTERNAL_PROJECT_PARALLELISM} )
endif()

# FabMap
# uncomment this part to enable fabmap
#add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap)
#include_directories(${PROJECT_SOURCE_DIR}/thirdparty/openFabMap/include)
#add_definitions("-DHAVE_FABMAP")
#set(FABMAP_LIB openFABMAP )


##==== Check and define the architecture
## Architecture-specific optimizations
option( CUDA_VERSION "Use CUDA" "")

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )
if( ${ARCHITECTURE} STREQUAL "aarch64" )
	# Jetson version uses OpenCV4Tegra from NVidia

	## For compilation on Jetson TX1
	set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a") # -DENABLE_NEON" )

elseif( ${ARCHITECTURE} STREQUAL "x86_64" )

  # -DENABLE_SSE enables Jakob's original SSE optimizations in his code,
	# it has nothing to do with compiler behavior
  set( CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -msse4.1 -msse3 -msse2 -msse -march=native -DENABLE_SSE" )
endif()

set( OPENCV_COMPONENTS core highgui imgproc video calib3d )
find_package(OpenCV COMPONENTS ${OPENCV_COMPONENTS} REQUIRED)

if( OpenCV_VERSION_MAJOR EQUAL "3" )
  message( STATUS "Using OpenCV 3.x")
  add_definitions( -DOPENCV3 )
else()
  message( STATUS "Using OpenCV 2.x")
  add_definitions( -DOPENCV2 )
endif()

if( CUDA_VERSION )
	find_package( CUDA REQUIRED ${CUDA_VERSION} )
  SET( CUDA_USE_STATIC_CUDA_RUNTIME OFF )
endif()


## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set( LSDSLAM_INCLUDE_DIRS
	${CMAKE_SOURCE_DIR}/lib
	${CMAKE_SOURCE_DIR}/lib/lsd_core
	${EIGEN3_INCLUDE_DIR}
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	${Pangolin_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/thirdparty/Sophus
  ${G3LOG_INCLUDE_DIR}
  ${LIBACTIVE_OBJECT_INCLUDE_DIRS}
  SYSTEM ${G2O_INCLUDE_DIR} )
include_directories( ${LSDSLAM_INCLUDE_DIRS} )


## This seems strange to me
if( APPLE)
  include_directories( /usr/local/include )
  link_directories( /usr/local/lib )
	set( CMAKE_MACOSX_RPATH OFF )
endif()


add_subdirectory( lib/ )
add_subdirectory( tools/ )

option( BUILD_UNIT_TESTS "Build unit tests" OFF )
if( BUILD_UNIT_TESTS )
  ## Build a local copy of gtest (as per the GTest FAQ)
	include( ${CMAKE_SOURCE_DIR}/cmake/BuildGTestLibraries.cmake )
	list( APPEND EXTERNAL_PROJECTS gtest )

  add_subdirectory( test/unit/ )
endif()

# Custom target to build dependencies
add_custom_target( deps DEPENDS ${EXTERNAL_PROJECTS} )
