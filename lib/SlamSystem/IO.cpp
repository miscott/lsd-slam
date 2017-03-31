
#include "SlamSystem.h"

namespace lsd_slam {

    IO::IO( SlamSystem &sys )
    : _system(sys),
      _outputWrapper( nullptr )
    {}

  void IO::set3DOutputWrapper( const shared_ptr<Output3DWrapper> &outputWrapper)
  {
    _outputWrapper = outputWrapper;
  }

  void IO::publishPose(const Sophus::Sim3f &pose )
  {
    if( _outputWrapper ) _outputWrapper->publishPose(pose);
  }

  void IO::publishTrackedFrame( const Frame::SharedPtr &frame )
  {
    if( _outputWrapper ) _outputWrapper->publishTrackedFrame( frame );
  }

  void IO::publishKeyframeGraph( void )
  {
    if( _outputWrapper ) _outputWrapper->publishKeyframeGraph( _system.keyFrameGraph() );
  }

  void IO::publishKeyframe(  const Frame::SharedPtr &frame )
  {
    if( _outputWrapper ) _outputWrapper->publishKeyframe( frame );
  }

  void IO::publishDepthImage( unsigned char* data  )
  {
    if( _outputWrapper ) _outputWrapper->updateDepthImage( data );
  }

}
