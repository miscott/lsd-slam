
#include <memory>

#include "IOWrapper/Output3DWrapper.h"


namespace lsd_slam {

  using std::shared_ptr;

  class SlamSystem;

  /// A thin wrapper to gather all the IO functions in a "namespace" within SlamSystem
  struct IO {

    IO( SlamSystem &system );

    SlamSystem &_system;

    //=== Debugging output functions =====
    void set3DOutputWrapper( const shared_ptr<Output3DWrapper> &outputWrapper);
    const shared_ptr<Output3DWrapper> &outputWrapper() { return _outputWrapper; }

//    void set3DOutputWrapper( Output3DWrapper* outputWrapper );

    void publishPose(const Sophus::Sim3f &pose ) ;
    void publishTrackedFrame( const Frame::SharedPtr &frame ) ;
    void publishKeyframeGraph( void ) ;
    void publishKeyframe(  const Frame::SharedPtr &frame ) ;
    void publishDepthImage( unsigned char* data  );

    void publishCurrentKeyframe();

  private:
    shared_ptr<Output3DWrapper> _outputWrapper;	// no lock required

  };
}
