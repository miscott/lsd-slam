

#include "KeyframeLibrary.h"


namespace lsd_slam {

  using namespace std;

  KeyframeLibrary::KeyframeLibrary()
    : _idToKeyFrame(),
     _idToKeyFrameMutex(),
    _keyframesAll(),
     _keyframesMutex()
     {;}




     void KeyframeLibrary::addKeyframe( const Frame::SharedPtr &frame)
     {
       addJustKeyframe(frame);
       addJustIdToKeyframe( frame );
     }

     void KeyframeLibrary::dropKeyframe( const Frame::SharedPtr &frame)
     {
         LockGuard lock(_idToKeyFrameMutex;
         _idToKeyFrame.erase(frame.id());
     }

     void KeyframeLibrary::dropCurrentKeyframe()
     {
       dropKeyframe( _system.currentKeyFrame() );
     }


       // isolate these two functions so I can trace when one is used w/o the other
       void KeyframeLibrary::addJustIdToKeyframe( const Frame::SharedPtr &frame)
       {
         lock_guard<mutex> guard(_idToKeyFrameMutex);
         _idToKeyFrameMutex.insert( frame->id(), frame );
       }

       void KeyframeLibrary::addJustKeyframe( const Frame::SharedPtr &frame);
       {
         lock_guard<mutex> guard( _keyframesMutex );
         newKeyframe->idxInKeyframes = keyframesAll.const_ref().size();
         _keyframes.push_back( frame );
       }
     }

}
