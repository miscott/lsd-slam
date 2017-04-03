

#include "KeyframeLibrary.h"


namespace lsd_slam {

  using namespace std;

  KeyframeLibrary::KeyframeLibrary()
    : _idToKeyFrame(),
     _idToKeyFrameMutex(),
    _keyframes(),
     _keyframesMutex()
     {;}

     KeyframeLibrary::~KeyframeLibrary()
     {;}



     void KeyframeLibrary::addKeyframe( const Frame::SharedPtr &frame)
     {
       addJustKeyframe(frame);
       addJustIdToKeyframe( frame );
     }

     void KeyframeLibrary::dropKeyframe( const Frame::SharedPtr &frame)
     {
         LockGuard lock(_idToKeyFrameMutex);
         _idToKeyFrame.erase(frame->id());
     }

       // isolate these two functions so I can trace when one is used w/o the other
       void KeyframeLibrary::addJustIdToKeyframe( const Frame::SharedPtr &frame)
       {
         LockGuard guard(_idToKeyFrameMutex);
         _idToKeyFrame.insert( std::make_pair(frame->id(), frame) );
       }

       void KeyframeLibrary::addJustKeyframe( const Frame::SharedPtr &frame)
       {
         LockGuard guard( _keyframesMutex );
         frame->idxInKeyframes = _keyframes.size();
         _keyframes.push_back( frame );
       }

}
