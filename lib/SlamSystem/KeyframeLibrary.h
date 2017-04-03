#pragma once

#include <vector>
#include <unordered_map>
#include <memory>
#include <mutex>

#include "DataStructures/Frame.h"

namespace lsd_slam {

  class KeyframeLibrary {
  public:

    typedef std::mutex Mutex;
    typedef std::lock_guard<Mutex> LockGuard;

    typedef std::vector< Frame::SharedPtr > KeyframesAll;
    typedef std::unordered_map< int, Frame::SharedPtr > IdToKeyFrame;

    KeyframeLibrary();
    ~KeyframeLibrary();

    Mutex &mutex() { return _keyframesMutex; }

    void addKeyframe( const Frame::SharedPtr &frame);

    void dropKeyframe( const Frame::SharedPtr &frame );
    //void dropCurrentKeyframe();

    int size() const { return _keyframes.size(); }

    void addJustKeyframe( const Frame::SharedPtr &frame);
    void addJustIdToKeyframe( const Frame::SharedPtr &frame);

    const Frame::SharedPtr &at( int i ) { return _keyframes.at(i); }
    const Frame::SharedPtr &operator[]( int i ) { return _keyframes[i]; }

    KeyframesAll::iterator begin() { return _keyframes.begin(); }
    KeyframesAll::iterator end() { return _keyframes.end(); }


  private:

    /** Maps frame ids to keyframes. Contains ALL Keyframes allocated, including the one that currently being created. */
    /* this is where the shared pointers of Keyframe Frames are kept, so they are not deleted ever */
    IdToKeyFrame _idToKeyFrame;
    std::mutex _idToKeyFrameMutex;

    // TODO.   Move back to private
    // contains ALL keyframes, as soon as they are "finished".
    // does NOT yet contain the keyframe that is currently being created.
    //boost::shared_mutex keyframesAllMutex;
    KeyframesAll _keyframes;
    std::mutex _keyframesMutex;

  };

}
