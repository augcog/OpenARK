//
// Created by Kuan Lu on 1/28/18.
//
#pragma once

#include <functional>
#include <iostream>
#include <unordered_map>
#include <opencv2/opencv.hpp>
#include "openark/util/Types.h"


namespace ark {

    typedef std::function<void(MultiCameraFrame::Ptr)> KeyFrameAvailableHandler;
    typedef std::function<void(MultiCameraFrame::Ptr)> FrameAvailableHandler;
    typedef std::function<void(void)> LoopClosureDetectedHandler;
    typedef std::function<void(int)> SparseMapCreationHandler;
    typedef std::function<void(int, int)> SparseMapMergeHandler;
    typedef std::unordered_map<std::string, KeyFrameAvailableHandler> MapKeyFrameAvailableHandler;
    typedef std::unordered_map<std::string, FrameAvailableHandler> MapFrameAvailableHandler;
    typedef std::unordered_map<std::string, LoopClosureDetectedHandler> MapLoopClosureDetectedHandler;
    typedef std::unordered_map<std::string, SparseMapCreationHandler> MapSparseMapCreationHandler;
    typedef std::unordered_map<std::string, SparseMapMergeHandler> MapSparseMapMergeHandler;

    class SLAMSystem {
    public:

        /** Push an image with the given time stamp to the SLAM system */
        virtual void PushFrame(const MultiCameraFrame::Ptr) = 0;

        /** Start the SLAM system */
        virtual void Start() = 0;

        /** Request the SLAM system to stop */
        virtual void RequestStop() = 0;

        /** Force shutdown the SLAM system */
        virtual void ShutDown() = 0;

        /** Returns true if the SLAM system is running */
        virtual bool IsRunning() = 0;

        /** Add a handler that will be called each time a KEYframe becomes available. */
        virtual void AddKeyFrameAvailableHandler(KeyFrameAvailableHandler handler, std::string handlerName);

        /** Remove a keyframe available handler with the specified name. */
        virtual void RemoveKeyFrameAvailableHandler(std::string handlerName);

        /** Add a handler that will be called each time a frame becomes available. */
        virtual void AddFrameAvailableHandler(FrameAvailableHandler handler, std::string handlerName);

        /** Remove a frame available handler with the specified name. */
        virtual void RemoveFrameAvailableHandler(std::string handlerName);

        /** Add a handler that will be called each time a loop closure occurs. */
        virtual void AddLoopClosureDetectedHandler(LoopClosureDetectedHandler handler, std::string handlerName);

        /** Remove a loop closure handler with the specified name. */
        virtual void RemoveLoopClosureDetectedHandler(std::string handlerName);

        /** Add a handler that will be called each time two sparse maps are merged. */
        virtual void AddSparseMapMergeHandler(SparseMapMergeHandler handler, std::string handlerName);

        /** Remove a sparse map merge handler with the specified name. */
        virtual void RemoveSparseMapMergeHandler(std::string handlerName);

        /** Add a handler that will be called each time a sparse map is created. */
        virtual void AddSparseMapCreationHandler(SparseMapCreationHandler handler, std::string handlerName);

        /** Remove a sparse map creation handler with the specified name. */
        virtual void RemoveSparseMapCreationHandler(std::string handlerName);

        /** Destructor for the SLAM system. */
        virtual ~SLAMSystem() = default;

    protected:
        MapKeyFrameAvailableHandler mMapKeyFrameAvailableHandler;
        MapFrameAvailableHandler mMapFrameAvailableHandler;
        MapLoopClosureDetectedHandler mMapLoopClosureHandler;
        MapSparseMapCreationHandler mMapSparseMapCreationHandler;
        MapSparseMapMergeHandler mMapSparseMapMergeHandler;
    };
}
