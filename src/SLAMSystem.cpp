# include "slam/SLAMSystem.h"

namespace ark {

    void SLAMSystem::AddKeyFrameAvailableHandler(KeyFrameAvailableHandler handler, std::string handlerName)
    {
        mMapKeyFrameAvailableHandler[handlerName] = handler;
    }

    void SLAMSystem::RemoveKeyFrameAvailableHandler(std::string handlerName)
    {
        auto handler = mMapKeyFrameAvailableHandler.find(handlerName);
        if (handler != mMapKeyFrameAvailableHandler.end())
            mMapKeyFrameAvailableHandler.erase(handler);
    }

    void SLAMSystem::AddFrameAvailableHandler(FrameAvailableHandler handler, std::string handlerName)
    {
        mMapFrameAvailableHandler[handlerName] = handler;
    }

    void SLAMSystem::RemoveFrameAvailableHandler(std::string handlerName)
    {
        auto handler = mMapFrameAvailableHandler.find(handlerName);
        if (handler != mMapFrameAvailableHandler.end())
            mMapFrameAvailableHandler.erase(handler);
    }

    void SLAMSystem::AddLoopClosureDetectedHandler(LoopClosureDetectedHandler handler, std::string handlerName)
    {
        mMapLoopClosureHandler[handlerName] = handler;
    }

    void SLAMSystem::RemoveLoopClosureDetectedHandler(std::string handlerName)
    {
        auto handler = mMapLoopClosureHandler.find(handlerName);
        if (handler != mMapLoopClosureHandler.end())
            mMapLoopClosureHandler.erase(handler);
    }

    void SLAMSystem::AddSparseMapMergeHandler(SparseMapMergeHandler handler, std::string handlerName)
    {
        mMapSparseMapMergeHandler[handlerName] = handler;
    }

    void SLAMSystem::RemoveSparseMapMergeHandler(std::string handlerName)
    {
        auto handler = mMapSparseMapMergeHandler.find(handlerName);
        if (handler != mMapSparseMapMergeHandler.end())
            mMapSparseMapMergeHandler.erase(handler);
    }

    void SLAMSystem::AddSparseMapCreationHandler(SparseMapCreationHandler handler, std::string handlerName)
    {
        mMapSparseMapCreationHandler[handlerName] = handler;
    }

    void SLAMSystem::RemoveSparseMapCreationHandler(std::string handlerName)
    {
        auto handler = mMapSparseMapCreationHandler.find(handlerName);
        if (handler != mMapSparseMapCreationHandler.end())
            mMapSparseMapCreationHandler.erase(handler);
    }

}
