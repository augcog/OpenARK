#pragma once
#include "Types.h"

namespace ark{

    class CameraSetup{
    public:
        virtual ~CameraSetup(){};

        virtual const std::string getModelName() const
        {
            return "CameraSetup";
        }

        virtual void start() =0;

        virtual void update(MultiCameraFrame & frame) =0;

        virtual std::vector<float> getColorIntrinsics() =0;

        virtual cv::Size getImageSize() const =0;

    }; //CameraSetup

} //ark
