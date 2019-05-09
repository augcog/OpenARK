#ifndef _CAMERA_SETUP_H_
#define _CAMERA_SETUP_H_

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

    }; //CameraSetup

} //ark

#endif //_CAMERA_SETUP_H_