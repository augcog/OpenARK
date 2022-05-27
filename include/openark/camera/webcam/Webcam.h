#pragma once

#include "Version.h"
#include "openark/camera/RGBCamera.h"

namespace ark {
    /**
    * Class defining the behavior of a standard webcam.
    */
    class Webcam : public RGBCamera
    {
    public:
        /**
        * Constructs a new webcam instance.
        * @param code the code for the webcam. Default is 0 if there is only one webcam connected
        */
        explicit Webcam(int code = 0);

        /**
        * Deconstructs a webcam instance.
        */
        ~Webcam();

        /**
        * Updates the webcam infomration with the current frame.
        */
        void update() override;

    private:
    };
}