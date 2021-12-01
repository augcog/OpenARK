#include "stdafx.h"
#include "Version.h"
#include "openark/camera/RGBCamera.h"

namespace ark {
    void RGBCamera::update()
    {
    }

    /***
    Returns the next frame if next frame is recorded
    Returns the previous frame if next frame is not recorded
    ***/
    cv::Mat RGBCamera::getFrame() const
    {
        return frame;
    }
}
