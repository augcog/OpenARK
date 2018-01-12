#include "stdafx.h"
#include "version.h"
#include "RGBCamera.h"

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
