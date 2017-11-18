#include "stdafx.h"
#include "RGBCamera.h"

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
