#pragma once
#include "FrameObject.h"

namespace ark {
	/** Class representing a human body in the current frame
	* @see HumanBody.cpp
	*/
	class HumanBody
	{
	public:
		HumanBody();
		
		~HumanBody();

		std::vector<cv::Point2i> getMPIISkeleton2D();

		cv::Vec3f getHeadDirection();

	private:
		std::vector<cv::Point2i> MPIISkeleton2D;

		cv::Vec3f headDirection;
	};
}