#include "HumanBody.h"

namespace ark {
	HumanBody::HumanBody() {
		MPIISkeleton2D.resize(15);
	}

	HumanBody::~HumanBody() {

	}

	std::vector<cv::Point2i> HumanBody::getMPIISkeleton2D() {
		return MPIISkeleton2D;
	}

	cv::Vec3f HumanBody::getHeadDirection() {
		return headDirection;
	}
}