#pragma once

#include "hand_and_avatar/Detector.h"
#include "hand_and_avatar/FramePlane.h"

namespace ark {
    /** Plane detector class supporting the detection of multiple planes within a depth projection image (xyz map).
     * @see PlaneDetector
     */
    class PlaneDetector : public Detector {
    public:
        /** Construct a new plane detector instance
          * @param params detection parameters. If not specified, uses default parameter values.
          */
        PlaneDetector(DetectionParams::Ptr params = nullptr);

        /**
         * Obtain a list of planes in the current frame from this detector.
         */
        const std::vector<FramePlane::Ptr> & getPlanes() const;

        /** Shared pointer to PlaneDetector instance */
        typedef std::shared_ptr<PlaneDetector> Ptr;

        /** 
         * Get the normal map from this plane detector, if available 
         * @return the normal map; if one is not available, returns an empty image
         */
        cv::Mat getNormalMap();

    protected:
        /** Implementation of plane detection algorithm */
        void detect(cv::Mat & image) override;

    private:
        /** stores currently detected planes */
        std::vector<FramePlane::Ptr> planes;

        /**
         * Matrix storing the surface normal vectors (facing viewer) at each point in the observable world.
         * This is computed automatically from the depth map if required.
         * Implementers of subclasses do not need to deal with this at all.
         * Matrix type CV_32FC3
         */
        cv::Mat normalMap;

        /**
         * helper function for getting the equations of planes given xyz and normal maps.
         * @param[in] xyz_map the xyz map
         * @param[in] normal_map the normal map
         * @param[out] output_equations vector to be filled with equations of planes (in the form ax + by - z + c = 0)
         * @param[out] output_points vector to be filled with vectors of ij coordinate points on planes
         * @param[out] output_points_xyz vector to be filled with vectors of xyz coordinate points on planes
         * @param[in] params plane detection parameters
         */
        void detectPlaneHelper(const cv::Mat & xyz_map, const cv::Mat & normal_map, std::vector<Vec3f> & output_equations,
            std::vector<VecP2iPtr> & output_points, std::vector<VecV3fPtr> & output_points_xyz,
            DetectionParams::Ptr params = nullptr);
    };
}