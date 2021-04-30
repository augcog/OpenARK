#pragma once

#include <vector>
#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/registration/icp.h>


namespace ark{

template<typename PointT>
class CorrespondenceRansac{
    using CloudPtr = typename pcl::PointCloud<PointT>::Ptr;
public:

    //This function performs ransac on a set of feature
    //correspondences to determince which are inliers
    //also returns an estimate of the transform between clouds
    static void getInliersWithTransform(
            CloudPtr src, 
            CloudPtr tgt,
            const std::vector<int>& correspondences,
            int num_samples, 
            float inlier_threshold, 
            int num_iterations,
            int& num_inliers_out,
            std::vector<bool>& inliers_out,
            Eigen::Affine3d transform_out){ 

        //We need at least 3 samples to compute a transform
        if(num_samples<3){
            std::cerr << "Need at least 3 samples to perform RANSAC\n";
            std::cerr << "Setting num_samples to 3\n";
            num_samples =3;
        }

        //We don't know what was fed in with inliers_out so we should resize
        //and reset all values to false
        inliers_out.resize(correspondences.size());
        for(size_t i = 0; i<inliers_out.size(); i++){
            inliers_out[i] = false;
        }

        //Ensure correspondences is same size as src cloud
        if(src->size()!=correspondences.size()){
            std::cerr << 
                "There must be correspondence for each point in src cloud\n";
            std::cerr << 
                "src->size(): " << src->size() << " corr: " << correspondences.size() << '\n';

            transform_out = Eigen::Affine3d::Identity();
            num_inliers_out = 0;
            return;
        }

        //If we have less than the number of samples needed for ransac
        //just return identity transform
        if(inliers_out.size()<num_samples){
            std::cerr << "Not enough points for ransac\n";
            transform_out = Eigen::Affine3d::Identity();
            num_inliers_out = 0;
            return;
        }

        //Used for storing the best iteration result
        int best_inliers = 0;           
        Eigen::Affine3d best_transform = Eigen::Affine3d::Identity();

        //This will be for transforming cloud
        //I think using PCL transform cloud will be faster than 
        //Applying transform to each point individually, despite extra memory usage
        //Should test eventually though, similarly using Eigen to store clouds may be faster
        CloudPtr temp_cloud( new pcl::PointCloud<PointT>);


        //Run RANSAC the specified number of times 
        for ( int r = 0; r< num_iterations; r++){
            //Choose random points
            std::vector<Eigen::Vector3d> s_points(num_samples);
            std::vector<Eigen::Vector3d> t_points(num_samples);
            bool valid =true;
            for( int i = 0; i<num_samples; i++){

                int index(rand()%src->size());
                if(correspondences[index]<0){
                    valid =false;
                    break;
                }
                s_points[i] <<  (double)(src->points[index].x),
                                (double)(src->points[index].y),
                                (double)(src->points[index].z);
                t_points[i] <<  (double)(tgt->points[correspondences[index]].x),
                                (double)(tgt->points[correspondences[index]].y),
                                (double)(tgt->points[correspondences[index]].z);
            }
            if(!valid)
                continue;


            Eigen::Matrix3d W(Eigen::Matrix3d::Zero()); //point correspondence matrix used to compute R

            //compute mean points
            Eigen::Vector3d s_mu(Eigen::Vector3d::Zero()), t_mu(Eigen::Vector3d::Zero());
            for( int i = 0; i<num_samples; i++){
                s_mu+=s_points[i];
                t_mu+=t_points[i];
            }
            s_mu/=num_samples;
            t_mu/=num_samples;



            //compute W as W = sum(s_x*t_x.transpose())
            for( int i = 0; i<num_samples; i++){
                //std::cout<< (s_points[i]-s_mu)*(t_points[i]-t_mu).transpose() << '\n';
                W+=(s_points[i]-s_mu)*(t_points[i]-t_mu).transpose();
            }
            //std::cout << "W:= " << W << '\n';
            //compute transform from SVD
            Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
            //std::cout << "rank: " << svd.rank() << '\n';
            //ensure W was full rank
            //if(svd.rank()<3)
            //  continue;
            Eigen::Quaterniond R((svd.matrixU()*svd.matrixV().transpose()).transpose());
            Eigen::Translation3d t(t_mu - R*s_mu);
            Eigen::Affine3d transform = t*R;



            //apply transform to cloud
            pcl::transformPointCloud(*src,*temp_cloud,transform);


            //count inliers
            int inliers=0; 
            for (size_t i = 0; i < src->size(); ++i)
            {
                if(correspondences[i]>=0)
                    inliers+=checkThreshold(temp_cloud->points[i],tgt->points[correspondences[i]],inlier_threshold);
            }

            //save best result
            if(inliers>best_inliers){
                best_inliers=inliers;
                best_transform = transform;
            }        

        }
        //Fill in output with best result
        transform_out = best_transform;
        pcl::transformPointCloud(*src,*temp_cloud,best_transform);
        num_inliers_out= 0;
        for (size_t i = 0; i < src->size(); ++i)
        {
            if(correspondences[i]>=0)
                num_inliers_out+=inliers_out[i] = checkThreshold(temp_cloud->points[i],tgt->points[correspondences[i]],inlier_threshold);
        }



    }   

private:

    //Checks transformed points against threshold
    static bool checkThreshold(const PointT& p1, const PointT& p2, float inlier_threshold){
        return ((p1.x-p2.x)*(p1.x-p2.x)+
                (p1.y-p2.y)*(p1.y-p2.y)+
                (p1.z-p2.z)*(p1.z-p2.z))<
                (inlier_threshold*inlier_threshold);
    }

    CorrespondenceRansac(){}
};



}//namespace ICP

