#ifndef PCA_GRASP_PLANNER_H
#define PCA_GRASP_PLANNER_H

#include <pcl/point_types.h>

/**
* A PCA-based grasp planner.
*/
class PCAGraspPlanner{
    public:
        PCAGraspPlanner();

        /**
        * Compute the bounding box associated with an object cloud using PCA
        *
        * \param obj_color_cloud The cloud to bound
        * \param pose [output] The object's pose
        * \param width [output] The bounding box width (1st principal component range)
        * \param height [output] The bounding box height (2nd principal component range)
        * \param depth [output] The bounding box depth (3rd principal component range)
        */
        void findCloudBoundingBoxPCA(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& obj_cloud,
                                    geometry_msgs::Pose& pose, double& width, 
                                    double& height, double& depth);
        
    // private:
};

#endif
