#ifndef PCA_GRASP_PLANNER_H
#define PCA_GRASP_PLANNER_H

#include <pcl/point_types.h>

/**
* The definition of a bounding box.
*/
struct BoundingBox{
    // The pose of the bounding box center.
    geometry_msgs::PoseStamped& center_pose;
    // The rotation matrix of the center orientation.
    Eigen::Matrix3f center_ort_mat;
    // The bounding box size. The first to third value corresponds to the 
    // first to third principal component range.
    Eigen::Vector3f size;
};

/**
* A PCA-based grasp planner.
*/
class PCAGraspPlanner{
    public:
        PCAGraspPlanner(bool vis_bb=false): vis_bounding_box_(vis_bb) {};

        // TODO: transform the point cloud to make sure the z axis is the 
        // gravity direction.

        /**
        * Compute the bounding box associated with an object cloud using PCA.
        *
        * \param obj_cloud The object point cloud. 
        * \param bounding_box [output] The object bounding box.
        */
        void findCloudBoundingBoxPCA(
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& obj_cloud,
                            BoundingBox& bb);
        /**
        * Generate a heuristic overhead grasp using the object bounding box.
        * We assume the negative z axis of the bounding box is along the gravity direction.
        * \param bb The bounding box.
        * \param grasp_pose [output] The grasping pose.
        */
        void genOverheadGraspPoseBB(const BoundingBox& bb, 
                                    geometry_msgs::Pose &grasp_pose);
        
    private:
        // Visualize the bounding box or not.
        bool vis_bounding_box_;

};

#endif
