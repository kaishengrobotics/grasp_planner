#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/common/distances.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/common/pca.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <math.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <tf/tf.h>
#include <grasp_planners/pca_grasp_planner.h>

void PCAGraspPlanner::findCloudBoundingBoxPCA(
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& obj_cloud,
                            BoundingBox& bb);
{
    Eigen::Vector3f eigen_values;
    Eigen::Matrix3f eigen_vectors;
    Eigen::Vector4f centroid;
    // Remove the RGB components from the input cloud
    pcl::PointCloud<pcl::PointXYZ> obj_cloud;
    pcl::copyPointCloud(*obj_color_cloud, obj_cloud);
    pcl::PCA<pcl::PointCloud<pcl::PointXYZ>> pca;
    try{
      pca.setInputCloud(obj_cloud.makeShared());
      centroid = pca.getMean();
      eigen_values = pca.getEigenValues();
      eigen_vectors = pca.getEigenVectors();
    } catch(pcl::InitFailedException ife)
    {
      ROS_WARN_STREAM("Failed to compute PCA");
      ROS_WARN_STREAM("ife: " << ife.what());
    }
    // Ensure the coordinate system is right handed
    eigen_vectors.col(2) = eigen_vectors.col(0).cross(eigen_vectors.col(1));
    Eigen::Matrix4f proj_transform;
    proj_transform.block<3,3>(0,0) = eigen_vectors.transpose();
    proj_transform.block<3,1>(0,3) = -1.f * (proj_transform.block<3,3>(0,0) * centroid.head<3>());
    // Transform points into eigen space to find extrema and compute ranges
    pcl::PointCloud<pcl::PointXYZ> proj;
    // pca.project(obj_cloud, proj);
    pcl::transformPointCloud(obj_cloud, proj, proj_transform);
    
    pcl::PointCloud<pcl::PointXYZ> proj_min;
    pcl::PointCloud<pcl::PointXYZ> proj_max;
    pcl::getMinMax3D(proj, proj_min, proj_max);
    bb.size(0) = fabs(proj_max.x-proj_min.x);
    bb.size(1) = fabs(proj_max.y-proj_min.y);
    bb.size(2) = fabs(proj_max.z-proj_min.z);
    
    Eigen::Vector3f box_mean_eigenspace = 0.5f*(proj_max.getVector3fMap() +
                                                proj_min.getVector3fMap());
    
    // Compute the transform into the object's space
    Eigen::Quaternionf orientation(eigen_vectors);
    orientation.normalize();
    Eigen::Vector3f rectified_centroid = eigen_vectors*box_mean_eigenspace + \
                                         centroid.head<3>();
    bb.center_pose.pose.position.x = rectified_centroid[0];
    bb.center_pose.pose.position.y = rectified_centroid[1];
    bb.center_pose.pose.position.z = rectified_centroid[2];
    bb.center_pose.pose.orientation.x = orientation.x();
    bb.center_pose.pose.orientation.y = orientation.y();
    bb.center_pose.pose.orientation.z = orientation.z();
    bb.center_pose.pose.orientation.w = orientation.w();
    bb.center_pose.header.frame_id = obj_cloud->header.frame_id; 
    bb.center_ort_mat = eigen_vectors;
   
    if (debug_)
      ROS_INFO_STREAM("BB size: " << bb.size);
    
    if (vis_bounding_box_)
    {
      ROS_INFO_STREAM("eigen values:\n" << eigen_values);
      ROS_INFO_STREAM("eigen vectors\n" << eigen_vectors);
      // ROS_INFO_STREAM("e1 X e2 = " << eig3);
      ROS_INFO_STREAM("pca centroid:\n" << centroid);
      ROS_INFO_STREAM("box_mean:\n" << box_mean_eigenspace);
      ROS_INFO_STREAM("Recovered centroid:\n" << rectified_centroid);
      ROS_INFO_STREAM("Recovered orientation:\n"<< bb.center_pose.orientation);
      /* visualizes the inputed pointcloud */
      pcl::visualization::PCLVisualizer viewer("PCL Viewer");
      viewer.setBackgroundColor(0.0,0.0,0.7);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(obj_color_cloud);
      viewer.addPointCloud<pcl::PointCloud<pcl::PointXYZ>RGB>(obj_color_cloud, rgb, "object_cloud");
      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                              "object_cloud");
    
      viewer.addCube(rectified_centroid, orientation, bb.size(0), bb.size(1), bb.size(2));
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
    	                         pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
    				 "cube");
      while(!viewer.wasStopped())
      {
        viewer.spinOnce();
      }
}
}

void PCAGraspPlanner::genOverheadGraspPoseBB(const BoundingBox& bb, 
                            geometry_msgs::PoseStamped& grasp_pose)
{
    // Find the axis whose angle between itself and the z axis 
    // (grativity direction) is the smallest. 
    double max_z = fabs(bb.center_ort_mat(2, 0));
    int max_z_index = 0;
    for(int i = 1; i < 3; ++i) 
    {
        double z_val = fabs(bb.center_ort_mat(2, i));
        if(z_val > max_z)
        {
            max_z = z_val;
            max_z_index = i;
        }
    }
    
    // Compute the position of the overhead grasp.
    Eigen::Vector3f z_vec = bb.center_ort_mat.col(max_z_index);
    // Make sure the grasp position is at the top instead of bottom. 
    if(z_vec(2) < 0.)
        z_vec *= -1;
    z_vec *= bb.size(max_z_index);
    grasp_pose.pose.position.x = bb.center_pose.position.x + z_vec(0); 
    grasp_pose.pose.position.y = bb.center_pose.position.y + z_vec(1); 
    grasp_pose.pose.position.z = bb.center_pose.position.z + z_vec(2);

    // Compute the orientation of the overhead grasp.
    // We assume the x axis of the gripper palm/grasping link points
    // from the palm to the middle of the two finger tips.
    // The y axis points from the right finger to the left.
    Eigen::Matrix3f grasp_ort_mat;
    grasp_ort_mat.col(0) = -1. * z_vec;
    // Among the other two boudning box orientation axis, we select 
    // the one with a smaller size/eigenvalues to be the y axis. 
    // This means the robot grasps the thinner side of the object.

    grasp_pose.header.frame_id = bb.center_pose.header.frame_id;
    return;
}

