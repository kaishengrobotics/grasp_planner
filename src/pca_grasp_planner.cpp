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
#include <pca_grasp_planner.h>

// typedef sensor_msgs::PointCloud2 MsgCloud;
// typedef pcl::PointXYZRGB pcl::PointCloud<pcl::PointXYZ>RGB;
// typedef pcl::PointCloud<pcl::PointCloud<pcl::PointXYZ>RGB> PCXYZRGB;
// typedef pcl::Normal PtNormals;
// typedef pcl::PointCloud<PtNormals> PCNormals;
// typedef pcl::PointXYZ pcl::PointCloud<pcl::PointXYZ>;
// typedef pcl::PointCloud<pcl::PointCloud<pcl::PointXYZ>> PCXYZ;
// typedef pcl::search::KdTree<pcl::PointCloud<pcl::PointXYZ>RGB>::Ptr KdTreePtr;
// using boost::shared_ptr;

void PCAGraspPlanner::findCloudBoundingBoxPCA(
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr& obj_cloud, 
                            geometry_msgs::Pose& pose, double& width, 
                            double& height, double& depth)
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
      ROS_DEBUG_STREAM("Getting mean");
      centroid = pca.getMean();
      ROS_DEBUG_STREAM("Getting eigen values");
      eigen_values = pca.getEigenValues();
      ROS_DEBUG_STREAM("Getting eigen vectors");
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
    PCXYZ proj;
    // pca.project(obj_cloud, proj);
    pcl::transformPointCloud(obj_cloud, proj, proj_transform);
    
    pcl::PointCloud<pcl::PointXYZ> proj_min;
    pcl::PointCloud<pcl::PointXYZ> proj_max;
    pcl::getMinMax3D(proj, proj_min, proj_max);
    width = fabs(proj_max.x-proj_min.x);
    height = fabs(proj_max.y-proj_min.y);
    depth = fabs(proj_max.z-proj_min.z);
    
    Eigen::Vector3f box_mean_eigenspace = 0.5f*(proj_max.getVector3fMap() +
                                                proj_min.getVector3fMap());
    
    // Compute the transform into the object's space
    Eigen::Quaternionf orientation(eigen_vectors);
    orientation.normalize();
    Eigen::Vector3f rectified_centroid = eigen_vectors*box_mean_eigenspace + \
                                         centroid.head<3>();
    pose.position.x = rectified_centroid[0];
    pose.position.y = rectified_centroid[1];
    pose.position.z = rectified_centroid[2];
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    
    if (debug_)
      ROS_INFO_STREAM("width = " << width << "\theight = " << height <<
                      "\tdepth = " << depth);
    
    if (visualize_bounding_box_)
    {
      ROS_INFO_STREAM("eigen values:\n" << eigen_values);
      ROS_INFO_STREAM("eigen vectors\n" << eigen_vectors);
      // ROS_INFO_STREAM("e1 X e2 = " << eig3);
      ROS_INFO_STREAM("pca centroid:\n" << centroid);
      ROS_INFO_STREAM("box_mean:\n" << box_mean_eigenspace);
      ROS_INFO_STREAM("Recovered centroid:\n" << rectified_centroid);
      ROS_INFO_STREAM("Recovered orientation:\n"<< pose.orientation);
      /* visualizes the inputed pointcloud */
      pcl::visualization::PCLVisualizer viewer("PCL Viewer");
      viewer.setBackgroundColor(0.0,0.0,0.7);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(obj_color_cloud);
      viewer.addPointCloud<pcl::PointCloud<pcl::PointXYZ>RGB>(obj_color_cloud, rgb, "object_cloud");
      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2,
                                              "object_cloud");
    
      viewer.addCube(rectified_centroid, orientation, width, height, depth);
      viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
    	                         pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
    				 "cube");
      while(!viewer.wasStopped())
      {
        viewer.spinOnce();
      }
}
}


