#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>

#include <iostream>
#include <sstream>
#include <boost/make_shared.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/PointIndices.h>
#include <pcl/console/parse.h>
#include <eigen3/Eigen/Core>

// #include <types.h>
#include "types.h"

#include "read_pose.h"
#include "cloud_mapper.h"
#include "rectify.h"
#include "ground_segment.h"
#include "feature_points.h"
#include "curbfit.h"
#include "tracking.h"
#include "boundary_points.h"
#include "omp.h"
#include "ray_filter.h"




typedef std::pair<size_t, size_t> IndexRange;
typedef std::vector<IndexRange> scanIndices;

class RoadBoundaryDetectionNode {
  ros::Publisher road_boundary_left_pub_;
  ros::Publisher road_boundary_right_pub_;
  ros::Publisher ground_pub_;
  ros::Publisher road_pub_;
  // ros::Publisher obstacle_pub_;
  
public:
  RoadBoundaryDetectionNode(ros::NodeHandle& nh,
                            const std::string& ground_topic,
                            const std::string& left_boundary_topic,
                            const std::string& right_boundary_topic,
                            // const std::string& road_topic,
                            const bool& latch = false)
  {
    // road_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(road_topic, 1, latch);
    ground_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(ground_topic, 1, latch);
    road_boundary_left_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(left_boundary_topic, 1, latch);
    road_boundary_right_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(right_boundary_topic, 1, latch);

  }

  void scanCallback(const pcl::PointCloud<pcl::PointXYZI>& cloud) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);

    *cloud_in = cloud;
    
    //根据kitti点云数据存储特点计算点云laserID,并将ID存为intensity
    CloudMapper mapper1;
    pcl::PointCloud<pcl::PointXYZI>::Ptr completeCloudMapper(new pcl::PointCloud<pcl::PointXYZI>);
    mapper1.processByOri(cloud_in, completeCloudMapper);
    ROS_INFO("raw points number is %d", completeCloudMapper->points.size());

    //地面提取
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points_no(new pcl::PointCloud<pcl::PointXYZI>); //非地面点
    GroundSegmentation ground(completeCloudMapper);
    ground.groundfilter(ground_points, ground_points_no);
    
    // RayFilterGroundSegmentation ray_filter_segmenter(completeCloudMapper);
    // ray_filter_segmenter.process(ground_points, ground_points_no);
    
    ROS_INFO("ground points is %d", ground_points->points.size());
    ROS_INFO("no ground points is %d", ground_points_no->points.size());
    

    //根据之前计算的Intensity对地面点云进行mapper
    CloudMapper mapper2;
    scanIndices scanIDindices;
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_points_mapper(new pcl::PointCloud<pcl::PointXYZI>);
    mapper2.processByIntensity(ground_points, ground_points_mapper, scanIDindices);
    ROS_INFO("ground points mapper is %d", ground_points_mapper->points.size());

    //特征点提取
    pcl::PointCloud<pcl::PointXYZI>::Ptr featurePoints(new pcl::PointCloud<pcl::PointXYZI>);
    FeaturePoints curbExtract(ground_points_mapper, scanIDindices);
    curbExtract.extractFeatures(featurePoints);
    ROS_INFO("feature points is %d", featurePoints->points.size());

    //创建curb_refine对象
    BoundaryPoints refinePoints(*featurePoints);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusterPtr(2);
    
    for(int i = 0; i < clusterPtr.size(); i++)
    {
      clusterPtr[i] = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    }
    refinePoints.process(ground_points_no, clusterPtr);

    pcl::PointCloud<pcl::PointXYZI> left_boundary_cloud;
    pcl::PointCloud<pcl::PointXYZI> right_boundary_cloud;
    pcl::PointCloud<pcl::PointXYZI> ground_cloud;

    left_boundary_cloud = *clusterPtr[0];
    left_boundary_cloud.header = cloud.header;
    right_boundary_cloud = *clusterPtr[1];
    right_boundary_cloud.header = cloud.header;
    ground_cloud = *ground_points;
    ground_cloud.header = cloud.header;


    road_boundary_left_pub_.publish(left_boundary_cloud);
    road_boundary_right_pub_.publish(right_boundary_cloud);
    ground_pub_.publish(ground_cloud);

  }
};

int main(int argc, char** argv) {
  
  ros::init(argc, argv, "road_boundary_detection");
  ros::NodeHandle nh("~");
  ros::Subscriber cloud_sub;
  
  bool latch;
  std::string ground_topic, left_boundary_topic, right_boundary_topic, input_topic;
  
  latch = false;
  ground_topic = "ground_cloud";
  left_boundary_topic = "left_boundary";
  right_boundary_topic = "right_boundary";
  input_topic = "/kitti/velo/pointcloud";

  RoadBoundaryDetectionNode node(nh, ground_topic, left_boundary_topic,right_boundary_topic, latch);
  
  cloud_sub = nh.subscribe(input_topic, 1, &RoadBoundaryDetectionNode::scanCallback, &node);
  ros::spin();
}