#ifndef CONE_DETECTOR_HPP
#define CONE_DETECTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/bool.hpp"
#include <chrono>

// Inclusioni per PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>

class ConeDetectorNode : public rclcpp::Node {
public:
  ConeDetectorNode();
  ~ConeDetectorNode() = default;

private:
  void filter(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
};

#endif // CONE_DETECTOR_HPP
