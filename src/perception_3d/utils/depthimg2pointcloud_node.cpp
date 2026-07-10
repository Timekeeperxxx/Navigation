#include <chrono> // Date and time
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
 
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

// chrono_literals handles user-defined time durations (e.g. 500ms) 
using namespace std::chrono_literals;

namespace
{
uint16_t readUint16(const uint8_t* pixel, bool big_endian)
{
  return big_endian
             ? static_cast<uint16_t>((static_cast<uint16_t>(pixel[0]) << 8U) | pixel[1])
             : static_cast<uint16_t>(pixel[0] | (static_cast<uint16_t>(pixel[1]) << 8U));
}

float readFloat32(const uint8_t* pixel, bool big_endian)
{
  const uint32_t bits = big_endian
                            ? (static_cast<uint32_t>(pixel[0]) << 24U) |
                                  (static_cast<uint32_t>(pixel[1]) << 16U) |
                                  (static_cast<uint32_t>(pixel[2]) << 8U) | pixel[3]
                            : static_cast<uint32_t>(pixel[0]) |
                                  (static_cast<uint32_t>(pixel[1]) << 8U) |
                                  (static_cast<uint32_t>(pixel[2]) << 16U) |
                                  (static_cast<uint32_t>(pixel[3]) << 24U);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}
}  // namespace
 
class DepthImg2PointCloud : public rclcpp::Node
{
  public:
    DepthImg2PointCloud(std::string name);
    void cbDepthImg(const sensor_msgs::msg::Image::SharedPtr msg);
    void cbCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_camera_info_sub_;
    rclcpp::CallbackGroup::SharedPtr cbs_group_, cbs_group2_;
    std::string topic_, info_name_;
    bool has_info_;
    sensor_msgs::msg::CameraInfo camera_info_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
    rclcpp::Time last_pub_time_;
    double max_distance_;
    double frequency_;
    double leaf_size_;
    int sample_step_;
    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter_;
};

DepthImg2PointCloud::DepthImg2PointCloud(std::string name):Node(name){
  
  has_info_ = false;

  this->declare_parameter("topic", rclcpp::ParameterValue("/camera_left/depth/image_rect_raw"));
  this->get_parameter("topic", topic_);
  RCLCPP_INFO(this->get_logger(), "topic: %s" , topic_.c_str());

  this->declare_parameter("info", rclcpp::ParameterValue("/camera_left/depth/camera_info"));
  this->get_parameter("info", info_name_);
  RCLCPP_INFO(this->get_logger(), "info: %s" , info_name_.c_str());

  this->declare_parameter("max_distance", rclcpp::ParameterValue(4.0));
  this->get_parameter("max_distance", max_distance_);
  RCLCPP_INFO(this->get_logger(), "max_distance: %.2f" , max_distance_);

  this->declare_parameter("sample_step", rclcpp::ParameterValue(2));
  this->get_parameter("sample_step", sample_step_);
  RCLCPP_INFO(this->get_logger(), "sample_step: %d" , sample_step_);

  this->declare_parameter("leaf_size", rclcpp::ParameterValue(0.05));
  this->get_parameter("leaf_size", leaf_size_);
  RCLCPP_INFO(this->get_logger(), "leaf_size: %.2f" , leaf_size_);

  downSizeFilter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("~/point_cloud_from_depth", 2);

  cbs_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbs_group2_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  //@ Callback should be the last, because all parameters should be ready before cb
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbs_group_;
  depth_camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      topic_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable(),
      std::bind(&DepthImg2PointCloud::cbDepthImg, this, std::placeholders::_1), sub_options);

  rclcpp::SubscriptionOptions sub_options2;
  sub_options2.callback_group = cbs_group2_;
  depth_camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      info_name_, rclcpp::QoS(rclcpp::KeepLast(1)).durability_volatile().reliable(),
      std::bind(&DepthImg2PointCloud::cbCameraInfo, this, std::placeholders::_1), sub_options2);
  
}

void DepthImg2PointCloud::cbDepthImg(const sensor_msgs::msg::Image::SharedPtr msg){
  
  //RCLCPP_INFO_ONCE(this->get_logger(), "Got depth image.");
  if(!has_info_){
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
  cloud->is_dense = true; //single point of view, 2d rasterized

  float cx, cy, fx, fy;//principal point and focal lengths

  cloud->height = msg->height;
  cloud->width = msg->width;
  cx = camera_info_.k[2]; //(cloud->width >> 1) - 0.5f;
  cy = camera_info_.k[5]; //(cloud->height >> 1) - 0.5f;
  fx = 1.0f / camera_info_.k[0]; 
  fy = 1.0f / camera_info_.k[4]; 

  const bool is_u16 = msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                      msg->encoding == sensor_msgs::image_encodings::MONO16;
  const bool is_f32 = msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1;
  if (!is_u16 && !is_f32)
  {
    RCLCPP_ERROR(this->get_logger(), "Unsupported depth encoding: %s", msg->encoding.c_str());
    return;
  }
  const size_t bytes_per_pixel = is_u16 ? sizeof(uint16_t) : sizeof(float);
  const size_t min_step = static_cast<size_t>(msg->width) * bytes_per_pixel;
  const size_t required_size = static_cast<size_t>(msg->step) * msg->height;
  if (msg->step < min_step || msg->data.size() < required_size)
  {
    RCLCPP_ERROR(this->get_logger(), "Depth image dimensions do not match step/data size");
    return;
  }
  
  //RCLCPP_INFO(this->get_logger(), "%u",cv_image_->image.at<unsigned short>(0,0));
  for (unsigned int v = 0; v < msg->height; v+=sample_step_)
  {
    for (unsigned int u = 0; u < msg->width; u+=sample_step_)
    {
      pcl::PointXYZ pt;
      const uint8_t* pixel = msg->data.data() + static_cast<size_t>(v) * msg->step +
                             static_cast<size_t>(u) * bytes_per_pixel;
      const float z = is_u16 ? static_cast<float>(readUint16(pixel, msg->is_bigendian)) * 0.001F
                             : readFloat32(pixel, msg->is_bigendian);

      // Check for invalid measurements
      if (!std::isfinite(z) || z <= 0.0F || z > max_distance_)
      {
        continue;
        //pt.x = pt.y = pt.z = Z;
      }
      else // Fill in XYZ
      {
        pt.x = (u - cx) * z * fx;
        pt.y = (v - cy) * z * fy;
        pt.z = z;
      }
      cloud->push_back(pt); 
    }
  }

  downSizeFilter_.setInputCloud(cloud);
  downSizeFilter_.filter(*cloud);

  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*cloud, output);
  output.header = msg->header;
  cloud_pub_->publish(output);
}

void DepthImg2PointCloud::cbCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr msg){
  
  //RCLCPP_INFO_ONCE(this->get_logger(), "Got camera info.");
  if(!has_info_){
    has_info_ = true;
    camera_info_ = *msg;
  }

}

// Node execution starts here
int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  DepthImg2PointCloud DI2PC = DepthImg2PointCloud("depthimg2pointcloud_right");

  rclcpp::executors::MultiThreadedExecutor::SharedPtr mulexecutor_;

  mulexecutor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  mulexecutor_->add_node(DI2PC.get_node_base_interface());
   
  mulexecutor_->spin();

  // Shutdown the node when finished
  rclcpp::shutdown();
  return 0;

}
