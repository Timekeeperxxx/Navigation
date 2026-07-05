#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
// If using PCL, also include:
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudPublisher : public rclcpp::Node
{
public:

  PointCloudPublisher() : Node("point_cloud_publisher")
  {
    pub_pc_ = true;
    clock_ = this->get_clock();
    last_pub_  = clock_->now();

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("dummy_point_cloud", 1);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PointCloudPublisher::publish_point_cloud, this));
  }

private:

  rclcpp::Clock::SharedPtr clock_;
  bool pub_pc_;
  rclcpp::Time last_pub_;
  void publish_point_cloud();

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

void PointCloudPublisher::publish_point_cloud(){
  
  if(clock_->now() - last_pub_>rclcpp::Duration::from_seconds(15.0)){
    pub_pc_ = !pub_pc_;
    last_pub_  = clock_->now();
  }
  
  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::PointCloud<pcl::PointXYZI> pc;
  
  if(pub_pc_){
    for(float x=-0.1; x<=0.1; x+=0.1){
      for(float y=-5.5; y<=5.5; y+=0.1){
        for(float z=0.0; z<=1.5; z+=0.1){
          pcl::PointXYZI a_pt;
          a_pt.x = 8.23 + x;
          a_pt.y = 3.47 + y;
          a_pt.z = z;
          pc.push_back(a_pt);
        }
      }
    }
  }

  pcl::toROSMsg(pc, cloud_msg);
  cloud_msg.header.frame_id = "base_link";
  publisher_->publish(cloud_msg);
  
  
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudPublisher>());
  rclcpp::shutdown();
  return 0;
}