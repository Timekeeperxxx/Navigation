// Copyright 2024 Hongbiao Zhu
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Original work based on sensor_scan_generation package by Hongbiao Zhu.

#include <math.h>

#include "nav_msgs/msg/odometry.hpp"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

namespace
{
constexpr float kBaseFootprintFillRadius = 0.15f;
constexpr float kBaseFootprintGroundSearchRadius = 0.35f;
constexpr float kBaseFootprintGroundIntensityMax = 0.05f;
}

double scanVoxelSize = 0.05;
double decayTime = 2.0;
double noDecayDis = 4.0;
double clearingDis = 8.0;
bool clearingCloud = false;
bool useSorting = true;
double quantileZ = 0.25;
bool considerDrop = false;
bool limitGroundLift = false;
double maxGroundLift = 0.15;
bool clearDyObs = false;
double minDyObsDis = 0.3;
double minDyObsAngle = 0;
double minDyObsRelZ = -0.5;
double absDyObsRelZThre = 0.2;
double minDyObsVFOV = -16.0;
double maxDyObsVFOV = 16.0;
int minDyObsPointNum = 1;
bool noDataObstacle = false;
int noDataBlockSkipNum = 0;
int minBlockPointNum = 10;
double vehicleHeight = -0.6;
int voxelPointUpdateThre = 100;
double voxelTimeUpdateThre = 2.0;
double minRelZ = -0.2;
double maxRelZ = 0.2;
double disRatioZ = 0.2;

// terrain voxel parameters
float terrainVoxelSize = 1.0;
int terrainVoxelShiftX = 0;
int terrainVoxelShiftY = 0;
const int terrainVoxelWidth = 21;
int terrainVoxelHalfWidth = (terrainVoxelWidth - 1) / 2;
constexpr int kTerrainVoxelNum = terrainVoxelWidth * terrainVoxelWidth;

// planar voxel parameters
float planarVoxelSize = 0.2;
const int planarVoxelWidth = 51;
int planarVoxelHalfWidth = (planarVoxelWidth - 1) / 2;
constexpr int kPlanarVoxelNum = planarVoxelWidth * planarVoxelWidth;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudElev(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloud[kTerrainVoxelNum];

// 单独存储 base_footprint 附近填充的点云
pcl::PointCloud<pcl::PointXYZI>::Ptr baseFootprintFillCloud(new pcl::PointCloud<pcl::PointXYZI>());

int terrainVoxelUpdateNum[kTerrainVoxelNum] = {0};
float terrainVoxelUpdateTime[kTerrainVoxelNum] = {0};
int planarVoxelEdge[kPlanarVoxelNum] = {0};
int planarVoxelDyObs[kPlanarVoxelNum] = {0};
float planarVoxelElev[kPlanarVoxelNum] = {0};
std::vector<float> planarPointElev[kPlanarVoxelNum];

double laserCloudTime = 0;
bool newlaserCloud = false;

double systemInitTime = 0;
bool systemInited = false;
int noDataInited = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float vehicleXRec = 0, vehicleYRec = 0;

float sinVehicleRoll = 0, cosVehicleRoll = 0;
float sinVehiclePitch = 0, cosVehiclePitch = 0;
float sinVehicleYaw = 0, cosVehicleYaw = 0;

pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

bool getPlanarVoxelIndex(float x, float y, int & indX, int & indY)
{
  indX = static_cast<int>((x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
         planarVoxelHalfWidth;
  indY = static_cast<int>((y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
         planarVoxelHalfWidth;

  if (x - vehicleX + planarVoxelSize / 2 < 0) indX--;
  if (y - vehicleY + planarVoxelSize / 2 < 0) indY--;

  return indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth;
}

bool estimateBaseFootprintGroundZ(float baseX, float baseY, float & groundZ)
{
  float bestDistance2 = kBaseFootprintGroundSearchRadius * kBaseFootprintGroundSearchRadius;
  bool foundGroundPoint = false;

  for (const auto & point : terrainCloudElev->points) {
    if (point.intensity < 0.0f || point.intensity > kBaseFootprintGroundIntensityMax) {
      continue;
    }

    float dx = point.x - baseX;
    float dy = point.y - baseY;
    float distance2 = dx * dx + dy * dy;
    if (distance2 <= bestDistance2) {
      bestDistance2 = distance2;
      groundZ = point.z;
      foundGroundPoint = true;
    }
  }

  if (foundGroundPoint) {
    return true;
  }

  int indX = 0;
  int indY = 0;
  if (!getPlanarVoxelIndex(baseX, baseY, indX, indY)) {
    return false;
  }

  int idx = planarVoxelWidth * indX + indY;
  if (planarPointElev[idx].empty()) {
    return false;
  }

  groundZ = planarVoxelElev[idx];
  return true;
}

// state estimation callback function
void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
    .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  sinVehicleRoll = sin(vehicleRoll);
  cosVehicleRoll = cos(vehicleRoll);
  sinVehiclePitch = sin(vehiclePitch);
  cosVehiclePitch = cos(vehiclePitch);
  sinVehicleYaw = sin(vehicleYaw);
  cosVehicleYaw = cos(vehicleYaw);

  if (noDataInited == 0) {
    vehicleXRec = vehicleX;
    vehicleYRec = vehicleY;
    noDataInited = 1;
  }
  if (noDataInited == 1) {
    float dis = sqrt(
      (vehicleX - vehicleXRec) * (vehicleX - vehicleXRec) +
      (vehicleY - vehicleYRec) * (vehicleY - vehicleYRec));
    if (dis >= noDecayDis) noDataInited = 2;
  }
}

// registered laser scan callback function
void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
{
  laserCloudTime = rclcpp::Time(laserCloud2->header.stamp).seconds();
  if (!systemInited) {
    systemInitTime = laserCloudTime;
    systemInited = true;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloud2, *laserCloud);

  pcl::PointXYZI point;
  laserCloudCrop->clear();
  int laserCloudSize = laserCloud->points.size();
  for (int i = 0; i < laserCloudSize; i++) {
    point = laserCloud->points[i];

    float pointX = point.x;
    float pointY = point.y;
    float pointZ = point.z;

    float dis =
      sqrt((pointX - vehicleX) * (pointX - vehicleX) + (pointY - vehicleY) * (pointY - vehicleY));
    if (
      pointZ - vehicleZ > minRelZ - disRatioZ * dis &&
      pointZ - vehicleZ < maxRelZ + disRatioZ * dis &&
      dis < terrainVoxelSize * (terrainVoxelHalfWidth + 1)) {
      point.x = pointX;
      point.y = pointY;
      point.z = pointZ;
      point.intensity = laserCloudTime - systemInitTime;
      laserCloudCrop->push_back(point);
    }
  }

  newlaserCloud = true;
}

// joystick callback function
void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  if (joy->buttons[5] > 0.5) {
    noDataInited = 0;
    clearingCloud = true;
  }
}

// cloud clearing callback function
void clearingHandler(const std_msgs::msg::Float32::ConstSharedPtr dis)
{
  noDataInited = 0;
  clearingDis = dis->data;
  clearingCloud = true;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("terrainAnalysis");

  nh->declare_parameter<double>("scanVoxelSize", scanVoxelSize);
  nh->declare_parameter<double>("decayTime", decayTime);
  nh->declare_parameter<double>("noDecayDis", noDecayDis);
  nh->declare_parameter<double>("clearingDis", clearingDis);
  nh->declare_parameter<bool>("useSorting", useSorting);
  nh->declare_parameter<double>("quantileZ", quantileZ);
  nh->declare_parameter<bool>("considerDrop", considerDrop);
  nh->declare_parameter<bool>("limitGroundLift", limitGroundLift);
  nh->declare_parameter<double>("maxGroundLift", maxGroundLift);
  nh->declare_parameter<bool>("clearDyObs", clearDyObs);
  nh->declare_parameter<double>("minDyObsDis", minDyObsDis);
  nh->declare_parameter<double>("minDyObsAngle", minDyObsAngle);
  nh->declare_parameter<double>("minDyObsRelZ", minDyObsRelZ);
  nh->declare_parameter<double>("absDyObsRelZThre", absDyObsRelZThre);
  nh->declare_parameter<double>("minDyObsVFOV", minDyObsVFOV);
  nh->declare_parameter<double>("maxDyObsVFOV", maxDyObsVFOV);
  nh->declare_parameter<int>("minDyObsPointNum", minDyObsPointNum);
  nh->declare_parameter<bool>("noDataObstacle", noDataObstacle);
  nh->declare_parameter<int>("noDataBlockSkipNum", noDataBlockSkipNum);
  nh->declare_parameter<int>("minBlockPointNum", minBlockPointNum);
  nh->declare_parameter<double>("vehicleHeight", vehicleHeight);
  nh->declare_parameter<int>("voxelPointUpdateThre", voxelPointUpdateThre);
  nh->declare_parameter<double>("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nh->declare_parameter<double>("minRelZ", minRelZ);
  nh->declare_parameter<double>("maxRelZ", maxRelZ);
  nh->declare_parameter<double>("disRatioZ", disRatioZ);

  nh->get_parameter("scanVoxelSize", scanVoxelSize);
  nh->get_parameter("decayTime", decayTime);
  nh->get_parameter("noDecayDis", noDecayDis);
  nh->get_parameter("clearingDis", clearingDis);
  nh->get_parameter("useSorting", useSorting);
  nh->get_parameter("quantileZ", quantileZ);
  nh->get_parameter("considerDrop", considerDrop);
  nh->get_parameter("limitGroundLift", limitGroundLift);
  nh->get_parameter("maxGroundLift", maxGroundLift);
  nh->get_parameter("clearDyObs", clearDyObs);
  nh->get_parameter("minDyObsDis", minDyObsDis);
  nh->get_parameter("minDyObsAngle", minDyObsAngle);
  nh->get_parameter("minDyObsRelZ", minDyObsRelZ);
  nh->get_parameter("absDyObsRelZThre", absDyObsRelZThre);
  nh->get_parameter("minDyObsVFOV", minDyObsVFOV);
  nh->get_parameter("maxDyObsVFOV", maxDyObsVFOV);
  nh->get_parameter("minDyObsPointNum", minDyObsPointNum);
  nh->get_parameter("noDataObstacle", noDataObstacle);
  nh->get_parameter("noDataBlockSkipNum", noDataBlockSkipNum);
  nh->get_parameter("minBlockPointNum", minBlockPointNum);
  nh->get_parameter("vehicleHeight", vehicleHeight);
  nh->get_parameter("voxelPointUpdateThre", voxelPointUpdateThre);
  nh->get_parameter("voxelTimeUpdateThre", voxelTimeUpdateThre);
  nh->get_parameter("minRelZ", minRelZ);
  nh->get_parameter("maxRelZ", maxRelZ);
  nh->get_parameter("disRatioZ", disRatioZ);

  auto subOdometry =
    nh->create_subscription<nav_msgs::msg::Odometry>("/lio/odom", 5, odometryHandler);

  auto subLaserCloud =
    nh->create_subscription<sensor_msgs::msg::PointCloud2>("/lio/cloud_world", 5, laserCloudHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("joy", 5, joystickHandler);

  auto subClearing =
    nh->create_subscription<std_msgs::msg::Float32>("map_clearing", 5, clearingHandler);

  auto pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map", 2);
  // 发布 base_footprint 附近填充点云的话题
  auto pubBaseFootprintFillCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2>("base_footprint_fill_cloud", 2);

  // TF2 listener for base_footprint lookup
  tf2_ros::Buffer tfBuffer(nh->get_clock());
  tf2_ros::TransformListener tfListener(tfBuffer);

  for (int i = 0; i < kTerrainVoxelNum; i++) {
    terrainVoxelCloud[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }

  downSizeFilter.setLeafSize(scanVoxelSize, scanVoxelSize, scanVoxelSize);

  rclcpp::Rate rate(100);
  while (rclcpp::ok()) {
    rclcpp::spin_some(nh);
    if (newlaserCloud) {
      newlaserCloud = false;

      // terrain voxel roll over - 改进：同时滚动 updateNum 和 updateTime，不清空数据
      float terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      float terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;

      while (vehicleX - terrainVoxelCenX < -terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY];
          int updateNumTemp = terrainVoxelUpdateNum[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY];
          float updateTimeTemp = terrainVoxelUpdateTime[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY];
          for (int indX = terrainVoxelWidth - 1; indX >= 1; indX--) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * (indX - 1) + indY];
            terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY] =
              terrainVoxelUpdateNum[terrainVoxelWidth * (indX - 1) + indY];
            terrainVoxelUpdateTime[terrainVoxelWidth * indX + indY] =
              terrainVoxelUpdateTime[terrainVoxelWidth * (indX - 1) + indY];
          }
          terrainVoxelCloud[indY] = terrainVoxelCloudPtr;
          terrainVoxelUpdateNum[indY] = updateNumTemp;
          terrainVoxelUpdateTime[indY] = updateTimeTemp;
          // 不清空，保留旧数据作为过渡，让新数据逐渐覆盖
        }
        terrainVoxelShiftX--;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleX - terrainVoxelCenX > terrainVoxelSize) {
        for (int indY = 0; indY < terrainVoxelWidth; indY++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[indY];
          int updateNumTemp = terrainVoxelUpdateNum[indY];
          float updateTimeTemp = terrainVoxelUpdateTime[indY];
          for (int indX = 0; indX < terrainVoxelWidth - 1; indX++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * (indX + 1) + indY];
            terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY] =
              terrainVoxelUpdateNum[terrainVoxelWidth * (indX + 1) + indY];
            terrainVoxelUpdateTime[terrainVoxelWidth * indX + indY] =
              terrainVoxelUpdateTime[terrainVoxelWidth * (indX + 1) + indY];
          }
          terrainVoxelCloud[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] =
            terrainVoxelCloudPtr;
          terrainVoxelUpdateNum[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] = updateNumTemp;
          terrainVoxelUpdateTime[terrainVoxelWidth * (terrainVoxelWidth - 1) + indY] = updateTimeTemp;
          // 不清空，保留旧数据作为过渡
        }
        terrainVoxelShiftX++;
        terrainVoxelCenX = terrainVoxelSize * terrainVoxelShiftX;
      }

      while (vehicleY - terrainVoxelCenY < -terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)];
          int updateNumTemp = terrainVoxelUpdateNum[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)];
          float updateTimeTemp = terrainVoxelUpdateTime[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)];
          for (int indY = terrainVoxelWidth - 1; indY >= 1; indY--) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * indX + (indY - 1)];
            terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY] =
              terrainVoxelUpdateNum[terrainVoxelWidth * indX + (indY - 1)];
            terrainVoxelUpdateTime[terrainVoxelWidth * indX + indY] =
              terrainVoxelUpdateTime[terrainVoxelWidth * indX + (indY - 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX] = terrainVoxelCloudPtr;
          terrainVoxelUpdateNum[terrainVoxelWidth * indX] = updateNumTemp;
          terrainVoxelUpdateTime[terrainVoxelWidth * indX] = updateTimeTemp;
          // 不清空，保留旧数据作为过渡
        }
        terrainVoxelShiftY--;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      while (vehicleY - terrainVoxelCenY > terrainVoxelSize) {
        for (int indX = 0; indX < terrainVoxelWidth; indX++) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr =
            terrainVoxelCloud[terrainVoxelWidth * indX];
          int updateNumTemp = terrainVoxelUpdateNum[terrainVoxelWidth * indX];
          float updateTimeTemp = terrainVoxelUpdateTime[terrainVoxelWidth * indX];
          for (int indY = 0; indY < terrainVoxelWidth - 1; indY++) {
            terrainVoxelCloud[terrainVoxelWidth * indX + indY] =
              terrainVoxelCloud[terrainVoxelWidth * indX + (indY + 1)];
            terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY] =
              terrainVoxelUpdateNum[terrainVoxelWidth * indX + (indY + 1)];
            terrainVoxelUpdateTime[terrainVoxelWidth * indX + indY] =
              terrainVoxelUpdateTime[terrainVoxelWidth * indX + (indY + 1)];
          }
          terrainVoxelCloud[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] =
            terrainVoxelCloudPtr;
          terrainVoxelUpdateNum[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] = updateNumTemp;
          terrainVoxelUpdateTime[terrainVoxelWidth * indX + (terrainVoxelWidth - 1)] = updateTimeTemp;
          // 不清空，保留旧数据作为过渡
        }
        terrainVoxelShiftY++;
        terrainVoxelCenY = terrainVoxelSize * terrainVoxelShiftY;
      }

      // stack registered laser scans
      pcl::PointXYZI point;
      int laserCloudCropSize = laserCloudCrop->points.size();
      for (int i = 0; i < laserCloudCropSize; i++) {
        point = laserCloudCrop->points[i];

        int indX =
          static_cast<int>((point.x - vehicleX + terrainVoxelSize / 2) / terrainVoxelSize) +
          terrainVoxelHalfWidth;
        int indY =
          static_cast<int>((point.y - vehicleY + terrainVoxelSize / 2) / terrainVoxelSize) +
          terrainVoxelHalfWidth;

        if (point.x - vehicleX + terrainVoxelSize / 2 < 0) indX--;
        if (point.y - vehicleY + terrainVoxelSize / 2 < 0) indY--;

        if (indX >= 0 && indX < terrainVoxelWidth && indY >= 0 && indY < terrainVoxelWidth) {
          terrainVoxelCloud[terrainVoxelWidth * indX + indY]->push_back(point);
          terrainVoxelUpdateNum[terrainVoxelWidth * indX + indY]++;
        }
      }

      for (int ind = 0; ind < kTerrainVoxelNum; ind++) {
        if (
          terrainVoxelUpdateNum[ind] >= voxelPointUpdateThre ||
          laserCloudTime - systemInitTime - terrainVoxelUpdateTime[ind] >= voxelTimeUpdateThre ||
          clearingCloud) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr terrainVoxelCloudPtr = terrainVoxelCloud[ind];

          laserCloudDwz->clear();
          downSizeFilter.setInputCloud(terrainVoxelCloudPtr);
          downSizeFilter.filter(*laserCloudDwz);

          terrainVoxelCloudPtr->clear();
          int laserCloudDwzSize = laserCloudDwz->points.size();
          for (int i = 0; i < laserCloudDwzSize; i++) {
            point = laserCloudDwz->points[i];
            float dis = sqrt(
              (point.x - vehicleX) * (point.x - vehicleX) +
              (point.y - vehicleY) * (point.y - vehicleY));
            if (
              point.z - vehicleZ > minRelZ - disRatioZ * dis &&
              point.z - vehicleZ < maxRelZ + disRatioZ * dis &&
              (laserCloudTime - systemInitTime - point.intensity < decayTime || dis < noDecayDis) &&
              !(dis < clearingDis && clearingCloud)) {
              terrainVoxelCloudPtr->push_back(point);
            }
          }

          terrainVoxelUpdateNum[ind] = 0;
          terrainVoxelUpdateTime[ind] = laserCloudTime - systemInitTime;
        }
      }

      terrainCloud->clear();
      // 遍历所有 terrain voxel，基于世界坐标筛选出车辆周围 5.5m 范围内的点
      // 这样即使 voxel 滚动后数据位置偏移，也能正确筛选
      float terrainCloudRange = terrainVoxelSize * (terrainVoxelHalfWidth - 5) + terrainVoxelSize / 2;
      for (int ind = 0; ind < kTerrainVoxelNum; ind++) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr voxelCloud = terrainVoxelCloud[ind];
        int voxelSize = voxelCloud->points.size();
        for (int i = 0; i < voxelSize; i++) {
          pcl::PointXYZI pt = voxelCloud->points[i];
          float dx = pt.x - vehicleX;
          float dy = pt.y - vehicleY;
          if (fabs(dx) <= terrainCloudRange && fabs(dy) <= terrainCloudRange) {
            terrainCloud->push_back(pt);
          }
        }
      }

      // estimate ground and compute elevation for each point
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        planarVoxelElev[i] = 0;
        planarVoxelEdge[i] = 0;
        planarVoxelDyObs[i] = 0;
        planarPointElev[i].clear();
      }

      int terrainCloudSize = terrainCloud->points.size();
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];

        int indX = static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
                   planarVoxelHalfWidth;
        int indY = static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
                   planarVoxelHalfWidth;

        if (point.x - vehicleX + planarVoxelSize / 2 < 0) indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0) indY--;

        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
          for (int dX = -1; dX <= 1; dX++) {
            for (int dY = -1; dY <= 1; dY++) {
              if (
                indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 &&
                indY + dY < planarVoxelWidth) {
                planarPointElev[planarVoxelWidth * (indX + dX) + indY + dY].push_back(point.z);
              }
            }
          }
        }

        if (clearDyObs) {
          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth) {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            if (dis1 > minDyObsDis) {
              float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / M_PI;
              if (angle1 > minDyObsAngle) {
                float pointX2 = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
                float pointY2 = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
                float pointZ2 = pointZ1;

                float pointX3 = pointX2 * cosVehiclePitch - pointZ2 * sinVehiclePitch;
                float pointY3 = pointY2;
                float pointZ3 = pointX2 * sinVehiclePitch + pointZ2 * cosVehiclePitch;

                float pointX4 = pointX3;
                float pointY4 = pointY3 * cosVehicleRoll + pointZ3 * sinVehicleRoll;
                float pointZ4 = -pointY3 * sinVehicleRoll + pointZ3 * cosVehicleRoll;

                float dis4 = sqrt(pointX4 * pointX4 + pointY4 * pointY4);
                float angle4 = atan2(pointZ4, dis4) * 180.0 / M_PI;
                if (
                  (angle4 > minDyObsVFOV && angle4 < maxDyObsVFOV) ||
                  fabs(pointZ4) < absDyObsRelZThre) {
                  planarVoxelDyObs[planarVoxelWidth * indX + indY]++;
                }
              }
            } else {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] += minDyObsPointNum;
            }
          }
        }
      }

      if (clearDyObs) {
        for (int i = 0; i < laserCloudCropSize; i++) {
          point = laserCloudCrop->points[i];

          int indX =
            static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;
          int indY =
            static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0) indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0) indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth) {
            float pointX1 = point.x - vehicleX;
            float pointY1 = point.y - vehicleY;
            float pointZ1 = point.z - vehicleZ;

            float dis1 = sqrt(pointX1 * pointX1 + pointY1 * pointY1);
            float angle1 = atan2(pointZ1 - minDyObsRelZ, dis1) * 180.0 / M_PI;
            if (angle1 > minDyObsAngle) {
              planarVoxelDyObs[planarVoxelWidth * indX + indY] = 0;
            }
          }
        }
      }

      if (useSorting) {
        for (int i = 0; i < kPlanarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0) {
            sort(planarPointElev[i].begin(), planarPointElev[i].end());

            int quantileID = static_cast<int>(quantileZ * planarPointElevSize);
            if (quantileID < 0)
              quantileID = 0;
            else if (quantileID >= planarPointElevSize)
              quantileID = planarPointElevSize - 1;

            if (
              planarPointElev[i][quantileID] > planarPointElev[i][0] + maxGroundLift &&
              limitGroundLift) {
              planarVoxelElev[i] = planarPointElev[i][0] + maxGroundLift;
            } else {
              planarVoxelElev[i] = planarPointElev[i][quantileID];
            }
          }
        }
      } else {
        for (int i = 0; i < kPlanarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize > 0) {
            float minZ = 1000.0;
            int minID = -1;
            for (int j = 0; j < planarPointElevSize; j++) {
              if (planarPointElev[i][j] < minZ) {
                minZ = planarPointElev[i][j];
                minID = j;
              }
            }

            if (minID != -1) {
              planarVoxelElev[i] = planarPointElev[i][minID];
            }
          }
        }
      }

      // 计算每个 planar voxel 内点的 Z 值范围，用于检测墙壁区域
      // 如果某个 voxel 内 Z 值变化大（有墙壁），则该 voxel 内所有点都标记为障碍物
      float planarVoxelZRange[kPlanarVoxelNum] = {0};
      float planarVoxelZMin[kPlanarVoxelNum];
      float planarVoxelZMax[kPlanarVoxelNum];
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        planarVoxelZMin[i] = 1e9;
        planarVoxelZMax[i] = -1e9;
      }

      // 收集 terrainCloud 中每个 planar voxel 内点的 Z 值范围
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];
        int indX =
          static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
          planarVoxelHalfWidth;
        int indY =
          static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
          planarVoxelHalfWidth;

        if (point.x - vehicleX + planarVoxelSize / 2 < 0) indX--;
        if (point.y - vehicleY + planarVoxelSize / 2 < 0) indY--;

        if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth) {
          int idx = planarVoxelWidth * indX + indY;
          if (point.z < planarVoxelZMin[idx]) planarVoxelZMin[idx] = point.z;
          if (point.z > planarVoxelZMax[idx]) planarVoxelZMax[idx] = point.z;
        }
      }

      // 计算 Z 值范围
      for (int i = 0; i < kPlanarVoxelNum; i++) {
        if (planarVoxelZMin[i] < 1e8 && planarVoxelZMax[i] > -1e8) {
          planarVoxelZRange[i] = planarVoxelZMax[i] - planarVoxelZMin[i];
        }
      }

      terrainCloudElev->clear();
      int terrainCloudElevSize = 0;
      for (int i = 0; i < terrainCloudSize; i++) {
        point = terrainCloud->points[i];
        if (point.z - vehicleZ > minRelZ && point.z - vehicleZ < maxRelZ) {
          int indX =
            static_cast<int>((point.x - vehicleX + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;
          int indY =
            static_cast<int>((point.y - vehicleY + planarVoxelSize / 2) / planarVoxelSize) +
            planarVoxelHalfWidth;

          if (point.x - vehicleX + planarVoxelSize / 2 < 0) indX--;
          if (point.y - vehicleY + planarVoxelSize / 2 < 0) indY--;

          if (indX >= 0 && indX < planarVoxelWidth && indY >= 0 && indY < planarVoxelWidth) {
            if (
              planarVoxelDyObs[planarVoxelWidth * indX + indY] < minDyObsPointNum || !clearDyObs) {
              float disZ = point.z - planarVoxelElev[planarVoxelWidth * indX + indY];
              if (considerDrop) disZ = fabs(disZ);
              int planarPointElevSize = planarPointElev[planarVoxelWidth * indX + indY].size();
              int idx = planarVoxelWidth * indX + indY;
              if (disZ >= 0 && disZ < vehicleHeight && planarPointElevSize >= minBlockPointNum) {
                terrainCloudElev->push_back(point);
                // 如果该 voxel 内 Z 值范围大于 0.3m，说明有墙壁/障碍物，标记为障碍物
                if (planarVoxelZRange[idx] > 0.3) {
                  terrainCloudElev->points[terrainCloudElevSize].intensity = vehicleHeight;
                } else {
                  terrainCloudElev->points[terrainCloudElevSize].intensity = disZ;
                }
                terrainCloudElevSize++;
              }
            }
          }
        }
      }

      if (noDataObstacle && noDataInited == 2) {
        for (int i = 0; i < kPlanarVoxelNum; i++) {
          int planarPointElevSize = planarPointElev[i].size();
          if (planarPointElevSize < minBlockPointNum) {
            planarVoxelEdge[i] = 1;
          }
        }

        for (int noDataBlockSkipCount = 0; noDataBlockSkipCount < noDataBlockSkipNum;
             noDataBlockSkipCount++) {
          for (int i = 0; i < kPlanarVoxelNum; i++) {
            if (planarVoxelEdge[i] >= 1) {
              int indX = static_cast<int>(i / planarVoxelWidth);
              int indY = i % planarVoxelWidth;
              bool edgeVoxel = false;
              for (int dX = -1; dX <= 1; dX++) {
                for (int dY = -1; dY <= 1; dY++) {
                  if (
                    indX + dX >= 0 && indX + dX < planarVoxelWidth && indY + dY >= 0 &&
                    indY + dY < planarVoxelWidth) {
                    if (
                      planarVoxelEdge[planarVoxelWidth * (indX + dX) + indY + dY] <
                      planarVoxelEdge[i]) {
                      edgeVoxel = true;
                    }
                  }
                }
              }

              if (!edgeVoxel) planarVoxelEdge[i]++;
            }
          }
        }

        for (int i = 0; i < kPlanarVoxelNum; i++) {
          if (planarVoxelEdge[i] > noDataBlockSkipNum) {
            int indX = static_cast<int>(i / planarVoxelWidth);
            int indY = i % planarVoxelWidth;

            point.x = planarVoxelSize * (indX - planarVoxelHalfWidth) + vehicleX;
            point.y = planarVoxelSize * (indY - planarVoxelHalfWidth) + vehicleY;
            point.z = vehicleZ;
            point.intensity = vehicleHeight;

            point.x -= planarVoxelSize / 4.0;
            point.y -= planarVoxelSize / 4.0;
            terrainCloudElev->push_back(point);

            point.x += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.y += planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);

            point.x -= planarVoxelSize / 2.0;
            terrainCloudElev->push_back(point);
          }
        }
      }

      // ===== 查询 base_footprint 坐标并在其周围 XY 平面填充点云 =====
      // 清空上一帧的填充点云
      baseFootprintFillCloud->clear();
      try {
        geometry_msgs::msg::TransformStamped baseFootprintTransform =
          tfBuffer.lookupTransform("map", "base_footprint", tf2::TimePointZero);
        float baseX = baseFootprintTransform.transform.translation.x;
        float baseY = baseFootprintTransform.transform.translation.y;
        float baseZ = baseFootprintTransform.transform.translation.z;
        float fillZ = baseZ;
        bool hasGroundZ = estimateBaseFootprintGroundZ(baseX, baseY, fillZ);

        if (hasGroundZ) {
          // 在 base_footprint 中心附近的 XY 平面 0.3m 范围内生成密集点云。
          // z 使用 terrain 当前帧估计出的地面高度，避免把 base_footprint 的 TF 高度写入走过路径。
          float step = 2.0f * kBaseFootprintFillRadius / 14.0f;
          for (float dx = -kBaseFootprintFillRadius; dx <= kBaseFootprintFillRadius + 1e-6f;
               dx += step) {
            for (float dy = -kBaseFootprintFillRadius; dy <= kBaseFootprintFillRadius + 1e-6f;
                 dy += step) {
              pcl::PointXYZI fillPoint;
              fillPoint.x = baseX + dx;
              fillPoint.y = baseY + dy;
              fillPoint.z = fillZ;
              fillPoint.intensity = 0.0f;
              // 同时添加到 terrainCloudElev 和 baseFootprintFillCloud，保持原有发布与保存链路。
              terrainCloudElev->push_back(fillPoint);
              baseFootprintFillCloud->push_back(fillPoint);
            }
          }
        } else {
          RCLCPP_WARN(
            nh->get_logger(),
            "Skip base_footprint fill: no ground height around base_footprint, base_z=%.3f",
            baseZ);
        }
      } catch (const tf2::TransformException & ex) {
        // TF 查询失败时静默处理，不影响主流程
        RCLCPP_WARN(nh->get_logger(), "Could not get base_footprint transform: %s", ex.what());
      }

      clearingCloud = false;

      // publish points with elevation
      sensor_msgs::msg::PointCloud2 terrainCloud2;
      pcl::toROSMsg(*terrainCloudElev, terrainCloud2);
      terrainCloud2.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserCloudTime * 1e9));
      terrainCloud2.header.frame_id = "map";
      pubLaserCloud->publish(terrainCloud2);

      // 发布 base_footprint 附近填充点云
      sensor_msgs::msg::PointCloud2 baseFootprintFillCloud2;
      pcl::toROSMsg(*baseFootprintFillCloud, baseFootprintFillCloud2);
      baseFootprintFillCloud2.header.stamp = rclcpp::Time(static_cast<uint64_t>(laserCloudTime * 1e9));
      baseFootprintFillCloud2.header.frame_id = "map";
      pubBaseFootprintFillCloud->publish(baseFootprintFillCloud2);
    }

    rate.sleep();
  }

  return 0;
}
