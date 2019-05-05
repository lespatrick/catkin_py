//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include "HectorMappingRos.h"

#include "map/GridMap.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "sensor_msgs/PointCloud2.h"

#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "HectorMapMutex.h"
#include <stdio.h>
#include <fstream>
#include <boost/filesystem.hpp>

#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2/LinearMath/Quaternion.h>
#include <SDL2/SDL_image.h>
#include "yaml-cpp/yaml.h"

#ifndef TF_SCALAR_H
  typedef btScalar tfScalar;
#endif

HectorMappingRos::HectorMappingRos()
  : debugInfoProvider(0)
  , hectorDrawings(0)
  , lastGetMapUpdateIndex(-100)
  , tfB_(0)
  , map__publish_thread_(0)
  , initial_pose_set_(false)
{
  ros::NodeHandle private_nh_("~");

  std::string mapTopic_ = "map";

  private_nh_.param("pub_drawings", p_pub_drawings, false);
  private_nh_.param("pub_debug_output", p_pub_debug_output_, false);
  private_nh_.param("pub_map_odom_transform", p_pub_map_odom_transform_,true);
  private_nh_.param("pub_odometry", p_pub_odometry_,false);
  private_nh_.param("advertise_map_service", p_advertise_map_service_,true);
  private_nh_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 1000);

  private_nh_.param("map_resolution", p_map_resolution_, 0.025);
  private_nh_.param("map_size", p_map_size_, 1024);
  private_nh_.param("map_start_x", p_map_start_x_, 0.5);
  private_nh_.param("map_start_y", p_map_start_y_, 0.5);
  private_nh_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

  private_nh_.param("update_factor_free", p_update_factor_free_, 0.4);
  private_nh_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

  private_nh_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
  private_nh_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);

  private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
  private_nh_.param("sys_msg_topic", p_sys_msg_topic_, std::string("syscommand"));
  private_nh_.param("pose_update_topic", p_pose_update_topic_, std::string("poseupdate"));

  private_nh_.param("use_tf_scan_transformation", p_use_tf_scan_transformation_,true);
  private_nh_.param("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_,false);
  private_nh_.param("map_with_known_poses", p_map_with_known_poses_, false);

  private_nh_.param("base_frame", p_base_frame_, std::string("base_link"));
  private_nh_.param("map_frame", p_map_frame_, std::string("map"));
  private_nh_.param("odom_frame", p_odom_frame_, std::string("odom"));

  private_nh_.param("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_,true);
  private_nh_.param("tf_map_scanmatch_transform_frame_name", p_tf_map_scanmatch_transform_frame_name_, std::string("scanmatcher_frame"));

  private_nh_.param("output_timing", p_timing_output_,false);

  private_nh_.param("map_pub_period", p_map_pub_period_, 2.0);

  double tmp = 0.0;
  private_nh_.param("laser_min_dist", tmp, 0.4);
  p_sqr_laser_min_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_max_dist", tmp, 30.0);
  p_sqr_laser_max_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_z_min_value", tmp, -1.0);
  p_laser_z_min_value_ = static_cast<float>(tmp);

  private_nh_.param("laser_z_max_value", tmp, 1.0);
  p_laser_z_max_value_ = static_cast<float>(tmp);

  if (p_pub_drawings)
  {
    ROS_INFO("HectorSM publishing debug drawings");
    hectorDrawings = new HectorDrawings();
  }

  if(p_pub_debug_output_)
  {
    ROS_INFO("HectorSM publishing debug info");
    debugInfoProvider = new HectorDebugInfoProvider();
  }

  if(p_pub_odometry_)
  {
    odometryPublisher_ = node_.advertise<nav_msgs::Odometry>("scanmatch_odom", 50);
  }

  slamProcessor = new hectorslam::HectorSlamProcessor(static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_, Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_, hectorDrawings, debugInfoProvider);
  slamProcessor->setUpdateFactorFree(p_update_factor_free_);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

  int mapLevels = slamProcessor->getMapLevels();
  mapLevels = 1;

  for (int i = 0; i < mapLevels; ++i)
  {
    mapPubContainer.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr(mapTopic_);

    if (i != 0)
    {
      mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = mapPubContainer[i];
    tmp.mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
    tmp.mapMetadataPublisher_ = node_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

    if ( (i == 0) && p_advertise_map_service_)
    {
      tmp.dynamicMapServiceServer_ = node_.advertiseService("dynamic_map", &HectorMappingRos::mapCallback, this);
    }

    setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

    if ( i== 0){
      mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
    }
  }

  ROS_INFO("HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  ROS_INFO("HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  ROS_INFO("HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
  ROS_INFO("HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
  ROS_INFO("HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
  ROS_INFO("HectorSM p_map_pub_period_: %f", p_map_pub_period_);
  ROS_INFO("HectorSM p_update_factor_free_: %f", p_update_factor_free_);
  ROS_INFO("HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
  ROS_INFO("HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
  ROS_INFO("HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
  ROS_INFO("HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
  ROS_INFO("HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

  scanSubscriber_ = node_.subscribe(p_scan_topic_, p_scan_subscriber_queue_size_, &HectorMappingRos::scanCallback, this);
  sysMsgSubscriber_ = node_.subscribe(p_sys_msg_topic_, 2, &HectorMappingRos::sysMsgCallback, this);
  explorationModeSubscriber_ = node_.subscribe("exploration_on", 2, &HectorMappingRos::explorationModeHandler, this);
  saveMapSubscriber_ = node_.subscribe("save_map", 1, &HectorMappingRos::saveMapHandler, this);
  loadMapSubscriber_ = node_.subscribe("load_map", 1, &HectorMappingRos::loadMapHandler, this);

  poseUpdatePublisher_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_pose_update_topic_, 1, false);
  posePublisher_ = node_.advertise<geometry_msgs::PoseStamped>("slam_out_pose", 1, false);

  scan_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud>("slam_cloud",1,false);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(node_, "initialpose", 2);
  initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*initial_pose_sub_, tf_, p_map_frame_, 2);
  initial_pose_filter_->registerCallback(boost::bind(&HectorMappingRos::initialPoseCallback, this, _1));


  map__publish_thread_ = new boost::thread(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));

  map_to_odom_.setIdentity();

  lastMapPublishTime = ros::Time(0,0);
}

HectorMappingRos::~HectorMappingRos()
{
  delete slamProcessor;

  if (hectorDrawings)
    delete hectorDrawings;

  if (debugInfoProvider)
    delete debugInfoProvider;

  if (tfB_)
    delete tfB_;

  if(map__publish_thread_)
    delete map__publish_thread_;
}

void HectorMappingRos::scanCallback(const sensor_msgs::LaserScan& scan) {
    if (hectorDrawings) {
        hectorDrawings->setTime(scan.header.stamp);
    }

    ros::WallTime startTime = ros::WallTime::now();

    if (!p_use_tf_scan_transformation_) {
        if (rosLaserScanToDataContainer(scan, laserScanContainer,slamProcessor->getScaleToMap())) {
            slamProcessor->update(laserScanContainer,slamProcessor->getLastScanMatchPose());
        }
    } else {
        ros::Duration dur (0.5);

        if (tf_.waitForTransform(p_base_frame_,scan.header.frame_id, scan.header.stamp,dur)) {
            tf::StampedTransform laserTransform;
            tf_.lookupTransform(p_base_frame_,scan.header.frame_id, scan.header.stamp, laserTransform);

            projector_.projectLaser(scan, laser_point_cloud_,30.0);

            if (scan_point_cloud_publisher_.getNumSubscribers() > 0) {
                scan_point_cloud_publisher_.publish(laser_point_cloud_);
            }

            Eigen::Vector3f startEstimate(Eigen::Vector3f::Zero());

            if(rosPointCloudToDataContainer(laser_point_cloud_, laserTransform, laserScanContainer, slamProcessor->getScaleToMap())) {
                if (initial_pose_set_) {
                    initial_pose_set_ = false;
                    startEstimate = initial_pose_;
                } else if (p_use_tf_pose_start_estimate_) {
                    try {
                        tf::StampedTransform stamped_pose;

                        tf_.waitForTransform(p_map_frame_,p_base_frame_, scan.header.stamp, ros::Duration(0.5));
                        tf_.lookupTransform(p_map_frame_, p_base_frame_,  scan.header.stamp, stamped_pose);

                        tfScalar yaw, pitch, roll;
                        stamped_pose.getBasis().getEulerYPR(yaw, pitch, roll);

                        startEstimate = Eigen::Vector3f(stamped_pose.getOrigin().getX(),stamped_pose.getOrigin().getY(), yaw);
                    } catch(tf::TransformException e) {
                        ROS_ERROR("Transform from %s to %s failed\n", p_map_frame_.c_str(), p_base_frame_.c_str());
                        startEstimate = slamProcessor->getLastScanMatchPose();
                    }
                } else {
                    startEstimate = slamProcessor->getLastScanMatchPose();
                }

                if (p_map_with_known_poses_) {
                    slamProcessor->update(laserScanContainer, startEstimate, true);
                } else {
                    slamProcessor->update(laserScanContainer, startEstimate);
                }
            }

        } else {
            ROS_INFO("lookupTransform %s to %s timed out. Could not transform laser scan into base_frame.", p_base_frame_.c_str(), scan.header.frame_id.c_str());
            return;
        }
    }

    if (p_timing_output_) {
        ros::WallDuration duration = ros::WallTime::now() - startTime;
        ROS_INFO("HectorSLAM Iter took: %f milliseconds", duration.toSec()*1000.0f );
    }

    //If we're just building a map with known poses, we're finished now. Code below this point publishes the localization results.
    if (p_map_with_known_poses_) {
        return;
    }

    poseInfoContainer_.update(slamProcessor->getLastScanMatchPose(), slamProcessor->getLastScanMatchCovariance(), scan.header.stamp, p_map_frame_);

    poseUpdatePublisher_.publish(poseInfoContainer_.getPoseWithCovarianceStamped());
    posePublisher_.publish(poseInfoContainer_.getPoseStamped());

    if(p_pub_odometry_) {
        nav_msgs::Odometry tmp;
        tmp.pose = poseInfoContainer_.getPoseWithCovarianceStamped().pose;

        tmp.header = poseInfoContainer_.getPoseWithCovarianceStamped().header;
        odometryPublisher_.publish(tmp);
    }

    if (p_pub_map_odom_transform_) {
        tf::StampedTransform odom_to_base;

        try {
            tf_.waitForTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, ros::Duration(0.5));
            tf_.lookupTransform(p_odom_frame_, p_base_frame_, scan.header.stamp, odom_to_base);
        } catch(tf::TransformException e) {
            ROS_ERROR("Transform failed during publishing of map_odom transform: %s",e.what());
            odom_to_base.setIdentity();
        }

        map_to_odom_ = tf::Transform(poseInfoContainer_.getTfTransform() * odom_to_base.inverse());
        tfB_->sendTransform( tf::StampedTransform (map_to_odom_, scan.header.stamp, p_map_frame_, p_odom_frame_));
    }

    if (p_pub_map_scanmatch_transform_) {
        tfB_->sendTransform( tf::StampedTransform(poseInfoContainer_.getTfTransform(), scan.header.stamp, p_map_frame_, p_tf_map_scanmatch_transform_frame_name_));
    }
}

void HectorMappingRos::sysMsgCallback(const std_msgs::String& string)
{
  ROS_INFO("HectorSM sysMsgCallback, msg contents: %s", string.data.c_str());

  if (string.data == "reset")
  {
    ROS_INFO("HectorSM reset");
    slamProcessor->reset();
  }
}

void HectorMappingRos::explorationModeHandler(const std_msgs::String &message) {
  slamProcessor->dont_update_map = message.data != "ON";
}

bool HectorMappingRos::mapCallback(nav_msgs::GetMap::Request  &req,
                                   nav_msgs::GetMap::Response &res)
{
  ROS_INFO("HectorSM Map service called");
  res = mapPubContainer[0].map_;
  return true;
}

void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex)
{
  nav_msgs::GetMap::Response& map_ (mapPublisher.map_);

  //only update map if it changed
  if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
  {

    int sizeX = gridMap.getSizeX();
    int sizeY = gridMap.getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
    memset(&data[0], -1, sizeof(int8_t) * size);

    if (mapMutex)
    {
      mapMutex->lockMap();
    }

    for(int i=0; i < size; ++i)
    {
      if(gridMap.isFree(i))
      {
        data[i] = 0;
      }
      else if (gridMap.isOccupied(i))
      {
        data[i] = 100;
      }
    }

    lastGetMapUpdateIndex = gridMap.getUpdateIndex();

    if (mapMutex)
    {
      mapMutex->unlockMap();
    }
  }

  map_.map.header.stamp = timestamp;

  mapPublisher.mapPublisher_.publish(map_.map);
}

bool HectorMappingRos::rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }

  return true;
}

bool HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = pointCloud.points.size();
  //ROS_INFO("size: %d", size);

  dataContainer.clear();

  tf::Vector3 laserPos (laserTransform.getOrigin());
  dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y())*scaleToMap);

  for (size_t i = 0; i < size; ++i)
  {

    const geometry_msgs::Point32& currPoint(pointCloud.points[i]);

    float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

    if ( (dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_) ){

      if ( (currPoint.x < 0.0f) && (dist_sqr < 0.50f)){
        continue;
      }

      tf::Vector3 pointPosBaseFrame(laserTransform * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

      float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
      {
        dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
      }
    }
  }

  return true;
}

void HectorMappingRos::setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
{
  Eigen::Vector2f mapOrigin (gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
  mapOrigin.array() -= gridMap.getCellLength()*0.5f;

  map_.map.info.origin.position.x = mapOrigin.x();
  map_.map.info.origin.position.y = mapOrigin.y();
  map_.map.info.origin.orientation.w = 1.0;

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = p_map_frame_;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

void HectorMappingRos::publishMapLoop(double map_pub_period)
{
  ros::Rate r(1.0 / map_pub_period);
  while(ros::ok())
  {
    ros::Time mapTime (ros::Time::now());
    
    publishMap(mapPubContainer[0], slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

    r.sleep();
  }
}

void HectorMappingRos::staticMapCallback(const nav_msgs::OccupancyGrid& map)
{

}

void HectorMappingRos::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  initial_pose_set_ = true;

  tf::Pose pose;
  tf::poseMsgToTF(msg->pose.pose, pose);
  initial_pose_ = Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, tf::getYaw(pose.getRotation()));
  ROS_INFO("Setting initial pose with world coords x: %f y: %f yaw: %f", initial_pose_[0], initial_pose_[1], initial_pose_[2]);
}

void HectorMappingRos::saveMapHandler(const std_msgs::String &message) {
  nav_msgs::GetMap::Response& map_ (mapPubContainer[0].map_);

  this->saveMap(map_.map, "savedMap");
}

void HectorMappingRos::saveMap(const nav_msgs::OccupancyGrid& map, const std::string mapname_) {
  int threshold_occupied_ = 65;
  int threshold_free_ = 25;
  ROS_INFO("Received a %d X %d map @ %.3f m/pix",
            map.info.width,
            map.info.height,
            map.info.resolution);


  std::string mapdatafile = "/home/leszek/" + mapname_ + ".pgm";
  ROS_INFO("Writing map occupancy data to %s", mapdatafile.c_str());
  FILE* out = fopen(mapdatafile.c_str(), "w");
  if (!out)
  {
    ROS_ERROR("Couldn't save map file to %s", mapdatafile.c_str());
    return;
  }

  fprintf(out, "P5\n# CREATOR: map_saver.cpp %.3f m/pix\n%d %d\n255\n",
          map.info.resolution, map.info.width, map.info.height);
  for(unsigned int y = 0; y < map.info.height; y++) {
    for(unsigned int x = 0; x < map.info.width; x++) {
      unsigned int i = x + (map.info.height - y - 1) * map.info.width;
      if (map.data[i] >= 0 && map.data[i] <= threshold_free_) { // [0,free)
        fputc(254, out);
      } else if (map.data[i] >= threshold_occupied_) { // (occ,255]
        fputc(000, out);
      } else { //occ [0.25,0.65]
        fputc(205, out);
      }
    }
  }

  fclose(out);


  std::string mapmetadatafile = "/home/leszek/" + mapname_ + ".yaml";
  ROS_INFO("Writing map occupancy data to %s", mapmetadatafile.c_str());
  FILE* yaml = fopen(mapmetadatafile.c_str(), "w");


      /*
resolution: 0.100000
origin: [0.000000, 0.000000, 0.000000]
#
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
       */

  geometry_msgs::Quaternion orientation = map.info.origin.orientation;
  tf2::Matrix3x3 mat(tf2::Quaternion(
    orientation.x,
    orientation.y,
    orientation.z,
    orientation.w
  ));
  double yaw, pitch, roll;
  mat.getEulerYPR(yaw, pitch, roll);

  fprintf(yaml, "image: %s\nresolution: %f\norigin: [%f, %f, %f]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n\n",
          mapdatafile.c_str(), map.info.resolution, map.info.origin.position.x, map.info.origin.position.y, yaw);

  fclose(yaml);

  ROS_INFO("Done\n");
}

#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

#ifdef HAVE_YAMLCPP_GT_0_5_0
// The >> operator disappeared in yaml-cpp 0.5, so this function is
// added to provide support for code written under the yaml-cpp 0.3 API.
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

void HectorMappingRos::loadMapHandler(const std_msgs::String &message) {
  nav_msgs::GetMap::Response map_resp_;
  std::string mapfname = "";
  int negate;
  double res, occ_th, free_th, origin[3];
  MapMode mode = TRINARY;

  std::string fname("/home/leszek/savedMap.yaml");

  std::ifstream fin(fname.c_str());
  if (fin.fail()) {
    ROS_ERROR("Map_server could not open %s.", fname.c_str());
    exit(-1);
  }

  YAML::Node doc = YAML::Load(fin);
  try {
    doc["resolution"] >> res;
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
    exit(-1);
  }
  try {
    doc["negate"] >> negate;
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain a negate tag or it is invalid.");
    exit(-1);
  }
  try {
    doc["occupied_thresh"] >> occ_th;
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
    exit(-1);
  }
  try {
    doc["free_thresh"] >> free_th;
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
    exit(-1);
  }
  try {
    std::string modeS = "";
    doc["mode"] >> modeS;

    if(modeS=="trinary")
      mode = TRINARY;
    else if(modeS=="scale")
      mode = SCALE;
    else if(modeS=="raw")
      mode = RAW;
    else{
      ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
      exit(-1);
    }
  } catch (YAML::Exception &) {
    ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
    mode = TRINARY;
  }
  try {
    doc["origin"][0] >> origin[0];
    doc["origin"][1] >> origin[1];
    doc["origin"][2] >> origin[2];
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain an origin tag or it is invalid.");
    exit(-1);
  }
  try {
    doc["image"] >> mapfname;
    // TODO: make this path-handling more robust
    if(mapfname.size() == 0)
    {
      ROS_ERROR("The image tag cannot be an empty string.");
      exit(-1);
    }

    boost::filesystem::path mapfpath(mapfname);
    if (!mapfpath.is_absolute())
    {
      boost::filesystem::path dir(fname);
      dir = dir.parent_path();
      mapfpath = dir / mapfpath;
      mapfname = mapfpath.string();
    }
  } catch (YAML::InvalidScalar &) {
    ROS_ERROR("The map does not contain an image tag or it is invalid.");
    exit(-1);
  }

  this->loadMapFromFile(&map_resp_, mapfname.c_str(), res, negate, occ_th, free_th, origin, mode);

  this->mapPubContainer[0].map_ = map_resp_;

  hectorslam::GridMap& gridMap = slamProcessor->getGridMap(0);
  std::vector<int8_t>& data = map_resp_.map.data;

  int sizeX = gridMap.getSizeX();
  int sizeY = gridMap.getSizeY();

  int size = sizeX * sizeY;

  slamProcessor->reset();
  
  for(int i=0; i < size; ++i) {
    if(data[i] == 0) {
      gridMap.updateSetFree(i);
    } else if (data[i] == 100) {
      gridMap.updateSetOccupied(i);
    }
  }
}

void HectorMappingRos::loadMapFromFile(nav_msgs::GetMap::Response* resp,
                const char* fname, double res, bool negate,
                double occ_th, double free_th, double* origin,
                MapMode mode) {
  SDL_Surface* img;

  unsigned char* pixels;
  unsigned char* p;
  unsigned char value;
  int rowstride, n_channels, avg_channels;
  unsigned int i,j;
  int k;
  double occ;
  int alpha;
  int color_sum;
  double color_avg;

  // Load the image using SDL.  If we get NULL back, the image load failed.
  if(!(img = IMG_Load(fname)))
  {
    std::string errmsg = std::string("failed to open image file \"") +
            std::string(fname) + std::string("\": ") + IMG_GetError();
    throw std::runtime_error(errmsg);
  }

  // Copy the image data into the map structure
  resp->map.info.width = img->w;
  resp->map.info.height = img->h;
  resp->map.info.resolution = res;
  resp->map.info.origin.position.x = *(origin);
  resp->map.info.origin.position.y = *(origin+1);
  resp->map.info.origin.position.z = 0.0;
  tf2::Quaternion q;
  // setEulerZYX(yaw, pitch, roll)
  q.setEulerZYX(*(origin+2), 0, 0);
  resp->map.info.origin.orientation.x = q.x();
  resp->map.info.origin.orientation.y = q.y();
  resp->map.info.origin.orientation.z = q.z();
  resp->map.info.origin.orientation.w = q.w();

  // Allocate space to hold the data
  resp->map.data.resize(resp->map.info.width * resp->map.info.height);

  // Get values that we'll need to iterate through the pixels
  rowstride = img->pitch;
  n_channels = img->format->BytesPerPixel;

  // NOTE: Trinary mode still overrides here to preserve existing behavior.
  // Alpha will be averaged in with color channels when using trinary mode.
  if (mode==TRINARY || !img->format->Amask)
    avg_channels = n_channels;
  else
    avg_channels = n_channels - 1;

  // Copy pixel data into the map structure
  pixels = (unsigned char*)(img->pixels);
  for(j = 0; j < resp->map.info.height; j++)
  {
    for (i = 0; i < resp->map.info.width; i++)
    {
      // Compute mean of RGB for this pixel
      p = pixels + j*rowstride + i*n_channels;
      color_sum = 0;
      for(k=0;k<avg_channels;k++)
        color_sum += *(p + (k));
      color_avg = color_sum / (double)avg_channels;

      if (n_channels == 1)
          alpha = 1;
      else
          alpha = *(p+n_channels-1);

      if(negate)
        color_avg = 255 - color_avg;

      if(mode==RAW){
          value = color_avg;
          resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = value;
          continue;
      }


      // If negate is true, we consider blacker pixels free, and whiter
      // pixels occupied.  Otherwise, it's vice versa.
      occ = (255 - color_avg) / 255.0;

      // Apply thresholds to RGB means to determine occupancy values for
      // map.  Note that we invert the graphics-ordering of the pixels to
      // produce a map with cell (0,0) in the lower-left corner.
      if(occ > occ_th)
        value = +100;
      else if(occ < free_th)
        value = 0;
      else if(mode==TRINARY || alpha < 1.0)
        value = -1;
      else {
        double ratio = (occ - free_th) / (occ_th - free_th);
        value = 99 * ratio;
      }

      resp->map.data[MAP_IDX(resp->map.info.width,i,resp->map.info.height - j - 1)] = value;
    }
  }

  SDL_FreeSurface(img);
}

