/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Zhi Yan.
 *  Copyright (c) 2015-2016, Jiri Horner.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Jiri Horner nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <thread>

#include <map_merge/map_merge.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace map_merge
{
MapMerge::MapMerge() : subscriptions_size_(0)
{
  ros::NodeHandle private_nh("~");
  std::string frame_id;
  std::string merged_map_topic;

  private_nh.param("merging_rate", merging_rate_, 4.0);
  private_nh.param("discovery_rate", discovery_rate_, 0.05);
  private_nh.param("estimation_rate", estimation_rate_, 0.5);
  private_nh.param("known_init_poses", have_initial_poses_, true);
  private_nh.param("estimation_confidence", confidence_threshold_, 1.0);
  private_nh.param<std::string>("robot_map_topic", robot_map_topic_, "map");
  private_nh.param<std::string>("robot_map_updates_topic",
                                robot_map_updates_topic_, "map_updates");
  private_nh.param<std::string>("robot_namespace", robot_namespace_, "");
  private_nh.param<std::string>("merged_map_topic", merged_map_topic, "map");
  private_nh.param<std::string>("world_frame", world_frame_, "world");

  private_nh.param<std::string>("robot_name", robot_name_, "error_no_name");

  /* publishing */
  merged_map_publisher_ =
      node_.advertise<nav_msgs::OccupancyGrid>(merged_map_topic, 50, true);
}

/*
 * Subcribe to pose and map topics
 */
void MapMerge::topicSubscribing()
{
  ROS_DEBUG("Robot discovery started.");

//    ros::NodeHandle n_h;
//    std::string robot_name;
//    n_h.getParam("robot_name", robot_name);
    std::string local_map_topic = "/" + robot_name_ + "/local_map";
    std::string remote_map_topic = "/" + robot_name_ + "/remote_map";

  ros::master::V_TopicInfo topic_infos;
  geometry_msgs::Transform init_pose;
  std::string map_topic;
  std::string map_updates_topic;

  ros::master::getTopics(topic_infos);
  // default msg constructor does no properly initialize quaternion
  init_pose.rotation.w = 1;  // create identity quaternion

  ROS_WARN("SUBSCRIPIAOSDIAPSDASODBOASDING");

//   add the robot to the subscriptions
    ROS_INFO("adding robot [%s] to system", robot_name_.c_str());
    {
      // lock the subscriptions to our thread
      std::lock_guard<boost::shared_mutex> lock(subscriptions_mutex_);
      // create an empty MapSubscription at the front of the list
      subscriptions_.emplace_front();
      ++subscriptions_size_; //yep dont forget to do that 🤦
    }

    // no locking here. robots_ are used only in this procedure
    // grab the subscription that we just created
    MapSubscription& subscription_local = subscriptions_.front();
    // add the robot to the dictionary, formatted as - robot_name: subscription
//    robots_.insert({local_map_topic, &subscription});
//    robots_.insert({remote_map_topic, &subscription});

    // set the subscription's initial pose to whatever the robot's current pose is
    subscription_local.initial_pose = init_pose;

    robots_.insert({"local_robot", &subscription_local});
    subscription_local.map_sub =
    node_.subscribe<nav_msgs::OccupancyGrid>(local_map_topic.c_str(), 50,
        [this, &subscription_local](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
        fullMapUpdate(msg, subscription_local);
        });




    {
    // create an empty MapSubscription at the front of the list
    subscriptions_.emplace_front();
    ++subscriptions_size_; //yep dont forget to do that 🤦
    }

    // grab the subscription that we just created
    MapSubscription& subscription_remote = subscriptions_.front();

    // set the subscription's initial pose to whatever the robot's current pose is
    subscription_remote.initial_pose = init_pose; // this is kinda wrong

    robots_.insert({"remote_robot", &subscription_remote});
    subscription_remote.map_sub =
    node_.subscribe<nav_msgs::OccupancyGrid>(remote_map_topic.c_str(), 50,
        [this, &subscription_remote](const nav_msgs::OccupancyGrid::ConstPtr& msg) {
         fullMapUpdate(msg, subscription_remote);
        });

    ROS_WARN("Yep we have subscriped");
}

/*
 * mapMerging()
 */
void MapMerge::mapMerging()
{
  ROS_DEBUG("Map merging started.");

  // if we have the initial poses, loop oveer all the subscriptions
  // add all the grids and transforms to a Vector (list)
  // After that, add the vectors of grids and transforms to the merging pipeline
  if (have_initial_poses_) {
    // a list of grids, god knows why
    std::vector<nav_msgs::OccupancyGridConstPtr> grids;
    // a list of transforms :/
    std::vector<geometry_msgs::Transform> transforms;
    // reserve some memory space for the subscriptions
    grids.reserve(subscriptions_size_);
    {
      // lock the subscriptions to our thread
      boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);

      // loop over all the subscriptions
      for (auto& subscription : subscriptions_) {
        // lock the subscription to our thread
        std::lock_guard<std::mutex> s_lock(subscription.mutex);
        ROS_DEBUG("Map merging function is accessing the subscription mutex");

          // add the subscription's map to the list of grids
        grids.push_back(subscription.readonly_map);

        ROS_DEBUG("added subscription's map to list of grids");
//        ROS_DEBUG(std::string(subscription.readonly_map));

        // add the subscription's pose to the list of transforms
        transforms.push_back(subscription.initial_pose);
      }
    }


    // we don't need to lock here, because when have_initial_poses_ is true we
    // will not run concurrently on the pipeline
    pipeline_.feed(grids.begin(), grids.end());
    pipeline_.setTransforms(transforms.begin(), transforms.end());
  }

  // now that we have prepared the merging pipeline, do the actual merging part of the pipeline

  // store the merged maps
  nav_msgs::OccupancyGridPtr merged_map;
  {
    // lock the pipeline to our thread
    std::lock_guard<std::mutex> lock(pipeline_mutex_);

    merged_map = pipeline_.composeGrids();
  }

  // if something went wrong, abort
  if (!merged_map) {
      ROS_WARN("something went wrong, aborting map merging");
    return;
  }

  // wooohoooo we did ti!!!! now add some metadata to the merge map and publish it
  ROS_WARN("all maps merged, publishing");
  // add current time to merged map
  ros::Time now = ros::Time::now();
  merged_map->info.map_load_time = now;
  merged_map->header.stamp = now;
  merged_map->header.frame_id = world_frame_;

  // publish the merged map to our topic
  ROS_ASSERT(merged_map->info.resolution > 0.f);
  merged_map_publisher_.publish(merged_map);
}

void MapMerge::poseEstimation()
{
  ROS_DEBUG("Grid pose estimation started.");

  // create a list of grids
  std::vector<nav_msgs::OccupancyGridConstPtr> grids;

  // reserve as many spots as we have subscriptions
  grids.reserve(subscriptions_size_);
  {
    // lock the subscriptions
    boost::shared_lock<boost::shared_mutex> lock(subscriptions_mutex_);

    // for each subscription (basically every robot)
    for (auto& subscription : subscriptions_) {
      // lock the subscription to our thread
      std::lock_guard<std::mutex> s_lock(subscription.mutex);

      // add the MapSubscrition's OccupanceGrid to the list of grids
      grids.push_back(subscription.readonly_map);
    }
  }

  // feed the list of OccupancyGrids to the MapMergingPipeline
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  pipeline_.feed(grids.begin(), grids.end());


  // estimate the transforms using the occupancy grids, this information stayys within the MapmerginPipeline and we dont do anything here
  // TODO allow user to change feature type
  pipeline_.estimateTransforms(combine_grids::FeatureType::AKAZE,
                               confidence_threshold_);
}

void MapMerge::fullMapUpdate(const nav_msgs::OccupancyGrid::ConstPtr& msg,
                             MapSubscription& subscription)
{
  ROS_DEBUG("Received full map update");
  std::lock_guard<std::mutex> lock(subscription.mutex);
  if (subscription.readonly_map &&
      subscription.readonly_map->header.stamp > msg->header.stamp) {
      ROS_DEBUG("Received a faster map update");
    // we have been overrunned by faster update. our work was useless.

    // if the MapSubscription has more up-to-date data than the new subscription msg just ignore it
    return;
  }

  // set the readonly map to the map we just received
  subscription.readonly_map = msg;

  // clear the writable_map
  subscription.writable_map = nullptr;
}

void MapMerge::partialMapUpdate(
    const map_msgs::OccupancyGridUpdate::ConstPtr& msg,
    MapSubscription& subscription)
{
  ROS_DEBUG("received partial map update");

  if (msg->x < 0 || msg->y < 0) {
    ROS_ERROR("negative coordinates, invalid update. x: %d, y: %d", msg->x,
              msg->y);
    return;
  }

  size_t x0 = static_cast<size_t>(msg->x);
  size_t y0 = static_cast<size_t>(msg->y);
  size_t xn = msg->width + x0;
  size_t yn = msg->height + y0;

  nav_msgs::OccupancyGridPtr map;
  nav_msgs::OccupancyGridConstPtr readonly_map;  // local copy
  {
    // load maps
    std::lock_guard<std::mutex> lock(subscription.mutex);
    map = subscription.writable_map;
    readonly_map = subscription.readonly_map;
  }

  if (!readonly_map) {
    ROS_WARN("received partial map update, but don't have any full map to update. skipping.");
    return;
  }

  // we don't have partial map to take update, we must copy readonly map and
  // update new writable map
  if (!map) {
    map.reset(new nav_msgs::OccupancyGrid(*readonly_map));
  }

  size_t grid_xn = map->info.width;
  size_t grid_yn = map->info.height;

  if (xn > grid_xn || x0 > grid_xn || yn > grid_yn || y0 > grid_yn) {
    ROS_WARN("received update doesn't fully fit into existing map, "
             "only part will be copied. received: [%lu, %lu], [%lu, %lu] "
             "map is: [0, %lu], [0, %lu]",
             x0, xn, y0, yn, grid_xn, grid_yn);
  }

  // update map with data
  size_t i = 0;
  for (size_t y = y0; y < yn && y < grid_yn; ++y) {
    for (size_t x = x0; x < xn && x < grid_xn; ++x) {
      size_t idx = y * grid_xn + x;  // index to grid for this specified cell
      map->data[idx] = msg->data[i];
      ++i;
    }
  }
  // update time stamp
  map->header.stamp = msg->header.stamp;

  {
    // store back updated map
    std::lock_guard<std::mutex> lock(subscription.mutex);
    if (subscription.readonly_map &&
        subscription.readonly_map->header.stamp > map->header.stamp) {
      // we have been overrunned by faster update. our work was useless.
      return;
    }
    subscription.writable_map = map;
    subscription.readonly_map = map;
  }
}

std::string MapMerge::robotNameFromTopic(const std::string& topic)
{
  return ros::names::parentNamespace(topic);
}

/* identifies topic via suffix */
bool MapMerge::isRobotMapTopic(const ros::master::TopicInfo& topic)
{
  /* test whether topic is robot_map_topic_ */
  std::string topic_namespace = ros::names::parentNamespace(topic.name);
  bool is_map_topic =
      ros::names::append(topic_namespace, robot_map_topic_) == topic.name;

  /* test whether topic contains *anywhere* robot namespace */
  auto pos = topic.name.find(robot_namespace_);
  bool contains_robot_namespace = pos != std::string::npos;

  /* we support only occupancy grids as maps */
  bool is_occupancy_grid = topic.datatype == "nav_msgs/OccupancyGrid";

  /* we don't want to subcribe on published merged map */
  bool is_our_topic = merged_map_publisher_.getTopic() == topic.name;

  return is_occupancy_grid && !is_our_topic && contains_robot_namespace &&
         is_map_topic;
}

/*
 * Get robot's initial position
 */
bool MapMerge::getInitPose(const std::string& name,
                           geometry_msgs::Transform& pose)
{
  std::string merging_namespace = ros::names::append(name, "map_merge");
  double yaw = 0.0;

  bool success =
      ros::param::get(ros::names::append(merging_namespace, "init_pose_x"),
                      pose.translation.x) &&
      ros::param::get(ros::names::append(merging_namespace, "init_pose_y"),
                      pose.translation.y) &&
      ros::param::get(ros::names::append(merging_namespace, "init_pose_z"),
                      pose.translation.z) &&
      ros::param::get(ros::names::append(merging_namespace, "init_pose_yaw"),
                      yaw);

  tf2::Quaternion q;
  q.setEuler(0., 0., yaw);
  pose.rotation = toMsg(q);

  return success;
}

/*
 * execute()
 */
void MapMerge::executemapMerging()
{
  ros::Rate r(merging_rate_);
  while (node_.ok()) {
    mapMerging();
    r.sleep();
  }
}

// on its own thread
void MapMerge::executetopicSubscribing()
{
  ros::Rate r(discovery_rate_);

  // while the node is alive, subscripe to topics
  while (node_.ok()) {
    topicSubscribing();
    r.sleep();
  }
}

void MapMerge::executeposeEstimation()
{
  if (have_initial_poses_)
    return;

  // runs every 0.5 by default
  ros::Rate r(estimation_rate_);

  // while the node is alive
  while (node_.ok()) {
    // estimate the pose
    poseEstimation();
    r.sleep();
  }
}

/*
 * spin()
 */
void MapMerge::spin()
{
  ros::spinOnce();
  std::thread merging_thr([this]() { executemapMerging(); });
  std::thread subscribing_thr([this]() { executetopicSubscribing(); });
  std::thread estimation_thr([this]() { executeposeEstimation(); });
  ros::spin();
  estimation_thr.join();
  merging_thr.join();
  subscribing_thr.join();
}

}  // namespace map_merge

int main(int argc, char** argv)
{
  ros::init(argc, argv, "map_merge");
  // this package is still in development -- start wil debugging enabled
  if(ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
  }
  ROS_DEBUG("Map merging node: program started at debug level");
  map_merge::MapMerge map_merging;
  map_merging.spin();
  return 0;
}
