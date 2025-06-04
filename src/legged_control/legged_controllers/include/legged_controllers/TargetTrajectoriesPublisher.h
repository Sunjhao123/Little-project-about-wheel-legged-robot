//
// Created by qiayuan on 2022/7/24.
//

#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <mutex>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include "std_msgs/Float64MultiArray.h"
//#include "legged_interface/constraint/SwingTrajectoryPlanner.h"


namespace legged {
using namespace ocs2;
//extern scalar_t sw_Height;
scalar_t HEIGHT=0.5;
vector_t Vel = vector_t::Zero(4);
  
std_msgs::Float64MultiArray com_h;

class TargetTrajectoriesPublisher final {
 public:
  using CmdToTargetTrajectories = std::function<TargetTrajectories(const vector_t& cmd, const SystemObservation& observation)>;

  TargetTrajectoriesPublisher(::ros::NodeHandle& nh, const std::string& topicPrefix, CmdToTargetTrajectories goalToTargetTrajectories,
                              CmdToTargetTrajectories cmdVelToTargetTrajectories,CmdToTargetTrajectories HeightToTargetTrajectories)
      : goalToTargetTrajectories_(std::move(goalToTargetTrajectories)),
        cmdVelToTargetTrajectories_(std::move(cmdVelToTargetTrajectories)),
        HeightToTargetTrajectories_(std::move(HeightToTargetTrajectories)),
        tf2_(buffer_) {
    // Trajectories publisher
    targetTrajectoriesPublisher_.reset(new TargetTrajectoriesRosPublisher(nh, topicPrefix));

    // observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    };
    observationSub_ = nh.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    //vector_t cmdVel1 = vector_t::Zero(4);

    // goal subscriber
    auto goalCallback = [this](const geometry_msgs::PoseStamped::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }
      geometry_msgs::PoseStamped pose = *msg;
      try {
        buffer_.transform(pose, pose, "odom", ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("Failure %s\n", ex.what());
        return;
      }

      vector_t cmdGoal = vector_t::Zero(6);
      cmdGoal[0] = pose.pose.position.x;
      cmdGoal[1] = pose.pose.position.y;
      cmdGoal[2] = pose.pose.position.z;
      Eigen::Quaternion<scalar_t> q(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
      cmdGoal[3] = q.toRotationMatrix().eulerAngles(0, 1, 2).z();
      cmdGoal[4] = q.toRotationMatrix().eulerAngles(0, 1, 2).y();
      cmdGoal[5] = q.toRotationMatrix().eulerAngles(0, 1, 2).x();

      const auto trajectories = goalToTargetTrajectories_(cmdGoal, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };



    // cmd_vel subscriber
    auto cmdVelCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
      if (latestObservation_.time == 0.0) {
        return;
      }

      vector_t cmdVel = vector_t::Zero(4);
      cmdVel[0] = msg->linear.x;
      Vel[0] = msg->linear.x;
      cmdVel[1] = msg->linear.y;
      Vel[1] = msg->linear.y;
      cmdVel[2] = msg->linear.z;
      Vel[2] = msg->linear.z;
      cmdVel[3] = msg->angular.z;
      Vel[3] = msg->angular.z;

      const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
    };

    auto com_hCallback = [this](const std_msgs::Float64MultiArray::ConstPtr& msg) {
      com_h.data[0] = msg->data[0];
      HEIGHT = (const scalar_t)com_h.data[0];

      const auto trajectories = HeightToTargetTrajectories_(Vel, latestObservation_);
      targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);

      //const auto trajectories = cmdVelToTargetTrajectories_(cmdVel, latestObservation_);
      //targetTrajectoriesPublisher_->publishTargetTrajectories(trajectories);
      //target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories, &cmdVelToTargetTrajectories);
    };

    /*auto updateSwingHeight= [this](const std_msgs::Float64MultiArray::ConstPtr& msg) {
        sw_Height = (const float) msg->data[0];
        std::cout << "getdata:   {" << msg->data[0] << "}"<< std::endl;
        //std::cout << "swingHeight:   {" << legged_robot::sw_Height << "}"<< std::endl;
        // 可以在这里调用update函数重新计算轨迹
    };*/

    goalSub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_test/goal", 1, goalCallback);
    cmdVelSub_ = nh.subscribe<geometry_msgs::Twist>("/cmd_vel2", 1, cmdVelCallback);
    Com_hSub_ = nh.subscribe<std_msgs::Float64MultiArray>("/comHeight", 1, com_hCallback);
    /*leg_hSub_ = nh.subscribe<std_msgs::Float64MultiArray>("leg_height_topic", 1,updateSwingHeight);*/
  
    // std::cout << "OK" << std::endl;
    //cmdVelToTargetTrajectories_(cmdVel1, latestObservation_);
    
  }

 private:
  CmdToTargetTrajectories goalToTargetTrajectories_, cmdVelToTargetTrajectories_,HeightToTargetTrajectories_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisher_;
  ::ros::Subscriber Com_hSub_;
  ::ros::Subscriber observationSub_, goalSub_, cmdVelSub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;

  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;
};

}  // namespace legged
