//
// Created by qiayuan on 2022/7/24.
//

#include "legged_controllers/TargetTrajectoriesPublisher.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <numeric>
#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>

using namespace legged;

namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(12);
scalar_t TIME_TO_TARGET;
float_t FEET_HEIGHT = 0.0;


}  // namespace
/*
void feetHeightCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
  const auto& data = msg->data;

  if (!data.empty()){
      // Initialize sum to COM_HEIGHT
      double sum = std::accumulate(data.begin(), data.end(), 0.0);
      FEET_HEIGHT = sum / data.size();
  }
}
*/


void feetHeightCallback(const gazebo_msgs::LinkStates::ConstPtr& msg) {

  std::vector<std::string> foot_links = {
    "b2w::LF_FOOT", 
    "b2w::RF_FOOT", 
    "b2w::LH_FOOT", 
    "b2w::RH_FOOT"
  };
  std::vector<double> foot_z_values(foot_links.size(), 0.0);

  for (size_t i = 0; i < msg->name.size(); i++) {

    for (size_t j = 0; j < foot_links.size(); j++) {
        if (msg->name[i] == foot_links[j]) {
            foot_z_values[j] = msg->pose[i].position.z;
            break;
        }
    }
  }

  double sum_z = 0.0;
  for (const auto& z : foot_z_values) {
      sum_z += z;
  }
  double avg_z = sum_z / foot_z_values.size();
  FEET_HEIGHT = avg_z;

  // ROS_INFO("Average Foot Z: %.3f m", avg_z);
}

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dz = desiredBaseDisplacement(2);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  //const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy+ dz * dz);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
  
  // rotation
  const scalar_t& droll = desiredBaseDisplacement(4);
  const scalar_t& dpitch = desiredBaseDisplacement(5);
  const scalar_t rotation = std::sqrt(dyaw * dyaw + droll * droll + dpitch * dpitch);
  const scalar_t rotationTime = rotation / TARGET_ROTATION_VELOCITY;

  return std::max(rotationTime, displacementTime);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                  const scalar_t& targetReachingTime) {
  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_t currentPose = observation.state.segment<6>(6);
  currentPose(2) = HEIGHT + FEET_HEIGHT;
  currentPose(4) = 0;
  currentPose(5) = 0;
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    target(2) = HEIGHT + FEET_HEIGHT;
    target(3) = goal(3);
    target(4) = 0;
    target(5) = 0;
    return target;
  }();
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
}

TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

  const scalar_t timeToTarget = TIME_TO_TARGET;
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    target(2) = HEIGHT + FEET_HEIGHT;
    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

 

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;
  return trajectories;
}

  TargetTrajectories HeightToTargetTrajectories(const vector_t& Vel, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * Vel.head(3);

  const scalar_t timeToTarget = TIME_TO_TARGET;
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    target(2) = HEIGHT + FEET_HEIGHT;
    target(3) = currentPose(3) + Vel(3) * timeToTarget;
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

   const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;
  return trajectories;
}

/* void com_hCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  com_h.data[0] = msg->data[0];
  COM_HEIGHT = (scalar_t)com_h.data[0];
  //target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories, &cmdVelToTargetTrajectories);
}; */

int main(int argc, char** argv) {
  const std::string robotName = "legged_robot";
  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  ::ros::NodeHandle nh;
  // Get node parameters
  std::string referenceFile;
  std::string taskFile;
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/taskFile", taskFile);

  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);
  com_h.data={0.5};
  
  //ROS_INFO("swingHeight:%0.2f", sw_Height);
  TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories, &cmdVelToTargetTrajectories, &HeightToTargetTrajectories);
  //Com_hSub_ = nh.subscribe<std_msgs::Float64MultiArray>("/comHeight", 1, com_hCallback);
  
  // Add the subscriber for the feet heights
 // ros::Subscriber feetHeightSubscriber = nodeHandle.subscribe(
  //  "/controllers/legged_controller/contact_feet_z_coordinates", 10, feetHeightCallback);
  ros::Subscriber sub = nh.subscribe("/gazebo/link_states", 10, feetHeightCallback);
  ros::spin();
  // Successful exit
  return 0;
}
