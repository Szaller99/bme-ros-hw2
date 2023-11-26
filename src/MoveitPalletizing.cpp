// Copyright 2022 Áron Svastits
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

#include <math.h>

#include <memory>


#include "iiqka_moveit_palletizing/moveit_palletizing.hpp"

class Palletizer : public MoveitPalletizing {

  public: 
  
  void Palletize(
    const std::string & planning_pipeline = "pilz_industrial_motion_planner",    
    const std::string & planner_id = "PTP")
  {
    for (int j = 0; j < 4; j++) {
      for (int i = 0; i < 4; i++) {
        std::string object_name = "pallet_" + std::to_string(4 * j + i + 1);
        RCLCPP_INFO(LOGGER, "Going for object %s", object_name.c_str());

        // Go to pickup 
        Eigen::Isometry3d pickup_pose = Eigen::Isometry3d(
          Eigen::Translation3d(
            0.5, 
            0.0,
            0.3+0.055) *
          Eigen::Quaterniond(0, 1, 0, 0));

        auto planned_trajectory =
          planToPointUntilSuccess(pickup_pose, planning_pipeline, planner_id);

        if (planned_trajectory != nullptr) {
          move_group_interface_->execute(*planned_trajectory);
        } else {
          RCLCPP_ERROR(LOGGER, "Planning failed");
        }

        // this->addBreakPoint();
        // Attach object
        AttachObject(object_name);

        // this->addBreakPoint();

        // Drop off to -0.3, 0.0, 0.35 pointing down
        // TODO Drop off poses calculation
        Eigen::Isometry3d dropoff_pose = Eigen::Isometry3d(
          Eigen::Translation3d(
            -0.7  + 0.1 * i, 
            -0.15 + 0.1 * j, 
            0.45) * Eigen::Quaterniond(0, 1, 0, 0));

        auto drop_trajectory = planToPointUntilSuccess(
          dropoff_pose, planning_pipeline, planner_id);
        
        if (drop_trajectory != nullptr) {
          move_group_interface_->execute(*drop_trajectory);
        } else {
          RCLCPP_ERROR(LOGGER, "Planning failed");
        }

        // Detach
        DetachObject(object_name);

        // this->addBreakPoint();

        this->addPalletObject(4 * j + i + 2);
        // this->addBreakPoint();
        }
      }
    }

};

int main(int argc, char * argv[])
{
  std::shared_ptr<moveit_msgs::msg::RobotTrajectory> trajectory;

  // Setup
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const palletizing_node = std::make_shared<Palletizer>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(palletizing_node);
  std::thread(
    [&executor]()
    {executor.spin();})
  .detach();

  palletizing_node->initialize();
  palletizing_node->addBreakPoint();

  // palletizing


  palletizing_node->moveGroupInterface()->setMaxVelocityScalingFactor(1.0);
  palletizing_node->moveGroupInterface()->setMaxAccelerationScalingFactor(1.0);
  // Add robot platform
  // palletizing_node->addRobotPlatform();

  // Add pallets
  palletizing_node->addPalletObject(1);
  palletizing_node->addBreakPoint();

  palletizing_node->Palletize();

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
