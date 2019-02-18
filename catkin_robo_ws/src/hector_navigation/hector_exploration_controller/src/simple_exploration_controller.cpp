//=================================================================================================
// Copyright (c) 2012, Stefan Kohlbrecher, TU Darmstadt
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
    
    
    #include <ros/ros.h>
    #include <hector_path_follower/hector_path_follower.h>
    #include <hector_nav_msgs/GetRobotTrajectory.h>
    #include "std_msgs/String.h"
    
    class SimpleExplorationController: public pose_follower::HectorPathFollowerDelegate {
    public:
        SimpleExplorationController() {
            ros::NodeHandle nh;
            
            exploration_plan_service_client_ = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("get_exploration_path");
            navigation_plan_service_client_ = nh.serviceClient<hector_nav_msgs::GetRobotTrajectory>("get_navigation_path");
            
            path_follower_.initialize(&tfl_);
            path_follower_.delegate = this;
            
            exploration_plan_generation_timer_ = nh.createTimer(ros::Duration(10.0), &SimpleExplorationController::timerPlanExploration, this, false );
            cmd_vel_generator_timer_ = nh.createTimer(ros::Duration(0.05), &SimpleExplorationController::timerCmdVelGeneration, this, false );

            exploration_plan_generation_timer_.stop();
            cmd_vel_generator_timer_.stop();

            manual_goal_sub_ = nh.subscribe("manual_goal_received", 2, &SimpleExplorationController::manualGoalNavigation, this);
            exploration_mode_sub_ = nh.subscribe("exploration_on", 2, &SimpleExplorationController::explorationModeHandler, this);
            terminate_motors_sub_ = nh.subscribe("stop_motors", 2, &SimpleExplorationController::stopMotorsHandler, this);
            
            vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        }
        
        void timerPlanExploration(const ros::TimerEvent& e) {
            hector_nav_msgs::GetRobotTrajectory srv_exploration_plan;
            
            if (exploration_plan_service_client_.call(srv_exploration_plan)) {
                ROS_INFO("Generated exploration path with %u poses", (unsigned int)srv_exploration_plan.response.trajectory.poses.size());
                path_follower_.setPlan(srv_exploration_plan.response.trajectory.poses);
                exploration_plan_generation_timer_.stop();
            } else {
                ROS_WARN("Service call for exploration service failed");
            }
        }
        
        void timerCmdVelGeneration(const ros::TimerEvent& e) {
            geometry_msgs::Twist twist;
            path_follower_.computeVelocityCommands(twist);
            // ROS_INFO("cmd_vel: %.2f, %.2f", twist.linear.x, twist.angular.z);
            vel_pub_.publish(twist);
        }

        void stopMotorsHandler(const std_msgs::String &message) {
            ROS_INFO("Motors stopped from REST");
            exploration_plan_generation_timer_.stop();
            cmd_vel_generator_timer_.stop();
            geometry_msgs::Twist emptyTwist;
            vel_pub_.publish(emptyTwist);
        }

        void explorationModeHandler(const std_msgs::String &message) {
            if (message.data == "ON") {
                ROS_INFO("Exploration started from REST");
                manualTarget = false;
                exploration_plan_generation_timer_.start();
                cmd_vel_generator_timer_.start();
            } else {
                ROS_INFO("Exploration stopped from REST");
                manualTarget = true;
                exploration_plan_generation_timer_.stop();
                cmd_vel_generator_timer_.stop();
            }
        }

        void manualGoalNavigation(const std_msgs::String &message) {
            exploration_plan_generation_timer_.stop();
            manualTarget = true;
            ROS_INFO("Received maual goal pose");
            hector_nav_msgs::GetRobotTrajectory srv_navigation_plan;
            
            if (navigation_plan_service_client_.call(srv_navigation_plan)) {
                ROS_INFO("Generated navigation path with %u poses", (unsigned int)srv_navigation_plan.response.trajectory.poses.size());
                path_follower_.setPlan(srv_navigation_plan.response.trajectory.poses);
                cmd_vel_generator_timer_.start();
            } else {
                ROS_WARN("Service call for navigation service failed");
            }
        }
        
        virtual void explorationGoalAchieved() {
            if (!manualTarget)
                exploration_plan_generation_timer_.start();
            else
                cmd_vel_generator_timer_.stop();
        }
        
    protected:
        ros::ServiceClient exploration_plan_service_client_;
        ros::ServiceClient navigation_plan_service_client_;
        ros::Publisher vel_pub_;

        ros::Subscriber manual_goal_sub_;
        ros::Subscriber exploration_mode_sub_;
        ros::Subscriber terminate_motors_sub_;
        
        tf::TransformListener tfl_;
        
        pose_follower::HectorPathFollower path_follower_;
        
        ros::Timer exploration_plan_generation_timer_;
        ros::Timer cmd_vel_generator_timer_;

        bool manualTarget = false;
    };
    
    int main(int argc, char **argv) {
        ros::init(argc, argv, ROS_PACKAGE_NAME);
        
        SimpleExplorationController ec;
        
        ros::spin();
        
        return 0;
    }
    