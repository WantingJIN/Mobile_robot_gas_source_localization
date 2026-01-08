#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "sensor_msgs/msg/fluid_pressure.hpp" // Placeholder for gas/wind
#include "geometry_msgs/msg/pose2_d.hpp"
#include "tf2/utils.h"

#include "ste_interfaces/msg/wind_measure.hpp"

#include "ste_interfaces/action/navigate_sample.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

class RobotCoordinator : public rclcpp::Node {
public:
    using RobotMission = ste_interfaces::action::NavigateSample;
    using GoalHandleMission = rclcpp_action::ServerGoalHandle<RobotMission>;

    RobotCoordinator() : Node("robot_coordinator_node") {
        // 1. Handshake Service
        // TODO: ensure we can reveive all the data before confirming handshake (MCS, gas, wind)
        ready_service_ = this->create_service<std_srvs::srv::Trigger>(
            "/check_ready", std::bind(&RobotCoordinator::handle_handshake, this, std::placeholders::_1, std::placeholders::_2));

        // 2. Internal Subscriptions (Gathering data from other robot nodes)
        pose_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            "/vicon/nori/nori", 10, [this](const geometry_msgs::msg::TransformStamped::SharedPtr msg) { 
            {
                std::lock_guard<std::mutex> lock(data_mutex_);
                latest_pose_.x = msg->transform.translation.x; 
                latest_pose_.y = msg->transform.translation.y; 
                latest_pose_.theta = tf2::getYaw(msg->transform.rotation);
                new_pose_ = true;
                printf("Received new pose: (%.2f, %.2f, %.2f)\n", latest_pose_.x, latest_pose_.y, latest_pose_.theta);
            }
        });
        
        gas_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
            "/nori/concentration", 10, [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) { 
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    latest_gas_ = msg->vector.x; // gas level is in vector.x
                    new_gas_ = true;
                    printf("Received new gas concentration: %.2f\n", latest_gas_);
                }
        });
                
        wind_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>( // Replace with your actual msg type
            "/nori/wind", 10, [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr msg) { 
                {
                    std::lock_guard<std::mutex> lock(data_mutex_);
                    latest_wind_u_ = msg->vector.x;
                    latest_wind_v_ = msg->vector.y;
                    new_wind_ = true;
                    printf("Received new wind speed: (u: %.2f, v: %.2f)\n", latest_wind_u_, latest_wind_v_);
                }
        });

        planner_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/nori/pose_ctrl_array", 10);
        // 3. Action Server (Talking to the Laptop)
        this->action_server_ = rclcpp_action::create_server<RobotMission>(
            this, "robot_mission",
            std::bind(&RobotCoordinator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotCoordinator::handle_cancel, this, std::placeholders::_1),
            std::bind(&RobotCoordinator::handle_accepted, this, std::placeholders::_1));


        latest_gas_ = 0.0;
        latest_wind_u_ = 0.0;
        latest_wind_v_ = 0.0;
        latest_pose_.x = 0.0;
        latest_pose_.y = 0.0;
        latest_pose_.theta = 0.0;
        new_pose_ = false;
        new_gas_ = false;
        new_wind_ = false;
    
    }

private:
    // Handshake logic
    void handle_handshake(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        while (!check_setup_complete()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for all systems to be ready...");
            rclcpp::sleep_for(std::chrono::seconds(1));
        }
        response->success = true;
        response->message = "Robot hardware and sensors are online.";
        RCLCPP_INFO(this->get_logger(), "Handshake completed with laptop.");
    }

    // Action Server Callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const RobotMission::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleMission>) {
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleMission> goal_handle) {
        std::thread{std::bind(&RobotCoordinator::execute_mission, this, goal_handle)}.detach();
    }

    void execute_mission(const std::shared_ptr<GoalHandleMission> goal_handle) {
        // Extract the goal sent by the laptop
        printf("Robot Coordinator: Executing new mission...\n");
        auto feedback = std::make_shared<RobotMission::Feedback>();
        auto result = std::make_shared<RobotMission::Result>();
        auto way_points = goal_handle->get_goal();
        rclcpp::Rate loop_rate(10); // 10Hz feedback
        if (way_points->goal_positions.poses.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Received empty waypoint list!");
            result->success = false;
            goal_handle->abort(result);
            return;
        } 
        planner_pub_->publish(way_points->goal_positions);
        // while the current pose is far from the last waypoint
        auto goal_pose = way_points->goal_positions.poses.back();
        goal_location_.x = goal_pose.position.x;
        goal_location_.y = goal_pose.position.y;

        goal_location_.theta = 0.0; // Not used for now

        while (rclcpp::ok() && distance_to_goal(goal_location_) > 0.05) {
            // Check if laptop canceled
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                return;
            }

            if (new_pose_ && new_gas_ && new_wind_){
                std::lock_guard<std::mutex> lock(data_mutex_);
                // Only proceed if we have a full "packet" of new data
                auto feedback = std::make_shared<RobotMission::Feedback>();
                feedback->current_pose = latest_pose_;
                feedback->gas_level = latest_gas_;
                feedback->wind_speed_u = latest_wind_u_;
                feedback->wind_speed_v = latest_wind_v_;

                // Reset flags for next sync
                new_pose_ = false;
                new_gas_ = false;
                new_wind_ = false;
                goal_handle->publish_feedback(feedback);               
            }

        }
        // take one measurement or more at the goal position depending on the sample mode
        // TODO: impelement different sampling modes
        rclcpp::sleep_for(std::chrono::seconds(1));
        if (distance_to_goal(goal_location_) < 0.05){
            RCLCPP_INFO(this->get_logger(), "Reached goal location.");
            if (new_pose_ && new_gas_ && new_wind_){
                std::lock_guard<std::mutex> lock(data_mutex_);
                // Only proceed if we have a full "packet" of new data
                auto feedback = std::make_shared<RobotMission::Feedback>();
                feedback->current_pose = latest_pose_;
                feedback->gas_level = latest_gas_;
                feedback->wind_speed_u = latest_wind_u_;
                feedback->wind_speed_v = latest_wind_v_;

                // Reset flags for next sync
                new_pose_ = false;
                new_gas_ = false;
                new_wind_ = false;
                goal_handle->publish_feedback(feedback);               
            }
        }
        // wait for 1s
        rclcpp::sleep_for(std::chrono::seconds(1));
        printf("Mission completed successfully.\n");
        result->success = true;
        goal_handle->succeed(result);
    }

    bool check_setup_complete(){
        // Placeholder logic to check if all systems are go
        return true;
    }

    double distance_to_goal(const geometry_msgs::msg::Pose2D& goal) {
        double dx = goal.x - latest_pose_.x;
        double dy = goal.y - latest_pose_.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        return distance;
    }

    // Variables
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service_;
    rclcpp_action::Server<RobotMission>::SharedPtr action_server_;
    rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr gas_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr planner_pub_;
    
    geometry_msgs::msg::Pose2D latest_pose_;
    float latest_gas_;
    float latest_wind_u_;
    float latest_wind_v_;
    // current location
    geometry_msgs::msg::Pose2D goal_location_;
    std::mutex data_mutex_;

    bool new_pose_;
    bool new_gas_;
    bool new_wind_;
    
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<RobotCoordinator>());
    // rclcpp::shutdown();
    auto node = std::make_shared<RobotCoordinator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}