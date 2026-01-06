#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "sensor_msgs/msg/fluid_pressure.hpp" // Placeholder for gas/wind
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ste_interfaces/action/navigate_sample.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class RobotCoordinator : public rclcpp::Node {
public:
    using RobotMission = ste_interfaces::action::NavigateSample;
    using GoalHandleMission = rclcpp_action::ServerGoalHandle<RobotMission>;

    RobotCoordinator() : Node("robot_coordinator_node") {
        // 1. Handshake Service
        ready_service_ = this->create_service<std_srvs::srv::Trigger>(
            "check_ready", std::bind(&RobotCoordinator::handle_handshake, this, std::placeholders::_1, std::placeholders::_2));

        // 2. Internal Subscriptions (Gathering data from other robot nodes)
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) { latest_pose_ = msg->pose; });
        
        gas_sub_ = this->create_subscription<sensor_msgs::msg::Fluid_pressure>( // Replace with your actual msg type
            "/gas_sensor", 10, [this](const sensor_msgs::msg::Fluid_pressure::SharedPtr msg) { latest_gas_ = msg->fluid_pressure; });
        
        wind_sub_ = this->create_subscription<sensor_msgs::msg::Fluid_pressure>( // Replace with your actual msg type
            "/wind_sensor", 10, [this](const sensor_msgs::msg::Fluid_pressure::SharedPtr msg) { latest_wind_ = msg->fluid_pressure; });

        // 3. Action Server (Talking to the Laptop)
        this->action_server_ = rclcpp_action::create_server<RobotMission>(
            this, "robot_mission",
            std::bind(&RobotCoordinator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotCoordinator::handle_cancel, this, std::placeholders::_1),
            std::bind(&RobotCoordinator::handle_accepted, this, std::placeholders::_1));

        current_location_.position.x = 0.0;
        current_location_.position.y = 0.0;
        current_location_.orientation.w = 1.0;
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
        auto way_points = goal_handle->get_goal();
        planner_pub_->publish(way_points->goal_positions);
        auto feedback = std::make_shared<RobotMission::Feedback>();
        auto result = std::make_shared<RobotMission::Result>();
        rclcpp::Rate loop_rate(10); // 10Hz feedback
        
        // while the current pose is far from the last waypoint
        goal_location_ = way_points->goal_positions.poses.back();

        while (rclcpp::ok() && distance_to_goal(goal_location_) > 0.05) {
            // Check if laptop canceled
            if (goal_handle->is_canceling()) {
                goal_handle->canceled(result);
                return;
            }

            // Pack latest measurements into feedback
            // NOTE: fixed the logic here, we need to sync all the messages
            feedback->current_pose = latest_pose_;
            current_location_ = latest_pose_.pose;
            feedback->gas_level = latest_gas_;
            feedback->wind_speed = 1.23; // Placeholder for anemometer

            goal_handle->publish_feedback(feedback);

            // Logic to determine if "reached" (this would normally check Planner status)
            // For now, we simulate a finish after some time
            if (/* some condition from Path Planner */ false) { break; }

            loop_rate.sleep();
        }

        result->success = true;
        goal_handle->succeed(result);
    }

    bool check_setup_complete(){
        // Placeholder logic to check if all systems are go
        return true;
    }

    double distance_to_goal(const geometry_msgs::msg::Pose& goal) {
        double dx = goal.position.x - current_location_.position.x;
        double dy = goal.position.y - current_location_.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        return distance;
    }

    // Variables
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ready_service_;
    rclcpp_action::Server<RobotMission>::SharedPtr action_server_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Fluid_pressure>::SharedPtr gas_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Fluid_pressure>::SharedPtr wind_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr planner_pub_;
    
    geometry_msgs::msg::PoseStamped latest_pose_;
    double latest_gas_;
    double latest_wind_;
    // current location
    geometry_msgs::msg::Pose current_location_;
    geometry_msgs::msg::Pose goal_location_;
    
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotCoordinator>());
    rclcpp::shutdown();
    return 0;
}