#ifndef TIAGO_BT_NODE_HPP
#define TIAGO_BT_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/tree_node.h>
#include <behaviortree_cpp_v3/behavior_tree.h>  
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <vector>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

// Nav2 includes
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class TiagoBTNode : public rclcpp::Node
{
public:
    TiagoBTNode();
    ~TiagoBTNode();

private:
    // =================================================================
    // SETUP FUNCTIONS
    // =================================================================
    void registerNodes();
    void loadTree();
    void tickTree();
    void initializeBlackboard();
    void setupSubscriptions();
    void initializeWaypoints();
    
    // =================================================================
    // BT CONDITION FUNCTIONS
    // =================================================================
    BT::NodeStatus checkPersonEngaged(BT::TreeNode& self);
    BT::NodeStatus checkVoiceDetected(BT::TreeNode& self);
    BT::NodeStatus checkNavigationAllowed(BT::TreeNode& self);
 
    // =================================================================
    // BT ENGAGEMENT STATE CONTROL
    // =================================================================
    BT::NodeStatus isEngagementActive(BT::TreeNode& self);
    BT::NodeStatus isNotEngagementActive(BT::TreeNode& self);
    BT::NodeStatus checkEngagementEnding(BT::TreeNode& self);
    BT::NodeStatus setEngagementActive(BT::TreeNode& self);
    BT::NodeStatus setEngagementInactive(BT::TreeNode& self);
    
    // =================================================================
    // BT ACTION FUNCTIONS - TTS
    // =================================================================
    BT::NodeStatus speakGreetingOnce(BT::TreeNode& self);
    
    // =================================================================
    // BT ACTION FUNCTIONS - NAVIGATION
    // =================================================================
    BT::NodeStatus turnToVoiceDirection(BT::TreeNode& self);
    BT::NodeStatus navigateToWaypoint(BT::TreeNode& self);
    
    // =================================================================
    // BT ACTION FUNCTIONS - LED & LOGGING
    // =================================================================
    BT::NodeStatus logEngagementState(BT::TreeNode& self);
    
    // =================================================================
    // NAV2 HELPER FUNCTIONS
    // =================================================================
    geometry_msgs::msg::PoseStamped getCurrentPose();
    geometry_msgs::msg::Quaternion createQuaternionFromYaw(double yaw);
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat);
    
    // =================================================================
    // BT INFRASTRUCTURE
    // =================================================================
    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // =================================================================
    // NAV2 ACTION CLIENT & TF2
    // =================================================================
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav2_client_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // =================================================================
    // NAVIGATION STATE & WAYPOINTS
    // =================================================================
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    size_t current_waypoint_index_;
    bool navigation_goal_sent_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr current_goal_handle_;

    // =================================================================
    // ROS2 SUBSCRIPTIONS
    // =================================================================
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr engagement_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_texto_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr vad_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr nav_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr voice_direction_sub_;
    
    // =================================================================
    // ROS2 PUBLISHERS
    // =================================================================
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr bt_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr speech_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr nav_command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr led_command_pub_;
    
    // =================================================================
    // PARAMETERS
    // =================================================================
    double tick_rate_;
    bool debug_mode_;
    std::string tree_file_;
};

#endif