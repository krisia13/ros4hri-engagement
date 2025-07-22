#include "tiago_behavior_tree_cpp/tiago_bt_node.hpp"
#include <signal.h>

std::shared_ptr<TiagoBTNode> g_node = nullptr;

void signalHandler(int /*signum*/)  // ← Añadir comentario para suprimir warning
{
    if (g_node) 
    {
        RCLCPP_INFO(g_node->get_logger(), "🛑 Shutting down Tiago Behavior Tree gracefully...");
    }
    rclcpp::shutdown();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Install signal handler
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try 
    {
        g_node = std::make_shared<TiagoBTNode>();
        
        RCLCPP_INFO(g_node->get_logger(), "🚀 Starting Tiago Behavior Tree");
        
        rclcpp::spin(g_node);
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "❌ Exception in main: %s", e.what());
        return 1;
    }
    
    RCLCPP_INFO(rclcpp::get_logger("main"), "✅ Tiago Behavior Tree shutdown complete");
    rclcpp::shutdown();
    
    return 0;
}