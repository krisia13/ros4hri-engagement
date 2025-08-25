#include "tiago_behavior_tree_cpp/tiago_bt_node.hpp"
#include <signal.h>


/// Nodo global para el manejo del ciclo de vida del BT.
std::shared_ptr<TiagoBTNode> g_node = nullptr;

/**
 * @brief Manejador de señales para apagado seguro.
 *
 * Este manejador captura señales SIGINT y SIGTERM para realizar un apagado
 * ordenado del nodo y del sistema ROS2.
 *
 * @param signum Número de señal recibida (no usado).
 */
void signalHandler(int /*signum*/)  // ← Añadir comentario para suprimir warning
{
    if (g_node) 
    {
        RCLCPP_INFO(g_node->get_logger(), "🛑 Shutting down Tiago Behavior Tree gracefully...");
    }
    rclcpp::shutdown();
}

/**
 * @brief Función principal del programa.
 *
 * Inicializa ROS2, instala los manejadores de señal, crea el nodo principal
 * y ejecuta el ciclo de vida del Behavior Tree. Captura excepciones y realiza
 * el apagado seguro.
 *
 * @param argc Número de argumentos de línea de comandos.
 * @param argv Vector de argumentos de línea de comandos.
 * @return int Código de salida (0 si correcto, 1 si excepción).
 */
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    // Instala los manejadores de señal para apagado seguro.
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try 
    {
        // Crea el nodo principal del Behavior Tree.
        g_node = std::make_shared<TiagoBTNode>();
        
        RCLCPP_INFO(g_node->get_logger(), "🚀 Starting Tiago Behavior Tree");
        
        // Ejecuta el ciclo de vida del nodo.
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