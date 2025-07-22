#include "tiago_behavior_tree_cpp/tiago_bt_node.hpp"

TiagoBTNode::TiagoBTNode() : Node("tiago_bt_coordinator")
{
    RCLCPP_INFO(get_logger(), "🚀 Iniciando TiagoBTNode...");
    
    // =================================================================
    // DECLARE PARAMETERS
    // =================================================================
    this->declare_parameter("tick_rate", 10.0);
    this->declare_parameter("debug_mode", true);
    this->declare_parameter("tree_file", "tiago_main_tree.xml");
    
    tick_rate_ = this->get_parameter("tick_rate").as_double();
    debug_mode_ = this->get_parameter("debug_mode").as_bool();
    tree_file_ = this->get_parameter("tree_file").as_string();
    
    RCLCPP_INFO(get_logger(), "⚙️ Parámetros: tick_rate=%.1f, debug=%s, tree=%s", 
                tick_rate_, debug_mode_ ? "true" : "false", tree_file_.c_str());
    
    // =================================================================
    // INITIALIZE TF2
    // =================================================================
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    // =================================================================
    // INITIALIZE NAV2 ACTION CLIENT
    // =================================================================
    nav2_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
        this, "navigate_to_pose");
    
    RCLCPP_INFO(get_logger(), "🗺️ Esperando Nav2 action server...");
    if (!nav2_client_->wait_for_action_server(std::chrono::seconds(10))) {
        RCLCPP_WARN(get_logger(), "⚠️ Nav2 action server no disponible después de 10s");
    } else {
        RCLCPP_INFO(get_logger(), "✅ Nav2 action server conectado");
    }
    
    // =================================================================
    // INITIALIZE NAVIGATION STATE
    // =================================================================
    current_waypoint_index_ = 0;
    navigation_goal_sent_ = false;
    
    // =================================================================
    // SETUP PUBLISHERS
    // =================================================================
    bt_status_pub_ = create_publisher<std_msgs::msg::String>("bt_status", 10);
    speech_pub_ = create_publisher<std_msgs::msg::String>("speech_text", 10);
    nav_command_pub_ = create_publisher<std_msgs::msg::String>("nav_command", 10);
    
    // =================================================================
    // SETUP BT
    // =================================================================
    registerNodes();
    loadTree();
    initializeBlackboard();
    setupSubscriptions();
    initializeWaypoints();
    
    // =================================================================
    // START BT TIMER
    // =================================================================
    auto period = std::chrono::duration<double>(1.0 / tick_rate_);
    timer_ = create_wall_timer(period, std::bind(&TiagoBTNode::tickTree, this));
    
    RCLCPP_INFO(get_logger(), "✅ TiagoBTNode inicializado correctamente");
    RCLCPP_INFO(get_logger(), "🔄 BT ejecutándose a %.1f Hz", tick_rate_);
}

TiagoBTNode::~TiagoBTNode()
{
    RCLCPP_INFO(get_logger(), "🛑 Cerrando TiagoBTNode...");
    if (timer_) {
        timer_->cancel();
    }
}

// =================================================================
// REGISTER NODES
// =================================================================
void TiagoBTNode::registerNodes()
{
    RCLCPP_INFO(get_logger(), "📝 Registrando nodos BT con sintaxis v3...");

    
    // =================================================================
    // CONDITION NODES
    // =================================================================
    factory_.registerSimpleCondition("IsPersonEngaged", 
        [this](BT::TreeNode& node) -> BT::NodeStatus {
            return this->checkPersonEngaged(node);
        });
    
    factory_.registerSimpleCondition("IsVoiceDetected", 
        [this](BT::TreeNode& node) -> BT::NodeStatus {
            return this->checkVoiceDetected(node);
        });
    
    factory_.registerSimpleCondition("IsNavigationAllowed", 
        [this](BT::TreeNode& node) -> BT::NodeStatus {
            return this->checkNavigationAllowed(node);
        });
    
    factory_.registerSimpleCondition("IsEngagementActive", 
        [this](BT::TreeNode& node) -> BT::NodeStatus {
            return this->isEngagementActive(node);
        });

    factory_.registerSimpleCondition("IsEngagementEnding",
        [this](BT::TreeNode& node) { return this->checkEngagementEnding(node); 
        });

    factory_.registerSimpleCondition("IsNotEngagementActive",
        [this](BT::TreeNode& node) { 
            return this->isNotEngagementActive(node); 
        });
    
    // =================================================================
    // ACTION NODES - ENGAGEMENT
    // =================================================================
    factory_.registerSimpleAction("SetEngagementActive", 
        [this](BT::TreeNode& node) -> BT::NodeStatus {
            return this->setEngagementActive(node);
        });
    
    factory_.registerSimpleAction("SetEngagementInactive", 
        [this](BT::TreeNode& node) -> BT::NodeStatus {
            return this->setEngagementInactive(node);
        });
    
    // =================================================================
    // ACTION NODES - SPEECH
    // =================================================================
    factory_.registerSimpleAction("SpeakGreetingOnce", 
        [this](BT::TreeNode& node) -> BT::NodeStatus {
            return this->speakGreetingOnce(node);
        });
    

    // =================================================================
    // ACTION NODES - NAVIGATION
    // =================================================================
    
    factory_.registerSimpleAction("TurnToVoiceDirection", 
        [this](BT::TreeNode& node) -> BT::NodeStatus {
            return this->turnToVoiceDirection(node);
        });
    
    factory_.registerSimpleAction("NavigateToWaypoint", 
        [this](BT::TreeNode& node) -> BT::NodeStatus {
            return this->navigateToWaypoint(node);
        });
    
    // =================================================================
    // ACTION NODES - LED & LOGGING
    // =================================================================
    
    factory_.registerSimpleAction("LogEngagementState", 
        [this](BT::TreeNode& node) -> BT::NodeStatus {
            return this->logEngagementState(node);
        });
    
    RCLCPP_INFO(get_logger(), "✅ NODOS registrados:");
}

// =================================================================
// LOAD TREE
// =================================================================
void TiagoBTNode::loadTree()
{
    RCLCPP_INFO(get_logger(), "🌳 Cargando árbol BT: %s", tree_file_.c_str());
    
    try {
        std::string package_path = ament_index_cpp::get_package_share_directory("tiago_behavior_tree_cpp");
        std::string tree_path = package_path + "/trees/" + tree_file_;
        
        RCLCPP_WARN(get_logger(), "📁 Ruta completa: %s", tree_path.c_str());
        
        // Verificar que el archivo existe
        std::ifstream file_check(tree_path);
        if (!file_check.good()) {
            RCLCPP_ERROR(get_logger(), "❌ ARCHIVO NO ENCONTRADO: %s", tree_path.c_str());
            throw std::runtime_error("Archivo no existe: " + tree_path);
        }
        
        RCLCPP_WARN(get_logger(), "✅ Archivo XML encontrado, cargando...");
        tree_ = factory_.createTreeFromFile(tree_path);
        
        RCLCPP_WARN(get_logger(), "✅ Árbol BT cargado desde XML");
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "❌ Error cargando árbol: %s", e.what());
        RCLCPP_INFO(get_logger(), "🔧 Usando árbol por defecto...");
        
        // Árbol simplificado por defecto
        std::string xml_text = R"(
            <root BTCPP_format="4">
                <BehaviorTree ID="TiagoMainTree">
                    <ReactiveSequence name="MainBehavior">
                        
                        <!-- ENGAGEMENT: Entrada NUEVA -->
                        <Sequence name="NewEngagementHandler">
                            <IsPersonEngaged name="check_engagement"/>
                            <IsNewEngagementEntry name="check_if_new"/>
                            <SetEngagementActive name="mark_active"/>
                            <SpeakGreetingOnce name="greeting"/>
                            <LogEngagementState name="log_engagement"/>
                        </Sequence>
                        
                        <!-- ENGAGEMENT: Salida -->
                        <Sequence name="EngagementExitHandler">
                            <IsEngagementEnding name="check_ending"/>
                            <SetEngagementInactive name="mark_inactive"/>
                            <LogEngagementState name="log_exit"/>
                        </Sequence>
                        
                        <!-- VOZ: Solo si NO hay engagement -->
                        <Sequence name="VoiceHandler">
                            <IsVoiceDetected name="check_voice"/>
+                            <TurnToVoiceDirection name="turn_to_voice"/>
                            <SayVoiceResponse name="voice_response"/>
                            <LogEngagementState name="log_voice"/>
                            <AlwaysSuccess/>
                        </Sequence>
                        
                        <!-- NAVEGACIÓN: Solo si NO hay engagement -->
                        <Sequence name="NavigationHandler">
                            <IsNavigationAllowed name="check_nav_allowed"/>
                            <NavigateToWaypoint name="navigate"/>
++                            <LogEngagementState name="log_navigation"/>
                        </Sequence>
                        
                        <!-- IDLE: Por defecto -->
                        <Sequence name="IdleHandler">
                            <LogEngagementState name="log_idle"/>
                            <AlwaysSuccess/>
                        </Sequence>
                        
                    </ReactiveSequence>
                </BehaviorTree>
            </root>
        )";
        
        tree_ = factory_.createTreeFromText(xml_text);
        RCLCPP_INFO(get_logger(), "✅ Árbol por defecto cargado");
    }
}

// =================================================================
// TICK TREE
// =================================================================
void TiagoBTNode::tickTree()
{
    if (tree_.rootNode()) {
        auto status = tree_.tickRoot();
        
        if (debug_mode_) {
            static int tick_count = 0;
            if (++tick_count % 50 == 0) { // Log cada 5 segundos a 10Hz
                RCLCPP_DEBUG(get_logger(), "🔄 BT Status: %s", 
                           status == BT::NodeStatus::SUCCESS ? "SUCCESS" :
                           status == BT::NodeStatus::FAILURE ? "FAILURE" : "RUNNING");
            }
        }
    }
}

// =================================================================
// INITIALIZE BLACKBOARD
// =================================================================
void TiagoBTNode::initializeBlackboard()
{
    RCLCPP_INFO(get_logger(), "🧠 Inicializando blackboard...");
    
    // Engagement state
    tree_.blackboard_stack.front()->set("engagement_level", 0);
    tree_.blackboard_stack.front()->set("engagement_active", false);
    tree_.blackboard_stack.front()->set("previous_engagement_level", 0);
    
    // Voice detection
    tree_.blackboard_stack.front()->set("voice_detected", false);
    tree_.blackboard_stack.front()->set("voice_direction", 0);
    
    // Navigation state
    tree_.blackboard_stack.front()->set("navigation_paused", false);
    tree_.blackboard_stack.front()->set("waypoint_reached", false);
    
    // Speech flags
    tree_.blackboard_stack.front()->set("greeting_spoken", false);
    
    RCLCPP_INFO(get_logger(), "✅ Blackboard inicializado");
}

// =================================================================
// SETUP SUBSCRIPTIONS
// =================================================================
void TiagoBTNode::setupSubscriptions()
{
    RCLCPP_INFO(get_logger(), "📡 Configurando subscripciones...");
    
    // Engagement level subscription
    engagement_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/engagement/general_status", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            int previous_level = 0;
            tree_.blackboard_stack.front()->get("engagement_level", previous_level);
            tree_.blackboard_stack.front()->set("previous_engagement_level", previous_level);
            tree_.blackboard_stack.front()->set("engagement_level", msg->data);
            
            if (debug_mode_) {
                RCLCPP_INFO(get_logger(), "📊 Engagement: %d -> %d", previous_level, msg->data);
            }
        });
    
    // Suscripción al texto de voz reconocido
    voice_texto_sub_ = create_subscription<std_msgs::msg::String>(
        "/voz_texto", 10,
        [this](const std_msgs::msg::String::SharedPtr msg) {
            std::string texto = msg->data;
            bool detected = texto.find("hola") != std::string::npos;
            tree_.blackboard_stack.front()->set("voice_detected", detected);
    
            if (detected) {
                RCLCPP_INFO(get_logger(), "🗣️ Voz detectada true: %s", texto.c_str());
            } else {
                RCLCPP_INFO(get_logger(), "🗣️ Voz recibida false: %s", texto.c_str());
            }
        });
    
    // Voice direction subscription
    voice_direction_sub_ = create_subscription<std_msgs::msg::Int32>(
        "/voice_direction", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            tree_.blackboard_stack.front()->set("voice_direction", msg->data);
            
            std::string direction_str;
            switch (msg->data) {
                case 0: direction_str = "frente (0°)"; break;
                case 90: direction_str = "derecha (90°)"; break;
                case 180: direction_str = "detrás (180°)"; break;
                case 270: direction_str = "izquierda (270°)"; break;
                default: direction_str = "desconocida (" + std::to_string(msg->data) + "°)"; break;
            }
            
            RCLCPP_INFO(get_logger(), "📍 Dirección de voz: %s", direction_str.c_str());
        });
    
    RCLCPP_INFO(get_logger(), "✅ Subscripciones configuradas");
}

// =================================================================
// CONDITION FUNCTIONS
// =================================================================
BT::NodeStatus TiagoBTNode::checkPersonEngaged(BT::TreeNode& /*self*/)
{
    int engagement_level = 0;
    tree_.blackboard_stack.front()->get("engagement_level", engagement_level);
    if (engagement_level >= 2) {
        RCLCPP_INFO(get_logger(), "✅ checkPersonEngaged: nivel %d", engagement_level);
        return BT::NodeStatus::SUCCESS;
    }
    
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TiagoBTNode::checkVoiceDetected(BT::TreeNode& /*self*/)
{
    bool voice_detected = false;
    tree_.blackboard_stack.front()->get("voice_detected", voice_detected);

    RCLCPP_INFO(get_logger(), "🗣️ Voz detectada: %s", voice_detected ? "true" : "false");
    return voice_detected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus TiagoBTNode::checkNavigationAllowed(BT::TreeNode& /*self*/)
{
    int engagement_level = 0;
    bool engagement_active = false;
    bool voice_detected = false;
    bool turning_to_voice_in_progress = false;

    tree_.blackboard_stack.front()->get("engagement_level", engagement_level);
    tree_.blackboard_stack.front()->get("engagement_active", engagement_active);
    tree_.blackboard_stack.front()->get("voice_detected", voice_detected);
    tree_.blackboard_stack.front()->get("turning_to_voice_in_progress", turning_to_voice_in_progress);

    if (engagement_level >= 2 || engagement_active || voice_detected || turning_to_voice_in_progress) {
        RCLCPP_DEBUG(get_logger(), "🚫 Navegación NO permitida");
        return BT::NodeStatus::FAILURE;
    }

    RCLCPP_DEBUG(get_logger(), "✅ Navegación permitida");
    return BT::NodeStatus::SUCCESS;
}


//NO VAAAAA pongo greeting_spoken a false en nueva entrada engagement
BT::NodeStatus TiagoBTNode::checkEngagementEnding(BT::TreeNode& /*self*/)
{
    int engagement_level = 0;
    int previous_level = 0;

    tree_.blackboard_stack.front()->get("engagement_level", engagement_level);
    tree_.blackboard_stack.front()->get("previous_engagement_level", previous_level);

    if (engagement_level < 2 && previous_level >= 2) {
        RCLCPP_INFO(get_logger(), "🔵 FIN ENGAGEMENT: %d -> %d", previous_level, engagement_level);
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TiagoBTNode::isEngagementActive(BT::TreeNode& /*self*/)
{
    bool engagement_active = false;
    tree_.blackboard_stack.front()->get("engagement_active", engagement_active);
    


    
    return engagement_active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus TiagoBTNode::isNotEngagementActive(BT::TreeNode& /*self*/)
{
    bool engagement_active = false;
    tree_.blackboard_stack.front()->get("engagement_active", engagement_active);
    return !engagement_active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}


// =================================================================
// ACTION FUNCTIONS - ENGAGEMENT
// =================================================================

BT::NodeStatus TiagoBTNode::setEngagementActive(BT::TreeNode& /*self*/)
{
    bool nav_in_progress = false;
    tree_.blackboard_stack.front()->get("navigation_in_progress", nav_in_progress);

    if (nav_in_progress) {
        RCLCPP_INFO(get_logger(), "🙋 Engagement detectado, cancelando navegación");
        nav2_client_->async_cancel_all_goals();
        tree_.blackboard_stack.front()->set("navigation_in_progress", false);
    }
    tree_.blackboard_stack.front()->set("engagement_active", true);

    int engagement_level = 0;
    tree_.blackboard_stack.front()->get("engagement_level", engagement_level);
    tree_.blackboard_stack.front()->set("previous_engagement_level", engagement_level);

    RCLCPP_INFO(get_logger(), "🔴 ENGAGEMENT ACTIVADO");
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus TiagoBTNode::setEngagementInactive(BT::TreeNode& /*self*/)
{
    tree_.blackboard_stack.front()->set("engagement_active", false);
    tree_.blackboard_stack.front()->set("greeting_spoken", false);
    tree_.blackboard_stack.front()->set("voice_detected", false);

    int engagement_level = 0;
    tree_.blackboard_stack.front()->get("engagement_level", engagement_level);
    tree_.blackboard_stack.front()->set("previous_engagement_level", engagement_level);

    RCLCPP_INFO(get_logger(), "🟢 ENGAGEMENT DESACTIVADO");
    return BT::NodeStatus::SUCCESS;
}

// =================================================================
// ACTION FUNCTIONS - SPEECH
// =================================================================

// Saludo al entrar en engagement
BT::NodeStatus TiagoBTNode::speakGreetingOnce(BT::TreeNode& /*self*/)
{
    bool greeting_spoken = false;
    tree_.blackboard_stack.front()->get("greeting_spoken", greeting_spoken);

    if (!greeting_spoken) {
        auto msg = std_msgs::msg::String();
        msg.data = "ENGAGEMENT_GREETING"; 
        speech_pub_->publish(msg);

        tree_.blackboard_stack.front()->set("greeting_spoken", true);
        RCLCPP_INFO(get_logger(), "🗣️ SALUDO ENVIADO: %s", msg.data.c_str());
    }

    return BT::NodeStatus::SUCCESS;
}

// =================================================================
// ACTION FUNCTIONS - NAVIGATION
// =================================================================

BT::NodeStatus TiagoBTNode::turnToVoiceDirection(BT::TreeNode& /*self*/)
{
    // Si está navegando, cancela la navegación antes de girar
    bool navigation_in_progress = false;
    tree_.blackboard_stack.front()->get("navigation_in_progress", navigation_in_progress);

    bool turning_in_progress = false;
    tree_.blackboard_stack.front()->get("turning_to_voice_in_progress", turning_in_progress);

    // Si hay navegación en curso, cancela y espera al siguiente tick
    if (navigation_in_progress) {
        RCLCPP_INFO(get_logger(), "🛑 Cancelando navegación para girar hacia la voz");
        nav2_client_->async_cancel_all_goals();
        tree_.blackboard_stack.front()->set("navigation_in_progress", false);
        return BT::NodeStatus::FAILURE;
    }

    // Si ya está girando, espera a que termine
    if (turning_in_progress) {
        RCLCPP_INFO(get_logger(), "🔄 Still turning to voice...");
        return BT::NodeStatus::FAILURE;
    }

    // Prepara el goal de giro
    int voice_direction = 0;
    tree_.blackboard_stack.front()->get("voice_direction", voice_direction);

    // Anuncia que va a girar
    auto msg = std_msgs::msg::String();
    switch (voice_direction) {
        case 0:   msg.data = "VOICE FRONT"; break;
        case 90:  msg.data = "VOICE LEFT"; break;
        case 180: msg.data = "VOICE BACK"; break;
        case 270: msg.data = "VOICE RIGHT"; break;
        default:  msg.data = "VOICE UNKNOWN"; break;
    }
    speech_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "🔊 %s", msg.data.c_str());

    tree_.blackboard_stack.front()->set("voice_detected", false);

    // Prepara el goal Nav2 (igual que en navegación normal)
    auto current_pose = getCurrentPose();
    double voice_yaw_rad = (voice_direction * M_PI) / 180.0;
    double current_yaw = getYawFromQuaternion(current_pose.pose.orientation);
    double target_yaw = current_yaw + voice_yaw_rad;
    auto goal_pose = current_pose;
    goal_pose.header.stamp = this->get_clock()->now();
    goal_pose.pose.orientation = createQuaternionFromYaw(target_yaw);

    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = goal_pose;

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
            tree_.blackboard_stack.front()->set("turning_to_voice_in_progress", false);
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(get_logger(), "✅ Finished turning to voice direction");
                auto msg = std_msgs::msg::String();
                msg.data = "I am looking at you now. Did you want something?";
                speech_pub_->publish(msg);
            } else {
                RCLCPP_WARN(get_logger(), "❌ Failed turning to voice direction");
            }
        };

    if (!nav2_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_WARN(get_logger(), "⚠️ Nav2 server not available for turning");
        return BT::NodeStatus::FAILURE;
    }

    // Marca que está girando
    tree_.blackboard_stack.front()->set("turning_to_voice_in_progress", true);
    tree_.blackboard_stack.front()->set("voice_detected", false);

    nav2_client_->async_send_goal(goal_msg, send_goal_options);
    RCLCPP_INFO(get_logger(), "🔄 Turning to voice in progress...");
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TiagoBTNode::navigateToWaypoint(BT::TreeNode& /*self*/)
{
    // Lee el estado de navegación desde la blackboard
    bool navigation_in_progress = false;
    tree_.blackboard_stack.front()->get("navigation_in_progress", navigation_in_progress);

    RCLCPP_DEBUG(get_logger(), "🚗 Tick: current_waypoint_index_=%zu, navigation_in_progress_=%s",
                current_waypoint_index_, navigation_in_progress ? "true" : "false");

    if (current_waypoint_index_ >= waypoints_.size()) {
        RCLCPP_WARN(get_logger(), "🚫 No hay más waypoints disponibles");
        return BT::NodeStatus::FAILURE;
    }

    if (!navigation_in_progress) {
        auto goal_pose = waypoints_[current_waypoint_index_];
        goal_pose.header.stamp = this->get_clock()->now();

        if (!nav2_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_WARN(get_logger(), "🛑 Nav2 server no disponible");
            return BT::NodeStatus::FAILURE;
        }

        auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
        goal_msg.pose = goal_pose;

        RCLCPP_INFO(get_logger(), "🚩 Enviando goal a waypoint %zu: (%.2f, %.2f)",
                    current_waypoint_index_ + 1,
                    goal_pose.pose.position.x,
                    goal_pose.pose.position.y);

        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.result_callback =
            [this](const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result) {
                tree_.blackboard_stack.front()->set("navigation_in_progress", false);

                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    RCLCPP_INFO(get_logger(), "✅ Waypoint %zu alcanzado", current_waypoint_index_ + 1);
                    current_waypoint_index_ = (current_waypoint_index_ + 1) % waypoints_.size();
                } else {
                    RCLCPP_WARN(get_logger(), "❌ Fallo navegando a waypoint %zu", current_waypoint_index_ + 1);
                }
            };

        nav2_client_->async_send_goal(goal_msg, send_goal_options);
        tree_.blackboard_stack.front()->set("navigation_in_progress", true);
    } else {
        RCLCPP_INFO(get_logger(), "⏳ Navegación en curso hacia waypoint %zu", current_waypoint_index_ + 1);
    }

    return BT::NodeStatus::SUCCESS;
}

// =================================================================
// ACTION FUNCTIONS - LED & LOGGING
// =================================================================

BT::NodeStatus TiagoBTNode::logEngagementState(BT::TreeNode& /*self*/)
{
    int engagement_level = 0;
    bool engagement_active = false;
    
    tree_.blackboard_stack.front()->get("engagement_level", engagement_level);
    tree_.blackboard_stack.front()->get("engagement_active", engagement_active);
    
    RCLCPP_INFO(get_logger(), "📊 Estado: nivel=%d, activo=%s", 
                engagement_level, engagement_active ? "SÍ" : "NO");
    
    return BT::NodeStatus::SUCCESS;
}

// =================================================================
// HELPER FUNCTIONS
// =================================================================
void TiagoBTNode::initializeWaypoints()
{
    waypoints_.clear();
    
    // Waypoint 1
    geometry_msgs::msg::PoseStamped pose1;
    pose1.header.frame_id = "map";
    pose1.pose.position.x = 1.5596764087677002;
    pose1.pose.position.y = 1.8583741188049316;
    pose1.pose.position.z = 0.0;
    pose1.pose.orientation.x = 0.0;
    pose1.pose.orientation.y = 0.0;
    pose1.pose.orientation.z = 0.0;
    pose1.pose.orientation.w = 0.9999984067993181;
    waypoints_.push_back(pose1);
    
    // Waypoint 2
    geometry_msgs::msg::PoseStamped pose2;
    pose2.header.frame_id = "map";
    pose2.pose.position.x = 1.2248239517211914;
    pose2.pose.position.y = 6.7751898765563965;
    pose2.pose.position.z = 0.0;
    pose2.pose.orientation.x = 0.0;
    pose2.pose.orientation.y = 0.0;
    pose2.pose.orientation.z = 0.0;
    pose2.pose.orientation.w = 0.9999970106541881;
    waypoints_.push_back(pose2);
    
    RCLCPP_INFO(get_logger(), "🗺️ Waypoints inicializados: %zu puntos", waypoints_.size());
}

geometry_msgs::msg::PoseStamped TiagoBTNode::getCurrentPose()
{
    geometry_msgs::msg::PoseStamped current_pose;
    
    try {
        auto transform = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        
        current_pose.header.frame_id = "map";
        current_pose.header.stamp = this->get_clock()->now();
        current_pose.pose.position.x = transform.transform.translation.x;
        current_pose.pose.position.y = transform.transform.translation.y;
        current_pose.pose.position.z = transform.transform.translation.z;
        current_pose.pose.orientation = transform.transform.rotation;
        
    } catch (tf2::TransformException& ex) {
        RCLCPP_WARN(get_logger(), "⚠️ No se pudo obtener pose actual: %s", ex.what());
        
        // Pose por defecto
        current_pose.header.frame_id = "map";
        current_pose.header.stamp = this->get_clock()->now();
        current_pose.pose.position.x = 0.0;
        current_pose.pose.position.y = 0.0;
        current_pose.pose.position.z = 0.0;
        current_pose.pose.orientation.w = 1.0;
    }
    
    return current_pose;
}

geometry_msgs::msg::Quaternion TiagoBTNode::createQuaternionFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    
    geometry_msgs::msg::Quaternion quat_msg;
    quat_msg.x = q.x();
    quat_msg.y = q.y();
    quat_msg.z = q.z();
    quat_msg.w = q.w();
    
    return quat_msg;
}

double TiagoBTNode::getYawFromQuaternion(const geometry_msgs::msg::Quaternion& quat)
{
    tf2::Quaternion tf_quat(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 matrix(tf_quat);
    double roll, pitch, yaw;
    matrix.getRPY(roll, pitch, yaw);
    return yaw;
}