#include "odrive_can_node.hpp"
#include "epoll_event_loop.hpp"
#include "byte_swap.hpp"
#include <sys/eventfd.h>
#include <chrono>

// *******************************************************************************************************************************************
// ODRIVE ENUMS
// *******************************************************************************************************************************************
enum CmdId : uint32_t {
    kHeartbeat = 0x001,            // ControllerStatus  - publisher
    kGetError = 0x003,             // SystemStatus      - publisher
    kRxSdo,                        // SDO RECIEVE       - publisher
    kTxSdo,                        // SDO TRANSMIT      - subscriber
    kSetAxisState = 0x007,         // SetAxisState      - service
    kGetEncoderEstimates = 0x009,  // ControllerStatus  - publisher
    kSetControllerMode = 0x00b,    // ControlMessage    - subscriber
    kSetInputPos,                  // ControlMessage    - subscriber
    kSetInputVel,                  // ControlMessage    - subscriber
    kSetInputTorque,               // ControlMessage    - subscriber
    kSetLimits,                    // ControlMessage    - subscriber
    kSetTrajVelLimit=0x011,        // ControlMessage    - subscriber
    kSetTrajAccLimit,              // ControlMessage    - subscriber
    kSetTrajInertia,               // ControlMessage    - subscriber
    kGetIq = 0x014,                // ControllerStatus  - publisher
    kGetTemp,                      // SystemStatus      - publisher
    kGetBusVoltageCurrent = 0x017, // SystemStatus      - publisher
    kClearErrors,                  // ClearErrors       - service
    kGetTorques = 0x01c,           // ControllerStatus  - publisher
    kGetPower = 0x01d,             // SystemStatus      - publisher
};

enum OpCode : uint8_t {
    kRead = 0x00,
    kWrite,
};

enum Endpoints : uint16_t {
    kBrakeResMeasCurrent = 620,
    kBrakeResCurrStatus,
    kBrakeResDuty,
    kBrakeResAddDuty,
    kBrakeResCurrent,
    kBrakeResChopperTemp,
    kBrakeResIsArmed,
    kBrakeResWasSat,
};

enum ControlMode : uint64_t {
    kVoltageControl,
    kTorqueControl,
    kVelocityControl,
    kPositionControl,
};

// *******************************************************************************************************************************************
// INITIALIZATION FUNCTIONS
// *******************************************************************************************************************************************
ODriveCanNode::ODriveCanNode(const std::string& node_name) : rclcpp::Node(node_name) {
    
    rclcpp::Node::declare_parameter<std::string>("interface", "can0");
    rclcpp::Node::declare_parameter<uint16_t>("node_id", 0);

    rclcpp::QoS ctrl_stat_qos(rclcpp::KeepAll{});
    ctrl_publisher_ = rclcpp::Node::create_publisher<ControllerStatus>("controller_status", ctrl_stat_qos);
    
    rclcpp::QoS odrv_stat_qos(rclcpp::KeepAll{});
    odrv_publisher_ = rclcpp::Node::create_publisher<ODriveStatus>("odrive_status", odrv_stat_qos);

    rclcpp::QoS br_stat_qos(rclcpp::KeepAll{});
    br_publisher_ = rclcpp::Node::create_publisher<BrakeResistorStatus>("brake_resistor_status", br_stat_qos);

    rclcpp::QoS ctrl_msg_qos(rclcpp::KeepAll{});
    ctrl_msg_subscriber_ = rclcpp::Node::create_subscription<ControlMessage>("control_message", ctrl_msg_qos, std::bind(&ODriveCanNode::ctrl_msg_subscriber_callback, this, _1));

    rclcpp::QoS srv_qos(rclcpp::KeepAll{});
    axis_state_service_ = rclcpp::Node::create_service<AxisState>("request_axis_state", std::bind(&ODriveCanNode::axis_state_service_callback, this, _1, _2), srv_qos.get_rmw_qos_profile());
    clear_errors_service_ = rclcpp::Node::create_service<ClearErrors>("clear_errors", std::bind(&ODriveCanNode::clear_errors_service_callback, this, _1, _2), srv_qos.get_rmw_qos_profile());
    set_limits_service_ = rclcpp::Node::create_service<Limits>("set_limits", std::bind(&ODriveCanNode::limits_service_callback, this, _1, _2), srv_qos.get_rmw_qos_profile());
    set_traj_acc_limits_service_ = rclcpp::Node::create_service<TrajAccLimits>("set_traj_acc_limits", std::bind(&ODriveCanNode::traj_acc_limits_service_callback, this, _1, _2), srv_qos.get_rmw_qos_profile());
    set_traj_vel_limits_service_ = rclcpp::Node::create_service<TrajVelLimits>("set_traj_vel_limits", std::bind(&ODriveCanNode::traj_vel_limit_service_callback, this, _1, _2), srv_qos.get_rmw_qos_profile());
    set_traj_inertia_service_ = rclcpp::Node::create_service<TrajInertia>("set_traj_inertia", std::bind(&ODriveCanNode::traj_inertia_service_callback, this, _1, _2), srv_qos.get_rmw_qos_profile());
}

void ODriveCanNode::deinit() {
    sub_evt_.deinit();
    axis_state_srv_evt_.deinit();
    clear_errors_srv_evt_.deinit();
    set_limits_srv_evt_.deinit();
    traj_acc_limits_srv_evt_.deinit();
    traj_vel_limits_srv_evt_.deinit();
    traj_inertia_srv_evt_.deinit();
    can_intf_.deinit();
}

bool ODriveCanNode::init(EpollEventLoop* event_loop) {

    node_id_ = rclcpp::Node::get_parameter("node_id").as_int();
    std::string interface = rclcpp::Node::get_parameter("interface").as_string();

    if (!can_intf_.init(interface, event_loop, std::bind(&ODriveCanNode::recv_callback, this, _1))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize socket can interface: %s", interface.c_str());
        return false;
    }
    if (!sub_evt_.init(event_loop, std::bind(&ODriveCanNode::ctrl_msg_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize subscriber event");
        return false;
    }
    if (!axis_state_srv_evt_.init(event_loop, std::bind(&ODriveCanNode::request_state_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize axis state service event");
        return false;
    }
    if (!clear_errors_srv_evt_.init(event_loop, std::bind(&ODriveCanNode::clear_errors_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize clear errors service event");
        return false;
    }
    if (!set_limits_srv_evt_.init(event_loop, std::bind(&ODriveCanNode::set_limits_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize limits service event");
        return false;
    }
    if (!traj_acc_limits_srv_evt_.init(event_loop, std::bind(&ODriveCanNode::set_traj_acc_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize traj acc limits service event");
        return false;
    }
    if (!traj_vel_limits_srv_evt_.init(event_loop, std::bind(&ODriveCanNode::set_traj_vel_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize traj vel limits service event");
        return false;
    }
    if (!traj_inertia_srv_evt_.init(event_loop, std::bind(&ODriveCanNode::set_traj_inertia_callback, this))) {
        RCLCPP_ERROR(rclcpp::Node::get_logger(), "Failed to initialize traj inertia service event");
        return false;
    }
    RCLCPP_INFO(rclcpp::Node::get_logger(), "node_id: %d", node_id_);
    RCLCPP_INFO(rclcpp::Node::get_logger(), "interface: %s", interface.c_str());
    return true;
}

// *******************************************************************************************************************************************
// PUBLISHER FUNCTIONS
// *******************************************************************************************************************************************
void ODriveCanNode::recv_callback(const can_frame& frame) {

    if(((frame.can_id >> 5) & 0x3F) != node_id_) return;

    switch(frame.can_id & 0x1F) {
        case CmdId::kHeartbeat: {
            if (!verify_length("kHeartbeat", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.active_errors    = read_le<uint32_t>(frame.data + 0);
            ctrl_stat_.axis_state        = read_le<uint8_t>(frame.data + 4);
            ctrl_stat_.procedure_result  = read_le<uint8_t>(frame.data + 5);
            ctrl_stat_.trajectory_done_flag = read_le<bool>(frame.data + 6);
            ctrl_pub_flag_ |= 0b00001;
            fresh_heartbeat_.notify_one();
            break;
        }
        case CmdId::kGetError: {
            if (!verify_length("kGetError", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.active_errors = read_le<uint32_t>(frame.data + 0);
            odrv_stat_.disarm_reason = read_le<uint32_t>(frame.data + 4);
            odrv_pub_flag_ |= 0b001;
            break;
        }
        case CmdId::kGetEncoderEstimates: {
            if (!verify_length("kGetEncoderEstimates", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.pos_estimate = read_le<float>(frame.data + 0);
            ctrl_stat_.vel_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b00010;
            break;
        }
        case CmdId::kGetIq: {
            if (!verify_length("kGetIq", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.iq_setpoint = read_le<float>(frame.data + 0);
            ctrl_stat_.iq_measured = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b00100;
            break;
        }
        case CmdId::kGetTemp: {
            if (!verify_length("kGetTemp", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.fet_temperature   = read_le<float>(frame.data + 0);
            odrv_stat_.motor_temperature = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b010;
            break;
        }
        case CmdId::kGetBusVoltageCurrent: {
            if (!verify_length("kGetBusVoltageCurrent", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(odrv_stat_mutex_);
            odrv_stat_.bus_voltage = read_le<float>(frame.data + 0);
            odrv_stat_.bus_current = read_le<float>(frame.data + 4);
            odrv_pub_flag_ |= 0b100;
            break;
        }
        case CmdId::kGetTorques: {
            if (!verify_length("kGetTorques", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.torque_target   = read_le<float>(frame.data + 0);
            ctrl_stat_.torque_estimate = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b01000; 
            break;
        }
        case CmdId::kGetPower: {
            if (!verify_length("kGetPower", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(ctrl_stat_mutex_);
            ctrl_stat_.elec_power = read_le<float>(frame.data + 0);
            ctrl_stat_.mech_power = read_le<float>(frame.data + 4);
            ctrl_pub_flag_ |= 0b10000; 
            break;
        }
        case CmdId::kTxSdo: {
            if (!verify_length("kTxSdo", 8, frame.can_dlc)) break;
            std::lock_guard<std::mutex> guard(br_stat_mutex_);
            if(read_le<uint16_t>(frame.data+1) == Endpoints::kBrakeResMeasCurrent){
                br_stat_.brake_resistor_meas_current = read_le<float>(frame.data + 4);
                br_pub_flag_ |= 0b00001; 
            }
            else if(read_le<uint16_t>(frame.data+1) == Endpoints::kBrakeResCurrent){
                br_stat_.brake_resistor_calc_current = read_le<float>(frame.data + 4);
                br_pub_flag_ |= 0b00010; 
            }
            else if(read_le<uint16_t>(frame.data+1) == Endpoints::kBrakeResDuty){
                br_stat_.brake_resistor_duty_cycle = read_le<float>(frame.data + 4);
                br_pub_flag_ |= 0b00100; 
            }
            else if(read_le<uint16_t>(frame.data+1) == Endpoints::kBrakeResChopperTemp){
                br_stat_.brake_resistor_chopper_temp = read_le<float>(frame.data + 4);
                br_pub_flag_ |= 0b01000; 
            }
            else if(read_le<uint16_t>(frame.data+1) == Endpoints::kBrakeResCurrStatus){
                br_stat_.brake_resistor_current_status = read_le<uint32_t>(frame.data + 4);
                br_pub_flag_ |= 0b10000; 
            }
            break;
        }
        default: {
            RCLCPP_WARN(rclcpp::Node::get_logger(), "Received unused message: ID = 0x%x", (frame.can_id & 0x1F));
            break;
        }
    }
    
    if (ctrl_pub_flag_ == 0b11111) {
        ctrl_publisher_->publish(ctrl_stat_);
        ctrl_pub_flag_ = 0;
    }
    
    if (br_pub_flag_ == 0b11111) {
        br_publisher_->publish(br_stat_);
        br_pub_flag_ = 0;
    }

    if (odrv_pub_flag_ == 0b111) {
        odrv_publisher_->publish(odrv_stat_);
        odrv_pub_flag_ = 0;
    }
}

// *******************************************************************************************************************************************
// SERVICES FUNCTIONS
// *******************************************************************************************************************************************
void ODriveCanNode::axis_state_service_callback(const std::shared_ptr<AxisState::Request> request, std::shared_ptr<AxisState::Response> response) {
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        axis_state_ = request->axis_requested_state;
        RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting axis state: %d", axis_state_);
    }
    axis_state_srv_evt_.set();

    wait_for_result(); 

    response->axis_state = ctrl_stat_.axis_state;
    response->active_errors = ctrl_stat_.active_errors;
    response->procedure_result = ctrl_stat_.procedure_result;
}
void ODriveCanNode::request_state_callback() {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        write_le<uint32_t>(axis_state_, frame.data);
    }
    frame.can_dlc = 4;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::clear_errors_service_callback(const std::shared_ptr<ClearErrors::Request> request, std::shared_ptr<ClearErrors::Response> response) {
    RCLCPP_INFO(rclcpp::Node::get_logger(), "clearing errors");
    clear_errors_srv_evt_.set();

    wait_for_result();

    response->active_errors = ctrl_stat_.active_errors;
    response->procedure_result = ctrl_stat_.procedure_result;
}
void ODriveCanNode::clear_errors_callback() {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | CmdId::kClearErrors;
    {
        write_le<uint8_t>(1, frame.data);
    }
    frame.can_dlc = 4;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::limits_service_callback(const std::shared_ptr<Limits::Request> request, std::shared_ptr<Limits::Response> response) {
    {
        std::unique_lock<std::mutex> guard(limits_mutex_);
        vel_limit_ = request->velocity_limit;
        cur_limit_ = request->current_limit;
        RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting velocity limit: %f", vel_limit_);
        RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting current limit: %f", cur_limit_);
    }
    set_limits_srv_evt_.set();

    wait_for_result(); 

    response->active_errors = ctrl_stat_.active_errors;
    response->procedure_result = ctrl_stat_.procedure_result;
}
void ODriveCanNode::set_limits_callback() {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | CmdId::kSetLimits;
    {
        std::unique_lock<std::mutex> guard(axis_state_mutex_);
        write_le<float>(vel_limit_, frame.data);
        write_le<float>(cur_limit_, frame.data);
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::traj_acc_limits_service_callback(const std::shared_ptr<TrajAccLimits::Request> request, std::shared_ptr<TrajAccLimits::Response> response) {
    {
        std::unique_lock<std::mutex> guard(traj_acc_limits_mutex_);
        traj_acc_limit_ = request->traj_accel_limit;
        traj_deacc_limit_ = request->traj_deaccel_limit;
        RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting trajectory acceleration limit: %f", traj_acc_limit_);
        RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting trajectory deacceleration limit: %f", traj_deacc_limit_);
    }
    traj_acc_limits_srv_evt_.set();

    wait_for_result(); 

    response->active_errors = ctrl_stat_.active_errors;
    response->procedure_result = ctrl_stat_.procedure_result;
}
void ODriveCanNode::set_traj_acc_callback() {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | CmdId::kSetTrajAccLimit;
    {
        std::unique_lock<std::mutex> guard(traj_acc_limits_mutex_);
        write_le<float>(traj_acc_limit_, frame.data);
        write_le<float>(traj_deacc_limit_, frame.data);
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::traj_vel_limit_service_callback(const std::shared_ptr<TrajVelLimits::Request> request, std::shared_ptr<TrajVelLimits::Response> response) {
    {
        std::unique_lock<std::mutex> guard(traj_vel_limits_mutex_);
        traj_vel_limit_ = request->traj_vel_limit;
        RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting trajectory velocity limit: %f", traj_vel_limit_);
    }
    traj_vel_limits_srv_evt_.set();

    wait_for_result(); 

    response->active_errors = ctrl_stat_.active_errors;
    response->procedure_result = ctrl_stat_.procedure_result;
}
void ODriveCanNode::set_traj_vel_callback() {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | CmdId::kSetTrajVelLimit;
    {
        std::unique_lock<std::mutex> guard(traj_vel_limits_mutex_);
        write_le<float>(traj_vel_limit_, frame.data);
    }
    frame.can_dlc = 4;
    can_intf_.send_can_frame(frame);
}

void ODriveCanNode::traj_inertia_service_callback(const std::shared_ptr<TrajInertia::Request> request, std::shared_ptr<TrajInertia::Response> response) {
    {
        std::unique_lock<std::mutex> guard(traj_inertia_mutex_);
        traj_inertia_ = request->traj_inertia;
        RCLCPP_INFO(rclcpp::Node::get_logger(), "requesting trajectory inertia: %f", traj_inertia_);
    }
    traj_inertia_srv_evt_.set();

    wait_for_result(); 

    response->active_errors = ctrl_stat_.active_errors;
    response->procedure_result = ctrl_stat_.procedure_result;
}
void ODriveCanNode::set_traj_inertia_callback() {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | CmdId::kSetTrajInertia;
    {
        std::unique_lock<std::mutex> guard(traj_inertia_mutex_);
        write_le<float>(traj_inertia_, frame.data);
    }
    frame.can_dlc = 4;
    can_intf_.send_can_frame(frame);
}
// *******************************************************************************************************************************************
// SUBSCRIBER FUNCTIONS 
// *******************************************************************************************************************************************
void ODriveCanNode::ctrl_msg_subscriber_callback(const ControlMessage::SharedPtr msg) {
    std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
    ctrl_msg_ = *msg;
    sub_evt_.set();
}

void ODriveCanNode::ctrl_msg_callback() {

    uint32_t control_mode;
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | kSetControllerMode;
    {
        std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
        write_le<uint32_t>(ctrl_msg_.control_mode, frame.data);
        write_le<uint32_t>(ctrl_msg_.input_mode,   frame.data + 4);
        control_mode = ctrl_msg_.control_mode;
    }
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);
    
    frame = can_frame{};
    switch (control_mode) {
        case ControlMode::kVoltageControl: {
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "Voltage Control Mode (0) is not currently supported");
            return;
        }
        case ControlMode::kTorqueControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_torque");
            frame.can_id = node_id_ << 5 | kSetInputTorque;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_torque, frame.data);
            frame.can_dlc = 4;
            break;
        }
        case ControlMode::kVelocityControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_vel");
            frame.can_id = node_id_ << 5 | kSetInputVel;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_vel,       frame.data);
            write_le<float>(ctrl_msg_.input_torque, frame.data + 4);
            frame.can_dlc = 8;
            break;
        }
        case ControlMode::kPositionControl: {
            RCLCPP_DEBUG(rclcpp::Node::get_logger(), "input_pos");
            frame.can_id = node_id_ << 5 | kSetInputPos;
            std::lock_guard<std::mutex> guard(ctrl_msg_mutex_);
            write_le<float>(ctrl_msg_.input_pos,  frame.data);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_vel) * 1000)),    frame.data + 4);
            write_le<int8_t>(((int8_t)((ctrl_msg_.input_torque) * 1000)), frame.data + 6);
            frame.can_dlc = 8;
            break;
        }    
        default: 
            RCLCPP_ERROR(rclcpp::Node::get_logger(), "unsupported control_mode: %d", control_mode);
            return;
    }

    can_intf_.send_can_frame(frame);

    // Get Brake Resistor States 
    frame.can_id = node_id_ << 5 | kRxSdo;
    write_le<uint8_t>(OpCode::kRead, frame.data);
    write_le<uint16_t>(Endpoints::kBrakeResMeasCurrent, frame.data + 1);
    frame.can_dlc = 8;
    can_intf_.send_can_frame(frame);

    write_le<uint16_t>(Endpoints::kBrakeResCurrStatus, frame.data + 1);
    can_intf_.send_can_frame(frame);

    write_le<uint16_t>(Endpoints::kBrakeResCurrent, frame.data + 1);
    can_intf_.send_can_frame(frame);

    write_le<uint16_t>(Endpoints::kBrakeResChopperTemp, frame.data + 1);
    can_intf_.send_can_frame(frame);

    write_le<uint16_t>(Endpoints::kBrakeResDuty, frame.data + 1);
    can_intf_.send_can_frame(frame);
}

// *******************************************************************************************************************************************
// UTILITIES
// *******************************************************************************************************************************************
inline bool ODriveCanNode::verify_length(const std::string&name, uint8_t expected, uint8_t length) {
    bool valid = expected == length;
    RCLCPP_DEBUG(rclcpp::Node::get_logger(), "received %s", name.c_str());
    if (!valid) RCLCPP_WARN(rclcpp::Node::get_logger(), "Incorrect %s frame length: %d != %d", name.c_str(), length, expected);
    return valid;
}

void ODriveCanNode::wait_for_result(){
    std::unique_lock<std::mutex> guard(ctrl_stat_mutex_); // define lock for controller status
    auto call_time = std::chrono::steady_clock::now();
    fresh_heartbeat_.wait(guard, [this, &call_time]() {
        bool complete = (this->ctrl_stat_.procedure_result != 1) && // make sure procedure_result is not busy
            (std::chrono::steady_clock::now() - call_time >= std::chrono::seconds(1)); // wait for minimum one second 
        return complete; 
        }); // wait for procedure_result
}