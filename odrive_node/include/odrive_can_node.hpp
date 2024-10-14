#ifndef ODRIVE_CAN_NODE_HPP
#define ODRIVE_CAN_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include "odrive_can/msg/o_drive_status.hpp"
#include "odrive_can/msg/controller_status.hpp"
#include "odrive_can/msg/control_message.hpp"
#include "odrive_can/msg/brake_resistor_status.hpp"
#include "odrive_can/srv/axis_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "odrive_can/srv/limits.hpp"
#include "odrive_can/srv/traj_acc_limits.hpp"
#include "odrive_can/srv/traj_vel_limits.hpp"
#include "odrive_can/srv/traj_inertia.hpp"
#include "socket_can.hpp"

#include <mutex>
#include <condition_variable>
#include <array>
#include <algorithm>
#include <linux/can.h>
#include <linux/can/raw.h>

using std::placeholders::_1;
using std::placeholders::_2;

using ODriveStatus = odrive_can::msg::ODriveStatus;
using ControllerStatus = odrive_can::msg::ControllerStatus;
using ControlMessage = odrive_can::msg::ControlMessage;
using BrakeResistorStatus = odrive_can::msg::BrakeResistorStatus;

using AxisState = odrive_can::srv::AxisState;
using Empty = std_srvs::srv::Empty;
using Limits = odrive_can::srv::Limits;
using TrajVelLimits = odrive_can::srv::TrajVelLimits;
using TrajAccLimits = odrive_can::srv::TrajAccLimits;
using TrajInertia = odrive_can::srv::TrajInertia;

class ODriveCanNode : public rclcpp::Node {
public:
    ODriveCanNode(const std::string& node_name);
    bool init(EpollEventLoop* event_loop); 
    void deinit();
private:
    // CAN Recieve Event Callback
    void recv_callback(const can_frame& frame);
    // Subscriber Callbacks
    void ctrl_msg_subscriber_callback(const ControlMessage::SharedPtr msg);
    // Subscriber Event Callbacks
    void ctrl_msg_callback();

    // Service Callbacks
    void axis_state_service_callback(const std::shared_ptr<AxisState::Request> request, std::shared_ptr<AxisState::Response> response);
    void service_clear_errors_callback(const std::shared_ptr<Empty::Request> request, std::shared_ptr<Empty::Response> response);
    void limits_service_callback(const std::shared_ptr<Limits::Request> request, std::shared_ptr<Limits::Response> response);
    void traj_vel_limit_service_callback(const std::shared_ptr<TrajVelLimits::Request> request, std::shared_ptr<TrajVelLimits::Response> response);
    void traj_acc_limits_service_callback(const std::shared_ptr<TrajAccLimits::Request> request, std::shared_ptr<TrajAccLimits::Response> response);
    void traj_inertia_service_callback(const std::shared_ptr<TrajInertia::Request> request, std::shared_ptr<TrajInertia::Response> response);
    // Service Event Callback
    void request_state_callback();
    void request_clear_errors_callback();
    void set_limits_callback();
    void set_traj_vel_callback();
    void set_traj_acc_callback();
    void set_traj_inertia_callback();
    // Utilities
    inline bool verify_length(const std::string&name, uint8_t expected, uint8_t length);
    void wait_for_result();
    
    uint16_t node_id_;
    SocketCanIntf can_intf_ = SocketCanIntf();
    
    short int ctrl_pub_flag_ = 0;
    std::mutex ctrl_stat_mutex_;
    ControllerStatus ctrl_stat_ = ControllerStatus();
    rclcpp::Publisher<ControllerStatus>::SharedPtr ctrl_publisher_;
    
    short int odrv_pub_flag_ = 0;
    std::mutex odrv_stat_mutex_;
    ODriveStatus odrv_stat_ = ODriveStatus();
    rclcpp::Publisher<ODriveStatus>::SharedPtr odrv_publisher_;

    short int br_pub_flag_ = 0;
    std::mutex br_stat_mutex_;
    BrakeResistorStatus br_stat_ = BrakeResistorStatus();
    rclcpp::Publisher<BrakeResistorStatus>::SharedPtr br_publisher_;

    EpollEvent sub_evt_;
    std::mutex ctrl_msg_mutex_;
    ControlMessage ctrl_msg_ = ControlMessage();
    rclcpp::Subscription<ControlMessage>::SharedPtr ctrl_msg_subscriber_;

    EpollEvent axis_state_srv_evt_;
    uint32_t axis_state_;
    std::mutex axis_state_mutex_;
    std::condition_variable fresh_heartbeat_;
    rclcpp::Service<AxisState>::SharedPtr axis_state_service_;

    EpollEvent srv_clear_errors_evt_;
    rclcpp::Service<Empty>::SharedPtr service_clear_errors_;

    EpollEvent set_limits_srv_evt_;
    float vel_limit_;
    float cur_limit_;
    std::mutex limits_mutex_;
    rclcpp::Service<Limits>::SharedPtr set_limits_service_;

    EpollEvent traj_acc_limits_srv_evt_;
    float traj_acc_limit_;
    float traj_deacc_limit_;
    std::mutex traj_acc_limits_mutex_;
    rclcpp::Service<TrajAccLimits>::SharedPtr set_traj_acc_limits_service_;

    EpollEvent traj_vel_limits_srv_evt_;
    float traj_vel_limit_;
    std::mutex traj_vel_limits_mutex_;
    rclcpp::Service<TrajVelLimits>::SharedPtr set_traj_vel_limits_service_;

    EpollEvent traj_inertia_srv_evt_;
    float traj_inertia_;
    std::mutex traj_inertia_mutex_;
    rclcpp::Service<TrajInertia>::SharedPtr set_traj_inertia_service_;
};

#endif // ODRIVE_CAN_NODE_HPP
