#include "so_arm_100_hardware/so_arm_100_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <sstream>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace so_arm_100_controller
{
static bool get_joint_rad_limits(const std::string& joint_name, double& rad_min, double& rad_max)
{
    // These MUST match what the rest of the stack assumes (MoveIt / policy_bridge safety).
    // If you change these, update seract_policy_control joint_min/joint_max as well.
    static const std::unordered_map<std::string, std::pair<double, double>> kLimits = {
        {"Shoulder_Rotation", {-M_PI,  M_PI}},
        {"Shoulder_Pitch",    {-1.57,  1.57}},
        {"Elbow",             {-2.50,  2.50}},
        {"Wrist_Pitch",       {-2.50,  2.50}},
        {"Wrist_Roll",        {-2.80,  2.80}},
        // Gripper is commanded separately (Float64MultiArray) but it still appears as a joint in /joint_states.
        // Keep it in a normalized-ish range.
        {"Gripper",           {-1.0,   1.0}},
    };
    auto it = kLimits.find(joint_name);
    if (it == kLimits.end()) return false;
    rad_min = it->second.first;
    rad_max = it->second.second;
    return true;
}
SOARM100Interface::SOARM100Interface() 
{
}

SOARM100Interface::~SOARM100Interface()
{
    if (use_serial_) {
        st3215_.end();
    }
}

CallbackReturn SOARM100Interface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    use_serial_ = hardware_info.hardware_parameters.count("use_serial") ?
        (hardware_info.hardware_parameters.at("use_serial") == "true") : false;
    
    serial_port_ = hardware_info.hardware_parameters.count("serial_port") ?
        hardware_info.hardware_parameters.at("serial_port") : "/dev/ttyACM0";
    
    serial_baudrate_ = hardware_info.hardware_parameters.count("serial_baudrate") ?
        std::stoi(hardware_info.hardware_parameters.at("serial_baudrate")) : 1000000;

    size_t num_joints = info_.joints.size();
    position_commands_.resize(num_joints, 0.0);
    position_states_.resize(num_joints, 0.0);

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SOARM100Interface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SOARM100Interface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        // NOTE: position_commands_ are initialized to 0.0 in on_init()
        // They will be synced to actual positions in on_activate() before controllers can use them
        // The write() function has guards to prevent writing until commands_initialized_ is true
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
    }
    return command_interfaces;
}

CallbackReturn SOARM100Interface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Activating so_arm_100 hardware interface...");
    
    // Reset command initialization flag - will be set after first successful read
    commands_initialized_ = false;
    startup_write_cycles_ = 0;  // Reset startup cycle counter
    first_read_after_activate_ = false;  // Reset read tracking
    startup_read_cycles_ = 0;  // Reset read cycle counter for continuous syncing

    // CRITICAL: Load calibration BEFORE reading positions
    // Otherwise ticks_to_radians() will use fallback (wrong) conversion
    node_ = rclcpp::Node::make_shared("so_arm_100_driver");
    std::string calib_file = info_.hardware_parameters.count("calibration_file") ?
        info_.hardware_parameters.at("calibration_file") : "";
        
    if (!calib_file.empty()) {
        if (!load_calibration(calib_file)) {
            RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                       "Failed to load calibration file: %s - position conversion may be incorrect", calib_file.c_str());
        } else {
            RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                       "Calibration loaded successfully before reading positions");
        }
    } else {
        RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                   "No calibration file specified - using fallback conversion");
    }

    if (use_serial_) {
        if(!st3215_.begin(serial_baudrate_, serial_port_.c_str())) {
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), "Failed to initialize motors");
            return CallbackReturn::ERROR;
        }

        // Initialize each servo
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            
            // First ping the servo (with a few retries to tolerate transient timeouts)
            {
                const int max_ping_retries = 3;
                int ping_attempt = 0;
                int ping_res = -1;
                for (; ping_attempt < max_ping_retries; ++ping_attempt) {
                    ping_res = st3215_.Ping(servo_id);
                    if (ping_res != -1) break;
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                if (ping_res == -1) {
                    RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                                "No response from servo %d during initialization after %d attempts", servo_id, max_ping_retries);
                    return CallbackReturn::ERROR;
                } else if (ping_attempt > 0) {
                    RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"),
                                 "Servo %d ping succeeded after %d retries", servo_id, ping_attempt);
                }
            }
            
            // Set to position control mode (retry a few times to tolerate transient failures)
            {
                const int max_mode_retries = 3;
                int mode_attempt = 0;
                bool mode_ok = false;
                for (; mode_attempt < max_mode_retries; ++mode_attempt) {
                    int res = st3215_.Mode(servo_id, 0);
                    if (res) { mode_ok = true; break; }
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
                if (!mode_ok) {
                    RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                                "Failed to set mode for servo %d after %d attempts", servo_id, max_mode_retries);
                    return CallbackReturn::ERROR;
                } else if (mode_attempt > 0) {
                    RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"),
                                 "Mode set for servo %d succeeded after %d retries", servo_id, mode_attempt);
                }
            }

            // Read initial position and set command to match
            if (st3215_.FeedBack(servo_id) != -1) {
                int pos = st3215_.ReadPos(servo_id);
                // calibrate_servo(servo_id, pos);
                position_states_[i] = ticks_to_radians(pos, i);
                position_commands_[i] = position_states_[i];  // CRITICAL: Set command to match state immediately
                
                // Extra logging for Elbow (servo 3, index 2) to diagnose movement issues
                if (info_.joints[i].name == "Elbow") {
                    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                               "🔧 ELBOW (servo %d, index %zu): ticks=%d, rad=%.4f, command=%.4f (SYNCED)", 
                               servo_id, i, pos, position_states_[i], position_commands_[i]);
                } else {
                    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                               "Servo %d (%s) initialized at position %d (%.4f rad), command synced", 
                               servo_id, info_.joints[i].name.c_str(), pos, position_states_[i]);
                }
            } else {
                // If read failed, still sync command to state to prevent movement
                position_commands_[i] = position_states_[i];
                RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                           "Servo %d: Failed to read position, using last known state (%.4f rad)", servo_id, position_states_[i]);
            }
            
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Mark commands as initialized after setting all of them in on_activate
        // This ensures controllers see matching commands/states from the start
        commands_initialized_ = true;
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                    "All commands synced with actual positions in on_activate() - robot will hold pose");
        // Log all positions for debugging
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                       "  %s: state=%.4f rad, command=%.4f rad", 
                       info_.joints[i].name.c_str(), position_states_[i], position_commands_[i]);
        }
        
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                    "Serial communication initialized on %s", serial_port_.c_str());
    }

    // Initialize ROS node components (node_ already created above for calibration loading)
    feedback_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "feedback", 10, std::bind(&SOARM100Interface::feedback_callback, this, std::placeholders::_1));
    command_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("command", 10);

    // Add services
    calib_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "record_position",
        std::bind(&SOARM100Interface::calibration_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));
                  
    torque_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "toggle_torque",
        std::bind(&SOARM100Interface::torque_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Hardware interface activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SOARM100Interface::on_deactivate(const rclcpp_lifecycle::State &)
{
    if (executor_) {
        executor_->cancel();
    }
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    
    if (use_serial_) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            st3215_.EnableTorque(servo_id, 0);
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Hardware interface deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

void SOARM100Interface::feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_feedback_msg_ = msg;
}

hardware_interface::return_type SOARM100Interface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // CRITICAL: Don't write until commands have been initialized (synced with actual positions)
    // This prevents writing commands before read() has updated states with actual servo positions
    if (!commands_initialized_) {
        // Skip write entirely until first read() has synced commands with states
        return hardware_interface::return_type::OK;
    }
    
    // ADDITIONAL SAFETY: Don't write until at least one read() has occurred after activation
    // This catches the case where write() is called before the first read() in the control loop
    if (!first_read_after_activate_ && use_serial_) {
        // First read() hasn't occurred yet in the control loop - don't write
        RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"), 
                    "Skipping write() - waiting for first read() after activation");
        return hardware_interface::return_type::OK;
    }
    
    // CRITICAL: During startup, continuously check and correct any commands that differ from states
    // This catches cases where the controller writes between read() calls
    // We do this in write() as a final safety check before sending to hardware
    // Extended to 200 cycles to cover controller activation delay
    const int max_startup_write_cycles = 200;  // Extended startup period (controllers activate ~5-6 seconds after hardware)
    bool is_startup_write = (startup_write_cycles_ < max_startup_write_cycles);
    
    if (is_startup_write) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            double delta = std::abs(position_commands_[i] - position_states_[i]);
            // During startup, if command differs from state by more than 0.0005 rad, force match
            // Tighter threshold to catch even small controller-induced movements
            if (delta > 0.0005) {
                double old_cmd = position_commands_[i];
                position_commands_[i] = position_states_[i];  // Force exact match
                
                if (info_.joints[i].name == "Elbow") {
                    RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                               "🔧 ELBOW write() safety: FORCED command to match state (was %.4f, now %.4f, delta=%.4f rad)", 
                               old_cmd, position_commands_[i], delta);
                }
            }
        }
    }
    
    // Check for NaN or invalid commands - these indicate controller wrote before hardware was ready
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        if (std::isnan(position_commands_[i]) || std::isinf(position_commands_[i])) {
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                        "⚠️ Invalid command detected for joint %zu (%s): %f - forcing to match state", 
                        i, info_.joints[i].name.c_str(), position_commands_[i]);
            position_commands_[i] = position_states_[i];
        }
    }
    
    if (use_serial_ && torque_enabled_) {  // Only write if torque is enabled
        // Safety: On first few cycles, clamp commands to be very close to current states
        // This prevents sudden movements if controller tries to write before read() has synced
        // Extended to match max_startup_write_cycles to cover controller activation delay
        const int max_startup_cycles = 200;  // Extended to cover controller activation (~5-6 seconds after hardware)
        bool is_startup = (startup_write_cycles_ < max_startup_cycles);
        
        if (is_startup) {
            startup_write_cycles_++;
            for (size_t i = 0; i < info_.joints.size(); ++i) {
                // Clamp command to be within 0.005 rad of current state during startup (tighter)
                double max_delta = 0.005;  // Reduced from 0.01 for tighter control
                double delta = position_commands_[i] - position_states_[i];
                if (std::abs(delta) > max_delta) {
                    double old_cmd = position_commands_[i];
                    position_commands_[i] = position_states_[i];  // Set to EXACT state, not clamped delta
                    
                    // Extra warning for Elbow
                    if (info_.joints[i].name == "Elbow") {
                        RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                                   "⚠️ ELBOW STARTUP SAFETY: FORCED command to match state! (was %.4f, now %.4f, delta=%.4f rad, state=%.4f)", 
                                   old_cmd, position_commands_[i], delta, position_states_[i]);
                    } else {
                        RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                                   "Startup safety: FORCED joint %zu (%s) command to match state (was %.4f, now %.4f, delta=%.4f rad)", 
                                   i, info_.joints[i].name.c_str(), old_cmd, position_commands_[i], delta);
                    }
                }
            }
            if (startup_write_cycles_ == max_startup_cycles) {
                RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                           "Startup safety period complete (%d cycles) - normal operation resumed", max_startup_cycles);
            }
        } else {
            // Even after startup period, continue to check for large unexpected deltas
            // This catches any controller errors or unexpected commands
            for (size_t i = 0; i < info_.joints.size(); ++i) {
                double delta = std::abs(position_commands_[i] - position_states_[i]);
                // After startup, warn if delta is very large (indicates controller error)
                if (delta > 0.05) {  // 0.05 rad = ~2.9 degrees
                    if (info_.joints[i].name == "Elbow") {
                        RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                                   "⚠️ ELBOW: Large command-state delta detected (%.4f rad) - possible controller error", delta);
                    }
                }
            }
        }
        
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            // Convert from radians (-π to π) to servo ticks (0-4095)
            int joint_pos_cmd = radians_to_ticks(position_commands_[i], i);
            
            RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"), 
                       "Servo %d command: %.2f rad -> %d ticks", 
                       servo_id, position_commands_[i], joint_pos_cmd);
            
            // Try write with retries to tolerate transient serial timeouts
            // Use async write (RegWritePosEx) which is buffered, then execute with RegWriteAction
            {
                const int max_retries = 3;
                int attempt = 0;
                bool write_ok = false;
                for (; attempt < max_retries; ++attempt) {
                    // Use slower speed during startup to prevent sudden movements
                    int speed = (startup_write_cycles_ < max_startup_cycles) ? 2000 : 4500;  // Slower during startup
                    int res = st3215_.RegWritePosEx(servo_id, joint_pos_cmd, speed, 255);
                    if (res) {
                        write_ok = true;
                        break;
                    }
                    // Small delay before retry, with exponential backoff
                    std::this_thread::sleep_for(std::chrono::milliseconds(5 * (attempt + 1)));
                }
                if (!write_ok) {
                    // Only warn occasionally to reduce log spam (every 50th failure)
                    static std::map<uint8_t, int> failure_counts;
                    failure_counts[servo_id]++;
                    if (failure_counts[servo_id] % 50 == 1) {
                        RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                                   "Servo %d write failures (suppressing frequent warnings). Check servo connection.", servo_id);
                    }
                } else if (attempt > 0) {
                    RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"),
                                 "Write to servo %d succeeded after %d retries", servo_id, attempt);
                }
            }
        }
        st3215_.RegWriteAction();
    }

    if (command_publisher_) {
        sensor_msgs::msg::JointState cmd_msg;
        cmd_msg.header.stamp = node_->now();
        
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            cmd_msg.name.push_back(info_.joints[i].name);
            cmd_msg.position.push_back(position_commands_[i]);
        }
        
        command_publisher_->publish(cmd_msg);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SOARM100Interface::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
    bool all_reads_successful = true;
    
    // Mark that at least one read has occurred after activation
    first_read_after_activate_ = true;
    
    // CRITICAL: During startup, continuously sync commands to states in read()
    // This prevents the controller from writing bad commands that cause movement
    // The controller can write to command interfaces at any time (they're direct pointers),
    // so we need to correct them immediately in read() before write() sends them to servos
    // Extended to 200 cycles to match write() startup period and cover controller activation delay
    const int max_startup_read_cycles = 200;  // Sync for first 200 read cycles
    bool is_startup_read = (startup_read_cycles_ < max_startup_read_cycles);
    if (is_startup_read) {
        startup_read_cycles_++;
    }
    
    if (use_serial_) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            
            // Reduced delay between reads for better performance
            if (i > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));  // Reduced from 10ms
            }

            if (!torque_enabled_) {
                // When torque is disabled, only try to read position
                int raw_pos = st3215_.ReadPos(servo_id);
                if (raw_pos != -1) {
                    position_states_[i] = ticks_to_radians(raw_pos, i);
                    // Sync command with state on first read to prevent startup movement
                    if (!commands_initialized_) {
                        position_commands_[i] = position_states_[i];
                    }
                    // During startup, continuously force commands to match states
                    else if (is_startup_read) {
                        double delta = std::abs(position_commands_[i] - position_states_[i]);
                        if (delta > 0.001) {  // If command differs from state by more than 0.001 rad
                            position_commands_[i] = position_states_[i];  // Force match
                            if (info_.joints[i].name == "Elbow") {
                                RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                                           "🔧 ELBOW: read() syncing command to state (delta=%.4f rad)", delta);
                            }
                        }
                    }
                } else {
                    all_reads_successful = false;
                }
                // Continue with last known position if read fails
                continue;  // Skip other reads
            }

            // Full feedback read when torque is enabled
            // Try FeedBack first, if it fails, try direct ReadPos as fallback
            bool read_success = false;
            if (st3215_.FeedBack(servo_id) != -1) {
                int raw_pos = st3215_.ReadPos(servo_id);
                if (raw_pos != -1) {
                    position_states_[i] = ticks_to_radians(raw_pos, i);
                    // Sync command with state on first read to prevent startup movement
                    if (!commands_initialized_) {
                        position_commands_[i] = position_states_[i];
                    }
                    // During startup, continuously force commands to match states
                    else if (is_startup_read) {
                        double delta = std::abs(position_commands_[i] - position_states_[i]);
                        if (delta > 0.001) {  // If command differs from state by more than 0.001 rad
                            position_commands_[i] = position_states_[i];  // Force match
                            if (info_.joints[i].name == "Elbow") {
                                RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                                           "🔧 ELBOW: read() syncing command to state (delta=%.4f rad)", delta);
                            }
                        }
                    }
                    read_success = true;
                    
                    // Read additional feedback data (non-critical, so don't fail if these fail)
                    double speed = -1 * st3215_.ReadSpeed(servo_id) * 2 * M_PI / 4096.0;
                    double pwm = -1 * st3215_.ReadLoad(servo_id) / 10.0;
                    int move = st3215_.ReadMove(servo_id);
                    double temperature = st3215_.ReadTemper(servo_id);
                    double voltage = st3215_.ReadVoltage(servo_id) / 10;
                    double current = st3215_.ReadCurrent(servo_id) * 6.5 / 1000;

                    RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"), 
                                "Servo %d: raw_pos=%d (%.2f rad) speed=%.2f pwm=%.2f temp=%.1f V=%.1f I=%.3f", 
                                servo_id, raw_pos, position_states_[i], speed, pwm, temperature, voltage, current);
                }
            }
            
            // Fallback: try direct position read if FeedBack failed
            if (!read_success) {
                int raw_pos = st3215_.ReadPos(servo_id);
                if (raw_pos != -1) {
                    position_states_[i] = ticks_to_radians(raw_pos, i);
                    // Sync command with state on first read to prevent startup movement
                    if (!commands_initialized_) {
                        position_commands_[i] = position_states_[i];
                    }
                    // During startup, continuously force commands to match states
                    else if (is_startup_read) {
                        double delta = std::abs(position_commands_[i] - position_states_[i]);
                        if (delta > 0.001) {  // If command differs from state by more than 0.001 rad
                            position_commands_[i] = position_states_[i];  // Force match
                            if (info_.joints[i].name == "Elbow") {
                                RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                                           "🔧 ELBOW: read() syncing command to state (delta=%.4f rad)", delta);
                            }
                        }
                    }
                    read_success = true;
                    RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"), 
                                "Servo %d: fallback read successful, position=%d", servo_id, raw_pos);
                } else {
                    all_reads_successful = false;
                    // Only warn if we've tried both methods
                    RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"), 
                                "Failed to read position from servo %d (using last known value)", servo_id);
                }
            }
        }
        
        // Mark commands as initialized after first successful read cycle
        // (This is a backup - commands should already be initialized in on_activate)
        if (all_reads_successful && !commands_initialized_) {
            commands_initialized_ = true;
            RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                       "Commands synced with actual positions in read() - robot will hold current pose");
            // Log all positions for debugging
            for (size_t i = 0; i < info_.joints.size(); ++i) {
                RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                           "  %s: state=%.4f rad, command=%.4f rad", 
                           info_.joints[i].name.c_str(), position_states_[i], position_commands_[i]);
            }
        }
    }
    else {
        sensor_msgs::msg::JointState::SharedPtr feedback_copy;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            feedback_copy = last_feedback_msg_;
        }

        if (feedback_copy) {
            for (size_t i = 0; i < info_.joints.size(); ++i) {
                auto it = std::find(feedback_copy->name.begin(), feedback_copy->name.end(), info_.joints[i].name);
                if (it != feedback_copy->name.end()) {
                    size_t idx = std::distance(feedback_copy->name.begin(), it);
                    if (idx < feedback_copy->position.size()) {
                        position_states_[i] = ticks_to_radians(feedback_copy->position[idx], i);
                    }
                }
            }
        }
    }

    return hardware_interface::return_type::OK;
}

void SOARM100Interface::calibrate_servo(uint8_t servo_id, int current_pos) 
{
    size_t idx = servo_id - 1;
    // Calculate offset from current position to expected zero
    int offset = current_pos - zero_positions_[idx];
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
               "Servo %d: current=%d, zero=%d, offset=%d", 
               servo_id, current_pos, zero_positions_[idx], offset);
}

double SOARM100Interface::ticks_to_radians(int ticks, size_t servo_idx) 
{
    const std::string& joint_name = info_.joints[servo_idx].name;
    
    if (joint_calibration_.count(joint_name) > 0) {
        const auto& calib = joint_calibration_[joint_name];
        double rad_min = -M_PI, rad_max = M_PI;
        (void)get_joint_rad_limits(joint_name, rad_min, rad_max);

        // Piecewise linear mapping around center tick to ensure center maps to 0 rad.
        // This also handles reversed servo direction (min/max can be decreasing).
        const double t = static_cast<double>(ticks);
        const double t_center = static_cast<double>(calib.center_ticks);
        const double t_min = static_cast<double>(calib.min_ticks);
        const double t_max = static_cast<double>(calib.max_ticks);

        // Determine whether ticks is on the "min side" or "max side" of center in tick space.
        auto between = [](double x, double a, double b) {
            return (x - a) * (x - b) <= 0.0;  // x is between a and b (inclusive), regardless of order
        };

        if (between(t, t_center, t_min)) {
            const double denom = (t_min - t_center);
            if (std::abs(denom) < 1e-6) return 0.0;
            const double s = (t - t_center) / denom;   // 0 at center, 1 at min
            const double r = s * rad_min;
            return std::clamp(r, rad_min, rad_max);
        }
        if (between(t, t_center, t_max)) {
            const double denom = (t_max - t_center);
            if (std::abs(denom) < 1e-6) return 0.0;
            const double s = (t - t_center) / denom;   // 0 at center, 1 at max
            const double r = s * rad_max;
            return std::clamp(r, rad_min, rad_max);
        }

        // Out of calibrated span: clamp to nearest endpoint in radians.
        // (This avoids returning nonsense when feedback glitches.)
        const double d_to_min = std::abs(t - t_min);
        const double d_to_max = std::abs(t - t_max);
        return (d_to_min < d_to_max) ? rad_min : rad_max;
    }
    
    // Fallback to default calibration
    return servo_directions_[servo_idx] * 
           (ticks - zero_positions_[servo_idx]) * 2 * M_PI / 4096.0;
}

int SOARM100Interface::radians_to_ticks(double radians, size_t servo_idx) 
{
    const std::string& joint_name = info_.joints[servo_idx].name;
    
    if (joint_calibration_.count(joint_name) > 0) {
        const auto& calib = joint_calibration_[joint_name];
        double rad_min = -M_PI, rad_max = M_PI;
        (void)get_joint_rad_limits(joint_name, rad_min, rad_max);

        // Clamp to expected safe range
        const double r = std::clamp(radians, rad_min, rad_max);
        const double t_center = static_cast<double>(calib.center_ticks);
        const double t_min = static_cast<double>(calib.min_ticks);
        const double t_max = static_cast<double>(calib.max_ticks);

        // Piecewise mapping around center: ensures 0 rad -> center_ticks.
        if (r <= 0.0) {
            if (std::abs(rad_min) < 1e-6) return calib.center_ticks;
            const double s = r / rad_min;  // 0 at 0 rad, 1 at rad_min (rad_min is negative)
            const double t = t_center + s * (t_min - t_center);
            return static_cast<int>(std::lround(t));
        } else {
            if (std::abs(rad_max) < 1e-6) return calib.center_ticks;
            const double s = r / rad_max;  // 0 at 0 rad, 1 at rad_max
            const double t = t_center + s * (t_max - t_center);
            return static_cast<int>(std::lround(t));
        }
    }
    
    // Fallback to default calibration
    return zero_positions_[servo_idx] + 
           servo_directions_[servo_idx] * (int)(radians * 4096.0 / (2 * M_PI));
}

void SOARM100Interface::record_current_position() 
{
    std::stringstream ss;
    ss << "{";  // Start with just a curly brace
    
    bool first = true;  // To handle commas between entries
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        uint8_t servo_id = static_cast<uint8_t>(i + 1);
        
        // Add delay between reads
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Try multiple times to read the servo
        int pos = -1;
        for (int retry = 0; retry < 3 && pos == -1; retry++) {
            st3215_.FeedBack(servo_id);
            pos = st3215_.ReadPos(servo_id);
            if (pos == -1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        if (!first) {
            ss << ",";
        }
        first = false;
        
        ss << "\"" << info_.joints[i].name << "\": {"
           << "\"ticks\": " << (pos != -1 ? pos : 0) << ","
           << "\"speed\": " << st3215_.ReadSpeed(servo_id) << ","
           << "\"load\": " << st3215_.ReadLoad(servo_id)
           << "}";
    }
    ss << "}";  // Close the JSON object
    
    last_calibration_data_ = ss.str();
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                "Recorded positions: %s", last_calibration_data_.c_str());
}

void SOARM100Interface::calibration_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    record_current_position();
    response->success = true;
    response->message = last_calibration_data_;
}

void SOARM100Interface::set_torque_enable(bool enable) 
{
    if (use_serial_) {
        // First set all servos
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            
            if (!enable) {
                // When disabling:
                // 1. Set to idle mode first
                st3215_.Mode(servo_id, 2);  // Mode 2 = idle
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
                // 2. Disable torque
                st3215_.EnableTorque(servo_id, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
                // 3. Double check it's disabled
                st3215_.EnableTorque(servo_id, 0);
            } else {
                // When enabling:
                // 1. Set position mode
                st3215_.Mode(servo_id, 0);  // Mode 0 = position
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
                // 2. Enable torque
                st3215_.EnableTorque(servo_id, 1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Wait a bit to ensure commands are processed
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Update state after all servos are set
        torque_enabled_ = enable;
        
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                    "Torque %s for all servos", enable ? "enabled" : "disabled");
    }
}

void SOARM100Interface::torque_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    bool new_state = !torque_enabled_;
    
    // Set response before changing state
    response->success = true;
    response->message = std::string("Torque ") + (new_state ? "enabled" : "disabled");
    
    // Change state after setting response
    set_torque_enable(new_state);
    
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                "Torque service called, response: %s", response->message.c_str());
}

bool SOARM100Interface::load_calibration(const std::string& filepath) 
{
    try {
        YAML::Node config = YAML::LoadFile(filepath);
        auto joints = config["joints"];
        if (!joints) {
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                        "No joints section in calibration file");
            return false;
        }

        for (const auto& joint : joints) {
            std::string name = joint.first.as<std::string>();
            const auto& data = joint.second;
            
            if (!data["min"] || !data["center"] || !data["max"]) {
                RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                            "Missing calibration data for joint %s", name.c_str());
                continue;
            }

            JointCalibration calib;
            calib.min_ticks = data["min"]["ticks"].as<int>();
            calib.center_ticks = data["center"]["ticks"].as<int>();
            calib.max_ticks = data["max"]["ticks"].as<int>();
            calib.range_ticks = static_cast<double>(calib.max_ticks - calib.min_ticks);
            
            joint_calibration_[name] = calib;
            
            RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                       "Loaded calibration for %s: min=%d, center=%d, max=%d", 
                       name.c_str(), calib.min_ticks, calib.center_ticks, calib.max_ticks);
        }
        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                    "Failed to load calibration: %s", e.what());
        return false;
    }
}

double SOARM100Interface::normalize_position(const std::string& joint_name, int ticks) 
{
    if (joint_calibration_.count(joint_name) == 0) {
        return 0.0;
    }
    
    const auto& calib = joint_calibration_[joint_name];
    double normalized = (ticks - calib.min_ticks) / calib.range_ticks;
    return std::clamp(normalized, 0.0, 1.0);
}

}  // namespace so_arm_100_controller

PLUGINLIB_EXPORT_CLASS(so_arm_100_controller::SOARM100Interface, hardware_interface::SystemInterface)

