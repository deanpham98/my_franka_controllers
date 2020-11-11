////////////////// Improvement
////// - Force filter
////// - Friction model


#include <franka_example_controllers/variable_impedance_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_example_controllers {
    bool VariableImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                    ros::NodeHandle& node_handle) {

        node_handle.subscribe("/control_topic", 20,
                &VariableImpedanceController::controlCallback,
                this, ros::TransportHints().reliable().tcpNoDelay());

        // Read .yaml parameters
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id))
            return false;
        //
        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names))
            return false;

        double publish_rate;
        if (!node_handle.getParam("publish_rate", publish_rate))
            return false;

        // Joint interfaces and handles
        auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr)  return false;
        for (size_t i = 0; i < 7; ++i)
            joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));

        auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) return false;
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));

        auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) return false;
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));

        // Initialize value
        position_d_.setZero();
        orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;

        // Dynamic Reconfigure
        dynamic_server_node_ = ros::NodeHandle("dynamic_server_node");
        dynamic_server_param_ = std::make_unique<
            dynamic_reconfigure::Server<franka_example_controllers::task_paramConfig>>(
            dynamic_server_node_);
        dynamic_server_param_->setCallback(
            boost::bind(&VariableImpedanceController::taskParamCallback, this, _1, _2));

        // Realtime Publisher
        publisher_.init(node_handle, "/state_topic", 1);

        return true;
    }

    void VariableImpedanceController::starting(const ros::Time&) {
        elapsed_time_ = 0.0;
        franka::RobotState initial_state = state_handle_->getRobotState();

        // Initial external force
        Eigen::Map<Eigen::Matrix<double, 6, 1>> force_ext(initial_state.O_F_ext_hat_K.data());
        force_ext_initial_ = force_ext;

        // Initial target position
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        position_d_ = initial_transform.translation();
        orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

        // Initial target twist
        twist_d_.setZero();

        p_u_prev = position_d_(2);
    }

    void VariableImpedanceController::update(const ros::Time&, const ros::Duration& period) {
        elapsed_time_ += period.toSec();

        franka::RobotState robot_state = state_handle_->getRobotState();

        // Jacobian matrix
        std::array<double, 42> jacobian_array =
            model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

        // Coriolis
        std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

        // Robot state: Force/Torque Sensor Measurement
        Eigen::Map<Eigen::Matrix<double, 6, 1>> force_ext(robot_state.O_F_ext_hat_K.data());

        // Robot state: Joint Velocities
        Eigen::Map<Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());

        // Robot state: Desired joint torque without gravity (Needed for torque saturation)
        Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());

        // Robot state: End-Effector pose - 4 x 4 matrix
        transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
        Eigen::Vector3d position(transform_.translation()); // Position
        Eigen::Quaterniond orientation(transform_.linear()); // Quaternion

        Eigen::VectorXd desired_force_torque(6), tau_hybrid(7), tau_cmd(7);

        // Desired Force/Torque
        desired_force_torque.setZero();
        desired_force_torque(2) = -desired_force_;
        desired_force_torque(3) = -desired_force_ * position(1);
        desired_force_torque(4) =  desired_force_ * position(0);

        /**********Force Control*********/
        double dp_u = k_pf * (desired_force_torque(2) - force_ext(2));
        double p_u = p_u_prev + period.toSec() * dp_u;
        p_u_prev = p_u;
        position_d_(2) += p_u;
        twist_d_(2) += dp_u;
        /********************************/

        /*************Orientation Error*************/
        orientation_d_.coeffs() << 1.0, 0.0, 0.0, 0.0;
        if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation * orientation_d_.inverse());
        // convert to axis angle
        Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
        /*******************************************/

        /*************Position Error************/
        Eigen::Matrix<double, 6, 1> position_err;
        position_err.head(3) << position_d_ - position;
        position_err.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
        /***************************************/
        
        Eigen::Matrix<double, 6, 1> ee_vel = jacobian * dq;

        // Realtime publisher
        if (rate_trigger_() && publisher_.trylock()) {
            for (size_t i = 0; i < 6; ++i) {
                publisher_.msg_.ee_vel[i] = ee_vel(i);
                publisher_.msg_.force[i] = force_ext(i);
                publisher_.msg_.force_des[i] = desired_force_torque(i);
            }
            publisher_.unlockAndPublish();
        }

        // Find tau_hybrid = tau_force + tau_task
        tau_hybrid = jacobian.transpose() * (K_P.cwiseProduct(position_err) + K_D.cwiseProduct(twist_d_ - ee_vel));

        // Command torque
        tau_cmd = tau_hybrid + coriolis;

        // Torque saturation
        tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);
        
        for (size_t i = 0; i < 7; ++i)
            // joint_handles_[i].setCommand(tau_cmd(i));
            cout << tau_cmd(i) << std::endl;

        // Update dynamic parameters
        updateDynamicReconfigure();
    }

    void VariableImpedanceController::stopping(const ros::Time&) {}

    Eigen::Matrix<double, 7, 1> VariableImpedanceController::saturateTorqueRate(
                                const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
                                const Eigen::Matrix<double, 7, 1>& tau_J_d) {
        Eigen::Matrix<double, 7, 1> tau_d_saturated{};
        for (size_t i = 0; i < 7; i++) {
            double difference = tau_d_calculated[i] - tau_J_d[i];
            tau_d_saturated[i] = tau_J_d[i] + 
                    std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
        }
        return tau_d_saturated;
    }

    void VariableImpedanceController::updateDynamicReconfigure() {
        for (size_t i = 0; i < 3; ++i)
            position_d_(i) += twist_d_(i) * 0.001;
    }

    void VariableImpedanceController::controlCallback(const franka_example_controllers::ControlMsg& msg) {
        K_P.head(6) << msg.stiffness[0], msg.stiffness[1], msg.stiffness[2], msg.stiffness[3], msg.stiffness[4], msg.stiffness[5];
        K_D.head(6) << 2 * std::sqrt(msg.stiffness[0]), 2 * std::sqrt(msg.stiffness[1]), 2 * std::sqrt(msg.stiffness[2]), 2 * std::sqrt(msg.stiffness[3]), 2 * std::sqrt(msg.stiffness[4]), 2 * std::sqrt(msg.stiffness[5]);
    }

    void VariableImpedanceController::taskParamCallback(franka_example_controllers::task_paramConfig& config,
                            uint32_t level) {
        // Force gains
        k_pf = config.k_pf;

        target_twist_ << config.v_x, config.v_y, 0, 0, 0, 0;

        // Target force
        target_force_ = config.desired_force;
    }

} // end namespace

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::VariableImpedanceController,
                       controller_interface::ControllerBase)