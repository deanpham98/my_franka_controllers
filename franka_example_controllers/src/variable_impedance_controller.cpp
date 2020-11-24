////////////////// Improvement
////// - Force filter
////// - Friction model
#include <cmath>
#include <vector>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>

#include <franka_example_controllers/variable_impedance_controller.h>

namespace franka_example_controllers {
    bool VariableImpedanceController::init(hardware_interface::RobotHW* robot_hw,
                                    ros::NodeHandle& node_handle) {

        sub_command_ = node_handle.subscribe("command", 20,
                &VariableImpedanceController::commandCallback,
                this, ros::TransportHints().reliable().tcpNoDelay());

        sub_gain_config_ = node_handle.subscribe("gain_config", 20,
                &VariableImpedanceController::gainConfigCallback,
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

        // Setting initial gain
        std::vector<double> kp;
        if (!node_handle.getParam("gain/Kp", kp)) {
            return false;
        }

        K_P.setIdentity();
        K_D.setIdentity();

        for (size_t i = 0; i < 6; ++i) {
            K_P(i, i) = kp[i];
            K_D(i, i) = 2 * std::sqrt(K_P(i, i));
        }

        k_pf.setIdentity();

        k_pf(2, 2) = 0.5;

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

        // Realtime Publisher
        pub_state_.init(node_handle, "state", 1);

        return true;
    }

    void VariableImpedanceController::starting(const ros::Time&) {
        elapsed_time_ = 0.0;
        franka::RobotState initial_state = state_handle_->getRobotState();

        // Initial external force
        // Eigen::Map<Eigen::Matrix<double, 6, 1>> force_ext(initial_state.O_F_ext_hat_K.data());
        // force_ext_initial_ = force_ext;

        // Initial target position
        Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
        position_d_ = initial_transform.translation();
        orientation_d_ = Eigen::Quaterniond(initial_transform.linear());

        // Initial target command
        cmd ut;
        ut.f.setZero();
        ut.p = p;
        ut.q = q;
        ut.v.setZero();
        ut.S.setIdentity();
        ut.fd.setZero();
        ut_buffer_.writeFromNonRT(ut);

        // Initial desired command
        ud_.f.setZero();
        ud_.p = p;
        ud_.q = q;
        ud_.v.setZero();
        ud_.S.setIdentity();
        ud_.fd.setZero();

        // initial filter velocity
        dq_filter_ = Eigen::Vector7d::Map(initial_state.dq.data());

        // initial external force for compensation
        f0_ = Eigen::Vector6d::Map(initial_state.O_F_ext_hat_K.data());

        p_u_prev(2) = ud_.fd(2);
    }

    void VariableImpedanceController::update(const ros::Time&, const ros::Duration& period) {
        elapsed_time_ += period.toSec();

        franka::RobotState robot_state = state_handle_->getRobotState();

        // update filter velocity
        Eigen::Vector7d dq(Eigen::Vector7d::Map(robot_state.dq.data()));
        dq_filter_ = alpha_dq_ * dp + (1 - alpha_dq_) * dq_filter_;

        // read target command from buffer
        cmd& ut = *ut_buffer_.readFromRT();

        ud_.S = ut.S;
        ud_.p = ut.p;
        ud_.q = ut.q;
        ud_.f = ut.f;
        ud_.v = ut.v;
        ud_.fd = ut.fd;

        // Coriolis
        std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

        // Robot state: Desired joint torque without gravity (Needed for torque saturation)
        Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());

        // Robot state: End-Effector pose - 4 x 4 matrix
        transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
        Eigen::Vector3d position(transform_.translation()); // Position
        Eigen::Quaterniond orientation(transform_.linear()); // Quaternion

        Eigen::Vector7d tau_hybrid, tau_cmd;

        // Desired Force/Torque
        ud_.fd(3) = -ud_.fd(2) * position(1);
        ud_.fd(4) =  ud_.fd(2) * position(0);

        /**********Force Control*********/

        Eigen::Vector6d dp_u = k_pf * (ud_.fd - ud_.f);
        Eigen::Vector6d p_u = p_u_prev + period.toSec() * dp_u;
        p_u_prev = p_u;
        ud_.p += p_u;
        ud_.v += dp_u;

        /********************************/

        /*************Orientation Error*************/
        ud_.q.coeffs() << 1.0, 0.0, 0.0, 0.0;
        if (ud_.q.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(ud_.q * orientation.inverse());
        // convert to axis angle
        Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
        /*******************************************/

        /*************Position Error************/
        Eigen::Vector6d position_err;
        position_err.head(3) << ud_.p - position;
        position_err.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();
        /***************************************/
        
        // jacobian matrix
        std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

        // body jacobian
        std::array<double, 42> body_jacobian_array =
        model_handle_->getBodyJacobian(franka::Frame::kEndEffector);

        tau_cmd = jacobian.transpose() * (K_P * (position_err) 
                        + K_D * (ud_.v - jacobian * dq_filter_)) + coriolis;

        // Torque saturation
        tau_cmd = saturateTorqueRate(tau_cmd, tau_J_d);
        
        for (size_t i = 0; i < 7; ++i)
            joint_handles_[i].setCommand(tau_cmd(i));

        // Realtime publisher
        if (rate_trigger_() && pub_state_.trylock()) {
            for (size_t i = 0; i < 3; ++i) {
                pub_state_.msg_.ee_vel[i] = ud_.v(i);
                pub_state_.msg_.ee_vel[i + 3] = ud_.v(i + 3);
                pub_state_.msg_.pd[i] = ud_.p(i);
                pub_state_.msg_.p[i] = position(i);
                pub_state_.msg_.Kp[i] = K_P(i, i);
                pub_state_.msg_.Kd[i] = K_D(i, i);
                pub_state_.msg_.Kp[i+3] = K_P(i+3, i+3);
                pub_state_.msg_.Kd[i+3] = K_D(i+3, i+3);
            }
            
            for (size_t i=0; i<42; i++){
                pub_state_.msg_.jacobian[i] = jacobian_array[i];
                pub_state_.msg_.body_jacobian[i] = body_jacobian_array[i];
            }

            for (size_t i=0; i<7; i++) {
                pub_state_.msg_.tau_ext[i] = robot_state.tau_ext_hat_filtered[i];
                pub_state_.msg_.tau_s[i] = robot_state.tau_J[i];
                pub_state_.msg_.tau_d[i] = robot_state.tau_J_d[i];
                pub_state_.msg_.g[i] = gravity[i];
                pub_state_.msg_.C[i] = coriolis_array[i];
            }

            for (size_t i=0; i<49; i++) {
                pub_state_.msg_.M[i] = mass[i];
            }

            pub_state_.msg_.qd[0] = ud_.q.w();
            pub_state_.msg_.qd[1] = ud_.q.x();
            pub_state_.msg_.qd[2] = ud_.q.y();
            pub_state_.msg_.qd[3] = ud_.q.z();

            pub_state_.msg_.q[0] = q.w();
            pub_state_.msg_.q[1] = q.x();
            pub_state_.msg_.q[2] = q.y();
            pub_state_.msg_.q[3] = q.z();

            pub_state_.unlockAndPublish();
        }        
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

    void VariableImpedanceController::commandCallback(const franka_example_controllers::VariableImpedanceControllerCommand& msg) {
        cmd ut;
        ut.f = Eigen::Vector6d::Map(msg.f.data());
        ut.p = Eigen::Vector3d::Map(msg.p.data());
        ut.q = Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]);
        ut.v = Eigen::Vector6d::Map(msg.v.data());
        ut.S.setIdentity();
        ut.S.diagonal() = Eigen::Vector6d::Map(msg.S.data());
        ut.f = Eigen::Vector6d::Map(msg.fd.data());

        ut_buffer_.writeFromNonRT(ut);
    }

    void VariableImpedanceController::gainConfigCallback(const franka_example_controllers::Gain& msg) {
        K_P.setIdentity();
        K_D.setIdentity();
        if (msg.kDefineDamping==<1) {
            for (size_t i = 0; i < 6; ++i) {
                K_P(i, i) = msg.kp[i];
                K_D(i, i) = msg.kd[i];
            }
        } else {
            for (size_t i = 0; i < 6; ++i) {
                K_P(i, i) = msg.kp[i];
                K_D(i, i) = 2 * std::sqrt(msg.kp[i]);
            }
        }
    }

} // end namespace

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::VariableImpedanceController,
                       controller_interface::ControllerBase)