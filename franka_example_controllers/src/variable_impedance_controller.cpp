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

        k_pf.setZero();

        k_pf(2, 2) = 1e-2;

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

        // Realtime Publisher
        pub_state_.init(node_handle, "state", 1);

        // service
        reset_controller_serv_ =
            node_handle.advertiseService<ResetController::Request, ResetController::Response>(
                "reset_controller",
                boost::bind(&VariableImpedanceController::reset_controller, this, _1, _2));


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
        Eigen::Vector3d p = initial_transform.translation();
        Eigen::Quaterniond q = Eigen::Quaterniond(initial_transform.linear());

        // Initial target command
        cmd ut;
        ut.f.setZero();
        ut.p = p;
        ut.q = q;
        ut.v.setZero();
        ut_buffer_.writeFromNonRT(ut);

        // Initial desired command
        ud_.f.setZero();
        ud_.p = p;
        ud_.q = q;
        ud_.v.setZero();

        // initial filter velocity
        dq_filter_ = Eigen::Matrix<double, 7, 1>::Map(initial_state.dq.data());

        // initial external force for compensation
        f0_ = Eigen::Matrix<double, 6, 1>::Map(initial_state.O_F_ext_hat_K.data());

        // p_u_prev(2) = ud_.p(2);
        sum.setZero();
        kResetController = false;
    }

    void VariableImpedanceController::update(const ros::Time& t_clock, const ros::Duration& period) {
        elapsed_time_ += period.toSec();

        franka::RobotState robot_state = state_handle_->getRobotState();

        // update filter velocity
        Eigen::Matrix<double, 7, 1> dq(Eigen::Matrix<double, 7, 1>::Map(robot_state.dq.data()));
        dq_filter_ = alpha_dq_ * dq + (1 - alpha_dq_) * dq_filter_;

        // read target command from buffer
        cmd& ut = *ut_buffer_.readFromRT();

        Eigen::Matrix<double, 6, 1> f_ = Eigen::Matrix<double, 6, 1>::Map(robot_state.O_F_ext_hat_K.data());

        ud_.p = ut.p;
        ud_.q = ut.q;
        ud_.f = ut.f;
        ud_.v = ut.v;

        // mass
        std::array<double, 49> mass = model_handle_->getMass();
        std::array<double, 7> gravity = model_handle_->getGravity();

        // Coriolis
        std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
        Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());

        // Robot state: Desired joint torque without gravity (Needed for torque saturation)
        Eigen::Map<Eigen::Matrix<double, 7, 1> > tau_J_d(robot_state.tau_J_d.data());

        // Robot state: End-Effector pose - 4 x 4 matrix
        transform_ = Eigen::Matrix4d::Map(robot_state.O_T_EE.data());
        Eigen::Vector3d position(transform_.translation()); // Position
        Eigen::Quaterniond orientation(transform_.linear()); // Quaternion

        // service
        if (kResetController) {
          f0_ = f_;
          sum.setZero();
          kResetController = false;
        }


        if (!ud_.f.isZero(0)) {

            Eigen::Matrix<double, 6, 1> dp_u = k_pf * (ud_.f - (f_ - f0_));
            sum += period.toSec() * dp_u;
            for (size_t i = 0; i < 6; i++) {
                sum(i) = (sum(i) > 3e-2) ? 3e-2 : sum(i);
                sum(i) = (sum(i) < -3e-2) ? -3e-2 : sum(i);
            }
            ud_.p += sum.head(3);
            ud_.v += dp_u;
        }


        /*************Orientation Error*************/
        if (ud_.q.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(ud_.q * orientation.inverse());
        // convert to axis angle
        Eigen::AngleAxisd error_quaternion_angle_axis(error_quaternion);
        /*******************************************/

        /*************Position Error************/
        Eigen::Matrix<double, 6, 1> position_err;
        position_err.head(3) << ud_.p - position;
        position_err.tail(3) << error_quaternion_angle_axis.axis() * error_quaternion_angle_axis.angle();

        for (size_t i = 0; i < 3; i++){
            if (std::isnan(position_err(i + 3)))
                position_err(i + 3) = 0.;

        }

        /***************************************/

        // jacobian matrix
        std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
        Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());

        // body jacobian
        std::array<double, 42> body_jacobian_array =
        model_handle_->getBodyJacobian(franka::Frame::kEndEffector);

        Eigen::Matrix<double, 7, 1> tau_cmd = jacobian.transpose() * (K_P * (position_err)
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
                pub_state_.msg_.fe[i] = f_(i) - f0_(i);
                pub_state_.msg_.fe[i + 3] = f_(i + 3) - f0_(i + 3);
                pub_state_.msg_.fd[i] = ud_.f(i);
                pub_state_.msg_.fd[i + 3] = ud_.f(i + 3);
                pub_state_.msg_.pd[i] = ud_.p(i);
                pub_state_.msg_.p[i] = position(i);
                pub_state_.msg_.Kp[i] = K_P(i, i);
                pub_state_.msg_.Kd[i] = K_D(i, i);
                pub_state_.msg_.Kp[i+3] = K_P(i+3, i+3);
                pub_state_.msg_.Kd[i+3] = K_D(i+3, i+3);
            }

            for (size_t i=0; i < 42; i++){
                pub_state_.msg_.jacobian[i] = jacobian_array[i];
                pub_state_.msg_.body_jacobian[i] = body_jacobian_array[i];
            }

            for (size_t i=0; i < 7; i++) {
                pub_state_.msg_.tau_ext[i] = robot_state.tau_ext_hat_filtered[i];
                pub_state_.msg_.tau_s[i] = robot_state.tau_J[i];
                pub_state_.msg_.tau_d[i] = robot_state.tau_J_d[i];
                pub_state_.msg_.g[i] = gravity[i];
                pub_state_.msg_.C[i] = coriolis_array[i];
            }

            for (size_t i=0; i < 49; i++) {
                pub_state_.msg_.M[i] = mass[i];
            }

            pub_state_.msg_.qd[0] = ud_.q.w();
            pub_state_.msg_.qd[1] = ud_.q.x();
            pub_state_.msg_.qd[2] = ud_.q.y();
            pub_state_.msg_.qd[3] = ud_.q.z();

            pub_state_.msg_.q[0] = orientation.w();
            pub_state_.msg_.q[1] = orientation.x();
            pub_state_.msg_.q[2] = orientation.y();
            pub_state_.msg_.q[3] = orientation.z();

            pub_state_.msg_.time = elapsed_time_;

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

    void VariableImpedanceController::commandCallback(const franka_example_controllers::VariableImpedanceControllerCommandConstPtr& msg) {
        cmd ut;
        ut.f = Eigen::Matrix<double, 6, 1>::Map(msg->f.data());
        ut.p = Eigen::Vector3d::Map(msg->p.data());
        ut.q = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
        ut.v = Eigen::Matrix<double, 6, 1>::Map(msg->v.data());
        // if (ut.v(1) > 0.01 || ut.v(1) < -0.01) {
        //     std::cout << "ut.v = " << ut.v << std::endl;
        //     std::cout << "---------------" << std::endl;
        // } else
        // {
        //     std::cout << "LALALALALALA" << std::endl;
        // }

        ut_buffer_.writeFromNonRT(ut);
    }

    void VariableImpedanceController::gainConfigCallback(const franka_example_controllers::GainConstPtr& msg) {
        K_P.setIdentity();
        K_D.setIdentity();

        for (size_t i = 0; i < 6; ++i) {
            K_P(i, i) = msg->kp[i];
            K_D(i, i) = msg->kd[i];
        }
    }

    bool VariableImpedanceController::reset_controller(ResetController::Request &req, ResetController::Response &res){
      ROS_INFO("Received request to reset controller");
      kResetController = true;
      return true;
    }
} // end namespace

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::VariableImpedanceController,
                       controller_interface::ControllerBase)
