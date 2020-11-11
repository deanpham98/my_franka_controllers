// Can use
#include <memory>
#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <Eigen/Dense>

// Dynamic Reconfigure
#include <dynamic_reconfigure/server.h>
#include <franka_example_controllers/hybrid_paramConfig.h>

// realtime_publisher
#include <realtime_tools/realtime_publisher.h>
#include <franka_hw/trigger_rate.h>
#include <franka_example_controllers/StateMsg.h>
#include <franka_example_controllers/ControlMsg.h>

namespace franka_example_controllers {
    class VariableImpedanceController : public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaStateInterface,
                                        franka_hw::FrankaModelInterface,
                                        hardware_interface::EffortJointInterface> {
    public:
        bool init(hardware_interface::RobotHW* robot_hw, ros::NodeHandle& node_handle) override;
        void starting(const ros::Time&) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        void stopping(const ros::Time&) override;

    private:
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;

        // Torque saturation and limitation
        static constexpr double delta_tau_max_{5.0};
        Eigen::Matrix<double, 7, 1> saturateTorqueRate(
            const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
            const Eigen::Matrix<double, 7, 1>& tau_J_d);

        // z Force control
        double desired_force_{0.0};
        double k_pf{0.0};
        double p_u_prev{0.0};
        Eigen::Matrix<double, 6, 1> force_ext_initial_;

        // x-y Motion control
        Eigen::Affine3d transform_;
        Eigen::Matrix<double, 6, 1> twist_d_;
        Eigen::Vector3d position_d_;
        Eigen::Quaterniond orientation_d_;
        // Adjoint transform
        Eigen::Matrix<double, 6, 6> adjoint(Eigen::Affine3d transform);

        // Gain Values
        Eigen::Matrix<double, 6, 1> K_P;
        Eigen::Matrix<double, 6, 1> K_D;

        // Dynamic reconfigure
        double target_force_{0.0};
        Eigen::Matrix<double, 6, 1> target_twist_;
        void updateDynamicReconfigure();

        std::unique_ptr<dynamic_reconfigure::Server<franka_example_controllers::hybrid_paramConfig>>
            dynamic_server_param_;
        ros::NodeHandle dynamic_server_node_;
        void hybridParamCallback(franka_example_controllers::hybrid_paramConfig& config,
                                uint32_t level);

        //Realtime Publisher
        Eigen::Matrix<double, 7, 1> last_tau_d_;
        double elapsed_time_{0.0};
        realtime_tools::RealtimePublisher<franka_example_controllers::StateMsg> publisher_;
        franka_hw::TriggerRate rate_trigger_{500.0};

        ros::Subscriber control_sub;
        void controlCallback(const franka_example_controllers::ControlMsg& msg);
    };
} // end namespace