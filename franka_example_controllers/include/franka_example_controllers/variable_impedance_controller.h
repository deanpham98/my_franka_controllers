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

// realtime_publisher
#include <realtime_tools/realtime_publisher.h>
#include <realtime_tools/realtime_buffer.h>
#include <franka_hw/trigger_rate.h>
#include <franka_example_controllers/VariableImpedanceControllerCommand.h>
#include <franka_example_controllers/VariableImpedanceControllerState.h>
#include <franka_example_controllers/Gain.h>

namespace franka_example_controllers {
    class VariableImpedanceController : public controller_interface::MultiInterfaceController<
                                        franka_hw::FrankaStateInterface,
                                        franka_hw::FrankaModelInterface,
                                        hardware_interface::EffortJointInterface> {
    public:

        struct cmd {
            Eigen::Matrix<double, 6, 1> f; // force
            Eigen::Vector3d p; // position
            Eigen::Quaterniond q; // quaternion
            Eigen::Matrix<double, 6, 1> v; // velocity
            Eigen::Matrix<double, 6, 6> S; // selection matrix
            Eigen::Matrix<double, 6, 1> fd; // desired force
        };

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
        Eigen::Matrix<double, 6, 6> k_pf;
        Eigen::Matrix<double, 6, 1> sum;

        // x-y Motion control
        Eigen::Affine3d transform_;
        
        // Adjoint transform
        Eigen::Matrix<double, 6, 6> adjoint(Eigen::Affine3d transform);

        // Gain Values
        Eigen::Matrix<double, 6, 6> K_P;
        Eigen::Matrix<double, 6, 6> K_D;

        // Realtime Buffer
        realtime_tools::RealtimeBuffer<cmd> ut_buffer_;

        //Realtime Publisher
        Eigen::Matrix<double, 7, 1> last_tau_d_;
        double elapsed_time_{0.0};
        realtime_tools::RealtimePublisher<franka_example_controllers::VariableImpedanceControllerState> pub_state_;
        franka_hw::TriggerRate rate_trigger_{500.0};
        void publish_state(const double t);

        bool kReceiveCommand;
        bool kReceiveGainConfig;

        ros::Subscriber sub_command_, sub_gain_config_;
        void commandCallback(const franka_example_controllers::VariableImpedanceControllerCommandConstPtr& msg);
        void gainConfigCallback(const franka_example_controllers::GainConstPtr& msg);

        // desired command (control law)
        cmd ud_;

        // Filter joint velocity
        Eigen::Matrix<double, 7, 1> dq_filter_;

        // Offset for measured force
        Eigen::Matrix<double, 6, 1> f0_;

        // low-pass filter params
        double alpha_dq_{0.95};
    };
} // end namespace