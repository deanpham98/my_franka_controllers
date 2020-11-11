import numpy
import rospy
#from my_fetch_train import robot_gazebo_env_goal
import base_env
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from fetch_train.srv import EePose, EePoseRequest, EeRpy, EeRpyRequest, EeTraj, EeTrajRequest, JointTraj, JointTrajRequest


class FrankaEnv(base_env.BaseEnv):
    """Superclass for all Fetch environments.
    """

    def __init__(self):
        """Initializes a new Fetch environment.
        Args:
            
        """


        """
        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that the stream of data doesn't flow. This is for simulations
        that are paused for whatever reason
        2) If the simulation was running already for some reason, we need to reset the controllers.
        This has to do with the fact that some plugins with tf don't understand the reset of the simulation
        and need to be reset to work properly.
        """

        # Define pub/sub topics
        self.state_topic = "/state_topic"
        self.control_topic = "/control_topic"
        
        # Define state subscriber and control publisher
        self.state_sub = rospy.Subscriber(self.state_topic, StateMsg, self.state_callback)
        self.control_pub = rospy.Publisher(self.control_topic, ControlMsg, queue_size=20)

        rospy.init_node("python", anonymous=True)
        rospy.Rate(100) # Publish Impedance at freq = 100Hz

        self.stateMsg = StateMsg()

        self.controllers_list = []

        self.robot_name_space = ""
        
        # We launch the init function of the Parent Class robot_gazebo_env_goal.RobotGazeboEnv
        super(FrankaEnv, self).__init__(controllers_list=self.controllers_list,
                                        robot_name_space=self.robot_name_space,
                                        reset_controls=False)

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other simulation systems are
        operational.
        """
        self._check_all_sensors_ready()
        return True

    def _check_all_sensors_ready(self):
        self._check_joint_states_ready()
        
        rospy.logdebug("ALL SENSORS READY")

    def _check_joint_states_ready(self):
        self.stateMsg = None
        while self.stateMsg is None and not rospy.is_shutdown():
            try:
                self.stateMsg = rospy.wait_for_message(self.state_topic, StateMsg, timeout=1.0)
            except:
                rospy.logerr("Current state topic not ready yet, retrying for getting stateMsg")
        return self.stateMsg
    
    def state_callback(self, data):
        self.stateMsg = data

    def get_stateMsg(self):
        return self.stateMsg

    def get_state_val(self):
        msg = self.get_stateMsg()
        return msg.ee_pos_err, msg.ee_vel_err, msg.force, msg.force_des

    def set_impedance_val(self, action):
        """
        Helper function.

        """
        controlMsg = ControlMsg()
        controlMsg.stiffness = action
        self.control_pub.publish(controlMsg)

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at the start
        of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode done based on observations given.
        """
        raise NotImplementedError()