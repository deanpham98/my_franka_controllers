from gym import utils

import math
import rospy
from gym import spaces
import franka_env
from gym.envs.registration import register
import numpy as np

register(
        id='FrankaSliding-v0',
        entry_point='openai_ros:task_env.FrankaSlidingEnv',
        timestep_limit=1000,
    )


class FrankaSlidingEnv(franka_env.FrankaEnv, utils.EzPickle):
    def __init__(self):
        
        # self.get_params()
        
        franka_env.FrankaEnv.__init__(self)
        utils.EzPickle.__init__(self)
        
        print ("Call env setup")
        self._env_setup(initial_qpos=self.init_pos)
        
        print ("Call get_obs")
        obs = self._get_obs()
        
        self.V_MIN = 
        self.V_MAX = 
        self.F_MIN = 
        self.F_MAX = 
        self.K_MIN = 
        self.K_MAX =
        self.F_DES = 

        self.low_obs = np.array(
            [
                self.V_MIN, self.V_MIN, self.V_MIN, self.V_MIN, self.V_MIN, self.V_MIN,
                self.F_MIN, self.F_MIN, self.F_MIN, self.F_MIN, self.F_MIN, self.F_MIN,
                self.F_MIN, self.F_MIN, self.F_MIN, self.F_MIN, self.F_MIN, self.F_MIN
            ], dtype=np.float64
        )

        self.high_obs = np.array(
            [
                self.V_MAX, self.V_MAX, self.V_MAX, self.V_MAX, self.V_MAX, self.V_MAX,
                self.F_MAX, self.F_MAX, self.F_MAX, self.F_MAX, self.F_MAX, self.F_MAX,
                self.F_MAX, self.F_MAX, self.F_MAX, self.F_MAX, self.F_MAX, self.F_MAX
            ], dtype=np.float64
        )

        self.low_act = np.array([self.K_MIN, self.K_MIN, self.K_MIN, self.K_MIN, self.K_MIN, self.K_MIN], dtype=np.float64)
        self.high_act = np.array([self.K_MAX, self.K_MAX, self.K_MAX, self.K_MAX, self.K_MAX, self.K_MAX], dtype=np.float64)

        self.observation_space = spaces.Box(self.low_obs, self.high_obs, dtype=np.float64)
        self.action_space = spaces.Box(self.low_act, self.high_act, dtype=np.float64)
        
    def _set_action(self, action):
        
        # # Take action
        action = action.copy()
            
        # Publish impedance action to controller
        self.set_impedance_val(action)

    def _get_obs(self):
        ee_pos_err, ee_vel_err, force, force_des = self.get_state_val()
        return np.concatenate(ee_pos_err, ee_vel_err, force, force_des)
        
    def _is_done(self, observations):
        d = self.goal_distance(observations['achieved_goal'], self.goal)
        return (d < self.distance_threshold).astype(np.float32)
        
    def _compute_reward(self, observations, done):
        force = observations[8]
        force_des = observations[14]
        if (force >= 0.9 * force_des and force <= 1.1 * force_des):
            return 1 - math.fabs(force_des - force) / force_des
        return - math.fabs(force_des - force) / force_des
        
    def _init_env_variables(self):
        """
        Inits variables needed to be initialized each time we reset at the start
        of an episode.
        :return:
        """
        pass
    
    def _set_init_pose(self):
        """Sets the Robot in its init pose
        """
        self.set_trajectory_joints(self.init_pos)

        return True
        
    def goal_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)
        

    def _sample_goal(self):
        if self.has_object:
            goal = self.initial_gripper_xpos[:3] + self.np_random.uniform(-self.target_range, self.target_range, size=3)
            goal += self.target_offset
            goal[2] = self.height_offset
            if self.target_in_the_air and self.np_random.uniform() < 0.5:
                goal[2] += self.np_random.uniform(0, 0.45)
        else:
            goal = self.initial_gripper_xpos[:3] + self.np_random.uniform(-0.15, 0.15, size=3)
        
        #return goal.copy()
        return goal
        
    def _sample_achieved_goal(self, grip_pos_array, object_pos):
        if not self.has_object:
            achieved_goal = grip_pos_array.copy()
        else:
            achieved_goal = np.squeeze(object_pos.copy())
        
        #return achieved_goal.copy()
        return achieved_goal
        
    def _env_setup(self, initial_qpos):
        print ("Init Pos:")
        print (initial_qpos)
        #for name, value in initial_qpos.items():
        self.gazebo.unpauseSim()
        self.set_trajectory_joints(initial_qpos)
            #self.execute_trajectory()
        #utils.reset_mocap_welds(self.sim)
        #self.sim.forward()

        # Move end effector into position.
        gripper_target = np.array([0.498, 0.005, 0.431 + self.gripper_extra_height])# + self.sim.data.get_site_xpos('robot0:grip')
        gripper_rotation = np.array([1., 0., 1., 0.])
        #self.sim.data.set_mocap_pos('robot0:mocap', gripper_target)
        #self.sim.data.set_mocap_quat('robot0:mocap', gripper_rotation)
        action = np.concatenate([gripper_target, gripper_rotation])
        self.set_trajectory_ee(action)
        #self.execute_trajectory()
        #for _ in range(10):
            #self.sim.step()
            #self.step()

        # Extract information for sampling goals.
        #self.initial_gripper_xpos = self.sim.data.get_site_xpos('robot0:grip').copy()
        gripper_pos = self.get_ee_pose()
        gripper_pose_array = np.array([gripper_pos.pose.position.x, gripper_pos.pose.position.y, gripper_pos.pose.position.z])
        self.initial_gripper_xpos = gripper_pose_array.copy()
        if self.has_object:
            self.height_offset = self.sim.data.get_site_xpos('object0')[2]
            
        self.goal = self._sample_goal()
        self._get_obs()
        
    def robot_get_obs(self, data):
        
        """
        Returns all joint positions and velocities associated with a robot.
        """
    
        if data.position is not None and data.name:
            #names = [n for n in data.name if n.startswith('robot')]
            names = [n for n in data.name]
            i = 0
            r = 0
            for name in names:
                r += 1
                
            return (
                np.array([data.position[i] for i in range(r)]),
                np.array([data.velocity[i] for i in range(r)]),
            )
        return np.zeros(0), np.zeros(0)