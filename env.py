import gym
from gym import spaces
import numpy as np
import pybullet as p
import pybullet_data as pd
from robot import Robot
from utils.utils import euclidean_distance

class PickAndPlaceEnv(gym.Env):
    def __init__(self):
        super(PickAndPlaceEnv, self).__init__()
        
        # Initialize PyBullet
        self.client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pd.getDataPath())
        p.setGravity(0, 0, -9.81)

        # Load plane and robot
        self.plane_id = p.loadURDF("plane.urdf")
        self.robot = Robot(client=self.client)
        
        # Define action and observation space
        self.action_space = spaces.Box(low=-0.1, high=0.1, shape=(4,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=np.array([-1.0, -1.0, -1.0]),
            high=np.array([1.0, 1.0, 1.0]),
            dtype=np.float32
        )
        
        # Positions
        self.pick_position = np.array([0.5, 0.0, 0.1])
        self.place_position = np.array([0.0, 0.5, 0.1])

        # Reset the environment
        self.reset()

    def reset(self):
        p.resetSimulation(self.client)
        self.plane_id = p.loadURDF("plane.urdf")
        self.robot.reset()

        self.object_position = self.pick_position + np.random.uniform(-0.05, 0.05, size=3)
        
        return self._get_observation()

    def step(self, action):
        target_position = self.robot.get_end_effector_position() + action[:3]
        target_orientation = p.getQuaternionFromEuler([0, 0, action[3]])

        self.robot.move(target_position, target_orientation)
        observation = self._get_observation()
        reward = self._calculate_reward(target_position)
        done = self._check_done()

        return observation, reward, done, {}

    def _get_observation(self):
        end_effector_pos = self.robot.get_end_effector_position()
        joint_angles = self.robot.get_joint_angles()
        return np.concatenate([end_effector_pos, joint_angles])

    def _calculate_reward(self, target_position):
        distance_to_pick = euclidean_distance(self.object_position, target_position)
        distance_to_place = euclidean_distance(self.place_position, target_position)

        if distance_to_pick < 0.05:
            return 1.0
        elif distance_to_place < 0.05:
            return 10.0
        else:
            return -0.01 * distance_to_pick

    def _check_done(self):
        end_effector_pos = self.robot.get_end_effector_position()
        if euclidean_distance(end_effector_pos, self.place_position) < 0.05:
            return True
        return False

    def render(self, mode='human'):
        pass

    def close(self):
        p.disconnect(self.client)
