"""
An example of how one might use PyRep to create their RL environments.
In this case, the Franka Panda must reach a randomly placed target.
This script contains examples of:
    - RL environment example.
    - Scene manipulation.
    - Environment resets.
    - Setting joint properties (control loop disabled, motor locked at 0 vel)
"""
from os.path import dirname, join, abspath
from pyrep import PyRep
# from pyrep.robots.arms.panda import Panda
# from pyrep.objects.shape import Shape
from pyrep.robots.legged_robots.slalom import Slalom
import numpy as np
# import time

SCENE_FILE = join(dirname(abspath(__file__)),
                  'scene_slalom_rl_env.ttt')
# POS_MIN, POS_MAX = [0.8, -0.2, 1.0], [1.0, 0.2, 1.4]
EPISODES = 500
EPISODE_LENGTH = 200


class SlalomEnv(object):

    def __init__(self):
        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()
        # time.sleep(0.5)
        self.agent = Slalom()
        self.agent.set_leg_control_loop_enabled(False)
        self.agent.set_body_control_loop_enabled(False)

        self.initial_position = self.agent.get_imu_positions()
        self.initial_leg_joint_positions = self.agent.get_leg_joint_positions()
        self.initial_body_joint_positions = self.agent.get_body_joint_positions()

    def _get_state(self):
        # Return state containing arm joint angles/velocities & target position
        return np.concatenate([
                                self.agent.get_leg_joint_positions(),
                                self.agent.get_leg_joint_velocities(),
                                self.agent.get_body_joint_positions(),
                                self.agent.get_body_joint_velocities(),
                                ])

    def reset(self):
        # Get a random position within a cuboid and set the target position
        # pos = list(np.random.uniform(POS_MIN, POS_MAX))
        # self.target.set_position(pos)
        self.agent.set_leg_joint_target_positions(self.initial_leg_joint_positions)
        # print(self.initial_leg_joint_positions)
        self.agent.set_body_joint_target_positions(self.initial_body_joint_positions)
        return self._get_state()

    def step(self, action):
        self.agent.set_leg_joint_target_positions(action[:16])          # Execute action on leg
        self.agent.set_body_joint_target_positions(action[16:19])       # Execute action on body
        self.pr.step()                                                  # Step the physics simulation
        # ax, ay, az = self.agent_ee_tip.get_position()
        # tx, ty, tz = self.target.get_position()
        # Reward is negative distance to target
        # reward = -np.sqrt((ax - tx) ** 2 + (ay - ty) ** 2 + (az - tz) ** 2)

        robot_position = self.agent.get_imu_positions()
        reward = -robot_position[1] - (-self.initial_position[1]) 
        return reward, self._get_state()

    def shutdown(self):
        self.pr.stop()
        self.pr.shutdown()
    
    def starto(self):
        self.pr.start()
    
    def stopo(self):
        self.pr.stop()


class Agent(object):

    def act(self, state):
        del state
        return list(np.random.uniform(-0.3, 0.3, size=(19,)))

    def learn(self, replay_buffer):
        del replay_buffer
        pass


env = SlalomEnv()
agent = Agent()
replay_buffer = []

for e in range(EPISODES):

    print('Starting episode %d' % e)
    env.starto()
    state = env.reset()
    # print("state", state)
    for i in range(EPISODE_LENGTH):
        action = agent.act(state)
        # print("action", action, np.array(action).shape)
        reward, next_state = env.step(action)
        print("reward", reward)
        # print("next_state", next_state, np.array(next_state).shape)
        replay_buffer.append((state, action, reward, next_state))
        state = next_state
        agent.learn(replay_buffer)
    env.stopo()
print('Done!')
env.shutdown()
