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
import math
# import time

SCENE_FILE = join(dirname(abspath(__file__)),
                  'scene_slalom_rl_env.ttt')
# POS_MIN, POS_MAX = [0.8, -0.2, 1.0], [1.0, 0.2, 1.4]
EPISODES = 10 #500
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
        self.initial_orientation = self.agent.get_imu_orientations()
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
    
    def _slipping_detector(self, _stand_one: int, _vel_threshould: float):
        linear_vel, angular_vel = self.agent.get_leg_atip_velocity(_stand_one)
        abs_linear_vel = math.sqrt(linear_vel[0]**2 + linear_vel[1]**2)
        # print(linear_vel[0], linear_vel[1], abs_linear_vel)
        return 1 if abs_linear_vel > _vel_threshould else 0

    def _tip_rotate_detector(self, _stand_one: int, _tip_rot_threshould: float):
        atip_rot = self.agent.get_leg_atip_orientation(_stand_one)
        abs_atip_roll = abs(atip_rot[0])
        abs_atip_pitch = abs(atip_rot[1])

        _tip_rot_score = 0
        if abs_atip_roll > _tip_rot_threshould:
            _tip_rot_score += 1
        if abs_atip_pitch > _tip_rot_threshould:
            _tip_rot_score += 1

        return _tip_rot_score


    # reward function
    def _get_reward(self):
        # walking distance
        robot_position = self.agent.get_imu_positions()
        walking_dist = (-robot_position[1]) - (-self.initial_position[1])

        # -- stability
        _imu_rot = self.agent.get_imu_orientations()
        _imu_pos = self.agent.get_imu_positions()
        abs_roll = abs(_imu_rot[0])
        abs_pitch = abs(_imu_rot[1])
        _yaw = _imu_rot[2]

        abs_heading_direct = abs(_yaw - self.initial_orientation[2])
        abs_body_height = abs(_imu_pos[2] - self.initial_position[2])

        stability = abs_body_height + abs_roll + abs_pitch + abs_heading_direct
        # print(stability, abs_body_height, abs_roll, abs_pitch, abs_heading_direct)


        # -- collision
        collision_threshold = 0.005          # considered 0.5 cm got collision 
        # defualt distance Motor2 <-> floor = 0.08
        body_floor_collision = self.agent.get_dist_floor_motors_No2()

        # default distance front and hind tip = 0.278
        left_tip_collision = self.agent.get_dist_of_left_tips()
        right_tip_collision = self.agent.get_dist_of_right_tips()

        # default distance both front tip = 0.264
        front_tip_collision = self.agent.get_dist_of_front_tips()
        hind_tip_collision = self.agent.get_dist_of_hind_tips()

        # default distance motor2 and tip for each leg = 0.05
        motor2_tip_collision = self.agent.get_dist_motor2_tips()


        _collision_results = body_floor_collision + motor2_tip_collision + [left_tip_collision, right_tip_collision, front_tip_collision, hind_tip_collision]
        collision_results = [1 for _coll in _collision_results if _coll < collision_threshold]
        collision = sum(collision_results)
        # print(collision_results, collision)

        # -- slipping
        vel_threshold = 0.025
        min_dist = 0.01
        # check stadning leg and then that leg sloipping or not?
        slipping_results = [(self._slipping_detector(i[0], vel_threshold)) for i in enumerate(self.agent.get_dist_floor_tips()) if i[1] < min_dist]
        # slipping score (slipping_score = 1 if a leg slip or 4 if 4-leg slip)
        slipping_score = sum(slipping_results)
        # print(slipping_results, slipping_score)
        slipping = slipping_score

        # -- tip rotation during stand phase
        tip_rot_threshold = 0.52        # 0.52 rad or 30 deg
        # check stadning leg and then that leg sloipping or not?
        tip_rot_results = [(self._tip_rotate_detector(i[0], tip_rot_threshold)) for i in enumerate(self.agent.get_dist_floor_tips()) if i[1] < min_dist]
        tip_rot_score = sum(tip_rot_results)


        # -- average power
        # TODO ISSUE: I cannot access to joint torque!
        avgPower = 0

        # print(walking_dist, stability, collision, slipping, top_rot_score, avgPower)
        return 3*walking_dist - (stability + collision + slipping + tip_rot_score + avgPower)

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
        return self._get_reward(), self._get_state()

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
        return list(np.random.uniform(-5.4, 5.4, size=(19,)))

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
