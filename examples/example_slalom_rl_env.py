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
from pyrep.backend import sim
from pyrep import PyRep
# from pyrep.robots.arms.panda import Panda
from pyrep.objects.shape import Shape
from pyrep.robots.legged_robots.slalom import Slalom
import numpy as np
import math
# import time

SCENE_FILE = join(dirname(abspath(__file__)),
                  'scene_slalom_fixedbody_rl_terrainRand.ttt')
# POS_MIN, POS_MAX = [0.8, -0.2, 1.0], [1.0, 0.2, 1.4]
EPISODES = 3 #500
EPISODE_LENGTH = 500


class SlalomEnv(object):

    def __init__(self):
        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.terrain_rand()
        self.pr.start()
        # time.sleep(0.5)
        self.agent = Slalom()
        self.agent.set_leg_control_loop_enabled(False)
        # self.agent.set_body_control_loop_enabled(False)

        self.initial_position = self.agent.get_imu_positions()
        self.initial_orientation = self.agent.get_imu_orientations()
        self.initial_leg_joint_positions = self.agent.get_leg_joint_positions()
        # self.initial_body_joint_positions = self.agent.get_body_joint_positions()

    def _get_state(self):
        # print(f"body rot: {self.agent.get_body_orientations()} body pos: {self.agent.get_body_positions()}")
        # Return state containing arm joint angles/velocities & target position
        return np.concatenate([
                                self.agent.get_leg_joint_positions(),
                                self.agent.get_leg_joint_velocities(),
                                # self.agent.get_leg_joint_forces()
                                # self.agent.get_body_joint_positions(),
                                # self.agent.get_body_joint_velocities(),
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

        # add force when tip rotation is in the right orientation
        if abs_atip_roll <= _tip_rot_threshould and abs_atip_pitch <= _tip_rot_threshould:
            self.agent.add_force_atip(_stand_one)
            # print("add force", _stand_one)
            self.agent.set_color_atip(_stand_one, [1.0,0.0,0.0])
        else:
            self.agent.set_color_atip(_stand_one, [0.0,1.0,0.0])

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
        min_dist = 0.005
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
        # self.agent.set_body_joint_target_positions(self.initial_body_joint_positions)
        return self._get_state()

    def step(self, action):
        self.agent.set_leg_joint_target_positions(action[:16])          # Execute action on leg
        # self.agent.set_body_joint_target_positions(action[16:19])       # Execute action on body
        self.pr.step()                                                  # Step the physics simulation
        # self.agent.add_force_tips()
        # force = self.agent.get_force_sensors()
        # print(force)
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

    """
    Terrain Randomization
    """
    def terrain_rand(self):

        config = {'geom': [0.03, 0.4, 0.08, 
                           0.0, 0.0, 0.0, 
                           np.random.uniform(0.0, 0.2), np.random.uniform(0.0, 0.2), 0.0,
                           np.random.uniform(0.0, 0.5), np.random.uniform(0.0, 0.7), 0.0], 
                  'generateHf': True, 
                  'textured': True, 
                  'color': [0.25, 0.25, 0.25]}

        # 'geom': [0.034, 0.401, 0.088, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.8, 0.8, 0.0],

        """
        potions: bit-coded options:
        bit 0 set (1): back faces are culled
        bit 1 set (2): overlay mesh is visible
        bit 2 set (4): a simple shape is generated instead of a heightfield
        bit 3 set (8): the heightfield is not respondable
        """
        options=4
        if config['generateHf']:
            options=0

        xs=config['geom'][0]        # size
        ys=config['geom'][1]        # x/y proportion
        zs=config['geom'][2]        # height

        c1fs=config['geom'][3]      # ch1 ferq
        c1as=config['geom'][4]      # ch1 amp
        c1ps=config['geom'][5]      # ch1 phase
        
        c2fs=config['geom'][6]      # ch2 ferq
        c2as=config['geom'][7]      # ch2 amp
        c2ps=config['geom'][8]      # ch2 phase

        c3fs=config['geom'][9]      # ch3 ferq
        c3as=config['geom'][10]     # ch3 amp
        c3ps=config['geom'][11]     # ch3 phase

        dxs=5+195*xs
        dzs=0+20*zs
        
        objName = "shape"
        objProperty = 160           # what is this number mean?
        objSpecialProperty = 1008   # what is this number mean?


        # -- Hills -- #
        heights = []
        step_heights =  []
        stair_heights = []
        if (ys>0.5):
            yt=64
            xt=6+math.floor(29*(1-ys)*2)*2
        else:
            xt=64
            yt=6+math.floor(29*ys*2)*2

        minw=9000
        maxw=0
        for y in range(yt):
            y_=(y+1)/yt
            for x in range(xt):
                x_=(x+1)/xt
                channel1=(math.sin(c1ps*math.pi*2+c1fs*(x_)*20)+math.sin(c1ps*math.pi*2+c1fs*(y_)*20))*c1as*0.5
                channel2=(math.sin(c2ps*math.pi*2+c2fs*(x_)*60)+math.sin(c2ps*math.pi*2+c2fs*(y_)*60))*c2as*0.5
                channel3=(math.sin(c3ps*math.pi*2+c3fs*(x_)*150)+math.sin(c3ps*math.pi*2+c3fs*(y_)*150))*c3as*0.5
                xp=((x+1)-xt/2)/(xt/2)                
                yp=((y+1)-yt/2)/(yt/2)                
                r=1*math.sqrt(xp*xp+yp*yp)
                nh=dzs/((r*r)+1)
                if (nh<minw): minw=nh
                if (nh>maxw): maxw=nh
                nh=(channel1+channel2*0.5+channel3*0.1)*dzs
                heights.append(nh)

        # -- Steps -- #
        """
        previous_j = 0
        for i in range(len(step_heights)):
            if i > 10:
                step_min = min(step_heights[i-10:i+10])
                step_max = max(step_heights[i-10:i+10])
            else:
                step_min = min(step_heights[i:i+10])
                step_max = max(step_heights[i:i+10])

            j = step_heights[i]
            if j > previous_j:
                step_heights[i] = step_max
            else:
                step_heights[i] = step_min
            previous_j = j
        """
        
        # -- Stairs -- #
        step_w = 3          # Not dimention
        step_h = np.random.uniform(0.01, 0.03) #0.03
        for i in range(yt):
            stair_step = -0.4
            for j in range(xt):
                if j%step_w == 0 and j > math.floor(xt/2)+1:
                    stair_step += step_h 
                stair_heights.append(stair_step)

        # terrainShape=sim.simCreateHeightfieldShape(options,0.0,xt,yt,dxs,heights)
        terrainShape=sim.simCreateHeightfieldShape(options,0.0,xt,yt,dxs,stair_heights)
        sim.simSetObjectName(terrainShape,objName)
        sim.simSetModelProperty(terrainShape,objProperty)
        sim.simSetObjectSpecialProperty(terrainShape,objSpecialProperty)
        if config['generateHf']:
            sim.simSetShapeColor(terrainShape,'', 0, config['color'])
        else:
            sim.simSetShapeColor(terrainShape,'', sim.colorcomponent_ambient_diffuse, config['color'])

        """
        terrainShape=sim.createHeightfieldShape(options,0.0,xt,yt,dxs,heights)
        sim.setObjectAlias(terrainShape,objName)
        sim.setObjectProperty(terrainShape,objProperty)
        sim.setObjectSpecialProperty(terrainShape,objSpecialProperty)
        #sim.setObjectParent(terrainShape,h,False)
        if config['generateHf']:
            sim.setShapeColor(terrainShape,'',0,[0.5, 0.5, 0.5])
        else:
            sim.setShapeColor(terrainShape,'', sim.colorcomponent_ambient_diffuse,config['color'])
        """

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
