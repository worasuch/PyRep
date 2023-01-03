import unittest
from tests.core import TestCore
from pyrep.robots.legged_robots.slalom import Slalom

class TestSlalom(TestCore):

    def setUp(self):
        super().setUp()
        self.agent = Slalom()
        self.agent.set_leg_control_loop_enabled(False)
        self.agent.set_body_control_loop_enabled(False)

    def test_joints(self):
        # leg_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # self.agent.set_leg_joint_positions(leg_joints)
        # leg_joint_speed = self.agent.get_leg_joint_velocities()
        # leg_joint_speed = self.agent.get_leg_joint_forces()
        # print(leg_joint_speed)
        body_joints = [0.2, 0.2, 0.2]
        self.agent.set_body_joint_positions(body_joints)
        body = self.agent.get_body_joint_velocities()
        print(body)
    

    """
    def test_read_leg_joint(self):
        leg_joints = self.agent.get_leg_joint_positions()
        print(leg_joints)

    def test_set_leg_joint(self):
        leg_joints = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.agent.set_leg_joint_positions(leg_joints)
        print("Set Leg DONE!!")

    def test_set_body_joint(self):
        body_joints = [0.2, 0.2, 0.2]
        self.agent.set_body_joint_positions(body_joints)
        print("Set Body DONE!!")

    
    def test_read_body_joint(self):
        body_joints = self.agent.get_body_joint_positions()
        print(body_joints)
    
    def test_read_leg_tip(self):
        leg_tip = self.agent.get_leg_tip_position()
        print(leg_tip)
    
    def test_read_imu(self):
        imuData = self.agent.get_imu_data()
        print(imuData)
    """

    """
    def test_read(self):
        accelerations = self.sensor.read()
        self.assertEqual(len(accelerations), 3)
    """

if __name__ == '__main__':
    unittest.main()
