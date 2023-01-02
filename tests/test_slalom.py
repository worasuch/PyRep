import unittest
from tests.core import TestCore
from pyrep.robots.legged_robots.slalom import Slalom

class TestSlalom(TestCore):

    def setUp(self):
        super().setUp()
        self.agent = Slalom()

    def test_read_leg_joint(self):
        leg_joints = self.agent.get_leg_joint_position()
        print(leg_joints)

    def test_read_body_joint(self):
        body_joints = self.agent.get_body_joint_position()
        print(body_joints)
    
    def test_read_leg_tip(self):
        leg_tip = self.agent.get_leg_tip_position()
        print(leg_tip)
    
    # def test_read_shape(self):
    #     tipColor = self.agent.get_tip_color()
    #     print(tipColor)
    
    def test_read_imu(self):
        imuData = self.agent.get_imu_data()
        print(imuData)

    """
    def test_read(self):
        accelerations = self.sensor.read()
        self.assertEqual(len(accelerations), 3)
    """

if __name__ == '__main__':
    unittest.main()
