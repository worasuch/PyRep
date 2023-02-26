import unittest
import pyrep
from pyrep import PyRep
from os import path

# ASSET_DIR = path.join(path.dirname(path.abspath(__file__)), 'assets')
ASSET_DIR = path.join(path.dirname(path.abspath(__file__)), '/home/slalom/PyRep/tests/assets')
pyrep.testing = True


class TestCore(unittest.TestCase):

    def setUp(self):
        self.pyrep = PyRep()
        self.pyrep.launch(path.join(ASSET_DIR, 'scene_slalom_fixedbody_rl_env.ttt'), headless=False)
        self.pyrep.step()
        self.pyrep.start()

    def tearDown(self):
        self.pyrep.stop()
        self.pyrep.step_ui()
        self.pyrep.shutdown()
