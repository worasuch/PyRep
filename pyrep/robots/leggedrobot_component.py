import numpy as np

from typing import List, Tuple

from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.force_sensor import ForceSensor

from pyrep.backend import sim, utils
from pyrep.const import JointType
from pyrep.objects.object import Object
from pyrep.objects.joint import Joint
from pyrep.const import ObjectType, JointMode


class LeggedRobotComponent(Object):
    def __init__(self):
        super().__init__("/geckobotiv")

        # -- leg joints handle -- #
        leg_joint_name = [  
                        "/joint1_lf", "/joint2_lf", "/joint3_lf", "/joint4_lf",
                        "/joint1_lh", "/joint2_lh", "/joint3_lh", "/joint4_lh",
                        "/joint1_rh", "/joint2_rh", "/joint3_rh", "/joint4_rh",
                        "/joint1_rf", "/joint2_rf", "/joint3_rf", "/joint4_rf",
                        ]
        self.leg_joints = [Joint(_leg_joint_name)
                            for _leg_joint_name in leg_joint_name]
        self._leg_joints_handles = [_leg_joints.get_handle() for _leg_joints in self.leg_joints]
        
        """
        # leg ball joints handle
        leg_ball_joint_name = [
                            "/Leg4_ball1", "/Leg4_ball2",
                            "/Leg3_ball1", "/Leg3_ball2",
                            "/Leg2_ball1", "/Leg2_ball2",
                            "/Leg1_ball1", "/Leg1_ball2",
                            ]
        self.leg_ball_joints = [Joint(_leg_ball_joint_name)
                                for _leg_ball_joint_name in leg_ball_joint_name]
        self._leg_ball_joints_handles = [_leg_ball_joints.get_handle() for _leg_ball_joints in self.leg_ball_joints]
        """

        # -- body joints handle -- #
        body_joint_name = [
                        "/joint_b1", "/joint_b2", "/joint_b3"
                        ]
        self.body_joints = [Joint(_body_joint_name)
                            for _body_joint_name in body_joint_name]
        self._body_joints_handles = [_body_joints.get_handle() for _body_joints in self.body_joints]


        # -- pads handle -- #
        leg_tip_name = [
                    "/pad_lf", "/pad_lh", "/pad_rh", "/pad_rf"
                    ]
        self.leg_tip = [Shape(_leg_tip_name)
                        for _leg_tip_name in leg_tip_name]
        self._leg_tip_handles = [_leg_tip.get_handle() for _leg_tip in self.leg_tip]


        # -- motor2 handle -- #
        motor2_name = [
                    "/motor2_lf", "/motor2_lh", "/motor2_rh", "/motor2_rf"
                    ]
        self.motor2 = [Shape(_motor2_name)
                        for _motor2_name in motor2_name]
        self._motor2_handles = [_motor2.get_handle() for _motor2 in self.motor2]

        # -- force sensors  -- #
        force_sensors_name = [
                            "/fc_lf", "/fc_lh", "/fc_rh", "/fc_rf"
                            ]
        self.force_sensors = [ForceSensor(_force_sensors_name)
                                for _force_sensors_name in force_sensors_name]
        self._force_sensors_handles = [_force_sensors.get_handle() for _force_sensors in self.force_sensors]


        # -- others handle -- #
        self.imu = Dummy("/Imu")
        self._imu = self.imu.get_handle()
        self.floor = Shape("/floor")
        self._floor = self.floor.get_handle() 


    def _get_requested_type(self) -> ObjectType:
        """Gets the type of the object.

        :return: Type of the object.
        """
        return ObjectType(sim.simGetObjectType(self.get_handle()))




    # --- Leg Joints Function --- #
    # --------------------------- #

    def get_leg_joint_positions(self) -> List[float]:
        """Retrieves the intrinsic position of the joints.

        See :py:meth:`Joint.get_joint_position` for more information.

        :return: A list of intrinsic position of the joints.
        """
        return [_leg_joints.get_joint_position() for _leg_joints in self.leg_joints]
    
    def set_leg_joint_positions(self, positions: List[float],
                                disable_dynamics: bool = False) -> None:
        """Sets the intrinsic position of the joints.

        See :py:meth:`Joint.set_joint_position` for more information.

        :param disable_dynamics: If True, then the position can be set even
            when the joint mode is in Force mode. It will disable dynamics,
            move the joint, and then re-enable dynamics.

        :param positions: A list of positions of the joints (angular or linear
            values depending on the joint type).
        """
        if not disable_dynamics:
            [sim.simSetJointPosition(jh, p)     # type: ignore
            for jh, p in zip(self._leg_joints_handles, positions)]
            return
    
        is_model = self.is_model()
        if not is_model:
            self.set_model(True)
        
        prior = sim.simGetModelProperty(self.get_handle())
        p = prior | sim.sim_modelproperty_not_dynamic
        # Disable the dynamics
        sim.simSetModelProperty(self._handle, p)
        with utils.step_lock:
            sim.simExtStep(True)                # Have to step for change to take effect

        [sim.simSetJointPosition(jh, p)         # type: ignore
         for jh, p in zip(self._leg_joints_handles, positions)]
        [j.set_joint_target_position(p)         # type: ignore
         for j, p in zip(self.leg_joints, positions)]
        with utils.step_lock:
            sim.simExtStep(True)        # Have to step for change to take effect
        # Re-enable the dynamics
        sim.simSetModelProperty(self._handle, prior)
        self.set_model(is_model)
    
    def set_leg_joint_target_positions(self, positions: List[float]) -> None:
        """Sets the target positions of the joints.

        See :py:meth:`Joint.set_joint_target_position` for more information.
            get_orientationn of the joints (angular or
            linear values depending on the joint type).
        """
        [_leg_joints.set_joint_target_position(p)
         for _leg_joints, p in zip(self.leg_joints, positions)]

    # TODO NOT Work!!
    def get_leg_joint_forces(self) -> List[float]:
        """Retrieves the forces or torques of the joints.

        See :py:meth:`Joint.get_joint_force` for more information.

        :return: A list of the forces or the torques applied to the joints
            along/about their z-axis.
        """        
        return [_leg_joints.get_joint_force() for _leg_joints in self.leg_joints]

    def get_leg_joint_velocities(self) -> List[float]:
        """Get the current joint velocities.

        :return: List containing the velocities of the joints (linear or
            angular velocities depending on the joint-type).
        """
        return [_leg_joints.get_joint_velocity() for _leg_joints in self.leg_joints]
    
    def set_leg_control_loop_enabled(self, value: bool) -> None:
        """Sets whether the control loop is enable for all joints.

        :param value: The new value for the control loop state.
        """
        [_leg_joints.set_control_loop_enabled(value)        # type: ignore
         for _leg_joints in self.leg_joints]




    # --- Body Joints Function --- #
    # ---------------------------- #

    def get_body_joint_positions(self) -> List[float]:
        return [_body_joints.get_joint_position() for _body_joints in self.body_joints]

    def set_body_joint_positions(self, position: List[float],
                                disable_dynamics: bool = False) -> None:

        if not disable_dynamics:
            [sim.simSetJointPosition(jh, p)     # type: ignore
            for jh, p in zip(self._body_joints_handles, position)]
            return
        
        is_model = self.is_model()
        if not is_model:
            self.set_model(True)

        prior = sim.simGetModelProperty(self.get_handle())
        p = prior | sim.sim_modelproperty_not_dynamic
        # Disable the dynamics
        sim.simSetModelProperty(self._handle, p)
        with utils.step_lock:
            sim.simExtStep(True)            # Have to step for changes to take effect

        [sim.simSetJointPosition(jh, p)     # type: ignore 
         for jh, p in zip(self._body_joints_handles, position)]
        [j.set_joint_traget_position(p)     # type: ignore
         for j, p in zip(self.body_joints, position)]
        with utils.step_lock:
            sim.simExtStep(True)            # Have to step for change to take effect
        # Re-enable the dynamics
        sim.simSetModelProperty(self._handle, prior)
        self.set_model(is_model)

    def set_body_joint_target_positions(self, positions: List[float]) -> None:
        [_body_joints.set_joint_target_position(p)
         for _body_joints, p in zip(self.body_joints, positions)]

    # TODO NOT Work!!
    def get_body_joint_forces(self) -> List[float]:     
        return [_body_joints.get_joint_force() for _body_joints in self.body_joints]

    def get_body_joint_velocities(self) -> List[float]:
        return [_body_joints.get_joint_velocity() for _body_joints in self.body_joints]

    def set_body_control_loop_enabled(self, value: bool) -> None:
        [_body_joints.set_control_loop_enabled(value)        # type: ignore
         for _body_joints in self.body_joints]





    # --- Tip Joints Function --- #
    # --------------------------- #
    
    # leg tip position
    def get_leg_tip_position(self) -> List[float]:
        return [_leg_tip.get_position() for _leg_tip in self.leg_tip]

    def get_leg_tip_velocities(self) -> List[float]:
        return [_leg_tip.get_velocity() for _leg_tip in self.leg_tip]
    
    def get_leg_atip_velocity(self, tip: int) -> float:
        return self.leg_tip[tip].get_velocity()

    def get_leg_tip_orientation(self) -> List[float]:
        return [_leg_tip.get_orientation() for _leg_tip in self.leg_tip]
    
    def get_leg_atip_orientation(self, tip: int) -> float:
        return self.leg_tip[tip].get_orientation()


    # distance between floor and tips
    def get_dist_floor_tips(self) -> List[float]:
        return [_leg_tip.check_distance(self.floor)
                for _leg_tip in self.leg_tip]
    
    # distance between floor and "A tip"
    def get_dist_floor_atip(self, tip: int) -> float:
        return self.leg_tip[tip].check_distance(self.floor)

    # add force to all tip
    def add_force_tips(self) -> None:
        force_position = np.array([0,0,0])      # add force to center of pad
        force_to_apply = np.array([0,0,-5])     # 5 newton to -z
        reset_force_torque = True
        for _leg_tip in self.leg_tip:
            _leg_tip.add_force(force_position,force_to_apply,reset_force_torque)

    # add force to "A tip"
    def add_force_atip(self, tip: int) -> None:
        force_position = np.array([0,0,0])
        force_to_apply = np.array([0,0,-5])
        reset_force_torque = True
        self.leg_tip[tip].add_force(force_position,force_to_apply,reset_force_torque)

    # set color "A tip"
    def set_color_atip(self, tip: int, _colors: List[float]) -> None:
        self.leg_tip[tip].set_color(_colors) 



    # --- Sensor/Measurment Function --- #
    # ---------------------------------- #

    # imu data
    def get_imu_positions(self) -> np.ndarray:
        return self.imu.get_position()

    def get_imu_orientations(self) -> np.ndarray:
        return self.imu.get_orientation()

    # distance betweet front and hind tip in the same side 
    def get_dist_of_left_tips(self) -> float:
        return self.leg_tip[0].check_distance(self.leg_tip[1])      # distance between tip of LF and LH

    def get_dist_of_right_tips(self) -> float:
        return self.leg_tip[2].check_distance(self.leg_tip[3])      # distance between tip of RH and RF

    # distance betweet tip in front or hind
    def get_dist_of_front_tips(self) -> float:
        return self.leg_tip[0].check_distance(self.leg_tip[3])      # distance between tip of LF and RF

    def get_dist_of_hind_tips(self) -> float:
        return self.leg_tip[1].check_distance(self.leg_tip[2])      # distance between tip of LH and RH

    # distance between floor and motor2
    def get_dist_floor_motors_No2(self) -> List[float]:             # distance between floor and motor2 of each leg
        return[_motor2.check_distance(self.floor)
               for _motor2 in self.motor2]

    # distance between motor2 and tip for each leg
    def get_dist_motor2_tips(self) -> List[float]:
        return [self.leg_tip[i].check_distance(self.motor2[i])      # distance between motor2 and tip for ALL
                for i, _ in enumerate(self.leg_tip)]

    def get_dist_motor2_tip_lf(self) -> float:
        return self.leg_tip[0].check_distance(self.motor2[0])       # distance between motor2 and tip of LF

    def get_dist_motor2_tip_lh(self) -> float:
        return self.leg_tip[1].check_distance(self.motor2[1])       # distance between motor2 and tip of LH

    def get_dist_motor2_tip_rh(self) -> float:
        return self.leg_tip[2].check_distance(self.motor2[2])       # distance between motor2 and tip of RH

    def get_dist_motor2_tip_rf(self) -> float:
        return self.leg_tip[3].check_distance(self.motor2[3])       # distance between motor2 and tip of RF


    def get_force_sensors(self) -> List[float]:
        return [self.force_sensors[i].read()      # distance between motor2 and tip for ALL
                for i, _ in enumerate(self.force_sensors)]
