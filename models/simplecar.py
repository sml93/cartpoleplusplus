#!/usr/bin/env python
import pybullet as p
from math import pi
from time import sleep


class UAV:
    """ class for UAVwFluid """
    def __init__(self, client):
        self.client = client
        self.uav = p.loadURDF("uav_fluid.urdf",
                              basePosition=[0, 0, 0.15],
                              baseOrientation=[0, 0, 0, 1],
                              physicsClientId=client)

        # Joint indx as found by p.getJointInfo()
        self.joint = [0]
        self.joint_spd = 0
        self.c_drag = 0.01
        self.angle = 0

    def get_ids(self):
        return self.car, self.client

    def apply_angle(self, ang):
        """ Expects action to be 90 - tilt angle of the fluid (beta) """
        ang -= (pi/2)
        self.angle = max(min(ang, -0.349), 0.785)
        p.setJointMotorControlArray(self.uav, self.joint,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPositions=[self.angle]*2,
                                    physicsClientID=self.client)

        ''' Calculate drag / mechanical resistance '''
        friction = -(self.joint_spd**2) * self.c_drag
        if self.joint_spd < 0:
            self.joint_spd = 0

    def get_observation(self):
        (x, y, z), orient = p.getBasePositionAndOrientation(self.uav)
        ox, oy, oz = p.getEulerFromQuaternion(orient)  # Row / Pitch / Yaw
        return x, y, z, ox, oy, oz



# """ To view joints """
# p.connect(p.DIRECT)
#
# obj = p.loadURDF("uav_fluid.urdf")
# number_of_joints = p.getNumJoints(obj)
# for joint_num in range(number_of_joints):
#     info = p.getJointInfo(obj, joint_num)
#     # print(info)
#     print(info[0], ": ", info[1])

# """ To load GUI """
# p.connect(p.GUI)
# p.setGravity(0, 0, -9.81)
#
# angle = p.addUserDebugParameter('Angle', -0.5, 0.5, 0)
# throttle = p.addUserDebugParameter('Throttle', 0, 20, 0)
# p.loadURDF("ground.urdf", [0, 0, 0.0])
# car = p.loadURDF("uav_fluid.urdf", [0, 0, 0.1])
#
# sleep(1)
#
# # wheel_indices = [1, 3, 4, 5]
# # hinge_indices = [0, 2]
# wheel_indices = [0]
# hinge_indices = [0]
#
# while True:
#     user_angle = p.readUserDebugParameter(angle)
#     user_throttle = p.readUserDebugParameter(throttle)
#     for joint_index in wheel_indices:
#         p.setJointMotorControl2(car, joint_index,
#                                 p.VELOCITY_CONTROL,
#                                 targetVelocity=user_throttle)
#     for joint_index in hinge_indices:
#         p.setJointMotorControl2(car, joint_index,
#                                 p.POSITION_CONTROL,
#                                 targetPosition=user_angle)
#     p.stepSimulation()
