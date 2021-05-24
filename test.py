#!/usr/bin/env python
import pybullet as p

""" To view joints """
# p.connect(p.DIRECT)
#
# car = p.loadURDF("models/uav_fluid.urdf")
# number_of_joints = p.getNumJoints(car)
# for joint_num in range(number_of_joints):
#     info = p.getJointInfo(car, joint_num)
#     # print(info)
#     print(info[0], ": ", info[1])

# """ To load GUI """
from time import sleep

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

angle = p.addUserDebugParameter('Angle', -0.349, 0.785, 0)
throttle = p.addUserDebugParameter('Throttle', 0, 20, 0)
p.loadURDF("models/ground.urdf", [0, 0, 0.0])
car = p.loadURDF("models/uav_fluid.urdf", [0, 0, 0.1])

sleep(1)

# wheel_indices = [1, 3, 4, 5]
# hinge_indices = [0, 2]
wheel_indices = [0]
hinge_indices = [0]

while True:
    user_angle = p.readUserDebugParameter(angle)
    user_throttle = p.readUserDebugParameter(throttle)
    for joint_index in wheel_indices:
        print(joint_index)
        p.setJointMotorControl2(car, joint_index,
                                p.POSITION_CONTROL,
                                targetVelocity=user_throttle)
    for joint_index in hinge_indices:
        p.setJointMotorControl2(car, joint_index,
                                p.POSITION_CONTROL,
                                targetPosition=user_angle)
    p.stepSimulation()
