import pybullet as p
import pybullet_data
import os
import time
import math
import numpy as np


file_name = "2R_Robotic_Arm.urdf"
p.connect(p.GUI)
p.setGravity(0,0,-9.8)
p.setRealTimeSimulation(0)
p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
orn = p.getQuaternionFromEuler([0, 0, 0])
robot = p.loadURDF(file_name, [0, 0, 0], orn)
p.createConstraint(parentBodyUniqueId=robot, parentLinkIndex=0, childBodyUniqueId=-1,       # Fixing the robot in place.
                   childLinkIndex=-1, jointType=p.JOINT_POINT2POINT, jointAxis=[1, 0, 0],
                   parentFramePosition=[0, 0, 0], childFramePosition=[0, 0, 0])


def Forward_Kinematics(joint_angles):
    theta_1=joint_angles[0]
    theta_2=joint_angles[1]    
    y = a1 * np.cos(np.radians(theta_1)) + a2 * np.cos(np.radians(theta_1 + theta_2))
    z = a1 * np.sin(np.radians(theta_1)) + a2 * np.sin(np.radians(theta_1 + theta_2))
    global fk
    fk=np.array([y, z])
    return fk


def ik_cost(desired_pos, thetas_guess):
  # return cost function: C(theta_vec) >>> Euclidean distance between desired (end_effector_pos) and
  #  guess at joint angles to achieve desired end-effector position
  start=Forward_Kinematics(thetas_guess)
  global cost
  cost=np.sqrt((desired_pos[0] - start[0]) ** 2 + (desired_pos[1] - start[1]) ** 2)
  return cost


def calculate_jacobian(joint_angles):
    theta_1=joint_angles[0]
    theta_2=joint_angles[1]
    global jacobian
    jacobian = np.array([[-a1 * np.sin(np.radians(theta_1)) - a2 * np.sin(np.radians(theta_1 + theta_2)),-a2 * np.sin(np.radians(theta_1 + theta_2))]
                         ,[a1 * np.cos(np.radians(theta_1)) + a2 * np.cos(np.radians(theta_1 + theta_2)),a2 * np.cos(np.radians(theta_1 + theta_2))]])
    return jacobian


def IK_Gradient_Descent(a_1, a_2, Yd, Zd, theta_1, theta_2, rate):
    global a1,a2
    a1=a_1
    a2=a_2
    joint_angles=np.array([theta_1,theta_2])
    desired_pos = np.array([Yd,Zd])
    iteration = 0
    cost_fun=ik_cost(desired_pos, joint_angles)
    while cost_fun > 0.0001:
      desired_pos = np.array([Yd,Zd])
      jacobian = calculate_jacobian(joint_angles)
      jacobian_trans = np.transpose(jacobian)
      grad_des_cost_fun = np.dot(jacobian_trans,(Forward_Kinematics(joint_angles) - desired_pos))
      alpha = rate
      joint_angles = joint_angles - (alpha * grad_des_cost_fun)
      cost_fun = ik_cost(desired_pos, joint_angles)
      iteration += 1
    #return iteration, desired_pos, cost_fun ,joint_angles
    return np.radians(joint_angles[0]), np.radians(joint_angles[1])


#Inverse kinematics function inputs:
link1=1
link2=1
y_desired = -0.5
z_desired = 1
theta1_init=0
theta2_init=0
learning_rate=0.5


use_custom = True    # Switch to True, to use custom Inverse_Kinematics function.
if use_custom:
    angle_1, angle_2 = IK_Gradient_Descent(link1,link2,y_desired, z_desired,theta1_init,theta2_init,learning_rate)
else:
    rotation = p.getQuaternionFromEuler([0, math.pi, 0]) # For rotated configuration of the arm
    angle_1, angle_2 = p.calculateInverseKinematics(robot, 2, [0, y_desired, z_desired], rotation)


#Ball visualization: a ball is positioned in the end effector desired cartesian position
# Define the ball's visual properties
ball_radius = 0.05  # Radius of the ball
ball_color = [1, 0.40, 0, 1]  # Red color (R, G, B)
# Create a visual shape for the ball
visual_shape_id = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=ball_radius, rgbaColor=ball_color)
# Create the ball at position (0, 1, 1)
ball_position = [0, y_desired, z_desired]
# Create the ball as a multi-body
ball_id = p.createMultiBody(baseVisualShapeIndex=visual_shape_id, basePosition=ball_position)


#move the URDF ROBOT ARM end effector to the desired cartesian position using joint angles calculated by the IK function for validation
p.setJointMotorControl2(bodyIndex=robot,
                        jointIndex=0,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angle_1,
                        force=2000)

p.setJointMotorControl2(bodyIndex=robot,
                        jointIndex=1,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=angle_2,
                        force=2000)


while True:
    p.stepSimulation()
    time.sleep(0.01)