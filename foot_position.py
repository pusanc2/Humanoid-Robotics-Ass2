#!/usr/bin/env python3
import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from robotis_mini import RobotisMini
from dynamic_reconfigure.server import Server
from scipy.signal import sawtooth
from robotis_mini_control.cfg import RobotisMiniConfig
import math


"""
Parameters for generating sine and triangle wave inputs
TODO: Find amplitude values for Q1.2 and Q1.3
    Change signal_type_ to switch signal type used in
    execute_variable_foot_position() function.
"""
start_time = 0 # sec
end_time = 30 # sec
control_period_ = 0.001 # ms
freq_n = 0.25 # hz
amplitude = 50 # mm

signal_type_ = 'sine' # sine or triangle

def callback(config, level):
    """
    Dynamic reconfigure callback used for execute_static_foot_position().
    The robot will move to the specified position in sim when user changes 
    x_foot_pos, y_foot_pos, or z_foot_pos param in rqt_dynamic_reconfigure GUI.
    """
    x_foot_pos = round(config['x_foot_pos'] / 5) * 5  # Round to nearest 5 mm
    y_foot_pos = round(config['y_foot_pos'] / 5) * 5  # Round to nearest 5 mm
    z_foot_pos = config['z_foot_pos']

    execute_static_foot_position(robot, x_foot_pos=x_foot_pos, 
                                        y_foot_pos=y_foot_pos, 
                                        z_foot_pos=z_foot_pos, 
                                        time = 1.0)

    print("x_foot_pos: ", x_foot_pos, 
        "\ny_foot_pos: ", y_foot_pos,
        "\nz_foot_pos: ", z_foot_pos,
        "\n--------------------")

    return config

def execute_static_foot_position(robot, x_foot_pos, y_foot_pos, z_foot_pos, time):
    z_foot_pos = -166

    joint_values_right_hand = [0, 0, 0]
    joint_values_left_hand = [0, 0, 0]
    joint_values_right_foot = robot.ik_right_foot(robot.x_RF0+x_foot_pos, robot.y_RF0+y_foot_pos, z_foot_pos, robot.roll_RF0, robot.pitch_RF0)
    joint_values_left_foot = robot.ik_left_foot(robot.x_LF0+x_foot_pos, robot.y_LF0+y_foot_pos, z_foot_pos, robot.roll_LF0, robot.pitch_LF0)

    joint_pos_values = joint_values_right_hand + joint_values_left_hand \
                        + joint_values_right_foot + joint_values_left_foot
    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = ['r_shoulder_joint','r_biceps_joint', 'r_elbow_joint', 
                            'l_shoulder_joint','l_biceps_joint', 'l_elbow_joint', 
                            'r_hip_joint', 'r_thigh_joint', 'r_knee_joint', 'r_ankle_joint', 'r_foot_joint',
                            'l_hip_joint', 'l_thigh_joint', 'l_knee_joint', 'l_ankle_joint', 'l_foot_joint']

    point = JointTrajectoryPoint()                  
    point.positions = joint_pos_values
    point.time_from_start = rospy.Duration(time)
    traj_msg.points.append(point)

    robot.execute_pub.publish(traj_msg)
    rospy.sleep(time)

def sine_wave_input():
    """
    TODO: Implement sine wave signal with a natural frequency (freq_n) 
            of 0.25hz sampled every 0.001 sec (control_period_).
    """
    global start_time
    global end_time 
    global control_period_
    global freq_n
    global amplitude

    time_samples = np.arange(start_time, end_time, control_period_)
    return amplitude * np.sin(2 * np.pi * freq_n * time_samples - np.pi/2)
    

def triangle_wave_input(): 
    """
    TODO: Implement triangle wave signal with a natural frequency (freq_n) 
            of 0.25hz sampled every 0.001 sec (control_period_).
    """
    global start_time
    global end_time 
    global control_period_
    global freq_n
    global amplitude

    time_samples = np.arange(start_time, end_time, control_period_)
    return amplitude * sawtooth(2 * np.pi * freq_n * time_samples, width=0.5) 


def execute_variable_foot_position(robot, z_foot_pos):
    global control_period_
    global signal_type_

    if signal_type_ == 'sine':
        wave_signal = sine_wave_input()
    elif signal_type_ == 'triangle':
        wave_signal = triangle_wave_input()
    else:
        rospy.logerr("Invalid signal type selected. Choose either 'sine' or 'triangle'.")
        return

    time_from_start = 0.0
    x_offset = 0
    y_offset = 0

    traj_msg = JointTrajectory()
    traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = ['r_shoulder_joint','r_biceps_joint', 'r_elbow_joint', 
                           'l_shoulder_joint','l_biceps_joint', 'l_elbow_joint', 
                           'r_hip_joint', 'r_thigh_joint', 'r_knee_joint', 'r_ankle_joint', 'r_foot_joint',
                           'l_hip_joint', 'l_thigh_joint', 'l_knee_joint', 'l_ankle_joint', 'l_foot_joint']
    all_joint_values_deg = []
    
    joint_names = ['r_shoulder_joint','r_biceps_joint', 'r_elbow_joint', 
                            'l_shoulder_joint','l_biceps_joint', 'l_elbow_joint', 
                            'r_hip_joint', 'r_thigh_joint', 'r_knee_joint', 'r_ankle_joint', 'r_foot_joint',
                            'l_hip_joint', 'l_thigh_joint', 'l_knee_joint', 'l_ankle_joint', 'l_foot_joint']
    for i in range(len(wave_signal)):  
        """
        TODO: Implement variable foot position input using the sine 
        and triangle wave inputs. Calc joint angles using ik and append 
        all waypoints to a single JointTrajectory msg.

        Reference execute_static_foot_position() code above for how to use the 
        inverse kinematics functions, construct JointTrajectory msg, and publish 
        the final trajectory to the controller.

        Hint: You need to increment point.time_from_start by control_period_ 
        for every point you add to the trajectory.
        
       pass
    """
        x_foot_pos = 0
        y_foot_pos = wave_signal[i]
        
        joint_values_right_hand = [0, 0, 0]
        joint_values_left_hand = [0, 0, 0]

        joint_values_right_foot = robot.ik_right_foot(robot.x_RF0+x_foot_pos, robot.y_RF0+y_foot_pos, z_foot_pos, robot.roll_RF0, robot.pitch_RF0)
        joint_values_left_foot = robot.ik_left_foot(robot.x_LF0+x_foot_pos, robot.y_LF0+y_foot_pos, z_foot_pos, robot.roll_LF0, robot.pitch_LF0)

        joint_pos_values = joint_values_right_hand + joint_values_left_hand  + joint_values_right_foot + joint_values_left_foot

        # print("Time: {:.3f} sec".format(time_from_start))
        # for j, name in enumerate(traj_msg.joint_names):
        #     print(f"  {name}: {joint_pos_values[j]:.6f}")
        # print("-------------------------------------------------------")
        
        joint_pos_values_deg = [math.degrees(rad) for rad in joint_pos_values]
        all_joint_values_deg.append(joint_pos_values_deg)

        # Create a trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_pos_values
        point.time_from_start = rospy.Duration(time_from_start)
        traj_msg.points.append(point)

        # Increment time
        time_from_start += control_period_

    robot.execute_pub.publish(traj_msg)

    rospy.loginfo("Now printing angles in degrees DURING motion.")
    for i, joint_deg_list in enumerate(all_joint_values_deg):
        rospy.loginfo(f"Step {i}:")
        for name, angle_deg in zip(joint_names, joint_deg_list):
            rospy.loginfo(f"  {name}: {angle_deg:.2f} deg")

        
if __name__ == '__main__':
    rospy.init_node('foot_position')

    robot = RobotisMini()
    # init dynamic reconfigure server 
    srv = Server(RobotisMiniConfig, callback)

    robot.init_pose(z_foot_pos = -166)
    # Get user input for signal type
    user_input = input("Enter signal type (sine/triangle): ").strip().lower()
    if user_input in ['sine', 'triangle']:
        signal_type_ = user_input
    else:
        rospy.logwarn("Invalid input. Defaulting to 'sine' wave.")
        signal_type_ = 'sine'



    # Uncomment below to test variable foot position
    execute_variable_foot_position(robot, z_foot_pos= -166)
    
    rospy.spin()