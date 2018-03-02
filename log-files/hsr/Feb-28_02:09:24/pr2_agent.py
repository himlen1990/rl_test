#!/usr/bin/env python


from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from gym_gazebo.srv import *
import actionlib
import rospy
import time
import tf
from tf import TransformListener
import threading
import numpy as np


class pr2_agent:
    def __init__(self, arm_name):
        #arm_name should be l_arm or r_arm
        rospy.init_node('pr2_agent')
        self.arm_name = arm_name
        self.jta = actionlib.SimpleActionClient('/'+arm_name+'_controller/joint_trajectory_action',
                                                JointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.loginfo('Found joint trajectory action!')
        self.listener = tf.TransformListener()

        self.lock = threading.Lock()
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()


        self.obs = None
        self.done = None
        self.reward_dist = None
        self.reward = None

        self.ee_target = np.array([0.8593751227275368, -0.4833588674883953, 0.48373306006651945]) 


        #thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)

        
    #callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        self.lock.acquire()
        self.name = msg.name
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort
        self.lock.release()

    def return_joint_state(self, joint_name):

        #no messages yet
        if self.name == []:
            rospy.logerr("No robot_state messages received!\n")
            return (0, 0., 0., 0.)

        #return info for this joint
        self.lock.acquire()
        if joint_name in self.name:
            index = self.name.index(joint_name)
            position = self.position[index]
            velocity = self.velocity[index]
            effort = self.effort[index]

        #unless it's not found
        else:
            rospy.logerr("Joint %s not found!", (joint_name,))
            self.lock.release()
            return (0, 0., 0., 0.)
        self.lock.release()
        return (1, position, velocity, effort)


    #server callback: returns arrays of position, velocity, and effort
    #for a list of joints specified by name
    def get_observations(self):
        joint_names = ["r_shoulder_pan_joint",
                       "r_shoulder_lift_joint",
                       "r_upper_arm_roll_joint",
                       "r_elbow_flex_joint",
                       "r_forearm_roll_joint",
                       "r_wrist_flex_joint",
                       "r_wrist_roll_joint"]

        joints_found = []
        positions = []
        velocities = []
        efforts = []
        for joint_name in joint_names:
            (found, position, velocity, effort) = self.return_joint_state(joint_name)
            joints_found.append(found)
            positions.append(position)
            velocities.append(velocity)
            efforts.append(effort)
        

        #print "!!!!!!!!!!!!!",positions
        self.ee_target = np.random.uniform(-1,1,3)
        #error = self.ee_target - self.get_ee_position()[0]
        current_position = self.get_ee_position()[0]
        #print "error",error
        return np.r_[np.reshape(positions,-1),np.reshape(self.ee_target,-1),np.reshape(current_position,-1)]


    def angle_normalize(self):#normalize angle to [-1,1]
        pass

    def move_arm(self, angles): #input: angles

        goal = JointTrajectoryGoal()
        char = self.arm_name[0] #either 'r' or 'l'
        goal.trajectory.joint_names = [char+'_shoulder_pan_joint',
                                       char+'_shoulder_lift_joint',
                                       char+'_upper_arm_roll_joint',
                                       char+'_elbow_flex_joint',
                                       char+'_forearm_roll_joint',
                                       char+'_wrist_flex_joint',
                                       char+'_wrist_roll_joint']
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(3)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)
        

    def get_ee_position(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform('/base_link','/r_gripper_tool_frame', rospy.Time(0))

                #angular = 4 * math.atan2(trans[1], trans[0])
                #linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
                return trans,rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep(0.1)

    
    def reset(self):
        self.move_arm([0.0]*7)
        self.obs = self.get_observations()
        return self.obs
    
    def step(self,action):
        #print "before"
        #print self.get_ee_position()
        self.current_pose = self.get_observations()[0:7]
        #print "before moving arm"
        #print self.current_pose
        self.processed_action = np.array(action) + self.current_pose
        #print "processed action"
        #print self.processed_action
        self.move_arm(self.processed_action)        
        self.obs = self.get_observations()

        #error = self.obs[7:10]
        error = self.obs[7:10] - self.obs[10:13]
        print error
        self.reward_dist = self.rmse_func(error)
        self.reward = 1 - self.reward_dist
        done = bool(abs(self.reward_dist) < 0.02)
        #print "done?", done
        #print self.obs[0:7]

        obs_np = np.array(self.obs)
        return obs_np, self.reward, done, {}
        #print "after"
        #print self.get_ee_position()
        
        

    def rmse_func(self, ee_points):
        rmse = np.sqrt(np.mean(np.square(ee_points), dtype=np.float32))
        return rmse
'''
if __name__ == '__main__':

    arm = pr2_agent('r_arm')
    print "waiting for queries"
    arm.step([0.2]*7)
    print("done2")
    arm.step([-0.2]*7)
    print("done3")
    #arm.reset()
    #obs = arm.get_observations()
    #print "obs", obs

    #rospy.spin()
'''
