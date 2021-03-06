#!/usr/bin/env python

#from pr2_controllers_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal
#from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
#from gym_gazebo.srv import *
import actionlib
import control_msgs.msg
import controller_manager_msgs.srv
import trajectory_msgs.msg
import rospy
import time
import tf
from tf import TransformListener
import threading
import numpy as np


class hsr_agent:
    def __init__(self):
        #arm_name should be l_arm or r_arm
        rospy.init_node('hsr_agent')

        self.jta = actionlib.SimpleActionClient('/hsrb/head_trajectory_controller/follow_joint_trajectory', control_msgs.msg.FollowJointTrajectoryAction)                                                
        rospy.loginfo('Waiting for joint trajectory action')
        self.jta.wait_for_server()
        rospy.wait_for_service('/hsrb/controller_manager/list_controllers')
        list_controllers = rospy.ServiceProxy(
            '/hsrb/controller_manager/list_controllers',
            controller_manager_msgs.srv.ListControllers)
        running = False
        while running is False:
            rospy.sleep(0.1)
            for c in list_controllers().controller:
                if c.name == 'head_trajectory_controller' and c.state == 'running':
                    running = True
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

        self.ee_target = np.array([0.4910034377104193, 0.11953546509968205, 0.22147284562751313]) 

        #thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('/hsrb/joint_states', JointState, self.joint_states_callback)

        
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
        while self.name == []:
            rospy.logerr("No robot_state messages received!\n")
            rospy.sleep(0.3)
            #return (0, 0., 0., 0.)

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
        #joint_names = ["arm_flex_joint",
        #               "arm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        joint_names = ["head_pan_joint",
                       "head_tilt_joint"]

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
        
        error = self.ee_target - self.get_ee_position()[0]
        #print positions
        #print "error",error
        return np.r_[np.reshape(positions,-1),np.reshape(error,-1)]

    def move(self, angles): #input: angles
        print "start moving arm"
        # fill ROS message                         
        goal = control_msgs.msg.FollowJointTrajectoryGoal()
        #traj = trajectory_msgs.msg.JointTrajectory()
        goal.trajectory.joint_names = ["head_pan_joint", "head_tilt_joint"]

        p = trajectory_msgs.msg.JointTrajectoryPoint()
        p.positions = angles
        print "move to ", angles
        #p.velocities = [0, 0, 0, 0, 0]        
        p.time_from_start = rospy.Duration(2)
        #traj.points = [p]
        #goal.trajectory = traj
        goal.trajectory.points.append(p)
        # send message to the action server                                                          
        self.jta.send_goal(goal)
        # wait for the action server to complete the order                                           
        #rospy.sleep(3)
        print self.jta.wait_for_result()
        
        print "finish moving arm"        

    def get_ee_position(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans,rot) = self.listener.lookupTransform('/base_link','/hand_palm_link', rospy.Time(0))

                #angular = 4 * math.atan2(trans[1], trans[0])
                #linear = 0.5 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
                return trans,rot
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep(0.1)

    def reset(self):
        self.move([0,0])    
        self.obs = self.get_observations()
        rospy.sleep(10)
        print "reset"
        return self.obs
    
    def step(self,action):
        #print "before"
        #print self.get_ee_position()
        self.move_arm(action)
        self.obs = self.get_observations()
        error = self.obs[4:7]
        self.reward_dist = self.rmse_func(error)
        self.reward = 1 - self.reward_dist
        done = bool(abs(self.reward_dist) < 0.02)
        print "done?", done
        return self.obs, self.reward, done, {}
        #print "after"
        #print self.get_ee_position()
        
        

    def rmse_func(self, ee_points):
        rmse = np.sqrt(np.mean(np.square(ee_points), dtype=np.float32))
        return rmse

if __name__ == '__main__':
  
    robot = hsr_agent()
    robot.move[0.5,0.5]

    robot.reset()
