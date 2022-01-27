#!/usr/bin/env python


from __future__ import print_function
import rospy
import actionlib
import signal
import sys
import os
import numpy as np
from cartesian_control_msgs.msg import FollowCartesianTrajectoryAction, FollowCartesianTrajectoryGoal, CartesianTrajectoryPoint
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import PyKDL
import tf
from math             import radians

class Client(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            '/arm_cartesian_controller/follow_cartesian_trajectory',
            FollowCartesianTrajectoryAction)
        self.client.wait_for_server()

        # Suppress spam output of urdf parsing.
        # urdf_parser_py is unhappy with various visual tags in the robot_description.
        tmp = sys.stderr
        sys.stderr = open(os.devnull, 'w')
        robot = URDF.from_parameter_server()
        sys.stderr = tmp
        self._listener   = tf.TransformListener()

        _, tree = treeFromUrdfModel(robot) # tree:KDL Tree returns
        #
        self.chain = tree.getChain('arm_base_link', 'arm_link6')
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(tree.getChain('arm_base_link', 'arm_link6'))
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

        self.xx = 0
        self.yy = 0
        self.zz = 0

    def test(self):
        def inverse_point():
            #  inverse kinematic
            p_kdl = PyKDL.Frame()
            joints = PyKDL.JntArray(6)
            # joint <- angle
            for i in range(6):
                #numpy.random.random_sample(): 0.0 <= && < 0.1
                #joints[i] = (np.random.random_sample() * 2 - 1) * np.pi
                joints[i] = 0.0
            q_init=joints #initial angles
            x = self.xx
            y = self.yy
            z = self.zz
            vik = PyKDL.ChainIkSolverVel_pinv(self.chain) #inverse kinematics velocity
            #ik=PyKDL.ChainIkSolverPos_NR(self.chain, self.fk_solver, vik)
            ik=PyKDL.ChainIkSolverPos_NR(self.chain, self.fk_solver, vik,100,1e-3)

            desiredFrame=PyKDL.Frame(PyKDL.Vector(x,y,z))
            print ("Desired Position: ")
            print (desiredFrame.p)

            q_out=PyKDL.JntArray(6)
            ik.CartToJnt(q_init,desiredFrame,q_out)
            q_out[0] = q_out[0] % (np.pi * 2)
            q_out[1] = q_out[1] % (np.pi * 2)
            q_out[2] = q_out[2] % (np.pi * 2)
            q_out[3] = q_out[3] % (np.pi * 2)
            q_out[4] = q_out[4] % (np.pi * 2)
            q_out[5] = q_out[5] % (np.pi * 2)
            print ("Output angles in rads: ")
            print (q_out)

            self.fk_solver.JntToCart(q_out, p_kdl)

            print ("last x,y,z: ")
            print (p_kdl.p)

            p = CartesianTrajectoryPoint()
            p.pose.position.x = p_kdl.p[0]
            p.pose.position.y = p_kdl.p[1]
            p.pose.position.z = p_kdl.p[2]

            q = PyKDL.Rotation.GetQuaternion(p_kdl.M)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            print ("Quaternion: ")
            print (p.pose.orientation)
            return p

        def now_link6():
            try:
                self._listener.waitForTransform("arm_base_link", "arm_link6", rospy.Time(0), rospy.Duration(1.0))
                (now_position, now_quaternion) = self._listener.lookupTransform("arm_base_link", "arm_link6", rospy.Time(0))
                print ("now_position: ")
                print (now_position)
                self.xx = now_position[0]
                self.yy = now_position[1]
                self.zz = now_position[2]
                print ("now_quaternion: ")
                print (now_quaternion)
                self.now_quaternion = now_quaternion
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logerr('Unknown pose: %s' % e)

            euler = tf.transformations.euler_from_quaternion(now_quaternion)
            self.roll = euler[0]
            self.pitch = euler[1]
            self.yaw = euler[2]
            print ("roll: ",self.roll, ", pitch: ",self.pitch, ", yaw: ", self.yaw)


        def rpy_point():
            #  inverse kinematic
            p_kdl = PyKDL.Frame()
            joints = PyKDL.JntArray(6)
            # joint <- angle
            for i in range(6):
                #numpy.random.random_sample(): 0.0 <= && < 0.1
                #joints[i] = (np.random.random_sample() * 2 - 1) * np.pi
                joints[i] = 0.0
            q_init=joints #initial angles

            x = self.xx
            y = self.yy
            z = self.zz
            vik = PyKDL.ChainIkSolverVel_pinv(self.chain) #inverse kinematics velocity
            #ik=PyKDL.ChainIkSolverPos_NR(self.chain, self.fk_solver, vik)
            ik=PyKDL.ChainIkSolverPos_NR(self.chain, self.fk_solver, vik,100,1e-3)

            desiredFrame=PyKDL.Frame(PyKDL.Vector(x,y,z))
            print ("Desired Position: ")
            print (desiredFrame.p)

            q_out=PyKDL.JntArray(6)
            ik.CartToJnt(q_init,desiredFrame,q_out)
            q_out[0] = q_out[0] % (np.pi * 2)
            q_out[1] = q_out[1] % (np.pi * 2)
            q_out[2] = q_out[2] % (np.pi * 2)
            q_out[3] = q_out[3] % (np.pi * 2)
            q_out[4] = q_out[4] % (np.pi * 2)
            q_out[5] = q_out[5] % (np.pi * 2)
            print ("Output angles in rads: ")
            print (q_out)

            self.fk_solver.JntToCart(q_out, p_kdl)

            print ("last x,y,z: ")
            print (p_kdl.p)

            p = CartesianTrajectoryPoint()
            p.pose.position.x = self.xx
            p.pose.position.y = self.yy
            p.pose.position.z = self.zz

            #q = PyKDL.Rotation.GetQuaternion(p_kdl.M)
            roll = self.roll
            pitch = self.pitch
            yaw = self.yaw
            quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            p.pose.orientation.x = quat[0]
            p.pose.orientation.y = quat[1]
            p.pose.orientation.z = quat[2]
            p.pose.orientation.w = quat[3]
            print ("Quaternion: ")
            print (p.pose.orientation)
            return p

        def test_point1():
            p_kdl = PyKDL.Frame()
            joints = PyKDL.JntArray(6)
            for i in range(6):
                joints[i] = (np.random.random_sample() * 2 - 1) * np.pi
            self.fk_solver.JntToCart(joints, p_kdl)

            p = CartesianTrajectoryPoint()
            p.pose.position.x = -0.1
            p.pose.position.y = -0.1
            p.pose.position.z = 1.2
            q = PyKDL.Rotation.GetQuaternion(p_kdl.M)
            p.pose.orientation.x = 0
            p.pose.orientation.y = 0
            p.pose.orientation.z = 0
            p.pose.orientation.w = 1
            return p

        def org_point():
            p_kdl = PyKDL.Frame()
            joints = PyKDL.JntArray(6)
            for i in range(6):
                joints[i] = 0.0
            self.fk_solver.JntToCart(joints, p_kdl)
            print ("last x,y,z: ")
            print (p_kdl.p)

            p = CartesianTrajectoryPoint()
            p.pose.position.x = p_kdl.p[0]
            p.pose.position.y = p_kdl.p[1]
            p.pose.position.z = p_kdl.p[2]

            q = PyKDL.Rotation.GetQuaternion(p_kdl.M)
            p.pose.orientation.x = q[0]
            p.pose.orientation.y = q[1]
            p.pose.orientation.z = q[2]
            p.pose.orientation.w = q[3]
            print ("org Quaternion: ")
            print (p.pose.orientation)
            return p


        duration = 5
        while not rospy.is_shutdown():
            now_link6()
            #select_menue = input('select character : ')
            select_menue = raw_input('select character : ')

            print(type(select_menue))
            print('path1 ',select_menue)
 
            if select_menue == 'o' :
                p1 = org_point()
            elif select_menue == 'x' :
                self.xx = float(input('input x-axis : '))
                p1 = rpy_point()
            elif select_menue == 'x+' :
                self.xx += 0.05
                p1 = rpy_point()
            elif select_menue == 'x-' :
                self.xx -= 0.05
                p1 = rpy_point()
            elif select_menue == 'y' :
                self.yy = float(input('input y-axis : '))
                p1 = rpy_point()
            elif select_menue == 'y+' :
                self.yy += 0.05
                p1 = rpy_point()
            elif select_menue == 'y-' :
                self.yy -= 0.05
                p1 = rpy_point()
            elif select_menue == 'z' :
                self.zz = float(input('input z-axis : '))
                p1 = rpy_point()
            elif select_menue == 'z+' :
                self.zz += 0.05
                p1 = rpy_point()
            elif select_menue == 'z-' :
                self.zz -= 0.05
                p1 = rpy_point()
            elif select_menue == 'r' :
                self.roll = float(input('input roll : '))
                p1 = rpy_point()
            elif select_menue == 'r+' :
                self.roll += 0.2
                p1 = rpy_point()
            elif select_menue == 'r-' :
                self.roll -= 0.2
                p1 = rpy_point()
            elif select_menue == 'p' :
                self.pitch = float(input('input pitch : '))
                p1 = rpy_point()
            elif select_menue == 'p+' :
                self.pitch += 0.2
                p1 = rpy_point()
            elif select_menue == 'p-' :
                self.pitch -= 0.2
                p1 = rpy_point()
            elif select_menue == 'ya' :
                self.yaw = float(input('input yaw : '))
                p1 = rpy_point()
            elif select_menue == 'ya+' :
                self.yaw += 0.2
                p1 = rpy_point()
            elif select_menue == 'ya-' :
                self.yaw -= 0.2
                p1 = rpy_point()
            elif select_menue == 'a' :
                self.xx = float(input('input x-axi : '))
                self.yy = float(input('input y-axi : '))
                self.zz = float(input('input z-axi : '))
                self.roll = float(input('input roll : '))
                self.pitch = float(input('input pitch : '))
                self.yaw = float(input('input yaw : '))
                p1 = rpy_point()
            elif select_menue == 'h' OR select_menue == 'help':
                print("change parameter")
                print("o: origin, x: x-axis, y: y-axis, z: z-axis, r: roll, p: pitch, ya: yaw, a: all input")
                print("x+: x-axis 0.05m up, x-: x-axis 0.05m down")
                print("y+: y-axis 0.05m up, y-: y-axis 0.05m down")
                print("z+: z-axis 0.05m up, z-: z-axis 0.05m down")
                print("r+: roll 0.2radian up, r-: roll 0.2radian down")
                print("p+: pitch 0.2radian up, p-: pitch 0.2radian down")
                print("ya+: yaw 0.2radian up, ya-: yaw 0.2radian down")
                print("h or help: help message")
                continue
            else : 
                print("unknown menu character: ", select_menue)
                continue
            
            print('x-axi : ',self.xx, ', y-axi : ',self.yy, ', z-axi : ',self.zz)
            print('roll : ',self.roll, ', pitch : ',self.pitch, ', yaw : ',self.yaw)

            p1.time_from_start = rospy.Duration(duration)
            #p2.time_from_start = rospy.Duration(duration)

            goal = FollowCartesianTrajectoryGoal()
            goal.trajectory.points.append(p1)
            #goal.trajectory.points.append(p2)

            self.client.send_goal(goal)
            self.client.wait_for_result()
            print("Result: {}".format(self.client.get_result()))
        #return self.client.get_result()

    def clean_shutdown(self, msg=None):
        """ Cancel goal on Ctrl-C """
        self.client.cancel_goal()
        if msg is not None:
            print(msg)
        sys.exit(0)


if __name__ == '__main__':

    try:
        rospy.init_node('action_test_client')
        client = Client()
        signal.signal(signal.SIGINT, lambda sig, frame: client.clean_shutdown("\nGoal canceled."))
        #result = client.test()
        client.test()
        #print("Result: {}".format(result))

    except rospy.ROSInterruptException:
        pass
