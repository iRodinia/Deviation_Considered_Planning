#!/usr/bin/env python

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from pycrazyswarm import *

from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped

class CrazyflieDriver:

    def __init__(self):
        rospy.init_node("crazyflie_driver_node", anonymous=True)
        ctrl_rate = rospy.get_param("ctrl_rate", 10)
        ctrl_rate = max(10, ctrl_rate)
        pos_cmd_topic = rospy.get_param("pos_cmd_topic", "/crazyflie/pos_cmd")
        vel_cmd_topic = rospy.get_param("vel_cmd_topic", "/crazyflie/vel_cmd")
        acc_cmd_topic = rospy.get_param("acc_cmd_topic", "/crazyflie/acc_cmd")
        init_height = rospy.get_param("init_height", 1.0)

        self.target_pos = np.array([0, 0, init_height])
        self.target_vel = np.array([0, 0, 0])
        self.target_acc = np.array([0, 0, 0])
        self.target_att_quat = np.array([1, 0, 0, 0])
        self.target_ang_rate = np.array([0, 0, 0])

        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.num_cfs = len(self.swarm.allcfs.crazyflies)
        self.offb_ready = False

        pos_sub = rospy.Subscriber(pos_cmd_topic, PoseStamped, self.posSubCb)
        vel_sub = rospy.Subscriber(vel_cmd_topic, Twist, self.velSubCb)
        acc_sub = rospy.Subscriber(acc_cmd_topic, Vector3Stamped, self.accSubCb)
        timer = rospy.timer.Timer(rospy.Rate(ctrl_rate), self.timerCb)

    def timerCb(self, e: rospy.timer.TimerEvent):
        cf = self.swarm.allcfs.crazyflies[0]
        

    def posSubCb(self, msg: PoseStamped):
        self.target_pos = np.array([msg.pose.position.x,
                                    msg.pose.position.y,
                                    msg.pose.position.z])
        self.target_att_quat = np.array([msg.pose.orientation.w,
                                         msg.pose.orientation.x,
                                         msg.pose.orientation.y,
                                         msg.pose.orientation.z])

    def velSubCb(self, msg: Twist):
        self.target_vel = np.array([msg.linear.x,
                                    msg.linear.y,
                                    msg.linear.z])
        self.target_ang_rate = np.array([msg.angular.x,
                                         msg.angular.y,
                                         msg.angular.z])

    def accSubCb(self, msg: Vector3Stamped):
        self.target_acc = np.array([msg.vector.x,
                                    msg.vector.y,
                                    msg.vector.z])






def executeTrajectory(timeHelper, cf, trajpath, rate=100, offset=np.zeros(3)):
    traj = uav_trajectory.Trajectory()
    traj.loadcsv(trajpath)

    start_time = timeHelper.time()
    while not timeHelper.isShutdown():
        t = timeHelper.time() - start_time
        if t > traj.duration:
            break

        e = traj.eval(t)
        cf.cmdFullState(
            e.pos + np.array(cf.initialPosition) + offset,
            e.vel,
            e.acc,
            e.yaw,
            e.omega)

        timeHelper.sleepForRate(rate)


if __name__ == "__main__":
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    rate = 30.0
    Z = 0.5

    cf.takeoff(targetHeight=Z, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)

    executeTrajectory(timeHelper, cf, "figure8.csv", rate, offset=np.array([0, 0, 0.5]))

    cf.notifySetpointsStop()
    cf.land(targetHeight=0.03, duration=Z+1.0)
    timeHelper.sleep(Z+2.0)