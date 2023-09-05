#!/usr/bin/env python

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from pycrazyswarm import *

from std_msgs.msg import Bool, Int16
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped

class CrazyflieDriver:

    def __init__(self):
        rospy.init_node("crazyflie_driver_node", anonymous=True)
        ctrl_rate = rospy.get_param("ctrl_rate", 10)
        ctrl_rate = max(10, ctrl_rate)
        pos_cmd_topic = rospy.get_param("pos_cmd_topic", "/crazyflie/pos_cmd")
        vel_cmd_topic = rospy.get_param("vel_cmd_topic", "/crazyflie/vel_cmd")
        acc_cmd_topic = rospy.get_param("acc_cmd_topic", "/crazyflie/acc_cmd")
        mode_cmd_topic = rospy.get_param("mode_cmd_topic", "/crazyflie/mode_cmd")
        init_height = rospy.get_param("init_height", 1.0)

        self.target_pos = np.array([0, 0, init_height])
        self.target_vel = np.array([0, 0, 0])
        self.target_acc = np.array([0, 0, 0])
        self.target_att_quat = np.array([1, 0, 0, 0]) # [w, x, y ,z]
        self.target_ang_rate = np.array([0, 0, 0])

        self.flight_mode = 0   # 0: land, 1: takeoff, 2: offboard
        self.offb_ready = False
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        self.num_cfs = len(self.swarm.allcfs.crazyflies)
        self.cf = self.swarm.allcfs.crazyflies[0]

        self.pos_pub = rospy.Publisher("/crazyflie/realtime_pos", PoseStamped)
        self.ready_pub = rospy.Publisher("/crazyflie/offb_ready", Bool)
        pos_sub = rospy.Subscriber(pos_cmd_topic, PoseStamped, self.posSubCb)
        vel_sub = rospy.Subscriber(vel_cmd_topic, Twist, self.velSubCb)
        acc_sub = rospy.Subscriber(acc_cmd_topic, Vector3Stamped, self.accSubCb)
        takeoff_land_sub = rospy.Subscriber(mode_cmd_topic, Int16, self.modeSubCb)
        timer = rospy.timer.Timer(rospy.Rate(ctrl_rate), self.timerCb)

        self.tmp_pos = self.cf.position()   # wait for first position to arrive
        self.target_pos[0] = self.tmp_pos[0]
        self.target_pos[1] = self.tmp_pos[1]
        self.timeHelper.sleep(1.0)

        self.pos_cnt = 0

    def timerCb(self, e: rospy.timer.TimerEvent):
        self.tmp_pos = self.cf.position()
        if self.flight_mode == 0:
            return
        elif self.flight_mode == 1:
            if not self.offb_ready:
                if (self.tmp_pos[2]-self.target_pos[2])**2 < 0.05:
                    self.pos_cnt += 1
                else:
                    self.pos_cnt = 0
                if self.pos_cnt >= 50:
                    self.offb_ready = True
        else:
            des_att = Rot.from_quat([self.target_att_quat[1],
                                     self.target_att_quat[2],
                                     self.target_att_quat[3],
                                     self.target_att_quat[0]]) # [x, y, z, w]
            des_att_euler = des_att.as_euler('xyz')
            self.cf.cmdFullState(self.target_pos, self.target_vel, self.target_acc, des_att_euler[2], self.target_ang_rate)

        pos_msg = PoseStamped()
        pos_msg.header.stamp = rospy.Time.now()
        pos_msg.pose.position.x = self.tmp_pos[0]
        pos_msg.pose.position.y = self.tmp_pos[1]
        pos_msg.pose.position.z = self.tmp_pos[2]
        self.pos_pub.publish(pos_msg)
        


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
        
    def modeSubCb(self, msg: Int16):
        mode = msg.data
        if self.flight_mode == mode:
            return
        elif mode == 4:
            self.cf.stop()
            self.offb_ready = False
            self.flight_mode = 0
            return
        else:
            if self.flight_mode == 0:
                if mode == 1:
                    self.cf.takeoff(targetHeight=self.target_pos[2], duration=2.0)
                    self.flight_mode = mode
                else:
                    rospy.logwarn("Switch to offboard before takeoff not allowed!")
                    self.ready_pub.publish(False)
                    return
            elif self.flight_mode == 1:
                if mode == 2:
                    if self.offb_ready:
                        self.flight_mode = mode
                        self.ready_pub.publish(True)
                else:
                    self.cf.land(targetHeight=0.03, duration=2.0)
                    self.offb_ready = False
                    self.flight_mode = 0
            elif self.flight_mode == 2:
                if mode == 0:
                    self.cf.notifySetpointsStop()
                    self.cf.land(targetHeight=0.03, duration=2.0)
                    self.offb_ready = False
                    self.flight_mode = 0
                else:
                    self.cf.notifySetpointsStop()
                    self.cf.goTo(self.target_pos, 0.0, 1.0, False)
                    self.flight_mode = mode







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